#include <observation.hpp>

namespace urquhart {

Observation::Observation(Points& freshLandmarks) {
    landmarks = Eigen::Ref<Points>(freshLandmarks);
    computeHierarchy();
};

Observation::Observation(std::vector<std::vector<double>>& freshLandmarks) {
    // Legacy method for processing input from std::vector
    landmarks.resize(2, freshLandmarks.size());
    for (int i = 0; i < freshLandmarks.size(); ++i) {
        landmarks.col(i) = Eigen::Vector2d{freshLandmarks[i][0], freshLandmarks[i][1]};
    }

    computeHierarchy();
};

// CAREFUL!! destructor can purge the hierarchy before we actually want it gone
// Observation::~Observation() { delete hier; }
Observation::~Observation() {}

void Observation::view() { hier->viewPolygons(std::cout); }


void Observation::computeHierarchy() {

    // Initialize hierarchy with Delaunay triangulation
    std::vector<Polygon> triangles;
    delaunayTriangulationFromScratch(triangles);

    if (hier != NULL) delete hier;
    hier = new Hierarchy(triangles); // TODO use shared pointer here?

    // Merge triangles into new polygons by removing their longest edges
    urquhartTesselation();
}

void Observation::recomputeEdgeLengthsAndDescriptors() {
    // Recompute all polygon descriptors once
    for (Eigen::Index i = 0; i < triangulationEdges.cols(); ++i) {
        triangulationEdgeLengths(i) = euclideanDistance2D(landmarks.col(triangulationEdges(0, i)), landmarks.col(triangulationEdges(1, i)));
    }

    // Recompute all polygon descriptors once
    for (const auto& pIdx : hier->getChildrenIds(0)) {  // Iterate over polygons
        hier->getPolygon(pIdx).recomputeDescriptor(landmarks, triangulationEdgeLengths);
        if (hier->getPolygon(pIdx).n > 3) {
            for (const auto& tIdx : hier->getChildrenIds(pIdx)) {   // Iterate over triangles
                hier->getPolygon(tIdx).recomputeDescriptor(landmarks, triangulationEdgeLengths);
            }
        }
    }
}

PtLoc Observation::tfLdmk(Eigen::Index colNum, const Eigen::Matrix3d& tf) {
    // Obtain the position of this tree in the global frame
    Eigen::Matrix3d localPointTf {
        {1, 0, landmarks(0, colNum)},
        {0, 1, landmarks(1, colNum)},
        {0, 0, 1},
    };
    return (tf * localPointTf)(Eigen::seq(0,1), 2);
}


void Observation::urquhartTesselation() {
    // Process every triangle to construct an Urquhart tessellation
    for (const auto& leaf : hier->getChildrenIds(0)) {
        Polygon p = hier->getPolygon(leaf);
        
        // When the longest edge is at the outer boundary of the triangulation (aka doesn't neighbor another triangle), 
        // don't merge the triangle with anything --> it will still be a direct child of the Tree's root node
        if (p.neighbors(p.longestEdgeIdx) == -1) continue;

        // Continue processing if these two triangles have not already been merged with each other
        int leafAncestorIdx = hier->getAncestorId(leaf), neighAncestorIdx = hier->getAncestorId(p.neighbors(p.longestEdgeIdx));
        if (leafAncestorIdx != neighAncestorIdx) {

            // Set the current Polygon to its parent if this triangle has already been merged with something else
            int triangulationEdgeIdx = p.edgeRefs(p.longestEdgeIdx);
            if (leafAncestorIdx != leaf) p = hier->getPolygon(leafAncestorIdx);

            // Merge the neighboring polygon with whatever the current Polygon is
            // (drop the shared edge between the Polygons and combine their vertex and edge references)
            Polygon n = hier->getPolygon(neighAncestorIdx), merged = mergePolygons(p, n, triangulationEdgeIdx);

            // If "mergePolygons" was successful, merge the Tree nodes in the geometric hierarchy
            if (merged.n != -1) hier->mergeOp(leafAncestorIdx, neighAncestorIdx, merged);
        }
    }
}


Polygon Observation::mergePolygons(Polygon& p, Polygon& n, int edgeIndexInObs) {
    // ORIGINAL DOCSTRING
    // (0,1), (1,2), (2,0)
    //                     => (0,1), (1,2), (2,4), (4,0)
    // (2,0), (0,4), (4,2)
    // 
    // "rotate" puts idx as first element, for p we want the common edge to be the last elem
    // we want the element after commonEdge to be the first, i.e. commonEdge is the last element


    // Reverse the order of the nieghbor's elements if the order is opposite to what the merging polygon has
    bool hasReversed = p.isClockwise != n.isClockwise;
    if (hasReversed) n.reverse();

    // Ensure that the neighbor polygon is aware of the shared edge
    int edgeIndexInN = n.findEdgeIndex(edgeIndexInObs);
    if (edgeIndexInN == -1) return Polygon();   // if not, cancel the merge

    // REMINDER: "p" may no longer be the original triangle, so we need to relocate the index of the triangle's longest edge
    int edgeIndexInP = p.findEdgeIndex(edgeIndexInObs);


    if (!hasReversed) edgeIndexInN = edgeIndexInN+1 % n.n;
    // Rotate both polygons to so they follow the given configuration:
    //    - The vertices of the shared polygons should be the first and last element in the list of landmark references 
    //    - In p, the shared edge should be the LAST  element in its list
    //    - In n, the shared edge should be the FIRST or LAST element in its list
    p.rotateElements(1+edgeIndexInP % p.n);
    n.rotateElements(edgeIndexInN);


    // Combine the Polygon data
    Eigen::VectorXi combinedLdmkRef(p.n+n.n-2), combinedEdgeRef(p.n+n.n-2), combinedNeighbors(p.n+n.n-2);
    
    // We ALWAYS expect shared vertices to be on either end of polygon's vector (opposing order)
    // Concatenation of landmark references: (should be in correct order now)
    // 1st shared vertex << p's non-shared vertices << 2nd shared vertex << n's non-shared vertices
    combinedLdmkRef << p.landmarkRefs, n.landmarkRefs.middleRows(1,n.n-2);

    if (hasReversed) {
        // When the neighbor's element order has been flipped, the edge reference to delete lies at the end of p and beginning of n
        combinedEdgeRef << p.edgeRefs.head(p.n-1), n.edgeRefs.tail(n.n-1);
        combinedNeighbors << p.neighbors.head(p.n-1), n.neighbors.tail(n.n-1);
    } else {
        // When the neighbor's element order has NOT been flipped, the edge reference to delete lies at the end of both p and n
        combinedEdgeRef << p.edgeRefs.head(p.n-1), n.edgeRefs.head(n.n-1);
        combinedNeighbors << p.neighbors.head(p.n-1), n.neighbors.head(n.n-1);
    }

    // TODO if we want to preserve the longest edge for Polygons, need to do comparison of the two longest edges here and pass through the bigger one
    // Reminder: since we are dropping a triangle's longest edge, we'd need to find the next largest one
    //           hence we would probably need to rank the edges length to make that step easier here
    return Polygon(combinedLdmkRef, combinedEdgeRef, combinedNeighbors, p.isClockwise, landmarks, triangulationEdgeLengths);
}


void Observation::printPolygon(const Polygon& p) {
    std::cout << "***********************************************" << std::endl;
    std::cout << "N = " << p.n << std::endl;
    std::cout << "Points: (" << p.landmarkRefs.transpose() << ")" << std::endl << landmarks(Eigen::placeholders::all, p.landmarkRefs) << std::endl;
    std::cout << "Clockwise? = " << p.isClockwise << std::endl;
    std::cout << "Edges: (" << p.edgeRefs.transpose() << ")" << std::endl << triangulationEdges(Eigen::placeholders::all, p.edgeRefs) << std::endl;
    std::cout << "Edge Lengths:" << std::endl << triangulationEdgeLengths(p.edgeRefs).transpose() << std::endl;
    std::cout << "Neighbors:" << std::endl << p.neighbors.transpose() << std::endl;
    std::cout << "Descriptor:" << std::endl << p.descriptor.transpose() << std::endl;
    std::cout << "***********************************************" << std::endl;
}

void Observation::delaunayTriangulationFromScratch(std::vector<Polygon>& polygons) {

    // TODO insert data directly from Eigen Matrix, not some goofy vector
    std::vector<double> qhull_points_data(landmarks.size());
    for (size_t pidx = 0; pidx < landmarks.cols(); ++pidx) {
        qhull_points_data[pidx * 2 + 0] = landmarks(0, pidx);
        qhull_points_data[pidx * 2 + 1] = landmarks(1, pidx);
    }
    // Read tree positions into QHull
    orgQhull::PointCoordinates qhull_points(2, "");
    // qhull_points.append(landmarks.cols(), landmarks.data());
    qhull_points.append(qhull_points_data);

    // Compute Delaunay Triangulation
    orgQhull::Qhull q;
    q.runQhull(qhull_points.comment().c_str(), qhull_points.dimension(),
                   qhull_points.count(), qhull_points.coordinates(), "Qt Qbb Qc Qz Q12 d");

    // Facet ids are confusing; we want to map the good facets in order of appearance
    size_t fIdx = 0;                // <---- This value will equate to the number of Delaunay triangles
    std::map<size_t, size_t> id_map;
    for (const auto& e : q.facetList()) {
        if (e.isGood()) id_map[e.id()] = ++fIdx;
    }

    // n = #vertices, h = #vertices in convex hull 
    // edges in triangulation       = 3n - h - 3
    // triangles in triangulation   = 2n - h - 2
    // THEREFORE --> #edges = #triangles + n - 1
    int numEdges = fIdx + landmarks.cols() - 1, edgeListIdx = 0, polygonIdx = 0;

    // Reserve space to store polygon and edge data for geometric hierarchy
    polygons.resize(fIdx);
    triangulationEdges.resize(2, numEdges);
    triangulationEdgeLengths.resize(numEdges);
    edgeRefMap.clear(); edgeRefMap.reserve(numEdges);

    // Delaunay regions as a vector of vectors
    orgQhull::QhullFacetListIterator k(q.facetList());
    while (k.hasNext()) {
        orgQhull::QhullFacet f = k.next();
        if (!f.isGood()) continue;

        // Get the neighbors for each edge of this triangle
        // This only provides neighbors that are also a good facet, so edges without neighbors are assigned -1
        orgQhull::QhullFacetSet neighborsSet(f.neighborFacets());
        std::vector<int> auxNeighIds;
        for (const auto& e : neighborsSet) auxNeighIds.push_back(e.isGood() ? id_map[e.id()] : -1); // iterates three times
        // TODO change to Eigen::Vector3i? permute inplace to get final neighIDs?

        // Store references to this triangle's vertices
        orgQhull::QhullVertexSetIterator i(f.vertices());
        Eigen::Vector3i ldmkIds, neighIds;
        for (int idx = 0; i.hasNext(); ++idx) ldmkIds(idx) = i.next().point().id();

        // Process the edges of this triangle
        double vertexOrdering = 0;
        int longestEdgeIdx = -1;
        Eigen::Vector3i edgeIDs;
        for (int srcID = 0, dstID = 1; srcID < 3; dstID = (++srcID+1)%3) {
            // Compute this edge's bidirectional UID 
            size_t edgeID = cantorPairing(ldmkIds[srcID], ldmkIds[dstID]);

            // Store edge data only if it hasn't been seen yet
            if (edgeRefMap.find(edgeID) == edgeRefMap.end()) {
                // REMINDER: order of edges <src,dst> DOES NOT MATTER because Polygons preserve it through order of landmarks
                triangulationEdges.col(edgeListIdx) = Eigen::Vector2i{ldmkIds(srcID), ldmkIds(dstID)};
                triangulationEdgeLengths(edgeListIdx) = euclideanDistance2D(landmarks.col(ldmkIds(srcID)), landmarks.col(ldmkIds(dstID)));
                edgeRefMap[edgeID] = edgeListIdx++;
            }
            edgeIDs(srcID) = edgeRefMap[edgeID];

            // Track the triangle's longest edge
            if (longestEdgeIdx == -1 || isNextEdgeBigger(edgeIDs(longestEdgeIdx), edgeIDs(srcID))) longestEdgeIdx = srcID;

            // Accumulate the signed area of this triangle while processing edges
            // A negative value indicates the vertices/edges are ordered clockwise
            vertexOrdering += landmarks(Eigen::placeholders::all, {ldmkIds[srcID], ldmkIds[dstID]}).determinant();
            
            // LOOP SIDE EFFECT: shift neighbor order to align with edges (edge[i] will be shared with neighIds[i])
            neighIds(dstID) = auxNeighIds[srcID];
        }

        // Store the processed triangle and calculate its descriptor
        polygons[polygonIdx++] = Polygon(ldmkIds, edgeIDs, neighIds, vertexOrdering < 0, landmarks, triangulationEdgeLengths, longestEdgeIdx);
    }
};


bool Observation::isNextEdgeBigger(int prevIdx, int nextIdx) { return triangulationEdgeLengths(nextIdx) > triangulationEdgeLengths(prevIdx); }

const PtLoc& Observation::ldmk(Eigen::Index colNum) const { return landmarks.col(colNum); }
// const PtLoc& Observation::ldmk(Eigen::Index colNum) const { return Eigen::Ref<PtLoc>(landmarks.col(colNum)); }
const double& Observation::ldmkX(Eigen::Index colNum) const { return landmarks(0, colNum); }
const double& Observation::ldmkY(Eigen::Index colNum) const { return landmarks(1, colNum); }
const Points& Observation::ldmks(Eigen::VectorXi indices) const { return landmarks(Eigen::placeholders::all, indices); }
// const Points& Observation::ldmks(Eigen::VectorXi indices) const { return Eigen::Ref<Points>(landmarks(Eigen::placeholders::all, indices)); }
} // urquhart