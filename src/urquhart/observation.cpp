#include <observation.hpp>

namespace urquhart {

Observation::Observation(Points& freshLandmarks) {
    landmarks = freshLandmarks;
    std::vector<Polygon> triangles;
    delaunayTriangulationFromScratch(triangles);
    H = new Tree(triangles);
    urquhartTesselation_();
};

Observation::Observation(std::vector<std::vector<double>>& freshLandmarks) {
    // Legacy method for processing vector input
    landmarks.resize(2, freshLandmarks.size());
    for (int i = 0; i < freshLandmarks.size(); ++i) {
        landmarks.col(i) = Eigen::Vector2d{freshLandmarks[i][0], freshLandmarks[i][1]};
    }

    std::vector<Polygon> triangles;
    delaunayTriangulationFromScratch(triangles);
    H = new Tree(triangles);
    urquhartTesselation_();
};

void Observation::view() { H->view_h2_polygons(); }

// std::vector<size_t> Observation::getCurrentAndChildren() { return H->view_h2_polygons(); }

void Observation::urquhartTesselation_() {
    for (size_t leaf : H->traverse()) {

        // Identify the longest edge of this triangle
        Polygon p = H->get_vertex(leaf);
        int neighId = -1, longestEdgeIdx = -1;
        double longestEdgeLen = -1;
        for (int i=0; i < p.n; ++i) {
            double thisEdgeLen = triangulationEdgeLengths(p.edgeRefs(i));
            if (thisEdgeLen > longestEdgeLen) {
                longestEdgeLen = thisEdgeLen;
                neighId = p.neighbors(i);
                longestEdgeIdx = i;
            }
        }
        
        // When the longest edge is at the outer boundary of the triangulation,
        // don't merge the triangle with anything --> it will still be a direct child of the Tree's root node
        if (neighId == -1) continue;

        // If these two triangles have not already been merged with each other, continue
        size_t leafAncestorIdx = H->get_ancestor(leaf), neighAncestorIdx = H->get_ancestor(neighId);
        if (leafAncestorIdx != neighAncestorIdx) {

            // Set the current Polygon to its parent if this triangle has already been merged with something else
            int longestEdgeObsIdx = p.edgeRefs(longestEdgeIdx);
            if (leafAncestorIdx != leaf) p = H->get_vertex(leafAncestorIdx);

            // Merge the neighboring polygon with whatever the current Polygon is
            // (drop the shared edge between the Polygons and combine their vertex and edge references)
            Polygon n = H->get_vertex(neighAncestorIdx), merged = mergePolygons_(p, n, longestEdgeObsIdx);

            // If "mergePolygons" was successful, merge the Tree nodes in the geometric hierarchy
            if (merged.n != -1) H->merge_op(leafAncestorIdx, neighAncestorIdx, merged);
        }
    }
}


Polygon Observation::mergePolygons_(Polygon& p, Polygon& n, int edgeIndexInObs) {
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
    p.coolRotate(1+edgeIndexInP % p.n);
    n.coolRotate(edgeIndexInN);


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
    
    // Initialize empty structures for edge data
    triangulationEdges.resize(2, 1);
    triangulationEdgeLengths.resize(1);


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
    // TODO the option "Fx" computes the convex hull, too; would be useful to preallocate space for the edges and edgeLengths


    // the facet ids are confusing, we want to map the good facets to order of appearance
    size_t fIdx = 0;
    std::map<size_t, size_t> id_map;
    for (const auto& e : q.facetList()) {
        if (e.isGood()) id_map[e.id()] = ++fIdx;
    }

    // Store edge lengths in a matrix, store references to these lengths in each Polygon
    int edgeListSize = 0;
    // TODO resize "triangulationEdges" with number of unique edges in triangulation:
    // 3n - h - 3 <-- n=#vertices, h=#vertices in convex hull

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
        Eigen::Vector3i edgeIDs;
        for (int srcID = 0, dstID = 1; srcID < 3; dstID = (++srcID+1)%3) {
            // Compute this edge's bidirectional UID 
            size_t edgeID = cantorPairing(ldmkIds[srcID], ldmkIds[dstID]);

            // Only process new edges
            if (edgeRefMap.find(edgeID) == edgeRefMap.end()) {
                edgeRefMap[edgeID] = edgeListSize++;
                
                triangulationEdges.conservativeResize(2, edgeListSize);      // TODO until I'm able to get number of edges in the triangulation,
                triangulationEdgeLengths.conservativeResize(edgeListSize);   //      I will need to allocate space as I go
                
                // order of landmark references does not matter because "triangulationEdges" because we preserve vertex order in Polygons
                triangulationEdges.col(edgeListSize-1) = Eigen::Vector2i{ldmkIds(srcID), ldmkIds(dstID)};
                triangulationEdgeLengths(edgeListSize-1) = euclideanDistance2D(landmarks.col(ldmkIds(srcID)), landmarks.col(ldmkIds(dstID)));
                // TODO reference "landmarks"
            }
            edgeIDs(srcID) = edgeRefMap[edgeID];

            // Accumulate the signed area of this triangle while processing edges
            // A negative value indicates the vertices/edges are ordered clockwise
            vertexOrdering += landmarks(Eigen::placeholders::all, {ldmkIds[srcID], ldmkIds[dstID]}).determinant();
            
            // SIDE EFFECT: use this loop to shift neighbor order to align with edges (edge[i] will be shared with neighIds[i])
            neighIds(dstID) = auxNeighIds[srcID];
        }
        polygons.push_back(Polygon(ldmkIds, edgeIDs, neighIds, vertexOrdering < 0, landmarks, triangulationEdgeLengths));
    }
};


} // urquhart