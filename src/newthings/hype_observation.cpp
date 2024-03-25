#include <hype_observation.hpp>

namespace hype_urq {

Observation::Observation(Points& freshLandmarks){
    landmarks = freshLandmarks;
    std::vector<EpicPolygon> triangles;
    delaunayTriangulationFromScratch(freshLandmarks, triangles);
    H = new Tree(triangles);
    // abort();
    urquhartTesselation_();
};

void Observation::view() { H->view_h2_polygons(); }

// std::vector<size_t> Observation::getCurrentAndChildren() { return H->view_h2_polygons(); }

void Observation::urquhartTesselation_() {
    for (size_t leaf : H->traverse()) {
        // if (leaf == 12) abort();
        // if (leaf == 20) abort();

        // Identify the longest edge of this triangle
        EpicPolygon p = H->get_vertex(leaf);
        // std::cout << "Leaf #: " << leaf << ", n=" << p.n << std::endl;
        // printPolygon(p);

        int neighId = -1, longestEdgeIdx = -1;
        double longestEdgeLen = -1;
        for (int i=0; i < p.n; ++i) {
            double thisEdgeLen = triangulationEdgeLengths(p.edgeRefs[i]);
            // std::cout << i << "- thisEdgeLen = " << thisEdgeLen << std::endl;

            if (thisEdgeLen > longestEdgeLen) {
                // std::cout << "bigger than = " << longestEdgeLen << std::endl;
                longestEdgeLen = thisEdgeLen;
                neighId = p.neighbors[i];
                longestEdgeIdx = i;
            }
        }
        // std::cout << "neighId = " << neighId << std::endl;
        // std::cout << "longestEdgeIdx = " << longestEdgeIdx << std::endl;
        // if (leaf == 11) abort();
        
        // When the longest edge is at the outer boundary of the triangulation,
        // don't merge the triangle with anything --> it will still be a direct child of the root of the Tree
        if (neighId == -1) continue;
        // std::cout << "neighPos = " << longestEdgeIdx << std::endl;



        // Drop the edge if these two triangles have not already been merged with each other
        // size_t commonEdgeKey = p.edgeRefs[longestEdgeIdx], leafAncestorIdx = H->get_ancestor(leaf), neighAncestorIdx = H->get_ancestor(neighId);
        size_t leafAncestorIdx = H->get_ancestor(leaf), neighAncestorIdx = H->get_ancestor(neighId);
        // printPolygon(H->get_vertex(neighAncestorIdx));
        // std::cout << "leafAncestorIdx = " << leafAncestorIdx << ", neighAncestorIdx = " << neighAncestorIdx << std::endl;
        if (leafAncestorIdx != neighAncestorIdx) {
            // Get the parent Polygon of this triangle if it has already been merged with another
            int longestEdgeObsIdx = p.edgeRefs(longestEdgeIdx);
            if (leafAncestorIdx != leaf) p = H->get_vertex(leafAncestorIdx);
            // if (leafAncestorIdx != leaf) {
            //     p = H->get_vertex(leafAncestorIdx);
            // }


            // std::cout << "leafAncestorIdx: " << leafAncestorIdx << ", neighAncestorIdx: " << neighAncestorIdx << std::endl;

            // Merge the neighboring polygon with this triangle's highest available Polygon
            EpicPolygon n = H->get_vertex(neighAncestorIdx), merged = mergePolygons_(p, n, longestEdgeObsIdx);

            // If "mergePolygons" was successful, merge the tree elements in the geometric hierarchy
            // std::cout << "Merged poly vertex count: " << merged.n << std::endl;
            // if (leaf == 11) printPolygon(merged);
            if (merged.n != -1) H->merge_op(leafAncestorIdx, neighAncestorIdx, merged);
            // H->view_tree();
            // printPolygon(merged);
            // if (merged.n == 4) abort();
        }
    }
}


EpicPolygon Observation::mergePolygons_(EpicPolygon& p, EpicPolygon& n, int edgeIndexInObs) {
    // ORIGINAL DOCSTRING
    // (0,1), (1,2), (2,0)
    //                     => (0,1), (1,2), (2,4), (4,0)
    // (2,0), (0,4), (4,2)
    // 
    // "rotate" puts idx as first element, for p we want the common edge to be the last elem
    // we want the element after commonEdge to be the first, i.e. commonEdge is the last element


    // std::cout << "p.ldmkRefs  = " << p.landmarkRefs.transpose() << std::endl;
    // std::cout << "p.edgeRefs  = " << p.edgeRefs.transpose() << std::endl;
    // printPolygon(p);
    // printPolygon(n);
    // std::cout << "PREROT n.ldmkRefs  = " << n.landmarkRefs.transpose() << std::endl;
    // std::cout << "PREROT n.edgeRefs  = " << n.edgeRefs.transpose() << std::endl;

    // Reverse the order of the neighboring polygon's elements if it opposes the merging polygon's order
    bool hasReversed = p.isClockwise != n.isClockwise;
    if (hasReversed) n.reverse();
    // if (hasReversed) {n.reverse(); std::cout << "neighbor order flipped!" << std::endl;}
    
    // std::cout << "POSTROT n.ldmkRefs  = " << n.landmarkRefs.transpose() << std::endl;
    // std::cout << "POSTROT n.edgeRefs  = " << n.edgeRefs.transpose() << std::endl;

    // std::cout << "edgeIndexInP = " << edgeIndexInP << std::endl;
    // std::cout << "finding edge  = " << edgeIndexInObs << std::endl;

    // std::cout << "PRECOMB p.edgeRefs  = " << p.edgeRefs.transpose() << std::endl;
    // std::cout << "PRECOMB n.edgeRefs  = " << n.edgeRefs.transpose() << std::endl;


    // First ensure that the neighbor polygon is aware of the shared edge
    int edgeIndexInN = n.findEdgeIndex(edgeIndexInObs);
    // std::cout << "meep = " << edgeIndexInN << std::endl;
    if (edgeIndexInN == -1) return EpicPolygon();   // if not, cancel the merge

    // REMINDER: P might have changed, so we need to find where the longest edge is for this polygon
    int edgeIndexInP = p.findEdgeIndex(edgeIndexInObs);

    // std::cout << "edgeIndexInN = " << edgeIndexInN << std::endl;

    if (!hasReversed) edgeIndexInN = edgeIndexInN+1 % n.n;


    // Rotate both polygons to so they follow the given configuration:
    //    - The vertices of the shared polygons should be the first and last element in the list respectively 
    //    - In p, the shared edge should be the LAST  element in its list
    //    - In n, the shared edge should be the FIRST element in its list
    p.coolRotate(1+edgeIndexInP % p.n);
    n.coolRotate(edgeIndexInN);

    // std::cout << "PRECOMB p.ldmkRefs  = " << p.landmarkRefs.transpose() << std::endl;
    // std::cout << "PRECOMB p.edgeRefs  = " << p.edgeRefs.transpose() << std::endl;
    // std::cout << "PRECOMB n.ldmkRefs  = " << n.landmarkRefs.transpose() << std::endl;
    // std::cout << "PRECOMB n.edgeRefs  = " << n.edgeRefs.transpose() << std::endl;


    // combine the polygon data
    Eigen::VectorXi combinedLdmkRef(p.n+n.n-2), combinedEdgeRef(p.n+n.n-2), combinedNeighbors(p.n+n.n-2);
    // combinedLdmkRef << p.landmarkRefs.head(p.n-1), (hasReversed ? n.landmarkRefs.reverse() : n.landmarkRefs).tail(n.n-1);
    
    // combinedLdmkRef << p.landmarkRefs.head(p.n-1), n.landmarkRefs.tail(n.n-1);
    // combinedEdgeRef << p.edgeRefs.head(p.n-1), n.edgeRefs.tail(n.n-1);
    // combinedNeighbors << p.neighbors.head(p.n-1), n.neighbors.tail(n.n-1);
    
    // We ALWAYS expect shared vertices to be on either end of polygon's vector (opposing order)
    // Concatenation of landmark references: (should be in correct order now)
    // 1st shared vertex << p's non-shared vertices << n's non-shared vertices << 2nd shared vertex
    // combinedLdmkRef << p.landmarkRefs.head(p.n-1), n.landmarkRefs.middleRows(1,n.n-2), p.landmarkRefs.tail(1);
    
    // 1st shared vertex << p's non-shared vertices << 2nd shared vertex << n's non-shared vertices
    combinedLdmkRef << p.landmarkRefs, n.landmarkRefs.middleRows(1,n.n-2);
    if (hasReversed) {
        // When the direction is reversed, the deleted edge reference lies at the end of p and beginning of n
        combinedEdgeRef << p.edgeRefs.head(p.n-1), n.edgeRefs.tail(n.n-1);
        combinedNeighbors << p.neighbors.head(p.n-1), n.neighbors.tail(n.n-1);
    } else {
        // When the direction was not reversed, the deleted edge reference lies at the end of both p and n
        combinedEdgeRef << p.edgeRefs.head(p.n-1), n.edgeRefs.head(n.n-1);
        combinedNeighbors << p.neighbors.head(p.n-1), n.neighbors.head(n.n-1);
    }

    // std::cout << "COMB ldmkRefs  = " << combinedLdmkRef.transpose() << std::endl;
    // std::cout << "COMB edgeRefs  = " << combinedEdgeRef.transpose() << std::endl;

    // return EpicPolygon(combinedLdmkRef, combinedEdgeRef, combinedNeighbors, p.isClockwise, landmarks, edgeRefMap, triangulationEdgeLengths);
    return EpicPolygon(combinedLdmkRef, combinedEdgeRef, combinedNeighbors, p.isClockwise, landmarks, triangulationEdgeLengths);
}


// void updateCoordToLdmkIdxMap(const Points &ldmks, std::unordered_map<int, int> &coordToLdmkIdx, int inputID, PtLoc pt) {
//     // Search linearly for now because I'm lazy
//     for (int i = 0; i < ldmks.cols(); ++i) {
//         if (pt == ldmks.col(i)) {
//             coordToLdmkIdx[inputID] = i;
//             return;
//         }
//     }
// }

void Observation::printPolygon(const EpicPolygon& p) {
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

void Observation::delaunayTriangulationFromScratch(Points& points, std::vector<EpicPolygon>& polygons){
    
    // Initialize empty structures for edge data
    triangulationEdges.resize(2, 1);
    triangulationEdgeLengths.resize(1);

    std::unordered_map<int, int> coordToLdmkIdx;

    std::vector<double> qhull_points_data(points.size());
    for (size_t pidx = 0; pidx < points.cols(); ++pidx) {
        qhull_points_data[pidx * 2 + 0] = points(0, pidx);
        qhull_points_data[pidx * 2 + 1] = points(1, pidx);
    }
    // for (auto e : qhull_points_data) {
    //     std::cout << e << " ";
    // }
    // std::cout << std::endl;

    // Read tree positions into QHull
    orgQhull::PointCoordinates qhull_points(2, "");
    // qhull_points.append(points.cols(), points.data());
    qhull_points.append(qhull_points_data);

    
    
    // Compute Delaunay Triangulation
    orgQhull::Qhull q;
    q.runQhull(qhull_points.comment().c_str(), qhull_points.dimension(),
                   qhull_points.count(), qhull_points.coordinates(), "Qt Qbb Qc Qz Q12 d");
    // TODO the option "Fx" computes the convex hull, too; would be useful to preallocate space for the edges and edgeLengths

    // Delaunay regions as a vector of vectors
    orgQhull::QhullFacetListIterator k(q.facetList());

    // the facet ids are confusing, we want to map the good facets to order of appearance
    size_t fIdx = 0;
    std::map<size_t, size_t> id_map;
    for (const auto& e : q.facetList()) {
        if (e.isGood()) id_map[e.id()] = ++fIdx;
        // TODO: COUNT BAD FACETS (SHOULD BE ON BOUNDARY? --> SHOULD MATCH # POINTS ON CONVEX HULL?) no good?
    }

    // Store edge lengths in a matrix, store references to these lengths in each Polygon
    int edgeListSize = 0;
    // TODO resize "triangulationEdges" with number of unique edges in triangulation:
    // 3n - h - 3 <-- n=#vertices, h=#vertices in convex hull


    while (k.hasNext()) {
        orgQhull::QhullFacet f = k.next();
        if (!f.isGood()) continue;

        // Get the neighbors for each edge of this triangle
        // This only provides neighbors that are also a good facet, so edges without neighbors are assigned -1
        orgQhull::QhullFacetSet neighborsSet(f.neighborFacets());
        std::vector<int> auxNeighIds;
        for (const auto& e : neighborsSet) auxNeighIds.push_back(e.isGood() ? id_map[e.id()] : -1); // iterates three times
    
        // Shift order of neighbors to align with the edges (edge[i] will be shared with neighIds[i])
        Eigen::Vector3i neighIds;
        for (int i=1; i <= 3; ++i) neighIds(i%3) = auxNeighIds[i-1]; // iterates three times
        // TODO incorporate this in the edge loop below

        // Store references to this triangle's vertices
        orgQhull::QhullVertexSetIterator i(f.vertices());
        Eigen::Vector3i ldmkIds;
        for (int idx = 0; i.hasNext(); ++idx) ldmkIds(idx) = i.next().point().id();
        // std::cout << "Landmark IDs: " << ldmkIds.transpose() << std::endl;

        double vertexOrdering = 0;
        // Get the edges of this facet
        // Eigen::Vector3i edgeRefs;
        Eigen::Vector3i edgeIDs;
        for (int srcID = 0, dstID = 1; srcID < 3; dstID = (++srcID+1)%3) {
            size_t edgeID = cantorPairing(ldmkIds[srcID], ldmkIds[dstID]);
            // std::cout << "lalala" << std::endl;

            // Do not add duplicate edge data to the observation (keep data atomic)
            if (edgeRefMap.find(edgeID) == edgeRefMap.end()) {
                edgeRefMap[edgeID] = edgeListSize++;
                // std::cout << "New Edge found, nextEdgeCol=" << edgeListSize << std::endl;
                triangulationEdges.conservativeResize(2, edgeListSize);      // TODO until I'm able to get number of edges in the triangulation,
                triangulationEdgeLengths.conservativeResize(edgeListSize);   //      I will need to allocate space as I go
                
                // order of point refs doesn't matter because "triangulationEdges" should be accessed through "edgeRefMap" values, which is invariant to order
                // std::cout << ldmkIds(srcID) << "," << ldmkIds(dstID) << std::endl;
                // Eigen::Vector2i thisEdge;
                // thisEdge << ldmkIds(srcID), ldmkIds(dstID);
                // std::cout << "This edge: " << thisEdge.transpose() << std::endl;
                // triangulationEdges.col(edgeListSize-1) = thisEdge;
                triangulationEdges.col(edgeListSize-1) = Eigen::Vector2i{ldmkIds(srcID), ldmkIds(dstID)};

                // std::cout << "Distance between " << points.col(ldmkIds(srcID)).transpose() << " and " << points.col(ldmkIds(dstID)).transpose() << std::endl;
                // double myDistance = euclideanDistance2D(points.col(ldmkIds(srcID)), points.col(ldmkIds(dstID)));
                // std::cout << "= " << myDistance << std::endl;
                // triangulationEdgeLengths(edgeListSize-1) = myDistance;
                triangulationEdgeLengths(edgeListSize-1) = euclideanDistance2D(points.col(ldmkIds(srcID)), points.col(ldmkIds(dstID)));
            }
            // edgeRefs(srcID) = edgeID; // TODO store edge indices directly in polygon, not the keys
            edgeIDs(srcID) = edgeRefMap[edgeID];

            // Accumulate the signed area of this triangle while processing edges
            // A negative value indicates the vertices/edges are ordered clockwise
            vertexOrdering += landmarks(Eigen::placeholders::all, {ldmkIds[srcID], ldmkIds[dstID]}).determinant();
            // std::cout << "next one..." << std::endl;
        }

        // std::cout << "Poly Time!" << std::endl;
        // std::cout << "All edges: " << std::endl << triangulationEdges << std::endl;
        // std::cout << "All edge lengths: " << std::endl << triangulationEdgeLengths << std::endl;
        // std::cout << "ldmkIds: " << ldmkIds.transpose() << std::endl;
        // std::cout << "These landmarks: " << std::endl << landmarks(Eigen::placeholders::all, ldmkIds) << std::endl;
        // std::cout << "Edge refs: " << edgeRefs.transpose() << std::endl;
        // std::cout << "Edge col IDs: " << edgeIDs.transpose() << std::endl;
        // std::cout << "Edge Lens: " << triangulationEdgeLengths(edgeIDs) << std::endl;
        // std::cout << "NeighIds: " << neighIds.transpose() << std::endl;
        // EpicPolygon poly(ldmkIds, edgeRefs, neighIds, vertexOrdering < 0, landmarks, edgeRefMap, triangulationEdgeLengths);
        EpicPolygon poly(ldmkIds, edgeIDs, neighIds, vertexOrdering < 0, landmarks, triangulationEdgeLengths);
        // printPolygon(poly);
        polygons.push_back(poly);
        // abort();
    }
    // std::cout << "All done!" << std::endl;
};


} // urquhart