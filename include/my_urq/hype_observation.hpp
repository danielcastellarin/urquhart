#pragma once

#include <hype_distance.hpp>
#include <epic_polygon.hpp>
#include <hype_tree.hpp>
#include <map>

#include "libqhullcpp/RboxPoints.h"
#include "libqhullcpp/QhullError.h"
#include "libqhullcpp/QhullQh.h"
#include "libqhullcpp/QhullFacet.h"
#include "libqhullcpp/QhullFacetList.h"
#include "libqhullcpp/QhullFacetSet.h"
#include "libqhullcpp/QhullLinkedList.h"
#include "libqhullcpp/QhullPoint.h"
#include "libqhullcpp/QhullVertex.h"
#include "libqhullcpp/QhullVertexSet.h"
#include "libqhullcpp/Qhull.h"

namespace hype_urq {
    class Observation {
        public:
            explicit Observation(Points& freshLandmarks);
            void view();

            Points landmarks;
            EdgeSet triangulationEdges;
            Eigen::VectorXd triangulationEdgeLengths; // index should line up with EdgeSet
            std::unordered_map<size_t, int> edgeRefMap; // cantor pair of vertex IDs -> EdgeSet column ID (which contains the "landmarks" indices for the two points in this edge)
            // useful when recomputing all the edge lengths 

            // H stores all polygons in a tree structure, where each vertex represents a polygon.
            // The childs of a polygon are the triangles that were merged to compose it.
            // Triangles are the leaves of the tree.
            // The vertex 0 (root) is an empty polygon that is only used to connect all polygons
            // TODO: H should be private and have accessors
            Tree* H;
        private:
            // Computes a Delaunay triangulation using QHull from a set of landmarks.
            void delaunayTriangulationFromScratch(Points& points, std::vector<EpicPolygon>& polygons);
            // Uses the triangles of the delaunay triangulation to compute an urquhart tessellation
            void urquhartTesselation_();
            
            void printPolygon(const EpicPolygon& p);
            EpicPolygon mergePolygons_(EpicPolygon& p, EpicPolygon& n, int edgeIndexInP);
            // EpicPolygon Observation::mergePolygons_(EpicPolygon& p, EpicPolygon& n, int edgeId, int edgeIndexInP);
            // EpicPolygon mergePolygons_(EpicPolygon& p, EpicPolygon& n, size_t commonEdgeKey, int edgeIndexInP);
    };
}