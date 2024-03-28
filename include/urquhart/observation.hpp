#pragma once

#include <hierarchy.hpp>

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

namespace urquhart {
    class Observation {
        public:
            explicit Observation(Points& freshLandmarks);
            explicit Observation(std::vector<std::vector<double>>& freshLandmarks);
            ~Observation();
            void view();

            void computeHierarchy();
            void recomputeEdgeLengths();

            // Storage for the positions of all landmarks in this observation (one per column)
            // All polygons store references to these values
            Points landmarks;

            // Storage for each distinct edge in the Delaunay triangulation of this observation
            // Each column is a pair of column numbers in "landmarks" 
            EdgeSet triangulationEdges;
            Eigen::VectorXd triangulationEdgeLengths; // indices should match triangulationEdges
            
            // cantor pair of vertex IDs -> EdgeSet column ID (which contains the "landmarks" indices for the two points in this edge)
            // useful when preventing duplicate operations on 
            std::unordered_map<size_t, int> edgeRefMap;

            // H stores all polygons in a tree structure, where each vertex represents a polygon.
            // The childs of a polygon are the triangles that were merged to compose it.
            // Triangles are the leaves of the tree.
            // The vertex 0 (root) is an empty polygon that is only used to connect all polygons
            // TODO: hier should be private and have accessors
            Hierarchy* hier = NULL;
            // std::shared_ptr<Hierarchy> hier;

            const PtLoc& ldmk(Eigen::Index colNum) const;
            const double& ldmkX(Eigen::Index colNum) const;
            const double& ldmkY(Eigen::Index colNum) const;
            const Points& ldmks(Eigen::VectorXi indices) const;

        private:
            // Computes a Delaunay triangulation using QHull from a set of landmarks.
            void delaunayTriangulationFromScratch(std::vector<Polygon>& polygons);
            
            // Uses the triangles of the delaunay triangulation to build an "urquhart tessellation"
            void urquhartTesselation_();
            
            // Create a new polygon from existing polyons
            Polygon mergePolygons_(Polygon& p, Polygon& n, int edgeIndexInP);
            
            void printPolygon(const Polygon& p);

            Points* getPolygonPoints(const Polygon& p);
    };
}