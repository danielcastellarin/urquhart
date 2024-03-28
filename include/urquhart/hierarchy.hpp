#pragma once

#include <polygon.hpp>
#include <memory>
#include <list>
#include <unordered_set>

namespace urquhart {

    // struct HNode {
    //     std::shared_ptr<HNode> parent = NULL;
    //     Polygon data;
    //     std::list<std::shared_ptr<HNode>> children;
    //     HNode(int numTriangles) {children.resize(numTriangles);} // initialize root
    //     HNode(const std::shared_ptr<HNode>& root, const Polygon& triangle) : parent(root), data(triangle) {} // initialize triangle
    //     HNode(const std::shared_ptr<HNode>& root, const Polygon& poly, std::list<std::shared_ptr<HNode>> kids) : parent(root), data(poly), children(kids) {} // initialize polygon

        
    // };

    // struct SlimHNode {
    //     int parentId = NULL;
    //     Polygon data;
    //     std::unordered_set<int> childIds;
    //     SlimHNode(int numTriangles) {childIds.reserve(numTriangles);} // initialize root
    //     SlimHNode(const int& root, const Polygon& triangle) : parentId(root), data(triangle) {} // initialize triangle
    //     SlimHNode(const int& root, const Polygon& poly, std::unordered_set<int> children) : parentId(root), data(poly), childIds(children) {} // initialize polygon

        
    // };

    class Hierarchy {
        public:
            explicit Hierarchy(const std::vector<Polygon>& polygons);
            void viewTree(std::ostream& out);
            void viewPolygons(std::ostream& out);

            // Adding new polygons from Uqruhart tessellation to the hierarchy
            int mergeOp(int i, int j, const Polygon& data);
            void mergeHelper(int existingNodeId, std::unordered_set<int>& newPolygonChildren);

            Polygon getPolygon(const int v);
            // Gets the parent of a Polygon in the geometric hierarchy
            int getAncestorId(const int v);
            std::unordered_set<int> getChildrenIds(const int v);
            // need alternative in matching for when called on triangle to only return itself
            // maybe if no children, return itself in a vector?

            int rootId = 0, polyIdSequence = 0;
        
        private:
            std::unordered_map<int, Polygon> polygonLookup;
            std::unordered_map<int, int> parentLookup;
            std::unordered_map<int, std::unordered_set<int>> childrenLookup;
    };
}