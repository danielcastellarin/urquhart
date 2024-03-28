#include <hierarchy.hpp>

namespace urquhart {

// Constructor

Hierarchy::Hierarchy(const std::vector<Polygon>& triangles) {
    // Define root node (empty polygon)
    polygonLookup[rootId] = Polygon();

    std::unordered_set<int> rootChildren;
    for (const auto& t : triangles) {
        rootChildren.insert(++polyIdSequence);  // add triangle as child of root
        parentLookup[polyIdSequence] = rootId;  // add root as parent of triangle 
        polygonLookup[polyIdSequence] = t;      // add triangle to hierarchy
    }
    childrenLookup[rootId] = rootChildren;
}

// ============
// View Methods
// ============

void Hierarchy::viewTree(std::ostream& out) {
    for (const auto& [parentId, children] : childrenLookup) {
        for (const auto& childId : children) {
            out << parentId << "->" << childId << std::endl;
        }
    }
}

void Hierarchy::viewPolygons(std::ostream& out) {
    for (const auto& childId : childrenLookup[rootId]) {
        out << childId << ": ";
        for (int i = 0; i < polygonLookup[childId].n; ++i) {
            out << "[" << polygonLookup[childId].landmarkRefs(i) << "," << polygonLookup[childId].landmarkRefs((i+1)%polygonLookup[childId].n) << "] ";
        }
        out << std::endl;
    }
}

// =============
// Merge Methods
// =============

int Hierarchy::mergeOp(int i, int j, const Polygon& data) {

    polygonLookup[++polyIdSequence] = data;          // add polygon to hierarchy
    childrenLookup[rootId].insert(polyIdSequence);   // add polygon as child of root
    parentLookup[polyIdSequence] = rootId;           // add root as parent of polygon

    // Add child triangles of i and j (if any) as children of the new polygon
    std::unordered_set<int> newNodeChildren;

    mergeHelper(i, newNodeChildren);
    mergeHelper(j, newNodeChildren);
    
    childrenLookup[polyIdSequence] = newNodeChildren;
    return polyIdSequence;
}

void Hierarchy::mergeHelper(int existingNodeId, std::unordered_set<int>& newPolygonChildren) {
    childrenLookup[rootId].erase(existingNodeId);    // existing polygon is no longer child of root
    
    // Hierarchy modification depends on whether existing node represents a polygon or triangle
    if (polygonLookup[existingNodeId].n > 3) {
        
        // Replace references to/from its children
        for (const auto& c : childrenLookup[existingNodeId]) {
            newPolygonChildren.insert(c);   // assign child to the new node
            parentLookup[c] = polyIdSequence;    // assign the new node as the parent of the child
        }

        // Drop existing polygon from the hierarchy
        childrenLookup.erase(existingNodeId);
        parentLookup.erase(existingNodeId);
        polygonLookup.erase(existingNodeId);
    } else {
        parentLookup[existingNodeId] = polyIdSequence;   // assign the new node as the parent of the triangle
        newPolygonChildren.insert(existingNodeId);  // assign triangle as a child of the new node
    }
}

// =========
// Accessors
// =========

Polygon Hierarchy::getPolygon(const int v) { return polygonLookup[v]; }

int Hierarchy::getAncestorId(const int v) { return parentLookup[v] == rootId ? v : parentLookup[v]; }

std::unordered_set<int> Hierarchy::getChildrenIds(const int v) { return childrenLookup.find(v) == childrenLookup.end() ? std::unordered_set<int>{v} : childrenLookup[v]; }

}