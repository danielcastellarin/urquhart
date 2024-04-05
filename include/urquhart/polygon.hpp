#pragma once
#include <descriptor.hpp>
#include <distance.hpp>

namespace urquhart {
    class Polygon {
        public:
            // Empty constructor for root of hierarchy and failed merge attempts
            explicit Polygon() {
                landmarkRefs = {};
                neighbors = {};
                edgeRefs = {};
                descriptor = {};
                n = -1, longestEdgeIdx = -1;
            }

            // Constructor for non-triangles 
            explicit Polygon(const Eigen::VectorXi& pRefs,
                                const Eigen::VectorXi& eRefs,
                                const Eigen::VectorXi& nn,
                                bool isClock, 
                                const Points& ldmks, 
                                const Eigen::VectorXd& eLens) {
                landmarkRefs = pRefs;
                edgeRefs = eRefs;
                neighbors = nn;
                isClockwise = isClock;
                n = pRefs.size();
                longestEdgeIdx = -1;
                descriptor = poly_desc::compute(pRefs, eRefs, ldmks, eLens);
            }

            // Constructor for triangles, includes index of eRefs for its longest edge length
            explicit Polygon(const Eigen::VectorXi& pRefs,
                                const Eigen::VectorXi& eRefs,
                                const Eigen::VectorXi& nn,
                                bool isClock, 
                                const Points& ldmks, 
                                const Eigen::VectorXd& eLens,
                                int longestEdgeIndex) {
                landmarkRefs = pRefs;
                edgeRefs = eRefs;
                neighbors = nn;
                isClockwise = isClock;
                n = pRefs.size();
                longestEdgeIdx = longestEdgeIndex;
                descriptor = poly_desc::compute(pRefs, eRefs, ldmks, eLens);
            }

            void recomputeDescriptor(const Points& ldmks, const Eigen::VectorXd& eLens) {
                descriptor = poly_desc::compute(landmarkRefs, edgeRefs, ldmks, eLens);
            }

            // Reverse the elements of this polygon.
            // Executed when disagreeing with the direction of another polygon that is initiating a merge
            void reverse() {  
                landmarkRefs.reverseInPlace();
                edgeRefs.reverseInPlace();
                neighbors.reverseInPlace();
                isClockwise = !isClockwise;
            }

            // Retrieve the local index of an edge in the triangulation, -1 if not present
            int findEdgeIndex(const int& commonEdge) {
                for (int i = 0; i < n; ++i) {
                    if (edgeRefs(i) == commonEdge) return i;
                }
                return -1;
            }

            // Puts the elements in position "idx" at the beginning of each vector. Used during merging operations.
            void rotateElements(const int& idx) {
                Eigen::PermutationMatrix<-1, -1> perm(n);
                Eigen::VectorXi permIndices(n);

                // I don't understand the permutation matrices...
                // The permutation index order must be "inverted" in order for it to be applied correctly
                for (int j = 0, invertedIdx = n - idx; j < n; ++j) {
                    permIndices[j] = (invertedIdx+j) % n;
                }
                perm.indices() = permIndices;

                // Apply rotation in-place
                landmarkRefs = perm * landmarkRefs;
                edgeRefs = perm * edgeRefs;
                neighbors = perm * neighbors;
            }

            // Lists of reference IDs to various other data structures:
            Eigen::VectorXi landmarkRefs;   // columns of Observation.landmarks
            Eigen::VectorXi edgeRefs;       // columns of Observation.triangulationEdges (whose values are column numbers in Observation.landmarks)
                                            //      and columns of Observation.triangulationEdgeLengths
            Eigen::VectorXi neighbors;      // Polygon node IDs in Tree
            Eigen::VectorXd descriptor;

            // Whether vertex/edge order is clockwise (calculated during triangulation)
            bool isClockwise;
            
            int n;              // number of sides
            int longestEdgeIdx; // index of edgeRefs for longest edge (triangles only)
    };
}