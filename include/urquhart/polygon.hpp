#pragma once
#include <descriptor.hpp>
#include <distance.hpp>

namespace urquhart {
    class Polygon {
        public:
            explicit Polygon() {
                landmarkRefs = {};
                neighbors = {};
                edgeRefs = {};
                descriptor = {};
                n = -1;
            }

            // TODO test whether the polygon vectors can be passed in by reference
            explicit Polygon(Eigen::VectorXi pRefs, // Copy these values into new space?
                                Eigen::VectorXi eRefs,  // If not, vectors should have dynamic size when initialized outside
                                Eigen::VectorXi nn,
                                bool isClock, 
                                const Points& ldmks, 
                                const Eigen::VectorXd& eLens) {
                landmarkRefs = pRefs;
                edgeRefs = eRefs;
                neighbors = nn;
                isClockwise = isClock;
                // std::cout << "Defs done" << std::endl;
                n = pRefs.size();
                descriptor = poly_desc::compute(pRefs, eRefs, ldmks, eLens);
            }

            void reverse() { // Performed on this polygon when it disagrees with the direction of a polygon initiating a merge 
                landmarkRefs.reverseInPlace();
                edgeRefs.reverseInPlace();
                neighbors.reverseInPlace();
                isClockwise = !isClockwise;
            }

            int findEdgeIndex(const int& commonEdge) { // Retrieve the index of an edge, -1 if not present
                for (int i = 0; i < n; ++i) {
                    if (edgeRefs(i) == commonEdge) return i;
                }
                return -1;
            }

            // Puts the elements in position "idx" at the beginning of each vector. Used during merging operations.
            void coolRotate(const int& idx) {
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
            
            // number of sides
            int n;
    };
}