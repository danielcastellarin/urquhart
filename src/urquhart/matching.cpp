#include <matching.hpp>

namespace matching
{

void polygonMatching(
        const urquhart::Observation &ref, std::unordered_set<int> refIds,
        const urquhart::Observation &targ, std::unordered_set<int> targIds, double thresh,
        std::vector<std::pair<size_t, size_t>> &polygonMatches)
{
    std::set<size_t> matched;

    // For each reference Polygon, try to find its best matching polygon in the target set
    for (const auto& rIdx : refIds) {
        size_t bestMatch = 0, bestDist = 100000;
        urquhart::Polygon rp = ref.hier->getPolygon(rIdx);

        for (const auto& tIdx : targIds) {
            // Only attempt a match if the target polygon has not been matched with another reference polygon already
            if (matched.find(tIdx) == matched.end()) {
                urquhart::Polygon tp = targ.hier->getPolygon(tIdx);

                // Only attempt a match if the number of vertices each polygon has is within 3 of each other
                if (std::abs(rp.n - tp.n) <= 3) {
                    double d = descriptorDistance(rp.descriptor, tp.descriptor);
                    if (d < bestDist) {
                        bestDist = d;
                        bestMatch = tIdx;
                    }
                }
            }
        }
        // Store a match for later if one is found; early bird gets the worm here
        if (bestDist < thresh) {
            matched.insert(bestMatch);
            polygonMatches.push_back({rIdx, bestMatch});
        }
    }
}


std::vector<std::pair<Eigen::Index, Eigen::Index>> hierarchyIndexMatching(const urquhart::Observation &ref,
                                                        const urquhart::Observation &targ, double thresh)
{
    std::vector<std::pair<size_t, size_t>> polygonMatches, triangleMatches;

    // Polygon Matching (Level 2)
    polygonMatching(ref, ref.hier->getChildrenIds(0), targ, targ.hier->getChildrenIds(0), thresh, polygonMatches);

    // Triangle Matching (Level 1)
    for (const auto& [refPoly, targPoly] : polygonMatches) {
        // TODO: ADD CHECK IF % OF TRIANGLES THAT MACTHED IS LARGER THAN 1/2
        polygonMatching(ref, ref.hier->getChildrenIds(refPoly), targ, targ.hier->getChildrenIds(targPoly), thresh, triangleMatches);
    }

    // Vertex Matching (Level 0)
    return pointIndexMatching(ref, targ, triangleMatches);
}


std::vector<std::pair<Eigen::Index, Eigen::Index>> pointIndexMatching(const urquhart::Observation &ref, const urquhart::Observation &targ, const std::vector<std::pair<size_t, size_t>> &triangleMatches)
{
    std::set<size_t> uniqueMatches;
    std::vector<std::pair<Eigen::Index, Eigen::Index>> vertexMatches;

    Eigen::PermutationMatrix<3, 3> chi(Eigen::Vector3i{0,1,2});
    Eigen::Matrix<int, 3, 6> perms;
    perms << 0, 0, 1, 1, 2, 2,
             1, 2, 2, 0, 1, 0,
             2, 1, 0, 2, 0, 1;

    // At this point, we assume that all given triangle matches are probably correct, so we want to match up their vertices
    for (const auto& [refIdx, targIdx] : triangleMatches) {
        urquhart::Polygon refTriangle = ref.hier->getPolygon(refIdx), targTriangle = targ.hier->getPolygon(targIdx);


        double bestDist = 1000000;
        Eigen::Vector3i bestPermutation;

        // Try all six permutations of the edge lengths to determine the best match between vertices
        for (int c = 0; c < 6; ++c) {
            chi.indices() = perms.col(c);
            double d = descriptorDistanceAgain(ref.triangulationEdgeLengths(refTriangle.edgeRefs), targ.triangulationEdgeLengths(targTriangle.edgeRefs.transpose() * chi));
            if (d < bestDist) {
                bestDist = d;
                bestPermutation = chi.indices();
            }
        }

        // Associate the off-edge vertices for each matched edge 
        for (int i = 0; i < 3; ++i) {
            // TODO is there a cleaner way to do this besides clunky indexing? (foreach instead?)
            Eigen::Index refIdx = (i+2)%3, targIdx = (bestPermutation(i)+2)%3;

            // Only add unique vertex matches to our set
            size_t uid = cantorPairing(refTriangle.landmarkRefs(refIdx), targTriangle.landmarkRefs(targIdx));
            if (uniqueMatches.find(uid) == uniqueMatches.end()) {
                vertexMatches.push_back({refTriangle.landmarkRefs(refIdx), targTriangle.landmarkRefs(targIdx)});
                uniqueMatches.insert(uid);
            }
        }
    }
    return vertexMatches;
}


} // matching