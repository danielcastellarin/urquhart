#include <matching.hpp>

namespace matching
{

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

std::vector<std::pair<Eigen::Index, Eigen::Index>> nonGreedyHierarchyIndexMatching(const urquhart::Observation &ref,
                                                        const urquhart::Observation &targ, double thresh, int numSideBoundsForMatch, double reqMatchedPolygonRatio)
{
    std::vector<std::pair<size_t, size_t>> polygonMatches, triangleMatches;    

    // Polygon Matching (Level 2)
    nonGreedyPolygonMatching(ref, ref.hier->getChildrenIds(0), targ, targ.hier->getChildrenIds(0), thresh, numSideBoundsForMatch, 0, polygonMatches);

    // Triangle Matching (Level 1)
    for (const auto& [refPoly, targPoly] : polygonMatches) {
        nonGreedyPolygonMatching(ref, ref.hier->getChildrenIds(refPoly), targ, targ.hier->getChildrenIds(targPoly), thresh, numSideBoundsForMatch, reqMatchedPolygonRatio, triangleMatches);
    }

    // Vertex Matching (Level 0)
    return pointIndexMatching(ref, targ, triangleMatches);
}

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


void nonGreedyPolygonMatching(
        const urquhart::Observation &ref, std::unordered_set<int> refIds,
        const urquhart::Observation &targ, std::unordered_set<int> targIds,
        double thresh, int numSideBoundsForMatch, double reqMatchedPolygonRatio,
        std::vector<std::pair<size_t, size_t>> &polygonMatches)
{
    double distancePlaceholder;
    int numExistingMatches = polygonMatches.size();

    std::unordered_map<size_t, std::pair<double, size_t>> targetsClosestDist;
    std::unordered_map<size_t, std::map<double, size_t>> refClosestTargets;

    // For each reference Polygon, find its distance to each Polygon in the target set
    for (const auto& rIdx : refIds) {
        urquhart::Polygon rp = ref.hier->getPolygon(rIdx);

        for (const auto& tIdx : targIds) {
            urquhart::Polygon tp = targ.hier->getPolygon(tIdx);

            // Record match if target is the closest polygon under the threshold
            distancePlaceholder = std::abs(rp.n - tp.n) <= numSideBoundsForMatch ? descriptorDistance(rp.descriptor, tp.descriptor) : 1000000;

            if (distancePlaceholder < thresh) {
                refClosestTargets[rIdx].insert({distancePlaceholder, tIdx});
                if (targetsClosestDist.find(tIdx) == targetsClosestDist.end() || distancePlaceholder < targetsClosestDist[tIdx].first)
                    targetsClosestDist[tIdx] = {distancePlaceholder, rIdx};
            }
        }
    }

    // Add all pairs from the target polygon's perspective
    for (const auto& [tIdx, closestRef] : targetsClosestDist)
        polygonMatches.push_back({closestRef.second, tIdx});

    // For any pair that the reference disagrees with, add the next closest match (if possible)
    // for (const auto& [rIdx, closebyTargets] : refClosestTargets) {
    //     for (auto targIter = closebyTargets.begin(); targIter != closebyTargets.end() && )
    //     if (targIter->second)
    // }
    // ^^^^^ unnecessary

    // Invalidate these matches if not enough of them were made
    if (polygonMatches.size() - numExistingMatches < reqMatchedPolygonRatio * refIds.size()) polygonMatches.resize(numExistingMatches);
}


// void nonGreedyPolygonMatching(
//         const urquhart::Observation &ref, std::unordered_set<int> refIds,
//         const urquhart::Observation &targ, std::unordered_set<int> targIds,
//         double thresh, int numSideBoundsForMatch, double reqMatchedPolygonRatio,
//         std::vector<std::pair<size_t, size_t>> &polygonMatches)
// {

//     // Rows link to reference polygons, columns link to 
//     Eigen::MatrixXd descDistances(refIds.size(), targIds.size());

//     // NOTE: current hierarchy implementation does not have predictable polygon indexing, therefore indices must be mapped
//     std::unordered_map<int, int> refPolyIDMap(refIds.size()), targPolyIDMap(targIds.size()),
//                                 iiRefPolyIDMap(refIds.size()), iiTargPolyIDMap(targIds.size());

//     // Assign target polygons columns in the matrix
//     for (const auto& tIdx : targIds) {
//         iiTargPolyIDMap[targPolyIDMap.size()] = tIdx;
//         targPolyIDMap[tIdx] = targPolyIDMap.size();
//     }

//     // Find the distance between the descriptors of each pair of polygons
//     for (const auto& rIdx : refIds) {
//         urquhart::Polygon rp = ref.hier->getPolygon(rIdx);
//         iiRefPolyIDMap[refPolyIDMap.size()] = rIdx;
//         refPolyIDMap[rIdx] = refPolyIDMap.size();

//         for (const auto& tIdx : targIds) {
//             urquhart::Polygon tp = targ.hier->getPolygon(tIdx);

//             // Only attempt a match if the number of vertices each polygon has is within a certain amount
//             descDistances(refPolyIDMap[rIdx], targPolyIDMap[tIdx]) = std::abs(rp.n - tp.n) <= numSideBoundsForMatch ? descriptorDistance(rp.descriptor, tp.descriptor) : 10000;
//         }
//     }

//     // Get get distances and indices of the target polygon closest to each reference polygon
//     Eigen::VectorXd minDistances = descDistances.rowwise().minCoeff();
//     Eigen::ArrayXi distancesUnderThresh = minDistances.array() < thresh, minDistanceIdxs(refIds.size()); int i=0;
//     for (const auto& row : descDistances.rowwise()) row.minCoeff(&minDistanceIdxs(i++));

//     // As long as enough polygons are similar enough, return those valid matches
//     if (distancesUnderThresh.count() > reqMatchedPolygonRatio * refIds.size()) {
//         for (i = 0; i < distancesUnderThresh.size(); ++i) {
//             if (distancesUnderThresh(i)) polygonMatches.push_back({iiRefPolyIDMap[i], iiTargPolyIDMap[minDistanceIdxs[i]]});
//         }
//     }
// }


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