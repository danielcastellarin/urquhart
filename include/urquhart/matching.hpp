#pragma once

#include <observation.hpp>

namespace matching
{
    // Matches a pair of observations. Returns a vector of tuples of points that were considered matches.
    // pair.first refers to a point in ref, pair.second refers to a point in targ
    std::vector<std::pair<Eigen::Index, Eigen::Index>> hierarchyIndexMatching(const urquhart::Observation &ref,
                                                        const urquhart::Observation &targ, double thresh);
    std::vector<std::pair<Eigen::Index, Eigen::Index>> nonGreedyHierarchyIndexMatching(const urquhart::Observation &ref,
                                                        const urquhart::Observation &targ, double thresh, int numSideBoundsForMatch, double reqMatchedPolygonRatio);

    // Matches polygons (works for triangles as well). This function does a greedy assignment
    // based on the euclidean distance of the DFT descriptors. The distance to the best match must be under thresh to be accepted
    void polygonMatching(
        const urquhart::Observation &ref, std::unordered_set<int> refIds,
        const urquhart::Observation &targ, std::unordered_set<int> targIds, double thresh,
        std::vector<std::pair<size_t, size_t>> &polygonMatches);

    
    void nonGreedyPolygonMatching(
        const urquhart::Observation &ref, std::unordered_set<int> refIds,
        const urquhart::Observation &targ, std::unordered_set<int> targIds,
        double thresh, int numSideBoundsForMatch, double reqMatchedPolygonRatio,
        std::vector<std::pair<size_t, size_t>> &polygonMatches);

    
    // Expands triangle matches into edge and point correspondences.
    // This function will test all possible combinations of edge assignments and use the one
    // with smallest euclidean distance between the edge lengths
    std::vector<std::pair<Eigen::Index, Eigen::Index>> pointIndexMatching(const urquhart::Observation &ref, const urquhart::Observation &targ, const std::vector<std::pair<size_t, size_t>> &triangleMatches);
    

}