#pragma once

#include <set>
#include <epic_polygon.hpp>
#include <hype_observation.hpp>

namespace hype_matching
{
    // Expands triangle matches into edge and point correspondences.
    // This function will test all possible combinations of edge assignments and use the one
    // with smallest squared distance between the edge lengths
    std::vector<std::pair<PtLoc, PtLoc>> pointMatching(const hype_urq::Observation &ref, const hype_urq::Observation &targ, const std::vector<std::pair<size_t, size_t>> &triangleMatches);

    // Matches polygons (works for triangles as well). This function does a greedy assignment
    // based on the euclidean distance of the DFT descriptors. The best match has to be under thresh to be accepted
    void polygonMatching(
        const hype_urq::Observation &ref, std::vector<size_t> refIds,
        const hype_urq::Observation &targ, std::vector<size_t> targIds, double thresh,
        std::vector<std::pair<size_t, size_t>> &polygonMatches);

    // Matches a pair of observations. Returns a vector of tuples of points that were considered matches.
    // pair.first refers to a point in ref, pair.second refers to a point in targ
    std::vector<std::pair<PtLoc, PtLoc>> hierarchyMatching(const hype_urq::Observation &ref,
                                                             const hype_urq::Observation &targ, double thresh);
}