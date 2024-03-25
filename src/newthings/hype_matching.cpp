#include <hype_matching.hpp>

namespace hype_matching
{

void polygonMatching(
    const hype_urq::Observation &ref, std::vector<size_t> refIds,
    const hype_urq::Observation &targ, std::vector<size_t> targIds,
    double thresh, std::vector<std::pair<size_t, size_t>> &polygonMatches)
{
    std::set<size_t> matched;

    // For each reference Polygon, try to find the best polygon in the target set
    for (const auto& rIdx : refIds) {
        // std::cout << "Finding match for match for " << rIdx << std::endl;
        size_t bestMatch = 0, bestDist = 100000;
        hype_urq::EpicPolygon rp = ref.H->get_vertex(rIdx);
        // std::cout << "Descriptor: " << rp.descriptor.transpose() << std::endl;

        for (const auto& tIdx : targIds) {
            // Only attempt a match if the target polygon has not been matched with another reference polygon already
            if (matched.find(tIdx) == matched.end()) {
                hype_urq::EpicPolygon tp = targ.H->get_vertex(tIdx);
                // std::cout << "- " << tIdx << " descriptor: " << tp.descriptor.transpose() << std::endl;

                // Only attempt a match if the number of vertices each polygon has is within 3 of each other 
                // if (std::abs(int(rp.landmarkRefs.size() - tp.landmarkRefs.size())) <= 3) { // datatype of Eigen::Index should be signed
                if (std::abs(int(rp.n - tp.n)) <= 3) { // datatype of Eigen::Index should be signed
                    double d = descriptorDistance(rp.descriptor, tp.descriptor);
                    // std::cout << "- distance to " << tIdx << " is " << d << std::endl;
                    // std::cout << "--- distance = " << d << std::endl;

                    // std::cout << "d = " << d  << std::endl;
                    if (d < bestDist) {
                        bestDist = d;
                        bestMatch = tIdx;
                        // std::cout << "Best target available: " << tIdx << std::endl;
                    }
                }
            }
        }
        // Store a match for later if one is found; early bird gets the worm here
        if (bestDist < thresh) {
            matched.insert(bestMatch);
            polygonMatches.push_back({rIdx, bestMatch});
            // std::cout << "=====>>> MATCHED  " << rIdx << " - " << bestMatch << " with distance of " << bestDist << std::endl;
        }
        // abort();
    }
}


std::vector<std::pair<PtLoc, PtLoc>> pointMatching(const hype_urq::Observation &ref, const hype_urq::Observation &targ, const std::vector<std::pair<size_t, size_t>> &triangleMatches)
{
    std::set<size_t> uniqueMatches;
    std::vector<std::pair<PtLoc, PtLoc>> vertexMatches;
    // TODO make sure these don't get reinit every loop (unnecessary, chi should flip back to original state automatically)
    // Eigen::PermutationMatrix<3, 3> chi(Eigen::Vector3i{0,1,2}), shift(Eigen::Vector3i{2,0,1});
    Eigen::PermutationMatrix<3, 3> chi(Eigen::Vector3i{0,1,2});

    Eigen::Matrix<int, 3, 6> perms;
    perms << 0, 0, 1, 1, 2, 2,
             1, 2, 2, 0, 1, 0,
             2, 1, 0, 2, 0, 1;

    for (const auto& [refIdx, targIdx] : triangleMatches) {
        // std::cout << "Triangles " << refIdx << " and " << targIdx << std::endl;
        hype_urq::EpicPolygon refTriangle = ref.H->get_vertex(refIdx), targTriangle = targ.H->get_vertex(targIdx);

        // std::cout << "First lengths: " << ref.triangulationEdgeLengths(refTriangle.edgeRefs).transpose() << std::endl << std::endl;
        // std::cout << "Second lengths: " << targ.triangulationEdgeLengths(targTriangle.edgeRefs).transpose() << std::endl << std::endl;

        double bestDist = 1000000;
        Eigen::Vector3i bestPermutation;
        // for (int i = 0; i < 3; ++i, chi = shift * chi) {
        //     std::cout << "Target lengths: " << targ.triangulationEdgeLengths(chi * targTriangle.edgeRefs).transpose() << std::endl;

        //     double d = descriptorDistanceAgain(ref.triangulationEdgeLengths(refTriangle.edgeRefs), targ.triangulationEdgeLengths(chi * targTriangle.edgeRefs));
        //     std::cout << i << "- difference = " << d << std::endl;

        //     if (d < bestDist) {
        //         bestDist = d;
        //         bestPermutation = chi.indices();
        //     }
        // }

        for (int c = 0; c < 6; ++c) {
            chi.indices() = perms.col(c);
            // std::cout << "Target lengths: " << targ.triangulationEdgeLengths(targTriangle.edgeRefs.transpose() * chi).transpose() << " (" << chi.indices().transpose() << ")" << std::endl;

            double d = descriptorDistanceAgain(ref.triangulationEdgeLengths(refTriangle.edgeRefs), targ.triangulationEdgeLengths(targTriangle.edgeRefs.transpose() * chi));
            // std::cout << c << "- difference = " << d << std::endl;

            if (d < bestDist) {
                bestDist = d;
                bestPermutation = chi.indices();
            }
        }

        // std::cout << "Best Perm: " << bestPermutation.transpose() << std::endl << std::endl;


        // TODO determine vertex order of polygon during triangulation, store in Polygon, update for higher order
        for (int i = 0; i < 3; ++i) {
            Eigen::Index refIdx = (i+2)%3, targIdx = (bestPermutation(i)+2)%3; // TODO hopefully the off-edge technique still works
            // size_t uid = cantorPairing(refIdx, targIdx);
            size_t uid = cantorPairing(refTriangle.landmarkRefs(refIdx), targTriangle.landmarkRefs(targIdx));
            // std::cout << "Match UID " << i << ": " << uid << std::endl;

            if (uniqueMatches.find(uid) == uniqueMatches.end()) {
                vertexMatches.push_back({ref.landmarks.col(refTriangle.landmarkRefs(refIdx)), targ.landmarks.col(targTriangle.landmarkRefs(targIdx))});
                uniqueMatches.insert(uid);
            }
        }
    }
    return vertexMatches;
}

std::vector<std::pair<PtLoc, PtLoc>> hierarchyMatching(const hype_urq::Observation &ref,
                                                        const hype_urq::Observation &targ, double thresh)
{
    std::vector<std::pair<size_t, size_t>> polygonMatches, triangleMatches;

    // Polygon Matching (Level 2)
    polygonMatching(ref, ref.H->get_children(0), targ, targ.H->get_children(0), thresh, polygonMatches);

    // Triangle Matching (Level 1)
    // std::cout << "Triangles:" << std::endl;
    for (const auto& [refPoly, targPoly] : polygonMatches) {
        // std::cout << "Polygons " << refPoly << " and " << targPoly << std::endl;
        
        // TODO: ADD CHECK IF % OF TRIANGLES THAT MACTHED IS LARGER THAN 1/2
        polygonMatching(ref, ref.H->get_children(refPoly), targ, targ.H->get_children(targPoly), thresh, triangleMatches);
    }

    // for (const auto& [refTri, targTri] : triangleMatches) {
    //     std::cout << "Polygons " << refTri << " and " << targTri << std::endl;
    // }
    // abort();

    // Vertex Matching (Level 0)
    return pointMatching(ref, targ, triangleMatches);
}



} // matching