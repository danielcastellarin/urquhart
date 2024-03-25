#pragma once
#include <hype_distance.hpp>

namespace hype_descriptor {
    // Samples points around the perimeter of a polygon and returns a new set of points.
    // The size of the returned PointVector depends on step.
    // Smaller steps means more points will be sampled,
    // e.g. step=0.05 means a point every 5% of the perimeter, resulting in 20 points.
    Points samplePoints(const Eigen::VectorXi& vRefs, 
                        const Eigen::VectorXi& eRefs,
                        const Points& ldmks,
                        const Eigen::VectorXd& eLens,
                        double step);
    // Uses sampled points to compute a centroid distance descriptor
    Eigen::VectorXd findCentroidDistance(const Eigen::VectorXi& ldmkIndices, const Points& ldmks, Points sampledPoints);
    // Uses OpenCV to compute the magnitude of the DFT of the centroid descriptor
    Eigen::VectorXd invariantFourier(Eigen::VectorXd centroidDesc);
    // Runs the entire descriptor computation pipeline
    Eigen::VectorXd compute(const Eigen::VectorXi& vRefs, 
                            const Eigen::VectorXi& eRefs, 
                            const Points& ldmks,
                            const Eigen::VectorXd& eLens);
}