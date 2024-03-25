#include <descriptor.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>

namespace poly_desc {
    // Samples points around the perimeter of a polygon and returns a new set of points.
    // The size of the returned PointVector depends on step.
    // Smaller steps means more points will be sampled,
    // e.g. step=0.05 means a point every 5% of the perimeter, resulting in 20 points.
    Points samplePoints(const Eigen::VectorXi& vRefs, 
                        const Eigen::VectorXi& eRefs,
                        const Points& ldmks,
                        const Eigen::VectorXd& eLens,
                        double step) {

        // Create a list of cumulative edge lengths normalized
        Eigen::VectorXd normalizedAccumLens = eLens(eRefs) / eLens(eRefs).sum();
        for (int i = 1; i < normalizedAccumLens.size(); ++i) normalizedAccumLens(i) += normalizedAccumLens(i-1);

        // Prepare for polygon sampling
        Points sampledPoints(2, int(ceil(1/step)));
        size_t currLdmkIdx = 0, nextLdmkIdx = 1, prevLdmkIdx = vRefs.size()-1;
        int currSampledPointColumn = -1;
        double coveredPerimPercent = 0, d = 0;

        // Loop until we have sampled from the whole perimeter
        while (std::abs(coveredPerimPercent - 1.0) > 0.0001) {
            
            // Increment our indices to our landmark references once we have stepped past the previous edge
            if (coveredPerimPercent > normalizedAccumLens[currLdmkIdx]) {
                prevLdmkIdx = currLdmkIdx;
                nextLdmkIdx = (++currLdmkIdx+1) % vRefs.size();
                continue;
            }

            // Define how much of the perimeter we should move by:
            if (currLdmkIdx != 0) {
                // If we are not looking at the first line segment:
                d = (std::abs(coveredPerimPercent - normalizedAccumLens[prevLdmkIdx]) /
                    std::abs(normalizedAccumLens[currLdmkIdx] - normalizedAccumLens[prevLdmkIdx]));
            } else if (coveredPerimPercent > 0) {
                // If we are on the first line segment AND it is not the first iteration:
                d = coveredPerimPercent / normalizedAccumLens[currLdmkIdx];
            }
            // Otherwise d does not change
            
            // Sample point relative to current percentage of line segment
            // [sample] = [p1] + d * ([p2] - [p1])
            sampledPoints.col(++currSampledPointColumn) = ldmks.col(vRefs(currLdmkIdx)) + d * (ldmks.col(vRefs(nextLdmkIdx)) - ldmks.col(vRefs(currLdmkIdx)));
            coveredPerimPercent += step;
        }
        return sampledPoints;
    }

    
    // Uses sampled points to compute a centroid distance descriptor
    Eigen::VectorXd findCentroidDistance(const Eigen::VectorXi& ldmkIndices, const Points& ldmks, Points sampledPoints) {
        Eigen::Vector2d centroid = ldmks(Eigen::placeholders::all, ldmkIndices).rowwise().sum() / ldmkIndices.size();
        return centroidDistance(sampledPoints, centroid);

        // TODO maybe just do centroid distance here?
        // Points centroidMatrix(sampledPoints.cols());
        // centroidMatrix.colwise() = ldmks(Eigen::placeholders::all, ldmkIndices).rowwise().sum() / ldmkIndices.size();
        // return (centroidMatrix - sampledPoints).array().square().colwise().sum().sqrt();
    }

    // Uses OpenCV to compute the magnitude of the DFT of the centroid descriptor
    // based on the official OpenCV tutorial: https://docs.opencv.org/3.4/d8/d01/tutorial_discrete_fourier_transform.html
    Eigen::VectorXd invariantFourier(Eigen::VectorXd centroidDesc) {
        cv::Mat matDesc;
        cv::eigen2cv(centroidDesc, matDesc);
        cv::Mat planes[] = {matDesc,
                            cv::Mat::zeros(cv::Size(1, centroidDesc.size()), CV_64FC1)};
        cv::Mat complexI;
        cv::merge(planes, 2, complexI);         // Add to the expanded another plane with zeros
        cv::dft(complexI, complexI);            // this way the result may fit in the source matrix
        // compute the magnitude
        cv::split(complexI, planes);                   // planes[0] = Re(DFT(I), planes[1] = Im(DFT(I))
        cv::magnitude(planes[0], planes[1], planes[0]);// planes[0] = magnitude
        cv::Mat magI = planes[0];

        std::vector<double> dftDesc;
        if (magI.isContinuous()) {
            dftDesc.assign((double*)magI.data, (double*)magI.data + magI.total()*magI.channels());
        } else {
            for (int i = 0; i < magI.rows; ++i) {
                dftDesc.insert(
                    dftDesc.end(),
                    magI.ptr<double>(i), magI.ptr<double>(i)+magI.cols*magI.channels());
            }
        }
        Eigen::VectorXd IDFT(dftDesc.size());
        IDFT = Eigen::Map<Eigen::VectorXd>(dftDesc.data(), dftDesc.size(), 1);
        return IDFT;
    }


    // Runs the entire descriptor computation pipeline
    Eigen::VectorXd compute(const Eigen::VectorXi& vRefs, 
                            const Eigen::VectorXi& eRefs, 
                            const Points& ldmks,
                            const Eigen::VectorXd& eLens) {
        return invariantFourier(findCentroidDistance(vRefs, ldmks, samplePoints(vRefs, eRefs, ldmks, eLens, 0.05)));
    }
}