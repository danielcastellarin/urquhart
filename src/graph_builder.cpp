#include <matching.hpp>
#include <logging.hpp>
#include <memory>
#include <random>
#include <unordered_map>
#include <queue>
#include <unordered_set>
#include <map>
#include <algorithm>
#include <iostream>
#include <filesystem>
#include <sstream>
#include <list>
#include <chrono>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/String.h>

#include <ros/ros.h>
#include <ros/package.h>


// @@@@@@@@@@@@@@@@@@
// @ MISC FUNCTIONS @
// @@@@@@@@@@@@@@@@@@

// A hash function used to hash a pair of any kind
struct hash_pair {
    template <class T1, class T2>
    size_t operator()(const std::pair<T1, T2>& p) const
    {
        auto hash1 = std::hash<T1>{}(p.first);
        auto hash2 = std::hash<T2>{}(p.second);
 
        if (hash1 != hash2) {
            return hash1 ^ hash2;              
        }
         
        // If hash1 == hash2, their XOR is zero.
          return hash1;
    }
};

void writeHierarchyFiles(const urquhart::Observation& trees, std::string filePath, std::string fileName) {
    std::ofstream plyOut(filePath+"/p/"+fileName), triOut(filePath+"/t/"+fileName), 
                    hieOut(filePath+"/h/"+fileName), dscOut(filePath+"/d/"+fileName);
    
    logging::writeHierarchyToFile(trees, plyOut, triOut, hieOut, dscOut);
    
    hieOut.close();
    plyOut.close();
    triOut.close();
    dscOut.close();
}

Eigen::Matrix3d v2t(Eigen::Vector3d vec) {
    double c = cos(vec(2)), s = sin(vec(2));
    return Eigen::Matrix3d {
        {c, -s, vec(0)},
        {s,  c, vec(1)},
        {0,  0,      1}
    };
}

Eigen::Vector3d t2v(Eigen::Matrix3d tf) { 
    return Eigen::Vector3d{tf(0,2), tf(1,2), atan2(tf(1,0), tf(0,0))};
}



// @@@@@@@@@@@@@@@@@@@@@
// @ GRAPHSLAM STRUCTS @
// @@@@@@@@@@@@@@@@@@@@@

struct PoseNode { // Pose derived from matching local shapes with global representation
    int id, stateVectorIdx;
    Eigen::Vector3d pose;
    PoseNode(int i, Eigen::Vector3d& p, int svPos) : id(i), pose(p), stateVectorIdx(svPos) {}
    bool operator==(const PoseNode &other) const{ return id == other.id; }
};

struct LandmarkNode { // Point location derived from matching local shapes with global representation
    int globalReprIdx, stateVectorIdx; // ID corresponds to the landmark's column number in the global observation record
    LandmarkNode(int i, int svPos) : globalReprIdx(i), stateVectorIdx(svPos) {}
    bool operator==(const LandmarkNode &other) const{ return globalReprIdx == other.globalReprIdx; }
};


// Seems like common information matrix is setting 100s along the diagonal
// https://robotics.stackexchange.com/questions/22451/calculate-information-matrix-for-graph-slam

struct PPEdge { // Point location derived from matching local shapes with global representation
    int src, dst; // indices in the list of pose nodes for the source and destination of this edge
    Eigen::Vector3d distance;   // difference in position between the robot poses constrained by this edge
    Eigen::Matrix3d info;       // inverse covariance matrix for 'distance'
    PPEdge(int source, int destination, Eigen::Vector3d d) : src(source), dst(destination), distance(d) {
        info << 100,  0,   0,
                0,  100,   0,
                0,    0, 100;
    }
};

struct PLEdge {
    int src, dst;      // indices in the lists of pose and landmark nodes respectively for the source and destination of this edge
    Eigen::Vector2d distance;   // the observed position of a landmark relative to the robot
    Eigen::Matrix2d info;       // inverse covariance matrix for 'distance' 
    // PLEdge(int source, int destination, Eigen::Vector2d d) : src(source), dst(destination), distance(d) {
    PLEdge(int source, int destination, const Eigen::Ref<const Eigen::Vector2d>& d) : src(source), dst(destination), distance(d) {
        info << 100,  0,
                0,  100;
    }
};


struct SLAMGraph {
    std::shared_ptr<urquhart::Observation> geoHier = NULL;
    Eigen::VectorXd stateVector;

    std::vector<PoseNode> poseNodeList;
    std::vector<LandmarkNode> landmarkNodeList; // These should align with "landmarks" in Observation

    std::vector<PPEdge> ppEdges;
    std::vector<PLEdge> plEdges;
    
    std::vector<double> globalErrors;   // For performance measurement purposes...

    SLAMGraph() {}

    inline int getPoseSVIdx(int idx) { return poseNodeList[idx].stateVectorIdx; }
    inline int getLdmkSVIdx(int idx) { return landmarkNodeList[idx].stateVectorIdx; }

    void addPoseNode(Eigen::Vector3d& pose) {
        // Add new pose node to end of list
        int nodeId = poseNodeList.size();
        poseNodeList.push_back(PoseNode(nodeId, pose, stateVector.size())); 
        
        // Append (x,y,theta) to the end of state vector
        stateVector.conservativeResize(stateVector.size()+3);
        stateVector.tail(3) = pose;
    }
    void addPPEdge(Eigen::Vector3d& currentGlobalPose) {
        // Add an new constraint to a new global pose for the robot 
        // This only gets called once per keyframe (at most)
        ppEdges.push_back(PPEdge(poseNodeList.size()-1, poseNodeList.size(), currentGlobalPose - poseNodeList.back().pose));
        addPoseNode(currentGlobalPose);
    }
    
    void addLandmarkNode(int idxInHier) {
        // Precondition: landmark data must already be in geometric hierarchy at the given index 
        landmarkNodeList.push_back(LandmarkNode(idxInHier, -1));
    }
    void addLandmarkNode(int idxInHier, PtLoc& globalPosition) {
        // Precondition: landmark data must already be in geometric hierarchy at the given index 
        
        // Define landmark node
        landmarkNodeList.push_back(LandmarkNode(idxInHier, stateVector.size()));

        // Append (x,y) to the end of state vector
        stateVector.conservativeResize(stateVector.size()+2);
        stateVector.tail(2) = globalPosition;
    }
    int addLandmarkNode(const PtLoc& globalPosition) {

        // Add the point to the set of landmarks in the global geometric hierarcy
        int ldmkIdx = geoHier->landmarks.cols();
        geoHier->landmarks.conservativeResize(2, ldmkIdx+1);
        geoHier->landmarks.col(ldmkIdx) = globalPosition;
        
        // Define landmark node
        landmarkNodeList.push_back(LandmarkNode(ldmkIdx, stateVector.size()));

        // Append (x,y) to the end of state vector
        stateVector.conservativeResize(stateVector.size()+2);
        stateVector.tail(2) = globalPosition;

        return ldmkIdx;
    }
    void addPLEdge(Eigen::Index globalLandmarkIdx, const PtLoc& landmarkLocalPosition) {
        // Precondition: landmark node must already be in the graph's list of nodes
        
        // May be called with landmark nodes that existed before current keyframe or nodes that were just created
        // landmarkLocalPosition is taken directly from the landmark's position relative to the robot in the current keyframe
        plEdges.push_back(PLEdge(poseNodeList.size()-1, globalLandmarkIdx, landmarkLocalPosition));
        if (landmarkNodeList[globalLandmarkIdx].stateVectorIdx == -1) {
            landmarkNodeList[globalLandmarkIdx].stateVectorIdx = stateVector.size();
            stateVector.conservativeResize(stateVector.size()+2);
            stateVector.tail(2) = geoHier->landmarks.col(globalLandmarkIdx);
        }
    }

    void addPLEdge(int poseNodeIdx, Eigen::Index globalLandmarkIdx, const PtLoc& landmarkLocalPosition) {
        // Precondition: landmark node must already be in the graph's list of nodes
        
        // May be called with landmark nodes that existed before current keyframe or nodes that were just created
        // landmarkLocalPosition is taken directly from the landmark's position relative to the robot in the current keyframe
        plEdges.push_back(PLEdge(poseNodeIdx, globalLandmarkIdx, landmarkLocalPosition));
    }

    double computeGlobalError() {
        double gErr = 0;

        for (const auto& edge : ppEdges) {
            int iStart = poseNodeList[edge.src].stateVectorIdx, jStart = poseNodeList[edge.dst].stateVectorIdx;
            Eigen::Vector3d xI(stateVector.segment(iStart, 3)), xJ(stateVector.segment(jStart, 3)), eIJ;
            Eigen::Matrix3d XI = v2t(xI), XJ = v2t(xJ), ZIJ = v2t(edge.distance);
            Eigen::Matrix3d XI_inv = XI.inverse(), ZIJ_inv = ZIJ.inverse();

            eIJ = t2v(ZIJ_inv * (XI_inv * XJ)); // NOTE previous 5 lines are verbatim from eIJ calc when optimizing graph; make function
            gErr += eIJ.transpose() * edge.info * eIJ;
        }

        for (const auto& edge : plEdges) {
            int iStart = poseNodeList[edge.src].stateVectorIdx, jStart = landmarkNodeList[edge.dst].stateVectorIdx;
            Eigen::Vector3d x(stateVector.segment(iStart, 3));
            Eigen::Vector2d l(stateVector.segment(jStart, 2)), eIJ;
            double s = sin(x(2)), c = cos(x(2));
            Eigen::Matrix2d R_transposed {
                { c, s},
                {-s, c}
            };
            eIJ = (R_transposed * (l - x.head(2))) - edge.distance; // 2x1
            gErr += eIJ.transpose() * edge.info * eIJ;
        }

        return gErr;
    }

    double saveGlobalError(std::string filePath, std::string fileName) {
        double gErr = 0;

        std::ofstream allErrOut(filePath+"/err/"+fileName);

        for (const auto& edge : ppEdges) {
            int iStart = poseNodeList[edge.src].stateVectorIdx, jStart = poseNodeList[edge.dst].stateVectorIdx;
            Eigen::Vector3d xI(stateVector.segment(iStart, 3)), xJ(stateVector.segment(jStart, 3)), eIJ;
            Eigen::Matrix3d XI = v2t(xI), XJ = v2t(xJ), ZIJ = v2t(edge.distance);
            Eigen::Matrix3d XI_inv = XI.inverse(), ZIJ_inv = ZIJ.inverse();

            eIJ = t2v(ZIJ_inv * (XI_inv * XJ)); // NOTE previous 5 lines are verbatim from eIJ calc when optimizing graph; make function
            double eVal = eIJ.transpose() * edge.info * eIJ;
            allErrOut << "P" << edge.src << " P" << edge.dst << " " << eVal << std::endl;
            gErr += eVal;
        }

        allErrOut << "===================================" << std::endl;


        std::map<int, double> errByPose;
        for (const auto& edge : plEdges) {
            int iStart = poseNodeList[edge.src].stateVectorIdx, jStart = landmarkNodeList[edge.dst].stateVectorIdx;
            Eigen::Vector3d x(stateVector.segment(iStart, 3));
            Eigen::Vector2d l(stateVector.segment(jStart, 2)), eIJ;
            double s = sin(x(2)), c = cos(x(2));
            Eigen::Matrix2d R_transposed {
                { c, s},
                {-s, c}
            };
            eIJ = (R_transposed * (l - x.head(2))) - edge.distance; // 2x1
            double eVal = eIJ.transpose() * edge.info * eIJ;

            if (errByPose.find(edge.src) == errByPose.end()) errByPose[edge.src] = eVal;
            else errByPose[edge.src] += eVal;

            allErrOut << "P" << edge.src << " L" << edge.dst << " " << eVal << std::endl;
            gErr += eVal;
        }
        
        allErrOut << "===================================" << std::endl;
        for (const auto& [src, totErr] : errByPose) allErrOut << "P" << src << ": " << totErr << std::endl;

        allErrOut.close();

        return gErr;
    }

    bool optimizeGraph(bool isDebug) {
        
        // Initialize sparse system H and coefficient vector b with zeroes
        Eigen::MatrixXd H = Eigen::MatrixXd::Zero(stateVector.size(), stateVector.size());
        Eigen::VectorXd b = Eigen::VectorXd::Zero(stateVector.size());
        bool isFirstThereforeAddPrior = true;

        // Resolve pose-pose constraints
        for (auto& edge : ppEdges) {
            
            // Obtain node states from the state vector
            auto iSlice = Eigen::seqN(getPoseSVIdx(edge.src), 3), jSlice = Eigen::seqN(getPoseSVIdx(edge.dst), 3);
            Eigen::Vector3d xI(stateVector(iSlice)), xJ(stateVector(jSlice));

            // Use xI, xJ, and the edge distance (aka zIJ) to compute error eIJ and jacobians A,B
            Eigen::Vector3d eIJ;
            Eigen::Matrix3d A, B;
            { // Math block
                Eigen::Matrix3d XI = v2t(xI), XJ = v2t(xJ), ZIJ = v2t(edge.distance);
                Eigen::Matrix3d XI_inv = XI.inverse(), ZIJ_inv = ZIJ.inverse();

                eIJ = t2v(ZIJ_inv * (XI_inv * XJ));

                Eigen::Matrix2d Aii = -ZIJ.block(0,0,2,2).transpose() * XI.block(0,0,2,2).transpose(), 
                RotXI_derived {
                    { XI(0,1), XI(0,0)},
                    {-XI(0,0), XI(0,1)}
                };
                
                Eigen::Vector2d Aij = (ZIJ.block(0,0,2,2).transpose() * RotXI_derived) * (xJ.head(2) - xI.head(2)), Bij{0,0};
                Eigen::RowVector2d Aji{0,0}, Bji{0,0};
                
                A << Aii, Aij,
                     Aji,  -1;
                B << -Aii, Bij,
                      Bji,   1;
            }

            // Update H and b accordingly
            b(iSlice) += eIJ.transpose() * edge.info * A;
            b(jSlice) += eIJ.transpose() * edge.info * B;
            H(iSlice, iSlice) += A.transpose() * edge.info * A;
            H(iSlice, jSlice) += A.transpose() * edge.info * B;
            H(jSlice, iSlice) += H(iSlice, jSlice).transpose();
            H(jSlice, jSlice) += B.transpose() * edge.info * B;

            // Add a prior only to the first pose node of the graph
            // This will fix that node to remain at its current location (fixed reference frame)
            //   all the other measurements are relative to this one
            if (isFirstThereforeAddPrior) {
                H(iSlice, iSlice) += Eigen::Matrix3d::Identity(); // I think scalar "lambda" can be applied here too?
                isFirstThereforeAddPrior = false;
            }
        }

        // Resolve pose-landmark constraints
        for (auto& edge : plEdges) {

            // Obtain node states from the state vector
            auto iSlice = Eigen::seqN(getPoseSVIdx(edge.src), 3), jSlice = Eigen::seqN(getLdmkSVIdx(edge.dst), 2);
            Eigen::Vector3d x(stateVector(iSlice));
            Eigen::Vector2d l(stateVector(jSlice));

            // Use x, l, and the edge distance (aka zIJ) to compute error eIJ and jacobians A,B
            Eigen::Vector2d eIJ;
            Eigen::Matrix<double, 2, 3> A;
            Eigen::Matrix2d B;
            { // Math block
                double s = sin(x(2)), c = cos(x(2));
                B << c, s, // B is just the rotation matrix of x transposed
                    -s, c;
                Eigen::Matrix2d RotX_derived {
                    {-s,  c},
                    {-c, -s}
                };
                Eigen::Vector2d TransXIdiff = l - x.head(2), A2 = RotX_derived * TransXIdiff;

                eIJ = (B * TransXIdiff) - edge.distance; // 2x1
                A << -B, A2; // 2x3
            }

            // Update H and b accordingly
            b(iSlice) += eIJ.transpose() * edge.info * A;
            b(jSlice) += eIJ.transpose() * edge.info * B;
            H(iSlice, iSlice) += A.transpose() * edge.info * A;
            H(iSlice, jSlice) += A.transpose() * edge.info * B;
            H(jSlice, iSlice) += H(iSlice, jSlice).transpose();
            H(jSlice, jSlice) += B.transpose() * edge.info * B;
        }

        // Solve linear system
        Eigen::VectorXd dx = H.llt().solve(b);
        bool goodChange = true;

        // Validate output: make sure nothing changed too much
        // TODO use aggregated global error related to last pose to prompt rollback 
        if (isDebug) {
            Eigen::Vector3d diffPose;
            for (const auto& node : poseNodeList) {
                diffPose = dx.segment(node.stateVectorIdx, 3);
                if (std::abs(diffPose(0)) > 2 || std::abs(diffPose(1)) > 2 || std::abs(diffPose(2)) > 0.5) {
                    std::cout << "Pose node " << node.id << " changed to much, (" << diffPose.transpose() << "), cancelling optimization!!" << std::endl;
                    goodChange = false;
                }
            }

            PtLoc diffPosition;
            for (const auto& node : landmarkNodeList) {
                if (node.stateVectorIdx == -1) continue;
                diffPosition = dx.segment(node.stateVectorIdx, 2);
                if (std::abs(diffPosition(0)) > 4 || std::abs(diffPosition(1)) > 4) {
                    std::cout << "Landmark node " << node.globalReprIdx << " changed to much, (" << diffPosition.transpose() << "), cancelling optimization!!" << std::endl;
                    goodChange = false;
                }
            }
        }
        // if ((dx.array().abs() > 2).any()) return false;


        // Update the state vector; according to the result of the system of equations
        // stateVector -= H.llt().solve(b);
        stateVector -= dx;

        // Copy data from the state vector into graph nodes (for easier access)
        for (auto& node : poseNodeList) node.pose = stateVector.segment(node.stateVectorIdx, 3);
        for (auto& node : landmarkNodeList) {
            if (node.stateVectorIdx == -1) continue;
            geoHier->landmarks.col(node.globalReprIdx) = stateVector.segment(node.stateVectorIdx, 2);
        } 
        globalErrors.push_back(computeGlobalError());
        if (isDebug) std::cout << "New global error: " << globalErrors.back();

        return goodChange;
    }

    void writeGraphToFiles(std::string filePath, std::string fileName) {
        std::ofstream nodesOut(filePath+"/graph_nodes/"+fileName), edgesOut(filePath+"/graph_edges/"+fileName), errOut(filePath+"/!gError.txt");
        
        // Output Nodes
        for (const auto& pn : poseNodeList) nodesOut << "P" << pn.id << " " << pn.pose.transpose() << std::endl;
        for (const auto& ln : landmarkNodeList) nodesOut << "L" << ln.globalReprIdx << " " << geoHier->landmarks.col(ln.globalReprIdx).transpose() << std::endl;
        // TODO ignore landmark nodes if svID = -1 (landmark unobserved so far)

        // Output Edges
        for (const auto& ppe : ppEdges) edgesOut << "P" << ppe.src << " P" << ppe.dst << " " << ppe.distance.transpose() << std::endl;
        for (const auto& ple : plEdges) edgesOut << "P" << ple.src << " L" << ple.dst << " " << ple.distance.transpose() << std::endl;

        // Save global error to single file (overwriting)
        // globalErrors.push_back(computeGlobalError());
        // globalErrors.push_back(saveGlobalError(filePath, fileName));
        for (const auto& ge : globalErrors) errOut << ge << std::endl;

        nodesOut.close();
        edgesOut.close();
        errOut.close();
    }

};


Eigen::Matrix3d tfFromSubsetMatches(const Points& localLandmarks, const Points& globalLandmarks, const std::vector<std::pair<Eigen::Index, Eigen::Index>>& pointMatchRefs, const std::vector<int>& indices, int numMatches) {
    Eigen::MatrixXd A(numMatches+numMatches, 4);
    Eigen::VectorXd b(numMatches+numMatches);

    // https://math.stackexchange.com/questions/77462/finding-transformation-matrix-between-two-2d-coordinate-frames-pixel-plane-to
    int startRow = 0;
    for (int i = 0; i < numMatches; ++i) {
        // TODO perform more elegant block assignments here
        Eigen::Index lIdx = pointMatchRefs.at(indices.at(i)).first, gIdx = pointMatchRefs.at(indices.at(i)).second;
        A.row(startRow)   = (Eigen::RowVector4d() << localLandmarks(0, lIdx), -localLandmarks(1, lIdx), 1, 0).finished();
        A.row(startRow+1) = (Eigen::RowVector4d() << localLandmarks(1, lIdx),  localLandmarks(0, lIdx), 0, 1).finished();
        b[startRow]   = globalLandmarks(0, gIdx);
        b[startRow+1] = globalLandmarks(1, gIdx);
        startRow += 2;
    }

    // https://www.cs.cmu.edu/~16385/s17/Slides/10.1_2D_Alignment__LLS.pdf <-- slide 24
    // x = (A^T A)^-1 A^T b
    Eigen::Vector4d x = (A.transpose() * A).inverse() * A.transpose() * b;

    // Return the 3x3 matrix to transform frames: reference --> target
    return Eigen::Matrix3d{
        {x[0], -x[1], x[2]},
        {x[1],  x[0], x[3]},
        {   0,     0,    1}
    };
}


// Original RANSAC metrics from prior work
// euclid dist b/w points after transform (d) = 0.5
// ratio of outliers to cause exit (r) = 0.99
// max iterations (s) = 40000
using GlobalPt = std::pair<Eigen::Index, PtLoc>;
bool estimateBestTf(const Points& localLandmarks, const Points& globalLandmarks,
        const std::vector<std::pair<Eigen::Index, Eigen::Index>>& allMatches,
        std::unordered_map<Eigen::Index, GlobalPt>& acceptedMatches,
        Eigen::Vector3d& bestTf, std::vector<GlobalPt>& unmatchedLocals,
        int maxIter, int reqMatchesForTf, double threshForValidAssoc, double threshForAssocNetEligibility, double matchRatio, double sensorRange)
{
    bool hasEstimate = false;

    // Preallocate space to perform matrix operations (can be quite large)
    Points calcMat(2, globalLandmarks.cols());
    Eigen::VectorXi closebyPointsMask(globalLandmarks.cols());
    
    // Initialize components to randomly sample landmark matches
    std::random_device randomDevice;
    std::mt19937 rng(randomDevice());
    // rng.seed(10);
    std::vector<int> indices(allMatches.size());
    std::iota(indices.begin(), indices.end(), 0);


    // Loop until we tried too many times or we found the expected number of landmark associations
    int expectedNumMatchedLandmarks = localLandmarks.cols() * matchRatio;
    for (int i = 0; i < maxIter && acceptedMatches.size() <= expectedNumMatchedLandmarks; ++i) {
        std::unordered_map<Eigen::Index, std::pair<Eigen::Index, PtLoc>> invertedGoodMatchesSoFar; // targ --> ref
        std::unordered_set<Eigen::Index> problematicGlobals;
        std::vector<std::pair<Eigen::Index, PtLoc>> unmatchedPoints;

        // Take random subset of matches to use for calculating the global -> local transform
        std::shuffle(indices.begin(), indices.end(), rng);
        Eigen::Matrix3d tf = tfFromSubsetMatches(localLandmarks, globalLandmarks, allMatches, indices, reqMatchesForTf);

        // Determine which landmarks are "observable" from the given pose
        calcMat = globalLandmarks.colwise() - tf(Eigen::seq(0,1), 2);
        closebyPointsMask = (calcMat.array().abs() < sensorRange).colwise().all().cast<int>();
        int numClosePoints = closebyPointsMask.count();
        if (numClosePoints == 0) continue;

        // Construct a smaller matrix of global landmark positions for squared distance computations
        Points closePoints(2, numClosePoints), intMat(2, numClosePoints);
        Eigen::VectorXi closePointIndices(numClosePoints);
        for (int i = 0, j = 0; i < closebyPointsMask.rows(); ++i) {
            if (closebyPointsMask(i)) {
                closePoints.col(j) = globalLandmarks.col(i);
                closePointIndices(j++) = i;
            }
        }

        Eigen::Index closestLandmarkIdx;
        // Match as many points in the keyframe with landmarks in the global frame
        for (int lIdx = 0; lIdx < localLandmarks.cols(); ++lIdx) {
            // Define this tree's local position as a homogeneous matrix
            Eigen::Matrix3d localPointTf {
                {1, 0, localLandmarks(0, lIdx)},
                {0, 1, localLandmarks(1, lIdx)},
                {0, 0, 1},
            };
            // Obtain the position of this tree in the global frame
            GlobalPt gp = {lIdx, PtLoc{(tf * localPointTf)(Eigen::seq(0,1), 2)}};

            // Get the distance to the nearest global landmark (with that landmark's index)
            intMat = closePoints.colwise() - gp.second;
            double closestDistance = intMat.colwise().squaredNorm().minCoeff(&closestLandmarkIdx);
            closestLandmarkIdx = closePointIndices(closestLandmarkIdx);

            // Make an association with the nearest neighbor within a threshold
            if (closestDistance < threshForValidAssoc) {
                // Ensure the closest landmark has not already been matched with something...
                // Ignore the association if the closest global point was already the source of conflict with other local points
                if (problematicGlobals.find(closestLandmarkIdx) != problematicGlobals.end()) {
                    unmatchedPoints.push_back(gp);

                // Cancel a previous match if this local point is closest to a global point that has already been matched
                } else if (invertedGoodMatchesSoFar.find(closestLandmarkIdx) != invertedGoodMatchesSoFar.end()) {
                    problematicGlobals.insert(closestLandmarkIdx);
                    unmatchedPoints.push_back(gp);
                    unmatchedPoints.push_back(invertedGoodMatchesSoFar[closestLandmarkIdx]);
                    invertedGoodMatchesSoFar.erase(closestLandmarkIdx);

                // Otherwise, keep this association because it is probably right
                } else {
                    invertedGoodMatchesSoFar[closestLandmarkIdx] = gp;
                }

            // Elect this point for the association network if it is too far away from everything else
            } else if (threshForAssocNetEligibility <= closestDistance) unmatchedPoints.push_back(gp);
        }
        
        // Keep this result if we have not seen anything more fitting
        if (invertedGoodMatchesSoFar.size() > acceptedMatches.size()) { // TODO make these copies shallow
            acceptedMatches = invertedGoodMatchesSoFar;
            bestTf = t2v(tf);
            unmatchedLocals = unmatchedPoints;
            hasEstimate = true;
        }
    }

    return hasEstimate;
}

    // auto t1 = std::chrono::high_resolution_clock::now();
    // auto t2 = std::chrono::high_resolution_clock::now();
    // auto ms_int = std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1);
    // std::cout << ms_int.count() << "ms\n";


// @@@@@@@@@@@@@@@@@@@@
// @ GLOBAL VARIABLES @
// @@@@@@@@@@@@@@@@@@@@

SLAMGraph g;

// Global variables for Association Network
using PointRef = std::pair<int, int>;   // <frameID, ldmkIdx>
using AssocSet = std::unordered_set<PointRef, hash_pair>;
std::unordered_map<PointRef, AssocSet, hash_pair> unresolvedLandmarks;
std::map<int, Points> activeObsWindow;

// Output options (screen, logs, realtime)
bool isConsoleDebug = false, isLogging = false, isRealtimeVis = false;
ros::Publisher graphPub, hierPub;
std::string outputPath, startingMap;

// Parameters for keyframe-global polygon matching
double polyMatchThresh, polyMatchThreshStep, reqMatchedPolygonRatio;
int numSideBoundsForMatch; 

// Parameters for association filtering (my "RANSAC")
double ransacValidAssocThresh, ransacAssocNetThresh, ransacMatchRatio, maxSensorRange;
int ransacMaxIter, ransacMatchPrereq, ransacMatchSampleSize;

// Parameters for Association Network (new landmark discovery)
double ptAssocThresh;
int minAssocForNewLdmk, associationWindowSize;



// @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
// @ ASSOCIATION NETWORK HELPER METHODS @
// @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@

bool canFormKClique(const AssocSet& a, const AssocSet& b, const int &k) {
    // Escape early if the old point does not have enough associations to belong to a k-clique
    if (a.size() < k) return false;

    // Iterate over the set with fewer elements (we should be passing the sets by reference here)
    AssocSet s1, s2;
    if (a.size() < b.size()) s1 = a, s2 = b; else s1 = b, s2 = a;

    // Return true if the number of shared elements in the sets surpasses k
    int count = 0;
    for (const PointRef& pf : s1) if (s2.find(pf) != s2.end()) ++count;
    return count >= k;
}

void deleteTreeAssoc(std::unordered_map<PointRef, AssocSet, hash_pair>& map, PointRef treeRefToDelete) {
    for (const PointRef& pf : map[treeRefToDelete]) if (!(treeRefToDelete == pf)) map[pf].erase(treeRefToDelete);
    map.erase(treeRefToDelete);
}



// @@@@@@@@@@@@@@@@@
// @ NODE CALLBACK @
// @@@@@@@@@@@@@@@@@

void constructGraph(const sensor_msgs::PointCloud2ConstPtr& cloudMsg) {
    if (isConsoleDebug) std::cout << "Received keyframe " << cloudMsg->header.seq << std::endl;

    // Take the points from the PointCloud2 message and create a geometric hierarchy from it
    pcl::PointCloud<pcl::PointXY> localCloud;
    pcl::fromROSMsg(*cloudMsg, localCloud);
    Points vectorOfTrees(2, localCloud.size());
    int idx = 0;
    for (const auto& p : localCloud) vectorOfTrees.col(idx++) = PtLoc{p.x, p.y};
    urquhart::Observation localObs(vectorOfTrees);

    // Save local observation data (if desired)
    if (isConsoleDebug) std::cout << "Defined local observation for keyframe " << cloudMsg->header.seq << " with " << vectorOfTrees.cols() << " observed trees." << std::endl;
    if (isLogging) {
        writeHierarchyFiles(localObs, outputPath+"/local", std::to_string(g.poseNodeList.size()+1)+".txt");
        std::ofstream ptsOut(outputPath+"/local/pts/"+std::to_string(g.poseNodeList.size()+1)+".txt");
        for (const auto& c : localObs.landmarks.colwise()) ptsOut << c(0) << " " << c(1) << std::endl;
        ptsOut.close();
    }

    // Do different things depending on whether the global map is available
    if (g.geoHier == NULL) {

        // %%%%%%%%%%%%%%%%%%%%%%%%%
        // Global Map Initialization
        // %%%%%%%%%%%%%%%%%%%%%%%%%

        // TODO determine what gets done when the global map is unavailable (might also occur when keyframe does not match with anything on the map)
        //      maybe I could modify the matching procedure a bit to iteratively loosen the matching constraints if not enough matches are found?

        // Assign the graph's Observation to the current local one
        g.geoHier = std::make_shared<urquhart::Observation>(std::move(localObs));

        // Add robot pose (as an anchor)
        Eigen::Vector3d startPose{0,0,0};
        g.addPoseNode(startPose);

        // Add landmarks and constrain them to the initial pose node
        for (int lIdx = 0; lIdx < localObs.landmarks.cols(); ++lIdx) {
            PtLoc position(localObs.landmarks.col(lIdx));
            g.addLandmarkNode(lIdx, position);
            g.addPLEdge(lIdx, position);
        }

        if (isConsoleDebug) {
            std::cout << "Constructed initial global geometric hierarchy " << cloudMsg->header.seq << std::endl;
            std::cout << "Number of initial trees: " << localObs.landmarks.cols() << std::endl;
        }

        // Publish initial graph state for visualization
        if (isRealtimeVis) {
            std::stringstream graphStream;
            
            // Initial Pose
            graphStream << g.poseNodeList[0].pose(0) << " " << g.poseNodeList[0].pose(1) << " " << g.poseNodeList[0].pose(2);

            graphStream << "|";
            int tmpIdx = 0;     // Landmark Positions
            for (const PtLoc& ldmk : g.geoHier->landmarks.colwise()) graphStream << (tmpIdx++!=0 ? ":": "") << ldmk(0) << " " << ldmk(1);

            // No existing landmarks
            graphStream << "||";
            // References to landmarks newly seen in this frame
            for (int idx = 0; idx < g.geoHier->landmarks.cols(); ++idx) graphStream << (idx!=0 ? " ": "") << idx;

            graphStream << "|-1"; // Previous error (N/A)

            // Construct graph message and publish
            std_msgs::String graphMsg;
            graphMsg.data = graphStream.str();
            graphPub.publish(graphMsg);
        }
        g.globalErrors.push_back(g.computeGlobalError());

        // Do nothing else because we need to get more observations before optimizing



    } else {

        // %%%%%%%%%%%%%%%%%%%%%%%%
        // Initial Polygon Matching 
        // %%%%%%%%%%%%%%%%%%%%%%%%

        // Matching speed should not be a bottleneck when querying global representation
        // If faster speed desired, only elect global polygons that are a certain distance away from the previous pose  
        // if not enough matches are made, maybe drop the frame? or expand global polygon election range

        // Attempt to match the current local observation with the global representation
        double matchingThresh = polyMatchThresh;
        std::vector<std::pair<Eigen::Index, Eigen::Index>> matchingPointIndices;
        int maxIter = 100, numIt = 0;
        do {
            // matchingPointIndices = matching::hierarchyIndexMatching(localObs, *g.geoHier, matchingThresh, numSideBoundsForMatch, reqMatchedPolygonRatio);
            matchingPointIndices = matching::nonGreedyHierarchyIndexMatching(localObs, *g.geoHier, matchingThresh, numSideBoundsForMatch, reqMatchedPolygonRatio);
            matchingThresh += polyMatchThreshStep;
        } while (matchingPointIndices.size() < ransacMatchPrereq && ++numIt < maxIter);

        // Exit early if we couldn't find matches within a reasonable amount of time
        if (matchingPointIndices.size() < ransacMatchPrereq) {
            if (isConsoleDebug) std::cout << "Dropping keyframe " << cloudMsg->header.seq << " because no matches could be found." << std::endl;
            return;
        }

        // LOG MATCHES
        if (isConsoleDebug) std::cout << "Matched " << matchingPointIndices.size() << " points with the global frame using threshold=" << matchingThresh - polyMatchThreshStep << std::endl;
        if (isLogging) {
            std::ofstream matOut(outputPath+"/match/"+std::to_string(g.poseNodeList.size())+".txt");
            for (const auto& [lIdx, gIdx] : matchingPointIndices) {
                matOut << localObs.ldmkX(lIdx) << "," << localObs.ldmkY(lIdx) << "|";
                matOut << g.geoHier->ldmkX(gIdx) << "," << g.geoHier->ldmkY(gIdx) << std::endl;
            }
            matOut.close();
        }


        // %%%%%%%%%%%%%%%%%%%%
        // Transform Estimation
        // %%%%%%%%%%%%%%%%%%%%

        // Filter associations while preparing data for GraphSLAM
        std::unordered_map<Eigen::Index, GlobalPt> goodAssociationMap;
        Eigen::Vector3d bestGlobalRobotPose;
        std::vector<GlobalPt> unmatchedGlobalPoints;
        if (!estimateBestTf(localObs.landmarks,
                            g.geoHier->landmarks, 
                            matchingPointIndices, 
                            goodAssociationMap, 
                            bestGlobalRobotPose, 
                            unmatchedGlobalPoints, 
                            ransacMaxIter, 
                            ransacMatchSampleSize, 
                            ransacValidAssocThresh, 
                            ransacAssocNetThresh,
                            ransacMatchRatio,
                            maxSensorRange)) {
            if (isConsoleDebug) std::cout << "Dropping keyframe " << cloudMsg->header.seq << " because all potential matches could not be verified." << std::endl;
            return;
        }

        // LOG FINAL ASSOCIATIONS
        if (isLogging) { // TODO save the final associations and new landmarks so the visualization can color them differently
            std::ofstream ascOut(outputPath+"/finalAssoc/"+std::to_string(g.poseNodeList.size())+"m.txt");
            for (const auto& [gIdx, localInGlobal] : goodAssociationMap) {
                ascOut << localObs.ldmkX(localInGlobal.first) << "," << localObs.ldmkY(localInGlobal.first) << "|";
                ascOut << g.geoHier->ldmkX(gIdx) << "," << g.geoHier->ldmkY(gIdx) << std::endl;
            }
            ascOut.close();
            std::ofstream uasOut(outputPath+"/finalAssoc/"+std::to_string(g.poseNodeList.size())+"u.txt");
            for (const auto& [lIdx, globalPoint] : unmatchedGlobalPoints) {
                uasOut << localObs.ldmkX(lIdx) << "," << localObs.ldmkY(lIdx) << "|" << globalPoint(0) << "," << globalPoint(1) << std::endl;
            }
            uasOut.close();
        }
        if (isConsoleDebug) {
            std::cout << "Final association count is " << goodAssociationMap.size() << ", and " << unmatchedGlobalPoints.size() << " point(s) will be added to the association network." << std::endl;
            std::cout << "Estimated best global robot pose: " << bestGlobalRobotPose.transpose() << std::endl;
        }


        // %%%%%%%%%%%%%%%%%%%%%
        // Association Filtering
        // %%%%%%%%%%%%%%%%%%%%%

        // Store both the local and global positions of unmatched landmarks
        // idx 0,n = global; n+1,2n = local
        int numUnmatchedLdmks = unmatchedGlobalPoints.size(), ldmkRefIdx = 0; // reference idx
        Points keyframePoints = Eigen::MatrixXd::Zero(2, numUnmatchedLdmks + numUnmatchedLdmks);
        std::vector<std::pair<PtLoc, AssocSet>> newlyEstablishedLandmarkObservations;


        // NOTE: "unresolvedLandmarks" is the association network (map from "frameID" -> Points)
        for (const auto& [lIdx, globalPoint] : unmatchedGlobalPoints) {
            // Add global and local positions to the association network reference list
            keyframePoints.col(ldmkRefIdx) = globalPoint;
            keyframePoints.col(numUnmatchedLdmks+ldmkRefIdx) = localObs.landmarks.col(lIdx);

            // std::cout << "Processing PF: {"<< g.poseNodeList.size() << "," << ldmkRefIdx <<"}" << std::endl;
            PointRef pf = {g.poseNodeList.size(), ldmkRefIdx++}; // {"frameID", ldmkIdx}
            AssocSet newAssociations;
            
            // std::cout << "Current keyframePoints:\n" << keyframePoints << std::endl;

            // Attempt to associate the "Tree" with every point from previous observations in the active window
            for (auto& [observedTreeRef, associatedTreeRefs] : unresolvedLandmarks) {
                if (observedTreeRef.first == pf.first) continue;    // Do not associate points in the same frame

                // std::cout << "Associating with {"<< observedTreeRef.first << "," << observedTreeRef.second <<"}, ";
                // std::cout << "whose position is "<< activeObsWindow[observedTreeRef.first].col(observedTreeRef.second).transpose();

                // Using max feature distance instead of euclidean distance for increased speed
                if (((activeObsWindow[observedTreeRef.first].col(observedTreeRef.second) - keyframePoints.col(pf.second)).array().abs() < ptAssocThresh).all()) {
                    associatedTreeRefs.insert(pf);
                    newAssociations.insert(observedTreeRef);
                    // std::cout << "      YES";
                }
                // std::cout << std::endl;
            }

            // std::cout << "===================================================" << std::endl;
            // std::cout << "Associated with " << newAssociations.size() << " points from previous frames." << std::endl;

            // Check if the newly-observed tree has enough associated observations to establish a new landmark
            newAssociations.insert(pf);
            AssocSet maxSubset = newAssociations;
            AssocSet::iterator itr = maxSubset.begin();
            bool isLandmarkDefined = false;

            while (maxSubset.size() >= minAssocForNewLdmk && !isLandmarkDefined) {
                if (canFormKClique(unresolvedLandmarks[*itr], maxSubset, minAssocForNewLdmk)) {
                    // if the iterator reaches the end of this new tree's association set, 
                    // then all remaining points in the set should form a k-clique - satisfying the condition to make a landmark
                    isLandmarkDefined = ++itr == maxSubset.end();
                } else {
                    maxSubset.erase(itr);       // Remove any elements cannot compose a k-clique with the new point
                    itr = maxSubset.begin();    // Double-check any elements that have already been processed
                }
            }
            unresolvedLandmarks[pf] = newAssociations;

            // Collapse the observations into a single landmark if k-CLIQUE is present
            if (isLandmarkDefined) {
                PtLoc establishedLdmk = Eigen::Vector2d::Zero();
                // Delete all references to the trees that are about to become a landmark
                // Average out the position of the landmark
                for (const PointRef& essentialTree : maxSubset) {
                    deleteTreeAssoc(unresolvedLandmarks, essentialTree);
                    establishedLdmk += (essentialTree.first == g.poseNodeList.size() ? keyframePoints : activeObsWindow[essentialTree.first]).col(essentialTree.second);
                } 
                newlyEstablishedLandmarkObservations.push_back({establishedLdmk / maxSubset.size(), maxSubset});
                if (isConsoleDebug) std::cout << "NEW LANDMARK ESTABLISHED AT " << (establishedLdmk / maxSubset.size()).transpose() << std::endl;
            }
        }

        // Store the positions of this keyframe's unassociated points
        activeObsWindow[g.poseNodeList.size()] = keyframePoints;


        // %%%%%%%%%%%%%%%%
        // GraphSLAM Update
        // %%%%%%%%%%%%%%%%

        std::vector<int> existingLdmkIds, newLdmkIds;

        // Define a new node and edge for the robot's estimated odometry
        if (g.poseNodeList.size() > 0) g.addPPEdge(bestGlobalRobotPose);
        else g.addPoseNode(bestGlobalRobotPose);

        // Add constraints for all local points that matched with existing landmarks
        for (const auto& [gIdx, localInGlobal] : goodAssociationMap) {
            g.addPLEdge(gIdx, localObs.landmarks.col(localInGlobal.first));
            existingLdmkIds.push_back(gIdx);
        }

        // Add new landmark nodes (if any) and constraint them to the poses where they were observed original observations
        for (const auto& [globalPoint, as] : newlyEstablishedLandmarkObservations) {
            // Define the new landmark
            int newId = g.addLandmarkNode(globalPoint);
            newLdmkIds.push_back(newId);

            // Retroactively create constraints from the original observations
            for (const auto& [poseNodeIdx, ldmkRefIdx] : as) {
                int pointSetSize = activeObsWindow[poseNodeIdx].cols() >> 1;
                g.addPLEdge(poseNodeIdx, newId, activeObsWindow[poseNodeIdx].col(pointSetSize + ldmkRefIdx));
            }
        }

        if (isRealtimeVis && g.geoHier->landmarks.cols() < 1000) {
            // Publish graph state before optimization
            std::stringstream graphStream;

            int tmpIdx = 0; // Poses
            for (const auto& pn : g.poseNodeList) graphStream << (tmpIdx++!=0 ? ":": "") << pn.pose(0) << " " << pn.pose(1) << " " << pn.pose(2);

            graphStream << "|";
            tmpIdx = 0;     // Landmark Positions
            for (const PtLoc& ldmk : g.geoHier->landmarks.colwise()) graphStream << (tmpIdx++!=0 ? ":": "") << ldmk(0) << " " << ldmk(1);

            graphStream << "|";
            tmpIdx = 0;     // References to existing landmarks seen in this frame
            for (const int& id : existingLdmkIds) graphStream << (tmpIdx++!=0 ? " ": "") << id;

            graphStream << "|";
            tmpIdx = 0;     // References to landmarks newly seen in this frame
            for (const int& id : newLdmkIds) graphStream << (tmpIdx++!=0 ? " ": "") << id;

            graphStream << "|" << (g.globalErrors.size() > 0 ? g.globalErrors.back() : -1); // Previous error

            // Construct graph message and publish
            std_msgs::String graphMsg;
            graphMsg.data = graphStream.str();
            graphPub.publish(graphMsg);
        }

        // Optimize the graph once
        if (isConsoleDebug) std::cout << "Beginning graph optimization..." << std::endl;
        bool isGood = g.optimizeGraph(isConsoleDebug);  // TODO what if I optimized twice?
        if (!isGood) abort();
        if (isConsoleDebug) std::cout << "   Done!" << std::endl;

        if (isConsoleDebug) std::cout << "Now, we think the robot's pose is " << g.poseNodeList.back().pose.transpose() << std::endl;

        // TODO add verbose debug option
        // if (isConsoleDebug) std::cout << "Optimized tree positions:\n" << g.geoHier->landmarks.transpose() << std::endl;



        // %%%%%%%%%%%%%%%%%
        // Global Map Upkeep    (TODO improve geometric hierarchy update)
        // %%%%%%%%%%%%%%%%%

        // Overwrite or amend the geometric hierarchy depending on whether new landarks were found
        if (newlyEstablishedLandmarkObservations.size() > 0) {
            if (isConsoleDebug) std::cout << "Recomputing hierarchy." << std::endl;
            g.geoHier->computeHierarchy();
        } else {
            if (isConsoleDebug) std::cout << "Recomputing edge lengths." << std::endl;
            g.geoHier->recomputeEdgeLengthsAndDescriptors();
        }

        if (activeObsWindow.size() == associationWindowSize) {
            // Erase every unresolved tree position from the oldest observation in our window, skipping any that already became landmarks
            int frameID = activeObsWindow.begin()->first;
            for (int idx = 0; idx < activeObsWindow.begin()->second.cols() >> 1; ++idx) {
                PointRef pf = {frameID, idx};
                if (unresolvedLandmarks.find(pf) != unresolvedLandmarks.end())
                    deleteTreeAssoc(unresolvedLandmarks, pf);
            }
            activeObsWindow.erase(activeObsWindow.begin()); // Remove the oldest observation from our active window
        }

        
        // %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        // Geometric Hierarchy Incremental Update
        // %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        
        // NOTE g.geoHier polygons will be handled below
        //      If we assume that the landmark nodes hold references to the points in the geometric hierarchy,
        //      then maybe updating edgeLengths and descriptor will be simpler after? 

        // TODO Spooky aftermath update of geometric hierarchy 
        // if (hasNewLandmarks) { // these points SHOULD be on the outer boundary of the global map
        //     // TODO need to store references to these separately

        //     // traverse up the local tree for these points, find which triangles and polygons they were part of
        //     // obtain set of points from all polygons/triangles identified
        //     // -----------------
        //     // Compute the geometric hierarchy for those points
        //     // Magically slap that onto the existing hierarchy, bing bang boom
        //     //    Replace triangulation
        //     //    for each poly - replace it from the new hierarchy (if possible)
        //     // BE CAREFUL if polygons have merged/broken between the hierarchies
        //     // === OR ===
        //     // Not sure lol, figure something out that has more specific details than the option above
        //     // -----------------
        // }
        
        // TODO in addition, adjust all point positions within the hierarchy, hopefully globObs.H->points will store pointers to globObs.landmarks 
        // decide whether polygon "edgeLengths" and "descriptor" should be updated too  

    }

    // %%%%%%%%%%%%%%%%%%%%%%%
    // Post-Computation Output
    // %%%%%%%%%%%%%%%%%%%%%%%

    if (isConsoleDebug) std::cout << "-------------------------------------------------------------------" << std::endl;

    // Save global map state to files (for debugging)
    if (isLogging) {
        writeHierarchyFiles(*g.geoHier, outputPath+"/global", std::to_string(g.poseNodeList.size())+".txt");
        g.writeGraphToFiles(outputPath+"/global", std::to_string(g.poseNodeList.size())+".txt");
    }

    // Serialize geometric hierarchy for real-time visualization
    if (isRealtimeVis && g.geoHier->landmarks.cols() < 1000) {
        std::stringstream polyStream, triStream, ptStream, hierStream;
        hierStream << g.poseNodeList.size() << "!";
        
        // Iterate over the indices of the Polygons in the hierarchy
        int j = 0, k = 0;
        for (auto pIdx : g.geoHier->hier->getChildrenIds(0)) {
            polyStream << (j++!=0 ? "|": "");
            for (int i = 0; i < g.geoHier->hier->getPolygon(pIdx).landmarkRefs.size(); ++i) {
                auto myPoint = g.geoHier->landmarks.col(g.geoHier->hier->getPolygon(pIdx).landmarkRefs(i));
                polyStream << (i!=0 ? ":": "") << myPoint[0] << " " << myPoint[1];
            }
            
            // Iterate over the indices of the Triangles that compose this Polygon
            for (auto tIdx : g.geoHier->hier->getChildrenIds(pIdx)) {
                triStream << (k++!=0 ? "|": "");
                for (int i = 0; i < g.geoHier->hier->getPolygon(tIdx).landmarkRefs.size(); ++i) {
                    auto myPoint = g.geoHier->landmarks.col(g.geoHier->hier->getPolygon(tIdx).landmarkRefs(i));
                    triStream << (i!=0 ? ":": "") << myPoint[0] << " " << myPoint[1];
                }
            }
        }

        j = 0;
        for (const PtLoc& ldmk : g.geoHier->landmarks.colwise()) {
            ptStream << (j++ != 0 ? ":": "") << ldmk(0) << " " << ldmk(1);
        }

        // Construct message from string streams and publish
        std_msgs::String hierMsg;
        hierStream << polyStream.str() << "$" << triStream.str() << "$" << ptStream.str();
        hierMsg.data = hierStream.str();
        hierPub.publish(hierMsg);
    }

}


// TODO Validate matches in global are within twice the observation radius of the robot

// TODO Store rejected keyframes for a little bit
// Try matching them together to build additional maps
// occasionally try merging with main graph


int main(int argc, char **argv) {
    // Initialize Node and read in private parameters
    ros::init(argc, argv, "graph_builder");
    ros::NodeHandle n("~");

    // I/O parameters
    std::string absolutePackagePath = ros::package::getPath("urquhart");
    n.param<std::string>("/outputDirName", outputPath, "testOutput");
    outputPath = absolutePackagePath+"/output/"+outputPath;
    isLogging = n.param("/logging", false);
    isConsoleDebug = n.param("consoleDebug", true);
    isRealtimeVis = n.param("realtimeVis", false);

    // Hierarchy matching parameters
    polyMatchThresh = n.param("polyMatchThreshStart", 5.0);
    polyMatchThreshStep = n.param("polyMatchThreshStep", 1.0);
    numSideBoundsForMatch = n.param("numSideBoundsForMatch", 3);
    reqMatchedPolygonRatio = n.param("reqMatchedPolygonRatio", 0.5); // 0.0 disables this feature
    
    // Data association filtering parameters
    ransacMaxIter = n.param("ransacMaxIter", 20);
    ransacMatchPrereq = n.param("ransacMatchPrereq", 16);               // landmark matches via geometric hierarchy before RANSAC
    ransacMatchSampleSize = n.param("ransacMatchSampleSize", 4);        // number of matches to use when estimating TF
    ransacValidAssocThresh = n.param("ransacValidAssocThresh", 1.0);    // lower boundary (meters)
    ransacValidAssocThresh *= ransacValidAssocThresh;                   // <-- converting to squared distance
    ransacAssocNetThresh = n.param("ransacAssocNetThresh", 1.5);        // upper boundary (meters)
    ransacAssocNetThresh *= ransacAssocNetThresh;                       // <-- converting to squared distance
    ransacMatchRatio = n.param("ransacMatchRatio", 1.0);                // percentage [0-1]
    maxSensorRange = n.param("maxSensorRange", 40) * 1.1;

    // Association Network
    minAssocForNewLdmk = n.param("minAssocForNewLdmk", 5);              // # associations to create a new landmark
    associationWindowSize = n.param("associationWindowSize", 8);        // # sequential observations in a window
    ptAssocThresh = n.param("ptAssocThresh", 1.0);                      // meters (distance for association in network)

    // Initialize with map (if desired)
    n.param<std::string>("startingMap", startingMap, "");
    if (!startingMap.empty()) {
        startingMap = absolutePackagePath+"/maps/"+startingMap;
        if (std::filesystem::exists(startingMap)) {
            // TODO hook this up with the other forest read method to include radii in landmarks
            std::cout << "Initializing global map with data from '" << startingMap << "'" << std::endl;
            std::vector<PtLoc> intake;
            double xPosition, yPosition;
            std::string line;

            // Read tree positions from file
            std::ifstream infile(startingMap);
            while (std::getline(infile, line)) {
                std::istringstream iss(line);
                if (iss >> xPosition >> yPosition) intake.push_back(PtLoc{xPosition, yPosition});
            }
            infile.close();
            
            // Add landmarks to graph
            Points landmarks(2, intake.size());
            for (int i = 0; i < intake.size(); ++i) {
                landmarks.col(i) = intake[i];
                g.addLandmarkNode(i);    // maybe TODO need to only initialize nodes for landmarks the robot sees? 
                // g.addLandmarkNode(i, intake[i]); 
            }
            // Save global geometric hierarchy
            if (intake.size() > 0) {
                urquhart::Observation globalMap(landmarks);
                g.geoHier = std::make_shared<urquhart::Observation>(std::move(globalMap));

            } else std::cout << "'" << startingMap << "' did not contain a valid map, ignoring..." << std::endl;
        } else std::cout << "Could not find map at '" << startingMap << "', ignoring..." << std::endl;
    }

    // Initialize Pubs/Subs
    ros::Subscriber sub = n.subscribe("/keyframe_maker/keyframe", 10, constructGraph);
    if (isRealtimeVis) {
        hierPub = n.advertise<std_msgs::String>("hierarchy", 10);
        graphPub = n.advertise<std_msgs::String>("graph", 10);
    }
    ros::spin();

    return 0;
}