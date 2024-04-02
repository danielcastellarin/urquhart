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

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <sensor_msgs/PointCloud2.h>

#include <ros/ros.h>
#include <ros/package.h>


void writeHierarchyFiles(const urquhart::Observation& trees, std::string filePath, std::string fileName) {
    std::ofstream plyOut(filePath+"/p/"+fileName), triOut(filePath+"/t/"+fileName), 
                    hieOut(filePath+"/h/"+fileName), dscOut(filePath+"/d/"+fileName);
    
    logging::writeHierarchyToFile(trees, plyOut, triOut, hieOut, dscOut);
    
    hieOut.close();
    plyOut.close();
    triOut.close();
    dscOut.close();
}


Eigen::Matrix4d computeRigid2DEuclidTf(const Points& localLandmarks, const Points& globalLandmarks, const std::vector<std::pair<Eigen::Index, Eigen::Index>>& pointMatchRefs) {
    Eigen::MatrixXd A(pointMatchRefs.size()+pointMatchRefs.size(), 4);
    Eigen::VectorXd b(pointMatchRefs.size()+pointMatchRefs.size());
    // TODO if not all matches valid, count how many of them are, then do conservativeResize at the end to crop out extra space

    // https://math.stackexchange.com/questions/77462/finding-transformation-matrix-between-two-2d-coordinate-frames-pixel-plane-to
    int startRow = 0;
    for (const auto& [lIdx, gIdx] : pointMatchRefs) {
        // TODO perform more elegant block assignments here
        A.row(startRow)   = (Eigen::Vector4d() << localLandmarks(0, lIdx), -localLandmarks(1, lIdx), 1, 0).finished().transpose();
        A.row(startRow+1) = (Eigen::Vector4d() << localLandmarks(1, lIdx),  localLandmarks(0, lIdx), 0, 1).finished().transpose();
        b[startRow]   = globalLandmarks(0, gIdx);
        b[startRow+1] = globalLandmarks(1, gIdx);
        startRow += 2;
    }

    // https://www.cs.cmu.edu/~16385/s17/Slides/10.1_2D_Alignment__LLS.pdf <-- slide 24
    // x = (A^T A)^-1 A^T b
    Eigen::Vector4d x = (A.transpose() * A).inverse() * A.transpose() * b;

    // Return the 4x4 matrix to transform frames: reference --> target
    return Eigen::Matrix4d{
        {x[0], -x[1], 0, x[2]},
        {x[1],  x[0], 0, x[3]},
        {   0,     0, 1,    0},
        {   0,     0, 0,    1}
    };
}


/*
    For every n individual frames, construct a keyframe K (using the following customized local bundle adjustment steps) (NOTE: n should be chosen based on the robot's speed and observation range, such that sequential keyframes exhibit partial overlap for the observed landmarks)
        For each individual frame (a set of 2D landmarks given as input), construct the local geometric hierarchy (triangulation, polygons, etc.)
            Perform data association with the previous frame, ideally making enough links between the landmarks in each frame
            If a certain proportion of landmarks per frame do not have associations, perform association with preceeding frames until that is fixed
        Once enough associations have been made for the current individual frame, perform RANSAC and estimate the current position of the robot
        After all n frames have been processed, approximate the true positions of each landmark w.r.t. the fixed reference point (the location of the robot in either the first or last individual frame)
        At this point, keyframe K should consist of a robot pose, a set of 2D landmark positions, and a newly constructed geometric hierarchy for the finalized positions of the landmarks (throw away everything else)
    
    vvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvv
    Using the geometric hierarchy from keyframe K, perform point association with the global geometric hierarchy
        Maybe use the current odometry estimate from the global map to restrict the number of global polygons that would be eligible for matching with the local keyframe polygons
        If not enough matches are made, then the search space could incrementally expand to include more polygons. if this is too complicated, then we could just query all the global polygons and call it a day
    
    Once enough associations have been made, use RANSAC to estimate the current position of the robot with respect to the global reference frame
    
    Create graph nodes and edges:
        A new node Xi for the new position of the robot with respect to the global reference frame and other nodes for any newly observed landmarks from the most recent keyframe
        A new edge between Xi and Xi-1 (representing robot odometry)
        A new edge between Xi and the nodes corresponding to each landmark observed in this keyframe (representing the spatial relationship between the robot and the observed landmarks at the time of this keyframe)
    ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

    Using the updated graph structure, run back-end optimization to provide better robot pose and landmark location estimates
    
    Perform the following customized global bundle adjustment steps to ensure that I am making the minimum required changes to the global map to keep the system accurate and stable over time
        TODO: somehow use the intermediary outputs from the back-end optimization process to measure how much the landmark positions have changed?
        If enough landmarks have been noticeably moved and/or have added to the global map, recompute the global geometric hierarchy
            Potentially only do complete triangulation and polygon recomputation for specific region of the global map, otherwise just adjust point locations without modifying the configuration of parent triangles or polygons (which I imagine is much more scalable)

*/


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
    PPEdge() {}
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
    PLEdge() {}
    PLEdge(int source, int destination, Eigen::Vector2d d) : src(source), dst(destination), distance(d) {
        info << 100,  0,
                0,  100;
    }
};

// USED ONLY FOR GRAPHSLAM MATH
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



struct SLAMGraph {
    std::shared_ptr<urquhart::Observation> geoHier = NULL;
    Eigen::VectorXd stateVector; // stored as eigen vector for faster computations??

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
    
    void addLandmarkNode(int idxInHier, PtLoc& position) {
        // Precondition: landmark data must already be in geometric hierarchy at the given index 
        
        // Define landmark node
        landmarkNodeList.push_back(LandmarkNode(idxInHier, stateVector.size()));

        // Append (x,y) to the end of state vector
        stateVector.conservativeResize(stateVector.size()+2);
        stateVector.tail(2) = position;
    }
    void addPLEdge(Eigen::Index globalLandmarkIdx, PtLoc& landmarkLocalPosition) {
        // Precondition: landmark data must already be in geometric hierarchy at the given index 
        // Precondition: landmark node must already be in the graph's list of nodes
        
        // May be called with landmark nodes that existed before current keyframe or nodes that were just created
        // landmarkLocalPosition is taken directly from the landmark's position relative to the robot in the current keyframe
        plEdges.push_back(PLEdge(poseNodeList.size()-1, globalLandmarkIdx, landmarkLocalPosition));
    }

    void computeGlobalError() {
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

        globalErrors.push_back(gErr);
    }

    void optimizeGraph() {
        
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


        // Update the state vector; according to the result of the system of equations
        stateVector -= H.llt().solve(b);

        // Copy data from the state vector into graph nodes (for easier access)
        for (auto& node : poseNodeList) node.pose = stateVector.segment(node.stateVectorIdx, 3);
        for (auto& node : landmarkNodeList) geoHier->landmarks.col(node.globalReprIdx) = stateVector.segment(node.stateVectorIdx, 2);
    }

    void writeGraphToFiles(std::string filePath, std::string fileName) {
        std::ofstream nodesOut(filePath+"/graph_nodes/"+fileName), edgesOut(filePath+"/graph_edges/"+fileName), errOut(filePath+"/!gError.txt");
        
        // Output Nodes
        for (const auto& pn : poseNodeList) nodesOut << "P" << pn.id << " " << pn.pose.transpose() << std::endl;
        for (const auto& ln : landmarkNodeList) nodesOut << "L" << ln.globalReprIdx << " " << geoHier->landmarks.col(ln.globalReprIdx).transpose() << std::endl;

        // Output Edges
        for (const auto& ppe : ppEdges) edgesOut << "P" << ppe.src << " P" << ppe.dst << " " << ppe.distance.transpose() << std::endl;
        for (const auto& ple : plEdges) edgesOut << "P" << ple.src << " L" << ple.dst << " " << ple.distance.transpose() << std::endl;

        // Save global error to single file (overwriting)
        computeGlobalError();
        for (const auto& ge : globalErrors) errOut << ge << std::endl;

        nodesOut.close();
        edgesOut.close();
        errOut.close();
    }

};



// // calculates squared error from two point mapping; assumes rotation around Origin.
// inline double sqErr_3Dof(PtLoc p1, PtLoc p2, double cos_alpha, double sin_alpha, PtLoc T) {

//     double x2_est = T(0) + cos_alpha * p1(0) - sin_alpha * p1(1);
//     double y2_est = T(1) + sin_alpha * p1(0) + cos_alpha * p1(1);
//     PtLoc p2_est({x2_est, y2_est});

//     return (p2_est - p2).array().square().sum(); // squared distance
// }

// // calculate RMSE for point-to-point metrics
// float RMSE_3Dof(const std::vector<PtLoc>& src, const std::vector<PtLoc>& dst,
//         const float* param, const bool* inliers, const PtLoc center) {

//     const bool all_inliers = (inliers==NULL); // handy when we run QUADRTATIC will all inliers
//     unsigned int n = src.size();
//     assert(n>0 && n==dst.size());

//     float ang_rad = param[0];
//     PtLoc T({param[1], param[2]});
//     float cos_alpha = cos(ang_rad);
//     float sin_alpha = sin(ang_rad);

//     double RMSE = 0.0;
//     int ninliers = 0;
//     for (unsigned int i=0; i<n; i++) {
//         if (all_inliers || inliers[i]) {
//             RMSE += sqErr_3Dof(src[i]-center, dst[i]-center, cos_alpha, sin_alpha, T);
//             ninliers++;
//         }
//     }

    
//     if (ninliers>0)
//         return sqrt(RMSE/ninliers);
//     else
//         return 100000000;
// }

// // Sets inliers and returns their count
// inline int setInliers3Dof(const std::vector<PtLoc>& src, const std::vector<PtLoc>& dst,
//         bool* inliers,
//         const float* param,
//         const float max_er,
//         const PtLoc center) {

//     float ang_rad = param[0];
//     PtLoc T({param[1], param[2]});

//     // set inliers
//     unsigned int ninliers = 0;
//     unsigned int n = src.size();
//     assert(n>0 && n==dst.size());

//     float cos_ang = cos(ang_rad);
//     float sin_ang = sin(ang_rad);
//     float max_sqErr = max_er*max_er; // comparing squared values

//     // get the number of inliers and set them 
//     for (unsigned int i=0; i<n; i++) {

//         float sqErr = sqErr_3Dof(src[i]-center, dst[i]-center, cos_ang, sin_ang, T);
//         if ( sqErr < max_sqErr) {
//             inliers[i] = 1;
//             ninliers++;
//         } else {
//             inliers[i] = 0;
//         }
//     }

//     return ninliers;
// }



SLAMGraph g;
Eigen::Vector3d mostRecentGlobalPose; // if not tracking updates to all previous robot poses
bool isDebug = false, isOutput = false;
std::string outputPath;

void constructGraph(const sensor_msgs::PointCloud2ConstPtr& cloudMsg) {
    if (isDebug) std::cout << "Received keyframe " << cloudMsg->header.seq << std::endl;

    // Take the points from the PointCloud2 message and create a geometric hierarchy from it
    pcl::PointCloud<pcl::PointXY> localCloud;
    pcl::fromROSMsg(*cloudMsg, localCloud);
    Points vectorOfTrees(2, localCloud.size());
    int idx = 0;
    for (const auto& p : localCloud) vectorOfTrees.col(idx++) = PtLoc{p.x, p.y};
    urquhart::Observation localObs(vectorOfTrees);
    
    // Save local observation data (if desired)
    if (isDebug) std::cout << "Defined local observation for keyframe " << cloudMsg->header.seq << std::endl;
    if (isOutput) {
        writeHierarchyFiles(localObs, outputPath+"/local", std::to_string(g.poseNodeList.size()+1)+".txt");
        std::ofstream ptsOut(outputPath+"/local/pts/"+std::to_string(g.poseNodeList.size()+1)+".txt");
        for (const auto& c : localObs.landmarks.colwise()) ptsOut << c(0) << " " << c(1) << std::endl;
        ptsOut.close();
    }

    // Do different things depending on whether the global map is available
    if (g.geoHier == NULL) {
        // TODO determine what gets done when the global map is unavailable (might also occur when keyframe does not match with anything on the map)
        //      maybe I could modify the matching procedure a bit to iteratively loosen the matching constraints if not enough matches are found?

        // Assign the graph's Observation to the current local one
        g.geoHier = std::make_shared<urquhart::Observation>(std::move(localObs));

        // Add robot pose (as an anchor)
        Eigen::Vector3d startPose{0,0,0};
        g.addPoseNode(startPose);

        // Add initial landmarks and constrain them to the first pose node
        for (int lIdx = 0; lIdx < localObs.landmarks.cols(); ++lIdx) {
            PtLoc position(localObs.landmarks.col(lIdx));
            g.addLandmarkNode(lIdx, position);
            g.addPLEdge(lIdx, position);
        }

        if (isDebug) {
            std::cout << "Constructed initial global geometric hierarchy " << cloudMsg->header.seq << std::endl;
            std::cout << "Initial tree positions:\n" << localObs.landmarks.transpose() << std::endl;
        }

        // Do nothing else because we need to get more observations before optimizing

    } else {

        // Attempt to match the current local observation with the global representation
        int matchingThresh = 5;
        std::vector<std::pair<Eigen::Index, Eigen::Index>> matchingPointIndices;
        do {
            matchingPointIndices = matching::hierarchyIndexMatching(localObs, *g.geoHier, matchingThresh);
            if (isDebug) std::cout << "Matched " << matchingPointIndices.size() << " points with the global frame using threshold=" << matchingThresh << std::endl;
            matchingThresh += 2;
        } while (matchingPointIndices.size() < 2);


        // LOG MATCHES
        if (isOutput) {
            std::ofstream matOut(outputPath+"/match/"+std::to_string(g.poseNodeList.size())+".txt");
            for (const auto& [lIdx, gIdx] : matchingPointIndices) {
                matOut << localObs.ldmkX(lIdx) << "," << localObs.ldmkY(lIdx) << "|";
                matOut << g.geoHier->ldmkX(gIdx) << "," << g.geoHier->ldmkY(gIdx) << std::endl;
                std::cout << std::sqrt((localObs.landmarks.col(lIdx) - g.geoHier->landmarks.col(gIdx)).array().square().sum()) << std::endl;
            }
            matOut.close();
        }
        if (isDebug) {
            for (const auto& [lIdx, gIdx] : matchingPointIndices) 
                std::cout << euclideanDistance2D(localObs.landmarks.col(lIdx), g.geoHier->landmarks.col(gIdx)) << std::endl;
        }



        // Matching speed should not be a bottleneck when querying global representation
        // If faster speed desired, only elect global polygons that are a certain distance away from the previous pose  
        // if not enough matches are made, maybe drop the frame? or expand global polygon election range


        // TODO Use RANSAC to remove outliers when estimating robot's current position with respect to the global reference frame
        // At the very least, return the inliear associations from matchingPoints
        // If not confident enough in potential match, exit early
        // Post-process: double-check matches, remove any where pairs are not certain distance from each other
        // for (auto iter = matchingPoints.begin(); iter != matchingPoints.end(); ++iter) {
        //     // TODO determine if eigen can parallelize this operation (without my goofy logic)
        //     if (std::abs(iter->first(0) - iter->second(0)) > validPointMatchThresh || std::abs(iter->first(1) - iter->second(1)) > validPointMatchThresh)
        //         matchingPoints.erase(iter);
        // }
        // arrogance check: if matched points across differential observations are not very close, then the match is probably wrong 


        // Use the inlier point pairs to find the robot's current position wrt global frame
        Eigen::Matrix4d currentGlobalRobotPoseTf(computeRigid2DEuclidTf(localObs.landmarks, g.geoHier->landmarks, matchingPointIndices));
        if (isDebug) std::cout << "Calculated tf (local->global):\n" << currentGlobalRobotPoseTf << std::endl;

        // Define a new node and edge for the robot's estimated odometry
        Eigen::Vector3d robotPoseInGlobal{currentGlobalRobotPoseTf(0,3), 
                                            currentGlobalRobotPoseTf(1,3), 
                                            atan2(currentGlobalRobotPoseTf(1,0), currentGlobalRobotPoseTf(0,0))};
        g.addPPEdge(robotPoseInGlobal);
        if (isDebug) std::cout << "New robot global pose: " << robotPoseInGlobal.transpose() << std::endl;

        // Add constraints for all local points that matched with existing landmarks
        std::unordered_set<Eigen::Index> localMatchedPointIndices, globalMatchedPointIndices;
        for (const auto& [lIdx, gIdx] : matchingPointIndices) {
            PtLoc localPosition = localObs.landmarks.col(lIdx);
            g.addPLEdge(gIdx, localPosition); // TODO for some reason, compiler not coercing slice into PtLoc?
            localMatchedPointIndices.insert(lIdx);
            globalMatchedPointIndices.insert(gIdx);
            if (isDebug) std::cout << "Matched: L=" << lIdx << " -> G=" << gIdx << std::endl;
        }

        // Try to associate other local points in the global frame  
        bool hasNewLandmarks = false;
        for (int lIdx = 0; lIdx < localObs.landmarks.cols(); ++lIdx) {
            // Only process points that were not EXPLICITLY matched with existing landmarks
            if (localMatchedPointIndices.find(lIdx) == localMatchedPointIndices.end()) {

                // Define this tree's local position as a homogeneous matrix
                Eigen::Matrix4d localPointTf {
                    {1, 0, 0, localObs.landmarks(0, lIdx)},
                    {0, 1, 0, localObs.landmarks(1, lIdx)},
                    {0, 0, 1, 0},
                    {0, 0, 0, 1},
                };
                
                // Obtain this point's observed position relative to the global frame
                PtLoc globalXY{(currentGlobalRobotPoseTf * localPointTf)(Eigen::seq(0,1), 3)};
                double nearestSqDistToPoint = 1.125;  // meters

                // Find the nearest existing landmark below the nearness threshold for this given point
                int nearestPointId = -1;
                for (int gIdx = 0; gIdx < g.geoHier->landmarks.cols(); ++gIdx) {
                    if (globalMatchedPointIndices.find(gIdx) == globalMatchedPointIndices.end()) {
                        // TODO make this more robust
                        double sqDiff = squaredDistance2D(globalXY, g.geoHier->landmarks.col(gIdx));
                        if (sqDiff < nearestSqDistToPoint) nearestSqDistToPoint = sqDiff, nearestPointId = gIdx;
                    }
                }
                // TODO maybe use the pointcloud clustering/centroid stuff here too

                // If we didn't find a point close enough to this one, make a node for it
                if (nearestPointId == -1) {
                    hasNewLandmarks = true;

                    // Add the point to the set of landmarks in the global geometric hierarcy
                    nearestPointId = g.geoHier->landmarks.cols();
                    g.geoHier->landmarks.conservativeResize(2, nearestPointId+1);
                    g.geoHier->landmarks.col(nearestPointId) = globalXY;

                    // Define a graph node for this point and add it to the statevector
                    g.addLandmarkNode(nearestPointId, globalXY);
                    if (isDebug) std::cout << "NEW LANDMARK: " << globalXY.transpose() << std::endl;
                }

                // Create a constraint for this landmark
                PtLoc localPosition = localObs.landmarks.col(lIdx);
                g.addPLEdge(nearestPointId, localPosition); // TODO for some reason, compiler not coercing slice into PtLoc?
                if (isDebug) std::cout << "Matched: L=" << lIdx << " -> G=" << nearestPointId << std::endl;
            }
        }


        // Optimize the graph once
        if (isDebug) std::cout << "Optimizing graph... ";
        g.optimizeGraph();
        if (isDebug) std::cout << "   Done!" << std::endl;

        // TODO add verbose debug option
        // if (isDebug) std::cout << "Optimized tree positions:\n" << g.geoHier->landmarks.transpose() << std::endl;

        

        // Overwrite or amend the geometric hierarchy depending on whether new landarks were found
        // if (hasNewLandmarks) g.geoHier->computeHierarchy();
        // else g.geoHier->recomputeEdgeLengths();

        if (hasNewLandmarks) {
            if (isDebug) std::cout << "Recomputing hierarchy." << std::endl;
            g.geoHier->computeHierarchy();
        } else {
            if (isDebug) std::cout << "Recomputing edge lengths." << std::endl;
            g.geoHier->recomputeEdgeLengths();
        }
        
        
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

    // Print the graph to a file (for debugging)
    if (isOutput) {
        std::cout << "Logging global data... ";
        writeHierarchyFiles(*g.geoHier, outputPath+"/global", std::to_string(g.poseNodeList.size())+".txt");
        std::cout << "   Done!" << std::endl;
        std::cout << "Saving graph data... ";
        g.writeGraphToFiles(outputPath+"/global", std::to_string(g.poseNodeList.size())+".txt");
        std::cout << "   Done!" << std::endl;
    }

}


int main(int argc, char **argv) {
    // Initialize Node and read in private parameters
    ros::init(argc, argv, "graph_builder");
    ros::NodeHandle n("~");

    std::string absolutePackagePath = ros::package::getPath("urquhart");
    isDebug = n.param("debug", true);
    isOutput = n.param("output", true);
    n.param<std::string>("outputDirName", outputPath, "gTEST");


    if (isOutput) {
        std::cout << "Creating output directory: '" << outputPath << "' ... ";
        outputPath = absolutePackagePath+"/output/"+outputPath;
        std::filesystem::remove_all(outputPath);
        std::filesystem::create_directory(outputPath);
        std::filesystem::create_directory(outputPath+"/global");
        std::filesystem::create_directory(outputPath+"/global/p");
        std::filesystem::create_directory(outputPath+"/global/d");
        std::filesystem::create_directory(outputPath+"/global/t");
        std::filesystem::create_directory(outputPath+"/global/h");
        std::filesystem::create_directory(outputPath+"/global/graph_nodes");
        std::filesystem::create_directory(outputPath+"/global/graph_edges");
        std::filesystem::create_directory(outputPath+"/local");
        std::filesystem::create_directory(outputPath+"/local/p");
        std::filesystem::create_directory(outputPath+"/local/d");
        std::filesystem::create_directory(outputPath+"/local/t");
        std::filesystem::create_directory(outputPath+"/local/h");
        std::filesystem::create_directory(outputPath+"/local/pts");
        std::filesystem::create_directory(outputPath+"/match");
        std::cout << "   Done!" << std::endl;
    }

    // ros::Rate pub_rate(cfg.pubRate);    // 10Hz by default

    
    ros::Subscriber sub = n.subscribe("/keyframe_maker/keyframe", 10, constructGraph);

    ros::spin();


    return 0;
}