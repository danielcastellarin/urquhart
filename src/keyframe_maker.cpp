#include <ros/ros.h>
#include <matching.hpp>
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
#include <pcl/common/transforms.h>
#include <pcl/common/centroid.h>
#include <pcl/segmentation/extract_clusters.h>
#include <sensor_msgs/PointCloud2.h>



/*
    vvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvv
    For every n individual frames, construct a keyframe K (using the following customized local bundle adjustment steps) (NOTE: n should be chosen based on the robot's speed and observation range, such that sequential keyframes exhibit partial overlap for the observed landmarks)
        For each individual frame (a set of 2D landmarks given as input), construct the local geometric hierarchy (triangulation, polygons, etc.)
            Perform data association with the previous frame, ideally making enough links between the landmarks in each frame
            If a certain proportion of landmarks per frame do not have associations, perform association with preceeding frames until that is fixed
        Once enough associations have been made for the current individual frame, perform RANSAC and estimate the current position of the robot
        After all n frames have been processed, approximate the true positions of each landmark w.r.t. the fixed reference point (the location of the robot in either the first or last individual frame)
        At this point, keyframe K should consist of a robot pose, a set of 2D landmark positions, and a newly constructed geometric hierarchy for the finalized positions of the landmarks (throw away everything else)
    ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

    Using the geometric hierarchy from keyframe K, perform point association with the global geometric hierarchy
        Maybe use the current odometry estimate from the global map to restrict the number of global polygons that would be eligible for matching with the local keyframe polygons
        If not enough matches are made, then the search space could incrementally expand to include more polygons. if this is too complicated, then we could just query all the global polygons and call it a day
    
    Once enough associations have been made, use RANSAC to estimate the current position of the robot with respect to the global reference frame
    
    Create graph nodes and edges:
        A new node Xi for the new position of the robot with respect to the global reference frame and other nodes for any newly observed landmarks from the most recent keyframe
        A new edge between Xi and Xi-1 (representing robot odometry)
        A new edge between Xi and the nodes corresponding to each landmark observed in this keyframe (representing the spatial relationship between the robot and the observed landmarks at the time of this keyframe)
    
    Using the updated graph structure, run back-end optimization to provide better robot pose and landmark location estimates
    
    Perform the following customized global bundle adjustment steps to ensure that I am making the minimum required changes to the global map to keep the system accurate and stable over time
        TODO: somehow use the intermediary outputs from the back-end optimization process to measure how much the landmark positions have changed?
        If enough landmarks have been noticeably moved and/or have added to the global map, recompute the global geometric hierarchy
            Potentially only do complete triangulation and polygon recomputation for specific region of the global map, otherwise just adjust point locations without modifying the configuration of parent triangles or polygons (which I imagine is much more scalable)

*/

// Observations are not associated when tf is not initialized
struct ObsRecord {
    int frameId;
    urquhart::Observation obs;
    pcl::PointCloud<pcl::PointXY> cloud;
    Eigen::Matrix4f tf;
    ObsRecord(int id, PointVector pv, pcl::PointCloud<pcl::PointXY> pc) : frameId(id), obs(pv), cloud(pc) {}
    bool operator==(const ObsRecord &other) const{ return frameId == other.frameId; }
    bool operator<(const ObsRecord &other) const{ return frameId < other.frameId; }
};

pcl::PointCloud<pcl::PointXYZ>::Ptr bigPcPtr(new pcl::PointCloud<pcl::PointXYZ>());
ros::Publisher kfpub;

std::vector<ObsRecord> unassociatedObs;
std::set<ObsRecord> kfObs;

void publishKeyFrame(const pcl::PointCloud<pcl::PointXY>& kfPoints, int seqID, std::string frameName) {
    sensor_msgs::PointCloud2 pc2_msg;
    pcl::toROSMsg(kfPoints, pc2_msg);
    pc2_msg.header.frame_id = frameName;
    pc2_msg.header.stamp = ros::Time::now();
    pc2_msg.header.seq = seqID;
    kfpub.publish(pc2_msg);
}

// void publishPc(const pcl::PointCloud<pcl::PointXY>& points, int seqID, std::string frameName, ros::Publisher& pub) {
//     sensor_msgs::PointCloud2 pc2_msg;
//     pcl::toROSMsg(points, pc2_msg);
//     pc2_msg.header.frame_id = frameName;
//     pc2_msg.header.stamp = ros::Time::now();
//     pc2_msg.header.seq = seqID;
//     pub.publish(pc2_msg);
// }

pcl::PointXY pclConvert(vecPtT p) {
    pcl::PointXY myPoint;
    myPoint.x = p[0]; myPoint.y = p[1];
    return myPoint;
}
pcl::PointXY pclConvert(Eigen::Vector4f p) {
    pcl::PointXY myPoint;
    myPoint.x = p[0]; myPoint.y = p[1];
    return myPoint;
}

// Substituted for original for type compatibility with my code
void myPolygonMatching(
    const urquhart::Observation &ref, std::vector<size_t> refIds,
    const urquhart::Observation &targ, std::vector<size_t> targIds, double thresh,
    std::vector<std::pair<size_t, size_t>> &polygonMatches) {
    std::set<size_t> matched;
    for (auto rIdx : refIds) {
        size_t bestMatch = 0, bestDist = 100000;
        urquhart::Polygon rp = ref.H->get_vertex(rIdx);
        for (auto tIdx : targIds) {
            urquhart::Polygon tp = targ.H->get_vertex(tIdx);
            // if tIdx was not matched before and the difference of number of points is not larger than 5
            if (matched.find(tIdx) == matched.end() &&
                std::abs(int(rp.points.size() - tp.points.size())) <= 3)
            {
                double d = euclideanDistance(rp.descriptor, tp.descriptor);
                if (d < bestDist) {
                    bestDist = d;
                    bestMatch = tIdx;
                }
            }
        }

        if (bestDist < thresh) {
            matched.insert(bestMatch);
            polygonMatches.push_back({rIdx, bestMatch});
        }
    }
}

// first values are points from reference frame, second values are points from target frame
Eigen::Matrix4f computeRigid2DEuclidTf(std::vector<std::pair<pcl::PointXY, pcl::PointXY>> pointPairs) {
    Eigen::MatrixXf A(pointPairs.size()+pointPairs.size(), 4);
    Eigen::VectorXf b(pointPairs.size()+pointPairs.size());

    // https://math.stackexchange.com/questions/77462/finding-transformation-matrix-between-two-2d-coordinate-frames-pixel-plane-to
    int startRow = 0;
    for (auto& [refP, targP] : pointPairs) {
        A.row(startRow)   = (Eigen::Vector4f() << refP.x, -refP.y, 1, 0).finished().transpose();
        A.row(startRow+1) = (Eigen::Vector4f() << refP.y,  refP.x, 0, 1).finished().transpose();
        b[startRow]   = targP.x;
        b[startRow+1] = targP.y;
        startRow += 2;
    }

    // https://www.cs.cmu.edu/~16385/s17/Slides/10.1_2D_Alignment__LLS.pdf <-- slide 24
    // x = (A^T A)^-1 A^T b
    Eigen::Vector4f x = (A.transpose() * A).inverse() * A.transpose() * b;

    // Return the 4x4 matrix to transform frames: reference --> target
    Eigen::Matrix4f tf;
    tf << x[0], -x[1], 0, x[2],
          x[1],  x[0], 0, x[3],
             0,     0, 1,    0,
             0,     0, 0,    1;
    return tf;
}

std::vector<std::pair<pcl::PointXY, pcl::PointXY>> matchObs(const urquhart::Observation &ref, const urquhart::Observation &targ, double polyMatchThresh, double validPointMatchThresh) {
    std::vector<std::pair<size_t, size_t>> polygonMatches, triangleMatches;
    // std::vector<std::pair<vecPtT, vecPtT>> pointMatches;
    std::vector<std::pair<pcl::PointXY, pcl::PointXY>> vertexMatches;
    std::set<size_t> uniqueMatches;

    // Polygon Matching (Level 2)
    // std::vector<size_t> refIds = ref.H->get_children(0), targIds = targ.H->get_children(0);
    // matching::polygonMatching(ref, ref.H->get_children(0), targ, targ.H->get_children(0), thresh, polygonMatches);
    myPolygonMatching(ref, ref.H->get_children(0), targ, targ.H->get_children(0), polyMatchThresh, polygonMatches);

    // Triangle Matching (Level 1)
    for (auto pMatch : polygonMatches) {
        // refIds = ref.H->get_children(pMatch.first), targIds = targ.H->get_children(pMatch.second);
        // TODO: ADD CHECK IF % OF TRIANGLES THAT MACTHED IS LARGER THAN 1/2
        // matching::polygonMatching(ref, ref.H->get_children(pMatch.first), targ, targ.H->get_children(pMatch.second), thresh, triangleMatches);
        myPolygonMatching(ref, ref.H->get_children(pMatch.first), targ, targ.H->get_children(pMatch.second), polyMatchThresh, triangleMatches);
    }

    // Vertex Matching (Level 0)
    for (auto tMatch : triangleMatches) {   // FIXME? make the loop explicitly over constant references?
        urquhart::Polygon refTriangle = ref.H->get_vertex(tMatch.first), targTriangle = targ.H->get_vertex(tMatch.second);
        std::vector<size_t> chi = {0, 1, 2}, bestPermutation;

        // TODO change the edgeLengths to do squared distance instead of euclidean distance (unnecessary square root)

        // Permute the edges to find the best match between the triangles
        double bestDist = 1000000;
        do {
            double d = euclideanDistance(refTriangle.edgeLengths, std::vector<double>{targTriangle.edgeLengths[chi[0]], targTriangle.edgeLengths[chi[1]], targTriangle.edgeLengths[chi[2]]});
            if (d < bestDist) {
                bestDist = d;
                bestPermutation = chi;
            }
        } while (std::next_permutation(chi.begin(), chi.end()));

        for (size_t i = 0; i < 3; ++i) {
            int refIdx = (i+2)%3, targIdx = (bestPermutation[i]+2)%3;
            size_t uid = cantorPairing(refTriangle.edges[refIdx].first, targTriangle.edges[targIdx].first);
            if (uniqueMatches.find(uid) == uniqueMatches.end()) {
                vertexMatches.push_back({pclConvert(refTriangle.points[refIdx]), pclConvert(targTriangle.points[targIdx])});
                uniqueMatches.insert(uid);
            }
        }
    }

    // Post-process: double-check matches, remove any where pairs are not certain distance from each other
    for (auto iter = vertexMatches.begin(); iter != vertexMatches.end(); ++iter) {
        if (std::abs(iter->first.x - iter->second.x) > validPointMatchThresh || std::abs(iter->first.y - iter->second.y) > validPointMatchThresh)
            vertexMatches.erase(iter);
    }
    // arrogance check: if matched points across differential observations are not very close, then the match is probably wrong 

    return vertexMatches;
}

void tfCloud(Eigen::Matrix4f existingTfToBase, Eigen::Matrix4f refToTargTf, ObsRecord& obsRec) {
    Eigen::Matrix4f fullTfToBase = refToTargTf * existingTfToBase;
    // std::cout << "Frame " << obsRec.frameId << " TF to base:\n" << fullTfToBase << std::endl;

    // Temporarily convert the pointcloud to 3D to transform into base frame 
    pcl::PointCloud<pcl::PointXYZ> nonAssociatedCloud, newlyAssociatedCloud;
    pcl::copyPointCloud(obsRec.cloud, nonAssociatedCloud);
    pcl::transformPointCloud(nonAssociatedCloud, newlyAssociatedCloud, fullTfToBase);

    // Add transformed points to keyframe pointcloud to baseframe and store tf in the frame's observation record
    *bigPcPtr += newlyAssociatedCloud;
    obsRec.tf = fullTfToBase;
    kfObs.insert(obsRec);
}

void parse2DPC(const sensor_msgs::PointCloud2ConstPtr& cloudMsg) {
    // Take the points from the PointCloud2 message
    pcl::PointCloud<pcl::PointXY> localCloud;
    pcl::fromROSMsg(*cloudMsg, localCloud);

    // Construct a PointVector for the given tree positions
    PointVector vectorOfTrees;
    for (const auto& p : localCloud) vectorOfTrees.push_back(std::vector<double>{p.x, p.y});
    ObsRecord myObs(cloudMsg->header.seq, vectorOfTrees, localCloud);

    // Try to match the current frame with any that have been associated with a base frame
    if (!kfObs.empty()) {
        std::vector<std::pair<pcl::PointXY, pcl::PointXY>> pointMatches;
        std::cout << "Matching frame " << myObs.frameId << " with associated clouds." << std::endl;
        
        // Match (current frame --> associated frame) until match found
        auto revKfIter = kfObs.rbegin();
        do {
            pointMatches = matchObs(myObs.obs, revKfIter->obs, 5, 1.5);
        } while (pointMatches.size() < 2 && ++revKfIter != kfObs.rend());

        // Store observation data
        if (pointMatches.size() < 2) { unassociatedObs.push_back(myObs); std::cout << "No matches found." << std::endl;
        } else {
            // Estimate tf from reference to target frame (assuming we have gotten rid of outliers already)
            std::cout << "Found match with frame " << revKfIter->frameId << ", associating..." << std::endl;
            tfCloud(revKfIter->tf, computeRigid2DEuclidTf(pointMatches), myObs);

            // Try to match all unassociated observations with the current observation
            auto obsIter = unassociatedObs.begin();
            while (obsIter != unassociatedObs.end()) {
                pointMatches = matchObs(obsIter->obs, myObs.obs, 5, 1.5);
                if (pointMatches.size() >= 2) {
                    std::cout << "Also matched with frame " << obsIter->frameId << std::endl;
                    tfCloud(myObs.tf, computeRigid2DEuclidTf(pointMatches), *obsIter);
                    obsIter = unassociatedObs.erase(obsIter);
                }
                else ++obsIter;
            }
        }

    } else if (!unassociatedObs.empty()) {  // Try to match the current frame with any unassociated frames
        std::vector<std::pair<pcl::PointXY, pcl::PointXY>> pointMatches;
        std::cout << "Trying to find base frame for frame " << myObs.frameId << std::endl;

        // Match (current frame --> latest unassociated frame) until match found
        auto revIter = unassociatedObs.rbegin();
        do {
            pointMatches = matchObs(myObs.obs, revIter->obs, 5, 1.5);
        } while (pointMatches.size() < 2 && ++revIter != unassociatedObs.rend());

        // Store observation data
        if (pointMatches.size() < 2) { unassociatedObs.push_back(myObs); std::cout << "None found." << std::endl;
        } else {
            // Assign the target frame of this match as the base of the keyframe
            revIter->tf = Eigen::Matrix4f::Identity();
            pcl::copyPointCloud(revIter->cloud, *bigPcPtr);
            kfObs.insert(*revIter);

            // Estimate tf from reference to target frame (assuming we have gotten rid of outliers already)
            std::cout << "Base frame found at frame " << revIter->frameId << ", adding frame " << myObs.frameId << " behind it." << std::endl;
            tfCloud(revIter->tf, computeRigid2DEuclidTf(pointMatches), myObs);
            
            // Remove the base frame from the unassociated list and advance the iterator
            std::advance(revIter, 1);
            unassociatedObs.erase(revIter.base());

            // For every remaining unassociated observation, try to match with the current observation
            while (revIter != unassociatedObs.rend()) {
                pointMatches = matchObs(revIter->obs, myObs.obs, 5, 1.5);
                if (pointMatches.size() >= 2) {
                    std::cout << "Including frame " << revIter->frameId << " in the association." << std::endl;
                    tfCloud(myObs.tf, computeRigid2DEuclidTf(pointMatches), *revIter);
                    unassociatedObs.erase(std::next(revIter).base()); // iterator should not move
                }
                else ++revIter;
            }
        }
    } else {
        // If this is the first observation, add it to the unassociated ones
        unassociatedObs.push_back(myObs);
        std::cout << "Initializing unassociated observations at frame " << myObs.frameId << std::endl;
    }

    // std::cout << "Unassociated: ";
    // for (auto g : unassociatedObs) {
    //     std::cout << g.frameId << ", ";
    // }
    // std::cout << "\nKeyframe: ";
    // for (auto g : kfObs) {
    //     std::cout << g.frameId << ", ";
    // }
    // std::cout << std::endl;


    // n = max #associated frames before sending, m = #unassociated frames before sending any associated
    int n = 5, m = 3; // TODO get this value from a param
    // If number of associated frames exceeds limit or there have been too many unassociated frames since one was associated: send to backend
    if (kfObs.size() >= n || (!kfObs.empty() && kfObs.rbegin()->frameId + m <= cloudMsg->header.seq)) {
        // At this point, all pc in kfObs should be in the same reference frame
        std::cout << "Keyframe with IDs ";
        for (auto& obs : kfObs) std::cout << obs.frameId << "(" << obs.cloud.size() << "), ";
        std::cout << " sending..." << std::endl;

        // TODO talk to Bailey about efficiently deriving the cluster centers out in either nlogn or linear time
        // (maybe this greedy solution is good enough)
        // Combine them --> do standard clustering/point association
        pcl::EuclideanClusterExtraction<pcl::PointXYZ> ece;
        ece.setInputCloud(bigPcPtr);
        ece.setClusterTolerance(0.1); // Set the spatial cluster tolerance (in meters)
        ece.setMinClusterSize(1);
        ece.setMaxClusterSize(n);
        std::vector<pcl::PointIndices> cluster_indices;
        ece.extract(cluster_indices);

        // Put centroids of each cluster as points of cloud to send to backend 
        pcl::PointCloud<pcl::PointXY> outputCloud;
        Eigen::Vector4f myCentroidOutput;
        // Eigen::Matrix3f covarMat;
        for (auto& indices : cluster_indices) {
            // std::cout << indices << std::endl;
            pcl::compute3DCentroid(*bigPcPtr, indices, myCentroidOutput);
            outputCloud.push_back(pclConvert(myCentroidOutput));
            // pcl::computeCovarianceMatrix(*bigPcPtr, indices, myCentroidOutput, covarMat);
            // std::cout << myCentroidOutput << std::endl << covarMat << std::endl;
        }
        
        // TODO should timestamp of message be that of the base frame or whatever the current time is?
        // TODO frameId be fresh for keyframes?
        publishKeyFrame(outputCloud, kfObs.begin()->frameId, "sensor_frame");
        kfObs.clear();
        bigPcPtr->clear();  // TODO might be overkill because it gets overwritten when base frame re-inits anyway
    }

    // If an unassociated frame becomes too old before it can be associated, remove it
    if (!unassociatedObs.empty() && unassociatedObs.front().frameId + m <= cloudMsg->header.seq) {
        std::cout << "Deleting frame " << unassociatedObs.front().frameId << " from unassociated observations." << std::endl;
        unassociatedObs.erase(unassociatedObs.begin());
    }
    std::cout << "===============================================================" << std::endl;
    

    // if needing to debug...
    // I'm guessing I could estimate whether a pc was transformed correctly by checking pc similarity with the other pcs
        // I would imagine that the similarity value would be significantly bigger
        // https://stackoverflow.com/questions/55913968/metric-to-compare-two-point-clouds-similarity
}


// TODO implement keyframe width parameter (default should be 5)
int main(int argc, char **argv) {
    // Initialize Node and read in private parameters
    ros::init(argc, argv, "keyframe_maker");
    ros::NodeHandle n("~");
    // SimConfig cfg(n);
    // cfg.outputConfig(std::cout);
    // ros::Rate pub_rate(cfg.pubRate);    // 10Hz by default

    ros::Subscriber sub = n.subscribe("/sim_path/local_points", 10, parse2DPC);
    kfpub = n.advertise<sensor_msgs::PointCloud2>("keyframe", 10);

    ros::spin();


    return 0;
}