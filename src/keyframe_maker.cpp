#include <matching.hpp>
#include <memory>
#include <random>
#include <unordered_map>
#include <queue>
#include <unordered_set>
#include <map>
#include <algorithm>
#include <filesystem>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl/common/centroid.h>
#include <pcl/segmentation/extract_clusters.h>
#include <sensor_msgs/PointCloud2.h>
#include <ros/ros.h>


// Observations are not associated when tf is not initialized
struct ObsRecord {
    int frameId;
    urquhart::Observation obs;
    pcl::PointCloud<pcl::PointXY> cloud;
    Eigen::Matrix4d tf;
    ObsRecord(int id, Points pv, pcl::PointCloud<pcl::PointXY> pc) : frameId(id), obs(pv), cloud(pc) {}
    bool operator==(const ObsRecord &other) const{ return frameId == other.frameId; }
    bool operator<(const ObsRecord &other) const{ return frameId < other.frameId; }
};

pcl::PointCloud<pcl::PointXYZ>::Ptr bigPcPtr(new pcl::PointCloud<pcl::PointXYZ>());
ros::Publisher kfpub;

std::vector<ObsRecord> unassociatedObs;
std::set<ObsRecord> kfObs;
bool isDebug;
int maxKeyframeWidth, numSkippedFramesBeforeSend;



void publishKeyFrame(const pcl::PointCloud<pcl::PointXY>& kfPoints, int seqID, std::string frameName) {
    sensor_msgs::PointCloud2 pc2_msg;
    pcl::toROSMsg(kfPoints, pc2_msg);
    pc2_msg.header.frame_id = frameName;
    pc2_msg.header.stamp = ros::Time::now();
    pc2_msg.header.seq = seqID;
    kfpub.publish(pc2_msg);
}

pcl::PointXY pclConvert(const PtLoc& p) {
    pcl::PointXY myPoint;
    myPoint.x = p[0]; myPoint.y = p[1];
    return myPoint;
}
pcl::PointXY pclConvert(const Eigen::Vector4d& p) {
    pcl::PointXY myPoint;
    myPoint.x = p[0]; myPoint.y = p[1];
    return myPoint;
}

// first values are points from reference frame, second values are points from target frame
Eigen::Matrix4d computeRigid2DEuclidTfFromIndices(const std::vector<std::pair<Eigen::Index, Eigen::Index>>& ldmkIndexPairs,
                                                const urquhart::Observation &ref, const urquhart::Observation &targ) {
    Eigen::MatrixXd A(ldmkIndexPairs.size()+ldmkIndexPairs.size(), 4);
    Eigen::VectorXd b(ldmkIndexPairs.size()+ldmkIndexPairs.size());

    // https://math.stackexchange.com/questions/77462/finding-transformation-matrix-between-two-2d-coordinate-frames-pixel-plane-to
    int startRow = 0;
    for (const auto& [refIdx, targIdx] : ldmkIndexPairs) {
        A.row(startRow)   = (Eigen::Vector4d() << ref.ldmkX(refIdx), -ref.ldmkY(refIdx), 1, 0).finished().transpose();
        A.row(startRow+1) = (Eigen::Vector4d() << ref.ldmkY(refIdx),  ref.ldmkX(refIdx), 0, 1).finished().transpose();
        b[startRow]   = targ.ldmkX(targIdx);
        b[startRow+1] = targ.ldmkY(targIdx);
        startRow += 2;
    }

    // https://www.cs.cmu.edu/~16385/s17/Slides/10.1_2D_Alignment__LLS.pdf <-- slide 24
    // x = (A^T A)^-1 A^T b
    Eigen::Vector4d x = (A.transpose() * A).inverse() * A.transpose() * b;

    // Return the 4x4 matrix to transform frames: reference --> target
    Eigen::Matrix4d tf;
    tf << x[0], -x[1], 0, x[2],
          x[1],  x[0], 0, x[3],
             0,     0, 1,    0,
             0,     0, 0,    1;
    return tf;
}

std::vector<std::pair<Eigen::Index, Eigen::Index>> matchObsIdx(const urquhart::Observation &ref, const urquhart::Observation &targ, double polyMatchThresh, double validPointMatchThresh) {

    // Perform traditional data association 
    std::vector<std::pair<Eigen::Index, Eigen::Index>> vertexMatches = matching::hierarchyIndexMatching(ref, targ, polyMatchThresh);

    // Post-process: double-check matches, remove any where pairs are beyond a certain distance from each other
    for (auto iter = vertexMatches.begin(); iter != vertexMatches.end(); ++iter) {
        if (std::abs(ref.ldmkX(iter->first) - targ.ldmkX(iter->second)) > validPointMatchThresh || std::abs(ref.ldmkY(iter->first) - targ.ldmkY(iter->second)) > validPointMatchThresh)
        // if (((ref.landmarks.col(iter->first) - targ.landmarks.col(iter->second)).array().abs() > validPointMatchThresh).any())
            vertexMatches.erase(iter);
    }
    // arrogance check: if matched points across differential observations are not very close, then the match is probably wrong 

    return vertexMatches;
}

void tfCloud(Eigen::Matrix4d existingTfToBase, Eigen::Matrix4d refToTargTf, ObsRecord& obsRec) {
    Eigen::Matrix4d fullTfToBase = refToTargTf * existingTfToBase;
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
    Points vectorOfTrees(2, localCloud.size());
    int idx = 0;
    for (const auto& p : localCloud) vectorOfTrees.col(idx++) = Eigen::Vector2d{p.x, p.y};
    ObsRecord myObs(cloudMsg->header.seq, vectorOfTrees, localCloud);

    // Try to match the current frame with any that have been associated with a base frame
    if (!kfObs.empty()) {
        std::vector<std::pair<Eigen::Index, Eigen::Index>> pointMatchIndices;
        if (isDebug) std::cout << "Matching frame " << myObs.frameId << " with associated clouds." << std::endl;
        
        // Match (current frame --> associated frame) until match found
        auto revKfIter = kfObs.rbegin();
        do {
            pointMatchIndices = matchObsIdx(myObs.obs, revKfIter->obs, 5, 1.5);
        } while (pointMatchIndices.size() < 2 && ++revKfIter != kfObs.rend());

        // Store observation data
        if (pointMatchIndices.size() < 2) {
            unassociatedObs.push_back(myObs);
            if (isDebug) std::cout << "No matches found." << std::endl;
        } else {
            // Estimate tf from reference to target frame (assuming we have gotten rid of outliers already)
            if (isDebug) std::cout << "Found match with frame " << revKfIter->frameId << ", associating..." << std::endl;
            tfCloud(revKfIter->tf, computeRigid2DEuclidTfFromIndices(pointMatchIndices, myObs.obs, revKfIter->obs), myObs);

            // Try to match all unassociated observations with the current observation
            auto obsIter = unassociatedObs.begin();
            while (obsIter != unassociatedObs.end()) {
                pointMatchIndices = matchObsIdx(obsIter->obs, myObs.obs, 5, 1.5);
                if (pointMatchIndices.size() >= 2) {
                    if (isDebug) std::cout << "Also matched with frame " << obsIter->frameId << std::endl;
                    tfCloud(myObs.tf, computeRigid2DEuclidTfFromIndices(pointMatchIndices, obsIter->obs, myObs.obs), *obsIter);
                    obsIter = unassociatedObs.erase(obsIter);
                }
                else ++obsIter;
            }
        }

    } else if (!unassociatedObs.empty()) {  // Try to match the current frame with any unassociated frames
        std::vector<std::pair<Eigen::Index, Eigen::Index>> pointMatchIndices;
        if (isDebug) std::cout << "Trying to find base frame for frame " << myObs.frameId << std::endl;

        // Match (current frame --> latest unassociated frame) until match found
        auto revIter = unassociatedObs.rbegin();
        do {
            pointMatchIndices = matchObsIdx(myObs.obs, revIter->obs, 5, 1.5);
        } while (pointMatchIndices.size() < 2 && ++revIter != unassociatedObs.rend());

        // Store observation data
        if (pointMatchIndices.size() < 2) {
            unassociatedObs.push_back(myObs);
            if (isDebug) std::cout << "None found." << std::endl;
        } else {
            // Assign the target frame of this match as the base of the keyframe
            revIter->tf = Eigen::Matrix4d::Identity();
            pcl::copyPointCloud(revIter->cloud, *bigPcPtr);
            kfObs.insert(*revIter);

            // Estimate tf from reference to target frame (assuming we have gotten rid of outliers already)
            if (isDebug) std::cout << "Base frame found at frame " << revIter->frameId << ", adding frame " << myObs.frameId << " behind it." << std::endl;
            tfCloud(revIter->tf, computeRigid2DEuclidTfFromIndices(pointMatchIndices, myObs.obs, revIter->obs), myObs);
            
            // Remove the base frame from the unassociated list and advance the iterator
            std::advance(revIter, 1);
            unassociatedObs.erase(revIter.base());

            // For every remaining unassociated observation, try to match with the current observation
            while (revIter != unassociatedObs.rend()) {
                pointMatchIndices = matchObsIdx(revIter->obs, myObs.obs, 5, 1.5);
                if (pointMatchIndices.size() >= 2) {
                    if (isDebug) std::cout << "Including frame " << revIter->frameId << " in the association." << std::endl;
                    tfCloud(myObs.tf, computeRigid2DEuclidTfFromIndices(pointMatchIndices, revIter->obs, myObs.obs), *revIter);
                    unassociatedObs.erase(std::next(revIter).base()); // iterator should not move
                }
                else ++revIter;
            }
        }
    } else {
        // If this is the first observation, add it to the unassociated ones
        unassociatedObs.push_back(myObs);
        if (isDebug) std::cout << "Initializing unassociated observations at frame " << myObs.frameId << std::endl;
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


    // If number of associated frames exceeds limit or there have been too many unassociated frames since one was associated: send to backend
    if (kfObs.size() >= maxKeyframeWidth || (!kfObs.empty() && kfObs.rbegin()->frameId + numSkippedFramesBeforeSend <= cloudMsg->header.seq)) {
        // At this point, all pc in kfObs should be in the same reference frame
        if (isDebug) {
            std::cout << "Keyframe with IDs ";
            for (auto& obs : kfObs) std::cout << obs.frameId << "(" << obs.cloud.size() << "), ";
            std::cout << " sending..." << std::endl;
        }

        // TODO talk to Bailey about efficiently deriving the cluster centers out in either nlogn or linear time
        // (maybe this greedy solution is good enough)
        // Combine them --> do standard clustering/point association
        pcl::EuclideanClusterExtraction<pcl::PointXYZ> ece;
        ece.setInputCloud(bigPcPtr);
        ece.setClusterTolerance(0.1); // Set the spatial cluster tolerance (in meters)
        ece.setMinClusterSize(1);
        ece.setMaxClusterSize(maxKeyframeWidth);
        std::vector<pcl::PointIndices> cluster_indices;
        ece.extract(cluster_indices);

        // Put centroids of each cluster as points of cloud to send to backend 
        pcl::PointCloud<pcl::PointXY> outputCloud;
        Eigen::Vector4d myCentroidOutput;
        for (auto& indices : cluster_indices) {
            pcl::compute3DCentroid(*bigPcPtr, indices, myCentroidOutput);
            outputCloud.push_back(pclConvert(myCentroidOutput));
        }
        
        // TODO should timestamp of message be that of the base frame or whatever the current time is?
        // TODO frameId be fresh for keyframes?
        publishKeyFrame(outputCloud, kfObs.begin()->frameId, "sensor_frame");
        kfObs.clear();
        bigPcPtr->clear();  // TODO might be overkill because it gets overwritten when base frame re-inits anyway
    }

    // If an unassociated frame becomes too old before it can be associated, remove it
    if (!unassociatedObs.empty() && unassociatedObs.front().frameId + numSkippedFramesBeforeSend <= cloudMsg->header.seq) {
        if (isDebug) std::cout << "Deleting frame " << unassociatedObs.front().frameId << " from unassociated observations." << std::endl;
        unassociatedObs.erase(unassociatedObs.begin());
    }
    if (isDebug) std::cout << "===============================================================" << std::endl;
    

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
    isDebug = n.param("debug", true);
    maxKeyframeWidth = n.param("maxKeyframeWidth", 5);
    numSkippedFramesBeforeSend = n.param("numSkippedFramesBeforeSend", 3);

    ros::Subscriber sub = n.subscribe("/sim_path/local_points", 10, parse2DPC);
    kfpub = n.advertise<sensor_msgs::PointCloud2>("keyframe", 10);

    ros::spin();


    return 0;
}