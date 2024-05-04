#include <matching.hpp>
#include <memory>
#include <random>
#include <unordered_map>
#include <queue>
#include <unordered_set>
#include <map>
#include <algorithm>
#include <filesystem>
#include <chrono>


#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl/common/centroid.h>
#include <pcl/segmentation/extract_clusters.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/String.h>
#include <ros/ros.h>
#include <ros/package.h>


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
ros::Publisher kfpub, bigCloudPub, hierPub, donePub;

std::vector<ObsRecord> unassociatedObs;
std::set<ObsRecord> kfObs;
int maxKeyframeWidth, numSkippedFramesBeforeSend, numSideBoundsForMatch, reqMatchesForFrameAssoc;
double polygonMatchThresh, reqMatchedPolygonRatio, validPointMatchThresh, clusterTolerance;
bool isDebug, isIndivFramePub, pubAllPoints, isLogging, isOffline;
std::string keyframePath, allKfPtsPath;
int numFramesSent = 0;


void publishPointCloud(ros::Publisher& pub, sensor_msgs::PointCloud2& pc2_msg, int seqID, std::string frameName) {
    pc2_msg.header.frame_id = frameName;
    pc2_msg.header.stamp = ros::Time::now();
    pc2_msg.header.seq = seqID;
    pub.publish(pc2_msg);
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

std::vector<std::pair<Eigen::Index, Eigen::Index>> matchObsIdx(const urquhart::Observation &ref, const urquhart::Observation &targ, double polyMatchThresh, double pointMatchThresh) {
    // Perform traditional data association
    // std::vector<std::pair<Eigen::Index, Eigen::Index>> finalVertexMatches, vertexMatches = matching::hierarchyIndexMatching(ref, targ, polyMatchThresh, numSideBoundsForMatch, reqMatchedPolygonRatio);
    std::vector<std::pair<Eigen::Index, Eigen::Index>> finalVertexMatches, vertexMatches = matching::nonGreedyHierarchyIndexMatching(ref, targ, polyMatchThresh, numSideBoundsForMatch, reqMatchedPolygonRatio);

    // Post-process: double-check matches, remove any where pairs are beyond a certain distance from each other
    // "if matched points across differential observations are not very close relative to the observer, then the match is probably wrong"
    for (const auto& [refIdx, targIdx] : vertexMatches) {
        if (((ref.landmarks.col(refIdx) - targ.landmarks.col(targIdx)).array().abs() < pointMatchThresh).all())
            finalVertexMatches.push_back({refIdx, targIdx});
    }

    return finalVertexMatches;
}

void tfCloud(Eigen::Matrix4d existingTfToBase, Eigen::Matrix4d refToTargTf, ObsRecord& obsRec) {
    Eigen::Matrix4d fullTfToBase = refToTargTf * existingTfToBase;
    // Temporarily convert the pointcloud to 3D to transform into base frame 
    pcl::PointCloud<pcl::PointXYZ> nonAssociatedCloud, newlyAssociatedCloud;
    pcl::copyPointCloud(obsRec.cloud, nonAssociatedCloud);
    pcl::transformPointCloud(nonAssociatedCloud, newlyAssociatedCloud, fullTfToBase);
    // std::cout << "transforming cloud with tf: " << std::endl << fullTfToBase << std::endl;

    // Add transformed points to keyframe pointcloud to baseframe and store tf in the frame's observation record
    *bigPcPtr += newlyAssociatedCloud;
    obsRec.tf = fullTfToBase;
    kfObs.insert(obsRec);
}


void parse2DPC(const int frameID, const pcl::PointCloud<pcl::PointXY>& localCloud) {
    if (localCloud.size() < 3) {
        if (isDebug) {
            std::cout << "Observation does not contain enough points to construct triangulation, discarding...\n";
            std::cout << "===============================================================" << std::endl;
        }
        return;
    }
    // Construct a PointVector for the given tree positions
    Points vectorOfTrees(2, localCloud.size());
    int idx = 0;
    for (const auto& p : localCloud) vectorOfTrees.col(idx++) = PtLoc{p.x, p.y};
    ObsRecord myObs(frameID, vectorOfTrees, localCloud);

    // Try to match the current frame with any that have been associated with a base frame
    if (!kfObs.empty()) {
        std::vector<std::pair<Eigen::Index, Eigen::Index>> pointMatchIndices;
        if (isDebug) std::cout << "Matching frame " << myObs.frameId << " with associated clouds." << std::endl;
        
        // Match (current frame --> associated frame) until match found
        auto revKfIter = kfObs.rbegin();
        do {
            pointMatchIndices = matchObsIdx(myObs.obs, revKfIter->obs, polygonMatchThresh, validPointMatchThresh);
        } while (pointMatchIndices.size() < reqMatchesForFrameAssoc && ++revKfIter != kfObs.rend());

        // Store observation data
        if (pointMatchIndices.size() < reqMatchesForFrameAssoc) {
            unassociatedObs.push_back(myObs);
            if (isDebug) std::cout << "No matches found." << std::endl;
        } else {
            // Estimate tf from reference to target frame (assuming we have gotten rid of outliers already)
            if (isDebug) std::cout << "Found " << pointMatchIndices.size() << " matches with frame " << revKfIter->frameId << ", associating..." << std::endl;
            tfCloud(revKfIter->tf, computeRigid2DEuclidTfFromIndices(pointMatchIndices, myObs.obs, revKfIter->obs), myObs);

            // Try to match all unassociated observations with the current observation
            auto obsIter = unassociatedObs.begin();
            while (obsIter != unassociatedObs.end()) {
                pointMatchIndices = matchObsIdx(obsIter->obs, myObs.obs, polygonMatchThresh, validPointMatchThresh);
                if (pointMatchIndices.size() >= reqMatchesForFrameAssoc) {
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
            pointMatchIndices = matchObsIdx(myObs.obs, revIter->obs, polygonMatchThresh, validPointMatchThresh);
        } while (pointMatchIndices.size() < reqMatchesForFrameAssoc && ++revIter != unassociatedObs.rend());

        // Store observation data
        if (pointMatchIndices.size() < reqMatchesForFrameAssoc) {
            unassociatedObs.push_back(myObs);
            if (isDebug) std::cout << "None found." << std::endl;
        } else {
            // Assign the target frame of this match as the base of the keyframe
            revIter->tf = Eigen::Matrix4d::Identity();
            pcl::copyPointCloud(revIter->cloud, *bigPcPtr);
            kfObs.insert(*revIter);

            // Estimate tf from reference to target frame (assuming we have gotten rid of outliers already)
            if (isDebug) std::cout << "With " << pointMatchIndices.size() << " matches, base frame " << revIter->frameId << " established; adding frame " << myObs.frameId << " behind it." << std::endl;
            tfCloud(revIter->tf, computeRigid2DEuclidTfFromIndices(pointMatchIndices, myObs.obs, revIter->obs), myObs);
            
            // Remove the base frame from the unassociated list and advance the iterator
            std::advance(revIter, 1);
            unassociatedObs.erase(revIter.base());

            // For every remaining unassociated observation, try to match with the current observation
            while (revIter != unassociatedObs.rend()) {
                pointMatchIndices = matchObsIdx(revIter->obs, myObs.obs, polygonMatchThresh, validPointMatchThresh);
                if (pointMatchIndices.size() >= reqMatchesForFrameAssoc) {
                    if (isDebug) std::cout << "Including frame " << revIter->frameId << " in the association (" << pointMatchIndices.size() << " matches)." << std::endl;
                    tfCloud(myObs.tf, computeRigid2DEuclidTfFromIndices(pointMatchIndices, revIter->obs, myObs.obs), *revIter);
                    std::advance(revIter, 1);   // reverse iterators are wierd
                    unassociatedObs.erase(revIter.base());  // iterator "advances" to look at previous value
                }
                else ++revIter;
            }
        }
    } else {
        // If this is the first observation, add it to the unassociated ones
        unassociatedObs.push_back(myObs);
        if (isDebug) std::cout << "Initializing unassociated observations at frame " << myObs.frameId << std::endl;
    }

    // Publish geometric hierarchy for the input frame (if desired)
    if (isIndivFramePub) {
        std::stringstream polyStream, triStream, ptStream, hierStream;
        hierStream << myObs.frameId << "!";
        
        // Iterate over the indices of the Polygons in the hierarchy
        int j = 0, k = 0;
        for (auto pIdx : myObs.obs.hier->getChildrenIds(0)) {
            polyStream << (j++!=0 ? "|": "");
            for (int i = 0; i < myObs.obs.hier->getPolygon(pIdx).landmarkRefs.size(); ++i) {
                auto myPoint = myObs.obs.landmarks.col(myObs.obs.hier->getPolygon(pIdx).landmarkRefs(i));
                polyStream << (i!=0 ? ":": "") << myPoint[0] << " " << myPoint[1];
            }
            
            // Iterate over the indices of the Triangles that compose this Polygon
            for (auto tIdx : myObs.obs.hier->getChildrenIds(pIdx)) {
                triStream << (k++!=0 ? "|": "");
                for (int i = 0; i < myObs.obs.hier->getPolygon(tIdx).landmarkRefs.size(); ++i) {
                    auto myPoint = myObs.obs.landmarks.col(myObs.obs.hier->getPolygon(tIdx).landmarkRefs(i));
                    triStream << (i!=0 ? ":": "") << myPoint[0] << " " << myPoint[1];
                }
            }
        }

        j = 0;
        for (const PtLoc& ldmk : myObs.obs.landmarks.colwise()) {
            ptStream << (j++ != 0 ? ":": "") << ldmk(0) << " " << ldmk(1);
        }

        // Construct message from string streams and publish
        std_msgs::String hierMsg;
        hierStream << polyStream.str() << "$" << triStream.str() << "$" << ptStream.str();
        hierMsg.data = hierStream.str();
        hierPub.publish(hierMsg);
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
    if (kfObs.size() >= maxKeyframeWidth || (!kfObs.empty() && kfObs.rbegin()->frameId + numSkippedFramesBeforeSend <= frameID)) {
        // At this point, all pc in kfObs should be in the same reference frame
        if (isDebug) {
            std::cout << "Keyframe with IDs ";
            for (auto& obs : kfObs) std::cout << obs.frameId << "(" << obs.cloud.size() << "), ";
            std::cout << " sending..." << std::endl;
        }

        // Transmit cumulative point cloud (if desired)
        if (pubAllPoints) {
            if (isLogging) {
                std::ofstream allOut(allKfPtsPath+std::to_string(1+numFramesSent)+".txt"); 
                for (const ObsRecord& kf : kfObs) allOut << (kf == *kfObs.begin() ? "" : " ") << kf.frameId; 
                for (const auto& p : *bigPcPtr) allOut << std::endl << p.x << " " << p.y;
                allOut.close();
            }

            if (!isOffline) {
                sensor_msgs::PointCloud2 allPointsMsg;
                pcl::toROSMsg(*bigPcPtr, allPointsMsg);
                publishPointCloud(bigCloudPub, allPointsMsg, kfObs.begin()->frameId, "sensor_frame");
            }
        }

        // TODO talk to Bailey about efficiently deriving the cluster centers out in either nlogn or linear time
        // (maybe this greedy solution is good enough)
        // Combine them --> do standard clustering/point association
        pcl::EuclideanClusterExtraction<pcl::PointXYZ> ece;
        ece.setInputCloud(bigPcPtr);
        ece.setClusterTolerance(clusterTolerance);  // Set the spatial cluster tolerance (in meters)
        ece.setMinClusterSize(2);                   // Don't make a cluster for a single stray point 
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

        // Log keyframe data
        if (isLogging) {
            std::ofstream kfOut(keyframePath+std::to_string(++numFramesSent)+".txt");
            for (const ObsRecord& kf : kfObs) if (kf.tf.isIdentity()) kfOut << kf.frameId;
            for (const auto& p : outputCloud) kfOut << std::endl << p.x << " " << p.y;
            kfOut.close();
        }

        // Publish the keyframe
        if (!isOffline) {
            sensor_msgs::PointCloud2 kfMsg;
            pcl::toROSMsg(outputCloud, kfMsg);
            publishPointCloud(kfpub, kfMsg, kfObs.begin()->frameId, "sensor_frame");
        }

        // Clear the keyframe
        kfObs.clear();
        bigPcPtr->clear();  // TODO might be overkill because it gets overwritten when base frame re-inits anyway
    }

    // If an unassociated frame becomes too old before it can be associated, remove it
    if (!unassociatedObs.empty() && unassociatedObs.front().frameId + numSkippedFramesBeforeSend <= frameID) {
        if (isDebug) std::cout << "Deleting frame " << unassociatedObs.front().frameId << " from unassociated observations." << std::endl;
        unassociatedObs.erase(unassociatedObs.begin());
    }
    if (isDebug) std::cout << "===============================================================" << std::endl;
    
}


void buildKeyframes(const std::string& localObsPath) {
    // Sort the local observations in sequential order
    std::map<int, std::string> orderedInputPaths;
    for (const auto& entry : std::filesystem::directory_iterator(localObsPath)) {
        orderedInputPaths[atoi(entry.path().filename().replace_extension("").string().c_str())] = entry.path();
    }

    std::cout << "Processing observations from " << localObsPath << " into keyframes." << std::endl;

    Eigen::VectorXi runtimePerObs(orderedInputPaths.size());
    for (const auto& [id, obsPath] : orderedInputPaths) {
        std::ifstream inFile(obsPath);
        pcl::PointCloud<pcl::PointXY> inputCloud;
        pcl::PointXY p; double radius; std::string line;

        while (std::getline(inFile, line)) {
            std::istringstream iss(line);   // only collect landmark data
            if (iss >> p.x >> p.y >> radius) inputCloud.push_back(p);
        }
        inFile.close();

        auto t1 = std::chrono::high_resolution_clock::now();
        parse2DPC(id, inputCloud);
        auto t2 = std::chrono::high_resolution_clock::now();
        auto ms_int = std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1);
        runtimePerObs(id) = ms_int.count();
    }
    std::cout << "Max time to process observations: " << runtimePerObs.maxCoeff() << "ms" << std::endl;
}

void observationsReady(const std_msgs::String::ConstPtr& msg) {
    if (msg->data == "shutdown") {
        std_msgs::String doneMsg;
        doneMsg.data = "shutdown";
        donePub.publish(doneMsg);
        ros::shutdown();
    } else {
        keyframePath = msg->data+"/keyframe/";
        allKfPtsPath = msg->data+"/allKfPts/";
        buildKeyframes(msg->data+"/local_obs");
        donePub.publish(msg);
        unassociatedObs.clear(); kfObs.clear(); bigPcPtr->clear();
        numFramesSent = 0;
    } 
}



void readLiveObservation(const sensor_msgs::PointCloud2ConstPtr& cloudMsg) {
    // Take the points from the PointCloud2 message
    pcl::PointCloud<pcl::PointXY> localCloud;
    pcl::fromROSMsg(*cloudMsg, localCloud);
    parse2DPC(cloudMsg->header.seq, localCloud);
}


int main(int argc, char **argv) {
    // Initialize Node and read in private parameters
    ros::init(argc, argv, "keyframe_maker");
    ros::NodeHandle n("~");

    // booleans
    isLogging = n.param("/logging", true);
    isOffline = n.param("/offline", true);
    isDebug = n.param("debug", true);
    pubAllPoints = n.param("pubAllPoints", false);
    isIndivFramePub = n.param("indivFramePub", false) && !isOffline;

    // integers
    numSideBoundsForMatch = n.param("numSideBoundsForMatch", 3);
    maxKeyframeWidth = n.param("maxKeyframeWidth", 5);
    numSkippedFramesBeforeSend = n.param("numSkippedFramesBeforeSend", 3);
    reqMatchesForFrameAssoc = n.param("reqMatchesForFrameAssoc", 2);
    if (reqMatchesForFrameAssoc < 2) reqMatchesForFrameAssoc = 2;

    // doubles
    polygonMatchThresh = n.param("polygonMatchThresh", 3.0);
    reqMatchedPolygonRatio = n.param("reqMatchedPolygonRatio", 0.5); // 0.0 disables this feature
    validPointMatchThresh = n.param("validPointMatchThresh", 1.0);
    clusterTolerance = n.param("clusterTolerance", 0.1);

    ros::Subscriber sub;
    if (isOffline) {
        donePub = n.advertise<std_msgs::String>("doneFlag", 20);
        sub = n.subscribe("/sim_path/doneFlag", 25, observationsReady);
    } else {
        std::string absolutePackagePath = ros::package::getPath("urquhart"), outputPath;
        n.param<std::string>("/outputDirName", outputPath, "testOutput");
        keyframePath = absolutePackagePath+"/output/"+outputPath+"/keyframe/";
        allKfPtsPath = absolutePackagePath+"/output/"+outputPath+"/allKfPts/";

        sub = n.subscribe("/sim_path/local_points", 10, readLiveObservation);
        kfpub = n.advertise<sensor_msgs::PointCloud2>("keyframe", 10);
        if (isIndivFramePub) hierPub = n.advertise<std_msgs::String>("hierarchy", 10);
        if (pubAllPoints) bigCloudPub = n.advertise<sensor_msgs::PointCloud2>("allPoints", 10);
    }

    ros::spin();

    return 0;
}