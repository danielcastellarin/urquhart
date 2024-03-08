#include <ros/ros.h>
#include <observation.hpp>
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

std::vector<urquhart::Observation> kfObservations;
std::vector<pcl::PointCloud<pcl::PointXY>> pcTransformed;
std::vector<std::map<pcl::PointXY, pcl::PointXY>> obsToTfMap;
// TODO store tf matrix to get from the most recent observation in the keyframe back to the first observation
//      by default, this should be the identity matrix

// store dictionary of landmark locations across observations relative to the first frame
//      k=landmark id, v=((x,y), #frames observed in) 
std::map<int, std::pair<pcl::PointXY, int>> kfTreeDict;
std::map<std::pair<int, pcl::PointXY>, int> ii;     // inverted index to find which dict element should be updated after successful association
int landmarkIdx = 0;

pcl::PointXY pclConvert(vecPtT p) {
    pcl::PointXY myPoint;
    myPoint.x = p[0]; myPoint.y = p[1];
    return myPoint;
}
pcl::PointXY sumPoints(pcl::PointXY p1, pcl::PointXY p2) {
    pcl::PointXY summedPoint;
    summedPoint.x = p1.x + p2.x; summedPoint.y = p1.y + p2.y;
    return summedPoint;
}


// void parse2DPC(const sensor_msgs::PointCloud2ConstPtr& cloudMsg)
// {
//     // Take the points from the PointCloud2 message
//     pcl::PointCloud<pcl::PointXY> localCloud;
    
//     pcl::fromROSMsg(*cloudMsg, localCloud);
//     int frameID = cloudMsg->header.seq;     // Do I need this?

//     // Construct a PointVector for the given tree positions
//     PointVector vectorOfTrees;
//     for(const auto& p : localCloud) vectorOfTrees.push_back(std::vector<double>{p.x, p.y});
//     urquhart::Observation obs(vectorOfTrees);
    
//     // For now, I'm going to assume that the fixed reference frame of the keyframe will be taken from the first frame  
//     if (kfObservations.empty()) {
//         // Add geometric hierarchy to this keyframe's record and store the positions of the observed landmarks
//         kfObservations.push_back(obs);
//         pcTransformed.push_back(localCloud);
//         for (const auto& p : localCloud) {
//             ii.insert({{kfObservations.size(), p}, ++landmarkIdx});
//             kfTreeDict.insert({landmarkIdx, {p, 1}});
//         }
//     } else {
//         // Store set of individual points in current frame that have been matched across previous frames 
//         std::map<pcl::PointXY, int> thisObsMatches;
//         std::vector<std::pair<pcl::PointXY, pcl::PointXY>> matchesToPrevious;
//         std::set<size_t> uniqueLandmarkMatches;

//         // TODO: loop over each preceeding frames if not enough landmarks have been matched in the current frame (THIS MIGHT DEPEND ON FOREST DENSITY?...)
//         int numMatchesIWant = localCloud.size()/2, obsIdx = 0;
//         while (thisObsMatches.size() < numMatchesIWant && ++obsIdx > kfObservations.size()) {
//             std::vector<std::pair<size_t, size_t>> polygonMatches, triangleMatches;
//             auto& prevObs = kfObservations.end()[-obsIdx];   // use negative index

//             // Polygon Matching
//             std::vector<size_t> refIds = obs.H->get_children(0), targIds = prevObs.H->get_children(0);
//             matching::polygonMatching(obs, refIds, prevObs, targIds, 5, polygonMatches);

//             // Triangle Matching
//             for (auto pMatch : polygonMatches) {    // FIXME? make the loop explicitly over constant references?
//                 refIds = obs.H->get_children(pMatch.first);
//                 targIds = prevObs.H->get_children(pMatch.second);
//                 // TODO FROM URQ: ADD CHECK IF % OF TRIANGLES THAT MACTHED IS LARGER THAN 1/2
//                 matching::polygonMatching(obs, refIds, prevObs, targIds, 5, triangleMatches);
//             }

//             // Point Matching
//             for (auto tMatch : triangleMatches) {   // FIXME? make the loop explicitly over constant references?
//                 urquhart::Polygon refTriangle = obs.H->get_vertex(tMatch.first), targetTriangle = prevObs.H->get_vertex(tMatch.second);
//                 std::vector<size_t> chi = {0, 1, 2}, bestPermutation;

//                 // TODO change the edgeLengths to do squared distance instead of euclidean distance (unnecessary square root)

//                 // Permute the edges to find the best match between the triangles
//                 double bestDist = 1000000;
//                 do {
//                     double d = euclideanDistance(refTriangle.edgeLengths, std::vector<double>{targetTriangle.edgeLengths[chi[0]], targetTriangle.edgeLengths[chi[1]], targetTriangle.edgeLengths[chi[2]]});
//                     if (d < bestDist) {
//                         bestDist = d;
//                         bestPermutation = chi;
//                     }
//                 } while (std::next_permutation(chi.begin(), chi.end()));

//                 // Match the corresponding vertices using the best permutation obtained above
//                 // Specifically, the vertices across from the matched edges (invariant to vertex ordering of the convex hull)
//                 for (size_t i = 0; i < 3; ++i) {
//                     int refIdx = (i+2)%3, targetIdx = (bestPermutation[i]+2)%3;
//                     pcl::PointXY refPoint = pclConvert(refTriangle.points[refIdx]), targetPoint = pclConvert(targetTriangle.points[targetIdx]);
//                     std::pair<int, pcl::PointXY> refPointKey = {frameID, refPoint};
//                     // size_t idx = cantorPairing(refTriangle.edges[refIdx].first, targetTriangle.edges[targetIdx].first);   // todo change ids for target vertices to use the ids from II
//                     // if (ii.find({frameID, refPoint}) == ii.end()) {
//                     //     std::pair<int, pcl::PointXY> targetPointKey = {kfObservations.size()-obsIdx, pclConvert(targetTriangle.points[(bestPermutation[i]+2)%3])};
//                     //     int existingLandmarkId = ii[targetPointKey];
//                     //     ii[refPointKey] = existingLandmarkId;
//                     //     thisObsMatches[refPoint] = existingLandmarkId;
//                     //     // .insert({existingLandmarkId, {}})
//                     // }
//                     int existingLandmarkId = ii[{kfObservations.size()-obsIdx, targetPoint}];
//                     size_t idx = cantorPairing(refTriangle.edges[refIdx].first, existingLandmarkId);
//                     // if (ii.find({frameID, refPoint}) == ii.end()) {
//                     if (uniqueLandmarkMatches.find(idx) == uniqueLandmarkMatches.end()) {
//                         // ii[refPointKey] = existingLandmarkId;        // TODO
//                         thisObsMatches[refPoint] = existingLandmarkId;
//                         // matchesToPrevious.push_back({refPoint, pclConvert(targetTriangle.points[(bestPermutation[i]+2)%3])});
//                         // .insert({existingLandmarkId, {}})
//                         uniqueLandmarkMatches.insert(idx);
//                     }
//                     // if (unique_matches.find(idx) == unique_matches.end()) {
//                     //     vecPtT pA = refTriangle.points[refIdx];
//                     //     vecPtT pB = targetTriangle.points[targetIdx];
//                     //     pointMatches.push_back(std::make_pair(pA, pB));     // TODO use thisObsMatches instead
//                     //     unique_matches.insert(idx);                         //   "
//                     // }
//                 }
//             }
//         }

//         // Now add the geometric hierarchy to the record
//         kfObservations.push_back(obs);

//         // TODO call homography for the pairs of matched points
//         // get the transform between this frame and the previous

//         // TODO multiply existing matrix with the newly computed one to get the tf from this frame to the first
//         // 3T0 = 3T2 (from above) 2T0 (og)

//         // TODO for each landmark (id, pos) in the latest observation:
//             // apply that full transform to get its position relative to the first frame
//             // if ID not in dictionary, put it in with count=1
//             // else, sum pos with whatever is in there, increment count 
//     }

//     // Finalize keyframe, send it to global code / backend for processing, then reset keyframe data structures
//     int n = 5; // TODO get this value from a param
//     if (kfObservations.size() >= n) {
//         // TODO if only raw points in ref plane, cluster then to create trees? <--- need to be CERTAIN that extra trees are not being created

//         // TODO for each landmark, divide summed position by #times it was observed
//         //      copy them (or something)
//         // TODO if any landmarks are within a CERTAIN THRESHOLD OF EACH OTHER, combine them

//         // send the keyframe (set of 2D positions of landmarks) to the backend
//         // (For now, I will expect the backend to do create the geometric hierarchy once it arrives)

//         kfObservations.clear();
//         // set matrix back to the identity matrix
//         // clear dictionary
//     }
// }

void myPolygonMatching(
    const urquhart::Observation &ref, std::vector<size_t> refIds,
    const urquhart::Observation &targ, std::vector<size_t> targIds, double thresh,
    std::vector<std::pair<size_t, size_t>> &polygonMatches)
{

    std::set<size_t> matched;
    for (auto rIdx : refIds)
    {
        size_t bestMatch = 0;
        size_t bestDist = 100000;
        urquhart::Polygon rp = ref.H->get_vertex(rIdx);
        for (auto tIdx : targIds)
        {
            urquhart::Polygon tp = targ.H->get_vertex(tIdx);
            // if tIdx was not matched before and the difference of number of points is not larger than 5
            if (matched.find(tIdx) == matched.end() &&
                std::abs(int(rp.points.size() - tp.points.size())) <= 3)
            {
                double d = euclideanDistance(rp.descriptor, tp.descriptor);
                if (d < bestDist)
                {
                    bestDist = d;
                    bestMatch = tIdx;
                }
            }
        }

        if (bestDist < thresh)
        {
            matched.insert(bestMatch);
            std::pair<size_t, size_t> m = std::make_pair(rIdx, bestMatch);
            polygonMatches.push_back(m);
        }
    }
}

struct ObsRecord {
    int frameId;
    urquhart::Observation obs;
    pcl::PointCloud<pcl::PointXY> cloud;
    Eigen::Matrix3f tf;
    ObsRecord(int id, PointVector pv, pcl::PointCloud<pcl::PointXY> pc) : frameId(id), obs(pv), cloud(pc) {}
    bool operator==(const ObsRecord &other) const{ return frameId == other.frameId; }
};

// bool obsSorter(ObsRecord const& a, ObsRecord const& b) {
//     return a.frameId < b.frameId;
// }

std::vector<std::pair<pcl::PointXY, pcl::PointXY>> matchObs(const urquhart::Observation &ref, const urquhart::Observation &targ, double thresh) {
    std::vector<std::pair<size_t, size_t>> polygonMatches, triangleMatches;
    // std::vector<std::pair<vecPtT, vecPtT>> pointMatches;
    std::vector<std::pair<pcl::PointXY, pcl::PointXY>> vertexMatches;
    std::set<size_t> uniqueMatches;

    // Polygon Matching (Level 2)
    // std::vector<size_t> refIds = ref.H->get_children(0), targIds = targ.H->get_children(0);
    // matching::polygonMatching(ref, ref.H->get_children(0), targ, targ.H->get_children(0), thresh, polygonMatches);
    myPolygonMatching(ref, ref.H->get_children(0), targ, targ.H->get_children(0), thresh, polygonMatches);

    // Triangle Matching (Level 1)
    for (auto pMatch : polygonMatches) {
        // refIds = ref.H->get_children(pMatch.first), targIds = targ.H->get_children(pMatch.second);
        // TODO: ADD CHECK IF % OF TRIANGLES THAT MACTHED IS LARGER THAN 1/2
        // matching::polygonMatching(ref, ref.H->get_children(pMatch.first), targ, targ.H->get_children(pMatch.second), thresh, triangleMatches);
        myPolygonMatching(ref, ref.H->get_children(pMatch.first), targ, targ.H->get_children(pMatch.second), thresh, triangleMatches);
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

    return vertexMatches;
}

// std::vector<urquhart::Observation> kfObservations, unassociatedObs;
// std::vector<std::pair<urquhart::Observation, pcl::PointCloud<pcl::PointXY>>> kfObservations, unassociatedObs;
// std::vector<pcl::PointCloud<pcl::PointXY>> ogPointClouds;
// int baseFrameIdx = 0;
std::vector<ObsRecord> unassociatedObs;
std::set<ObsRecord> kfObs;

void parse2DPC(const sensor_msgs::PointCloud2ConstPtr& cloudMsg) {
    // Take the points from the PointCloud2 message
    pcl::PointCloud<pcl::PointXY> localCloud;
    pcl::fromROSMsg(*cloudMsg, localCloud);

    // Construct a PointVector for the given tree positions
    PointVector vectorOfTrees;
    for(const auto& p : localCloud) vectorOfTrees.push_back(std::vector<double>{p.x, p.y});
    ObsRecord myObs(cloudMsg->header.seq, vectorOfTrees, localCloud);

    if (!kfObs.empty()) {
        auto revKfIter = kfObs.rbegin();
        std::vector<std::pair<pcl::PointXY, pcl::PointXY>> pointMatches;
        
        // Match (current frame --> associated frame) until match found
        do {
            pointMatches = matchObs(myObs.obs, revKfIter->obs, 5);
            // TODO maybe ransac validate with available point matches here before leaving loop?
        } while (pointMatches.empty() && ++revKfIter != kfObs.rend());

        // Store observation data
        if (pointMatches.empty()) unassociatedObs.push_back(myObs);
        else {
            // TODO maybe get tf before leaving loop
            // use point matches to estimate transform from current frame into previous
            // use forward kinematics to compute full tf to base frame (new tf * other obs's tf = full tf)
            // assign that tf to myObs.tf
            // apply tf to myObs.cloud (inplace)
            kfObs.insert(myObs);

            auto obsIter = unassociatedObs.begin();
            while(obsIter != unassociatedObs.end()) {
                auto pointMatches = matchObs(obsIter->obs, myObs.obs, 5);
                if (!pointMatches.empty()) {
                    // use point matches to estimate transform from unassociated frame into current frame
                    // use forward kinematics to compute full tf to base frame (new tf * myObs tf = full tf)
                    // assign that tf to obsIter->tf
                    // apply tf to obsIter->cloud (inplace)
                    kfObs.insert(*obsIter);
                    obsIter = unassociatedObs.erase(obsIter);
                }
                else ++obsIter;
            }
        }
    } else if (!unassociatedObs.empty()) {
        auto revIter = unassociatedObs.rbegin();
        std::vector<std::pair<pcl::PointXY, pcl::PointXY>> pointMatches;

        // Match (current frame --> unassociated frame) until match found
        do {
            pointMatches = matchObs(myObs.obs, revIter->obs, 5);
            // TODO maybe ransac validate with available point matches here before leaving loop?
        } while (pointMatches.empty() && ++revIter != unassociatedObs.rend());

        // Store observation data
        if (pointMatches.empty()) unassociatedObs.push_back(myObs);
        else {
            kfObs.insert(*revIter);
            // TODO set *revIter.tf to identity matrix 
            
            // TODO maybe get tf before leaving loop
            // use point matches to estimate transform from current frame into previous
            // use forward kinematics to compute full tf to base frame (new tf * other obs's tf = full tf)
            // assign that tf to myObs.tf
            // apply tf to myObs.cloud (inplace)
            kfObs.insert(myObs);
            unassociatedObs.erase(std::next(revIter).base());   // iter should not move

            // For every remaining unassociated obs: match with the current obs
            while(revIter != unassociatedObs.rend()) {
                auto pointMatches = matchObs(revIter->obs, myObs.obs, 5);
                if(!pointMatches.empty()) {
                    // use point matches to estimate transform from unassociated frame into current frame
                    // use forward kinematics to compute full tf to base frame (new tf * myObs tf = full tf)
                    // assign that tf to revIter->tf
                    // apply tf to revIter->cloud (inplace)
                    kfObs.insert(*revIter);
                    unassociatedObs.erase(std::next(revIter).base()); // iter should not move
                }
                else ++revIter;
            }
        }
    } else {
        // If this is the first observation, add it to the unassociated ones
        unassociatedObs.push_back(myObs);
    }
    
    

    // TODO postprocessing
    // check size of kfObs to see if it is ready to be sent
        // should also be sent if newer observations are not being associated, too
        // At this point, all pc in kfObs should be in the same reference frame
        // Combine them --> do standard clustering/point association
            // TODO should timestamp of message be that of the base frame or whatever the current time is?
        // send point cloud to graph construction

    // check unassociated to see if any should be removed (old)

    // if needing to debug...
    // I'm guessing I could estimate whether a pc was transformed correctly by checking pc similarity with the other pcs
        // I would imagine that the similarity value would be significantly bigger
        // https://stackoverflow.com/questions/55913968/metric-to-compare-two-point-clouds-similarity


    
    // OLD STUFF VVVVVVVVVVVVVVVV

    // // For now, I'm going to assume that the fixed reference frame of the keyframe will be taken from the first frame  
    // if (!kfObservations.empty()) {
    //     // Store set of individual points in current frame that have been matched across previous frames 
    //     std::map<pcl::PointXY, int> thisObsMatches;
    //     std::vector<std::pair<pcl::PointXY, pcl::PointXY>> matchesToPrevious;
    //     std::set<size_t> uniqueLandmarkMatches;

    //     // TODO: loop over each preceeding frames if not enough landmarks have been matched in the current frame (THIS MIGHT DEPEND ON FOREST DENSITY?...)
    //     int numMatchesIWant = localCloud.size()/2, obsIdx = 0;
    //     while (thisObsMatches.size() < numMatchesIWant && ++obsIdx > kfObservations.size()) {
    //         std::vector<std::pair<size_t, size_t>> polygonMatches, triangleMatches;
    //         auto& prevObs = kfObservations.end()[-obsIdx];   // use negative index

    //         // Polygon Matching
    //         std::vector<size_t> refIds = obs.H->get_children(0), targIds = prevObs.H->get_children(0);
    //         matching::polygonMatching(obs, refIds, prevObs, targIds, 5, polygonMatches);

    //         // Triangle Matching
    //         for (auto pMatch : polygonMatches) {    // FIXME? make the loop explicitly over constant references?
    //             refIds = obs.H->get_children(pMatch.first);
    //             targIds = prevObs.H->get_children(pMatch.second);
    //             // TODO FROM URQ: ADD CHECK IF % OF TRIANGLES THAT MACTHED IS LARGER THAN 1/2
    //             matching::polygonMatching(obs, refIds, prevObs, targIds, 5, triangleMatches);
    //         }

    //         // Point Matching
    //         for (auto tMatch : triangleMatches) {   // FIXME? make the loop explicitly over constant references?
    //             urquhart::Polygon refTriangle = obs.H->get_vertex(tMatch.first), targetTriangle = prevObs.H->get_vertex(tMatch.second);
    //             std::vector<size_t> chi = {0, 1, 2}, bestPermutation;

    //             // TODO change the edgeLengths to do squared distance instead of euclidean distance (unnecessary square root)

    //             // Permute the edges to find the best match between the triangles
    //             double bestDist = 1000000;
    //             do {
    //                 double d = euclideanDistance(refTriangle.edgeLengths, std::vector<double>{targetTriangle.edgeLengths[chi[0]], targetTriangle.edgeLengths[chi[1]], targetTriangle.edgeLengths[chi[2]]});
    //                 if (d < bestDist) {
    //                     bestDist = d;
    //                     bestPermutation = chi;
    //                 }
    //             } while (std::next_permutation(chi.begin(), chi.end()));

    //             // Match the corresponding vertices using the best permutation obtained above
    //             // Specifically, the vertices across from the matched edges (invariant to vertex ordering of the convex hull)
    //             for (size_t i = 0; i < 3; ++i) {
    //                 int refIdx = (i+2)%3, targetIdx = (bestPermutation[i]+2)%3;
    //                 pcl::PointXY refPoint = pclConvert(refTriangle.points[refIdx]), targetPoint = pclConvert(targetTriangle.points[targetIdx]);
    //                 std::pair<int, pcl::PointXY> refPointKey = {frameID, refPoint};
    //                 // size_t idx = cantorPairing(refTriangle.edges[refIdx].first, targetTriangle.edges[targetIdx].first);   // todo change ids for target vertices to use the ids from II
    //                 // if (ii.find({frameID, refPoint}) == ii.end()) {
    //                 //     std::pair<int, pcl::PointXY> targetPointKey = {kfObservations.size()-obsIdx, pclConvert(targetTriangle.points[(bestPermutation[i]+2)%3])};
    //                 //     int existingLandmarkId = ii[targetPointKey];
    //                 //     ii[refPointKey] = existingLandmarkId;
    //                 //     thisObsMatches[refPoint] = existingLandmarkId;
    //                 //     // .insert({existingLandmarkId, {}})
    //                 // }
    //                 int existingLandmarkId = ii[{kfObservations.size()-obsIdx, targetPoint}];
    //                 size_t idx = cantorPairing(refTriangle.edges[refIdx].first, existingLandmarkId);
    //                 // if (ii.find({frameID, refPoint}) == ii.end()) {
    //                 if (uniqueLandmarkMatches.find(idx) == uniqueLandmarkMatches.end()) {
    //                     // ii[refPointKey] = existingLandmarkId;        // TODO
    //                     thisObsMatches[refPoint] = existingLandmarkId;
    //                     // matchesToPrevious.push_back({refPoint, pclConvert(targetTriangle.points[(bestPermutation[i]+2)%3])});
    //                     // .insert({existingLandmarkId, {}})
    //                     uniqueLandmarkMatches.insert(idx);
    //                 }
    //                 // if (unique_matches.find(idx) == unique_matches.end()) {
    //                 //     vecPtT pA = refTriangle.points[refIdx];
    //                 //     vecPtT pB = targetTriangle.points[targetIdx];
    //                 //     pointMatches.push_back(std::make_pair(pA, pB));     // TODO use thisObsMatches instead
    //                 //     unique_matches.insert(idx);                         //   "
    //                 // }
    //             }
    //         }
    //     }

    //     // Now add the geometric hierarchy to the record
    //     // kfObservations.push_back(obs);

    //     // TODO call homography for the pairs of matched points
    //     // get the transform between this frame and the previous

    //     // TODO multiply existing matrix with the newly computed one to get the tf from this frame to the first
    //     // 3T0 = 3T2 (from above) 2T0 (og)

    //     // TODO for each landmark (id, pos) in the latest observation:
    //         // apply that full transform to get its position relative to the first frame
    //         // if ID not in dictionary, put it in with count=1
    //         // else, sum pos with whatever is in there, increment count 
    // }

    // // Finalize keyframe, send it to global code / backend for processing, then reset keyframe data structures
    // int n = 5; // TODO get this value from a param
    // if (kfObservations.size() >= n) {
    //     // TODO if only raw points in ref plane, cluster then to create trees? <--- need to be CERTAIN that extra trees are not being created

    //     // TODO for each landmark, divide summed position by #times it was observed
    //     //      copy them (or something)
    //     // TODO if any landmarks are within a CERTAIN THRESHOLD OF EACH OTHER, combine them

    //     // send the keyframe (set of 2D positions of landmarks) to the backend
    //     // (For now, I will expect the backend to do create the geometric hierarchy once it arrives)

    //     kfObservations.clear();
    //     // set matrix back to the identity matrix
    //     // clear dictionary
    // }
}


// TODO implement keyframe width parameter (default should be 5)
int main(int argc, char **argv) {
    // Initialize Node and read in private parameters
    ros::init(argc, argv, "keyframe_maker");
    ros::NodeHandle n("~");
    // SimConfig cfg(n);
    // cfg.outputConfig(std::cout);
    // ros::Rate pub_rate(cfg.pubRate);    // 10Hz by default

    ros::Subscriber sub = n.subscribe("local_points", 10, parse2DPC);

    ros::spin();


    return 0;
}