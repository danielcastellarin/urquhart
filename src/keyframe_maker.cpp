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
// TODO store tf matrix to get from the most recent observation in the keyframe back to the first observation
//      by default, this should be the identity matrix
// TODO store dictionary of landmark locations across observations relative to the first frame, only need position and landmark id (for position approximation later)
//      k=landmark id, v=((x,y), #frames observed in) 

void parse2DPC(const sensor_msgs::PointCloud2ConstPtr& cloudMsg)
{
    // Take the points from the PointCloud2 message
    pcl::PointCloud<pcl::PointXY> localCloud;
    pcl::fromROSMsg(*cloudMsg, localCloud);
    int frameID = cloudMsg->header.seq;     // Do I need this?

    // Construct a PointVector for the given tree positions
    PointVector vectorOfTrees;
    for(const auto& p : localCloud) vectorOfTrees.push_back(std::vector<double>{p.x, p.y});
    urquhart::Observation obs(vectorOfTrees);
    
    // For now, I'm going to assume that the fixed reference frame of the keyframe will be taken from the first frame  
    if (kfObservations.empty()) {
        kfObservations.push_back(obs);
        // TODO add landmarks to dictionary
    } else {
        // TODO: make these vectors one layer deeper to store matches from the most recent observation to potentially each preceding frame
        std::vector<std::pair<size_t, size_t>> polygonMatches, triangleMatches;
        std::vector<std::pair<vecPtT, vecPtT>> pointMatches;
        // TODO store set of individual points in current frame that have been matched across previous frames 
        // TODO store vector of tf matrices between consecutive observations

        // TODO: loop over each preceeding frames if not enough landmarks have been matched in the current frame
        //       THIS MIGHT DEPEND ON FOREST DENSITY...
        // while (pointMatches.size() < 4) // -------------------------
        auto& prevObs = kfObservations.end()[-1];

        // Polygon Matching
        std::vector<size_t> refIds = obs.H->get_children(0), targIds = prevObs.H->get_children(0);
        matching::polygonMatching(obs, refIds, prevObs, targIds, 5, polygonMatches);
            
        // Triangle Matching
        for (auto pMatch : polygonMatches) {    // FIXME? make the loop explicitly over constant references?
            refIds = obs.H->get_children(pMatch.first);
            targIds = prevObs.H->get_children(pMatch.second);
            // TODO FROM URQ: ADD CHECK IF % OF TRIANGLES THAT MACTHED IS LARGER THAN 1/2
            matching::polygonMatching(obs, refIds, prevObs, targIds, 5, triangleMatches);
        }

        // Point Matching
        std::set<size_t> unique_matches;
        for (auto tMatch : triangleMatches) {   // FIXME? make the loop explicitly over constant references?
            urquhart::Polygon refTriangle = obs.H->get_vertex(tMatch.first), targetTriangle = prevObs.H->get_vertex(tMatch.second);
            matching::linePointMatching(refTriangle, targetTriangle, pointMatches, unique_matches);
        }
        // end loop ---------------------------------------------------

        // TODO call homography for the pairs of matched points
        // get the transform between this frame and the previous

        // TODO multiply existing matrix with the newly computed one to get the tf from this frame to the first
        // 3T0 = 3T2 (from above) 2T0 (og)

        // TODO for each landmark (id, pos) in the latest observation:
            // apply that full transform to get its position relative to the first frame
            // if ID not in dictionary, put it in with count=1
            // else, sum pos with whatever is in there, increment count 
    }

    // Finalize keyframe, send it to global code / backend for processing, then reset keyframe data structures
    int n = 5; // TODO get this value from a param
    if (kfObservations.size() >= n) {
        // TODO for each landmark, divide summed position by #times it was observed
        //      copy them (or something)
        // send the keyframe (set of 2D positions of landmarks) to the backend
        // (For now, I will expect the backend to do create the geometric hierarchy once it arrives)

        kfObservations.clear();
        // set matrix back to the identity matrix
        // clear dictionary
    }
}


// TODO implement keyframe width parameter (default should be 5)
int main(int argc, char **argv)
{
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