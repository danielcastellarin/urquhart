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


void constructGraph(const sensor_msgs::PointCloud2ConstPtr& cloudMsg) {
    // Take the points from the PointCloud2 message and create a geometric hierarchy from it
    pcl::PointCloud<pcl::PointXY> localCloud;
    pcl::fromROSMsg(*cloudMsg, localCloud);
    PointVector vectorOfTrees;
    for(const auto& p : localCloud) vectorOfTrees.push_back(std::vector<double>{p.x, p.y});
    urquhart::Observation obs(vectorOfTrees);

    // TODO do different things depending on whether the global map is available
    // TODO determine what gets done when the global map is unavailable (might also occur when keyframe does not match with anything on the map)
    //      maybe I could modify the matching procedure a bit to iteratively loosen the matching constraints if not enough matches are found?

    // When global map is present:
        // TODO perform landmark association between the keyframe and the global map
        // if last robot pose is available from the backend graph:
            // Use the polygons that are less than distance d away from that pose (implying that the global polygons are spatially organized for easy querying)
            // if not enough matches are made, then the search space could incrementally expand to include more polygons
        // otherwise, just use all the polygons from the global map

        // TODO Use RANSAC to estimate the current position of the robot with respect to the global reference frame

        // TODO Create graph nodes and edges:
            // node Xi : robot position at keyframe i
            // nodes for any newly observed landmarks
            // edge Xi --- Xi-1 (representing difference in robot position from this frame to the previous)
            // edges Xi === landmark nodes that were observed this keyframe (storing their observed distances relative to the robot in this keyframe)
        
        // TODO perform backend optimization stuff (maybe this will be its own node/service)
        // TODO global bundle adjustment (maybe this is done where the backend optimization does, send the new graph structure/geometric hierarchy back to this node)
}


int main(int argc, char **argv)
{
    // Initialize Node and read in private parameters
    ros::init(argc, argv, "graph_builder");
    ros::NodeHandle n("~");
    // SimConfig cfg(n);
    // cfg.outputConfig(std::cout);
    // ros::Rate pub_rate(cfg.pubRate);    // 10Hz by default

    pcl::PointCloud<pcl::PointXY> localPC;
    ros::Subscriber sub = n.subscribe("keyframe_points", 10, constructGraph);

    ros::spin();


    return 0;
}