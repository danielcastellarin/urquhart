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



void parse2DPC(const sensor_msgs::PointCloud2ConstPtr& cloudMsg)
{
    // Take the points from the PointCloud2 message
    pcl::PointCloud<pcl::PointXY> localCloud;
    pcl::fromROSMsg(*cloudMsg, localCloud);
    int frameID = cloudMsg->header.seq;

    // Construct a PointVector for the given tree positions
    PointVector vectorOfTrees;
    for(const auto& p : localCloud) vectorOfTrees.push_back(std::vector<double>{p.x, p.y});
    urquhart::Observation obs(vectorOfTrees);
    
    // TODO: implement front-end "grace-period" to collect enough tree associations to build a global map
    // DOES THIS MEAN WE EXPECT THE ROBOT TO BE STATIONARY????

    // TODO: once global map is available, pass it to the backend for formal optimization

    // TODO: infer robot location from associations made from most recent observation and the global map

    // TODO: naively recompute the geometric hierarchy for the global representation every time we update map

    // RE-READ THE GRAPH-SLAM TUTORIAL PAPER TO REMEMBER HOW THE ALGO WORKS!
}


/**
 * This tutorial demonstrates simple sending of messages over the ROS system.
 */
int main(int argc, char **argv)
{
    // Initialize Node and read in private parameters
    ros::init(argc, argv, "forest_mapper");
    ros::NodeHandle n("~");
    // SimConfig cfg(n);
    // cfg.outputConfig(std::cout);
    // ros::Rate pub_rate(cfg.pubRate);    // 10Hz by default

    pcl::PointCloud<pcl::PointXY> localPC;
    ros::Subscriber sub = n.subscribe("local_points", 10, parse2DPC);

    ros::spin();


    return 0;
}