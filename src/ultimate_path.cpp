#include <path.hpp>
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
#include <geometry_msgs/Pose2D.h>

void writeObservationToFile(std::string filePath, const std::vector<Tree>& trees) {
    std::ofstream out(filePath+".txt");
    for (const Tree& t : trees) out << t.toString() << std::endl;
    out.close();
}

void publishObservation(const ros::Publisher& pub, const std::vector<Tree>& treePositions, int seqID, std::string frameName) {
    pcl::PointCloud<pcl::PointXY> localPC;
    for (const Tree& t : treePositions) {
        pcl::PointXY myPoint;
        myPoint.x = t.p.x; myPoint.y = t.p.y;
        localPC.push_back(myPoint);
    }

    sensor_msgs::PointCloud2 pc2_msg;
    pcl::toROSMsg(localPC, pc2_msg);
    pc2_msg.header.frame_id = frameName;
    pc2_msg.header.stamp = ros::Time::now();
    pc2_msg.header.seq = seqID;
    pub.publish(pc2_msg);
}



Path::Path(SimConfig cfg) : rng(randomDevice()) {
    env = cfg.forest;
    length = cfg.distanceToTravel;
    distBetweenObs = cfg.distanceBetweenObservations;
    collisionWidth = cfg.collisionRadius;
    obsRadius = cfg.observationRadius;
    obsRadiusSq = obsRadius*obsRadius;
    ldmkAssocThresh = cfg.landmarkAssociationThreshold;
    ptAssocThresh = cfg.treeAssociationThreshold;
    initAttempts = cfg.initializationAttempts;

    givenInitialPose = cfg.givenStartPose;
    globalPose = cfg.initialPose;
    landmarkWindowSize = cfg.numObservationsForValidLandmark;

    // Define randomizers for starting pose initialization in case valid pose not given
    // TODO use bounds provided by forest file
    if (cfg.randomSeed >= 0) rng.seed(cfg.randomSeed);
    double minX, maxX, minY, maxY;
    minX = minY = INT16_MAX; maxX = maxY = INT16_MIN;
    for (const Tree& t : env) {
        if (t.p.x < minX) minX = t.p.x;
        else if (t.p.x > maxX) maxX = t.p.x;
        if (t.p.y < minY) minY = t.p.y;
        else if (t.p.y > maxY) maxY = t.p.y;
    }
    randX = std::uniform_real_distribution<double>{minX, maxX};
    randY = std::uniform_real_distribution<double>{minY, maxY};
    randTheta = std::uniform_real_distribution<double>{-PI, PI};

    // Define randomizers for robot observations
    successfulObservationProbability = cfg.successfulObservationProbability;
    detectionNoise = std::uniform_real_distribution<double>{0, 1};
    positionNoise = std::normal_distribution<double>{0, cfg.treePositionStandardDeviation};
    treeRadiusNoise = std::normal_distribution<double>{0, cfg.treeRadiusStandardDeviation};
}

Point Path::findValidPoint() {    // TODO maybe exit early if cannot find valid point after certain threshold
    Point randomPoint;
    do { randomPoint = Point(randX(rng), randY(rng)); }
    while (!std::all_of(env.begin(), env.end(), Avoids(randomPoint, collisionWidth)));
    return randomPoint;
}

bool Path::validateStartingPose() {
    bool isPoseValid = givenInitialPose && isValid();
    for (int i = 0; i < initAttempts && !isPoseValid; ++i, isPoseValid = isValid()) {
        globalPose = Pose(findValidPoint(), randTheta(rng));
        std::cout << "Pose: " << globalPose.printPose() << "\n";
    }
    return isPoseValid;
}

SimKeyframe Path::observe() {
    SimKeyframe keyframe(currentObsIdx, globalPose);

    // The robot's base orientation is in +y direction
    // Rotate global points points by negative theta to align with robot's current frame
    double globSin = sin(-globalPose.theta), globCos = cos(globalPose.theta);
    double globalVisRange = obsRadius*1.5;

    // Observe trees within the robot's vicinity
    for (const Tree& t : env) {
        // Calculate the tree's position relative to the robot
        // Record the true position of the tree relative to the global frame
        double x = t.p.x - globalPose.p.x, y = t.p.y - globalPose.p.y, treeRadius = t.radius;
        if (x*x+y*y < obsRadiusSq) keyframe.globalTreePositions.push_back(t);
        if (std::abs(x) < globalVisRange && std::abs(y) < globalVisRange) keyframe.globalTreePositionsForVis.push_back(t);

        // If detection noise present, only a given percentage of potentially observable trees will actually be seen
        if (detectionNoise(rng) > successfulObservationProbability) continue;
        
        // If position noise present, the position of each tree will be perturbed prior to observation (same with their radii)
        x += positionNoise(rng), y += positionNoise(rng), treeRadius += treeRadiusNoise(rng);

        // If the tree is within the robot's observation range, rotate its position to its correct orientation (relative to the robot's current frame) 
        if (x*x + y*y < obsRadiusSq) keyframe.localTreePositions.push_back(Tree(x*globCos - y*globSin, x*globSin + y*globCos, treeRadius));
    }
    
    // Prepare for the next observation
    ++currentObsIdx;
    distance = getNextPoseOnPath(distance);
    if (distance >= length) keyframe.setLastFrame();
    return keyframe;
}


/**
 * This tutorial demonstrates simple sending of messages over the ROS system.
 */
int main(int argc, char **argv)
{
    // Initialize Node and read in private parameters
    ros::init(argc, argv, "sim_path");
    ros::NodeHandle n("~");
    SimConfig cfg(n);
    cfg.outputConfig(std::cout);
    ros::Rate pub_rate(cfg.pubRate);    // 10Hz by default
    bool isDebug = n.param("debug", true);

    // Initialize the path through the forest
    Path* robotPath;
    if (cfg.pathType == "circle") robotPath = new CirclePath(cfg);
    else if (cfg.pathType == "square") robotPath = new SquarePath(cfg);
    else if (cfg.pathType == "triangle") robotPath = new TrianglePath(cfg);
    else if (cfg.pathType == "line") robotPath = new LinePath(cfg);
    else if (cfg.pathType == "figure8") robotPath = new Figure8Path(cfg);
    else return -1;

    // Define publishers
    ros::Publisher gpPub = n.advertise<sensor_msgs::PointCloud2> ("global_points", 1);
    ros::Publisher lpPub = n.advertise<sensor_msgs::PointCloud2>("local_points", 1);
    ros::Publisher gPosePub = n.advertise<geometry_msgs::Pose2D>("global_pose", 1);

    // Create a directory to store the simulation data
    // std::filesystem::remove_all(cfg.outputDirPath);
    // std::filesystem::create_directories(cfg.globalPointsPath);
    // std::filesystem::create_directory(cfg.localPointsPath);
    // std::cout << "Created output directory: " << cfg.outputDirName << "\n";
    

    // Find a starting location and observe the environment
    if (robotPath->validateStartingPose()) {
        std::cout << "Robot will follow a " << cfg.pathType << " path starting at pose (x y theta): " << robotPath->globalPose.printPose() << "\n";
        int numFrames = 0;
        bool isFinished = false;
        while (ros::ok() && !isFinished) {
            SimKeyframe nextObs = robotPath->observe();
            numFrames = nextObs.id;
            if (isDebug) ROS_INFO("%d", numFrames);

            // global observation visualization hack
            std::string globalPoseHack = nextObs.globalPose.printPose() + " " + std::to_string(nextObs.localTreePositions.size());
            // NOTE: apparently PCL overwrites "seq" in message header...
            publishObservation(gpPub, nextObs.globalTreePositionsForVis, nextObs.id, globalPoseHack);
            // Publish data in ROS
            // publishObservation(gpPub, nextObs.globalTreePositions, nextObs.id, "global_frame");
            publishObservation(lpPub, nextObs.localTreePositions, nextObs.id, "sensor_frame");
            geometry_msgs::Pose2D gP;
            gP.x = nextObs.globalPose.p.x; gP.y = nextObs.globalPose.p.y; gP.theta = nextObs.globalPose.theta;
            gPosePub.publish(gP);


            // Save data to files
            if (cfg.isLogging) {
                writeObservationToFile(cfg.globalPointsPath+std::to_string(nextObs.id), nextObs.globalTreePositions);
                writeObservationToFile(cfg.localPointsPath+std::to_string(nextObs.id), nextObs.localTreePositions);
                std::ofstream gposeOut(cfg.outputDirPath+"/!gp.txt", std::ios_base::app);
                gposeOut << nextObs.globalPose.printPose() << " " << nextObs.globalPose.linearV << " " << nextObs.globalPose.angularV << std::endl;
                gposeOut.close();
            }

            // Prepare for next iteration
            isFinished = nextObs.isLast;
            ros::spinOnce();
            pub_rate.sleep();
        }

        // Also save this run's configuration
        if (cfg.isLogging) {
            std::ofstream configOut(cfg.outputDirPath+"/!config.txt");
            cfg.outputConfig(configOut);
            configOut << "Total observations: " << numFrames << std::endl;
            configOut.close();
        }

    } else {
        std::cout << "Could not find a valid starting position (random initialization attempts exceeded limit)." << std::endl;    
    }

    delete robotPath;

    return 0;
}