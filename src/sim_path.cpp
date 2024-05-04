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
#include <std_msgs/String.h>

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
    initAttempts = cfg.initializationAttempts;

    givenInitialPose = cfg.givenStartPose;
    globalPose = cfg.initialPose;

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

bool Path::validateStartingPose(bool isDebug) {
    bool isPoseValid = givenInitialPose && isValid();
    for (int i = 0; i < initAttempts && !isPoseValid; ++i, isPoseValid = isValid()) {
        globalPose = Pose(findValidPoint(), randTheta(rng));
        if (isDebug) std::cout << "Checking if (" << globalPose.printPose() << ") is a valid starting pose\n";
    }
    return isPoseValid;
}

SimKeyframe Path::observe() {
    SimKeyframe keyframe(currentObsIdx, globalPose, odomPose);

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

void Path::resetPath() {
    currentObsIdx = 0;
    distance = 0;
    odomPose = Pose();
    givenInitialPose = false;
}


/**
 * This tutorial demonstrates simple sending of messages over the ROS system.
 */
int main(int argc, char **argv) {
    // Initialize Node and read in private parameters
    ros::init(argc, argv, "sim_path");
    ros::NodeHandle n("~");

    // I/O parameters
    std::string absolutePackagePath = ros::package::getPath("urquhart");
    std::string outputPath;
    n.param<std::string>("/outputDirName", outputPath, "GH_OUT");
    outputPath = absolutePackagePath+"/output/"+outputPath;
    int totalRuns = n.param("/totalRuns", 1);
    if (totalRuns < 1) totalRuns = 1;
    bool isOffline = n.param("/offline", false) || totalRuns > 1;
    bool isLogging = n.param("/logging", true);


    SimConfig cfg(n);
    cfg.outputConfig(std::cout);
    ros::Rate pub_rate(cfg.pubRate);    // 10Hz by default
    bool isDebug = n.param("debug", true);
    

    // Define publishers
    ros::Publisher gpPub, lpPub, gPosePub, donePub;
    std_msgs::String doneMsg;
    if (!isOffline) {
        gpPub = n.advertise<sensor_msgs::PointCloud2> ("global_points", 1);
        lpPub = n.advertise<sensor_msgs::PointCloud2>("local_points", 1);
        gPosePub = n.advertise<geometry_msgs::Pose2D>("global_pose", 1);
    } else donePub = n.advertise<std_msgs::String>("doneFlag", 10);


    // Initialize the path through the forest
    Path* robotPath;
    if (cfg.pathType == "circle") robotPath = new CirclePath(cfg);
    else if (cfg.pathType == "square") robotPath = new SquarePath(cfg);
    else if (cfg.pathType == "triangle") robotPath = new TrianglePath(cfg);
    else if (cfg.pathType == "line") robotPath = new LinePath(cfg);
    else if (cfg.pathType == "figure8") robotPath = new Figure8Path(cfg);
    else return -1;


    std::string outputLocation = outputPath;
    for (int numRuns = 0; numRuns < totalRuns; ++numRuns) {
        if (totalRuns > 1) outputLocation = outputPath + "-" + std::to_string(numRuns+1);

        // Prepare output directory for logs 
        if (isLogging) {
            if (isDebug) std::cout << "Creating output directory: '" << outputLocation << "' ... ";
            std::filesystem::remove_all(outputLocation);
            std::filesystem::create_directory(outputLocation);
            std::filesystem::create_directory(outputLocation+"/global");
            std::filesystem::create_directory(outputLocation+"/global/p");
            std::filesystem::create_directory(outputLocation+"/global/d");
            std::filesystem::create_directory(outputLocation+"/global/t");
            std::filesystem::create_directory(outputLocation+"/global/h");
            std::filesystem::create_directory(outputLocation+"/global/graph_nodes");
            std::filesystem::create_directory(outputLocation+"/global/graph_edges");
            // std::filesystem::create_directory(outputLocation+"/global/err");
            std::filesystem::create_directory(outputLocation+"/keyframe");
            std::filesystem::create_directory(outputLocation+"/allKfPts");
            std::filesystem::create_directory(outputLocation+"/local");
            std::filesystem::create_directory(outputLocation+"/local/p");
            std::filesystem::create_directory(outputLocation+"/local/d");
            std::filesystem::create_directory(outputLocation+"/local/t");
            std::filesystem::create_directory(outputLocation+"/local/h");
            std::filesystem::create_directory(outputLocation+"/local/pts");
            std::filesystem::create_directory(outputLocation+"/match");
            std::filesystem::create_directory(outputLocation+"/finalAssoc");
            std::filesystem::create_directory(outputLocation+"/global_obs");
            std::filesystem::create_directory(outputLocation+"/local_obs");
            if (isDebug) std::cout << "   Done!" << std::endl;
        }

        // Find a starting location and observe the environment
        if (robotPath->validateStartingPose(isDebug)) {
            if (isDebug) std::cout << "Robot will follow a " << cfg.pathType << " path starting at pose (x y theta): " << robotPath->globalPose.printPose() << "\n";

            // Also save this run's configuration
            if (isLogging) {
                std::ofstream configOut(outputLocation+"/!config.txt");
                cfg.outputConfig(configOut);
                configOut << "Publish Rate: " << (isOffline ? "OFFLINE" : std::to_string(cfg.pubRate)+ " Hz") << std::endl;
                configOut << "Starting Pose: " << robotPath->globalPose.printPose() << std::endl;
                configOut.close();
            }

            bool isFinished = false;
            while (ros::ok() && !isFinished) {
                SimKeyframe nextObs = robotPath->observe();
                // if (isDebug) ROS_INFO("%d", nextObs.id);

                if (!isOffline) {
                    // NOTE: apparently PCL overwrites "seq" in message header...
                    // hack the "frame" header to visualize global observations easier
                    std::string globalPoseHack = nextObs.globalPose.printPose() + " " + std::to_string(nextObs.localTreePositions.size());
                    publishObservation(gpPub, nextObs.globalTreePositionsForVis, nextObs.id, globalPoseHack);
                    publishObservation(lpPub, nextObs.localTreePositions, nextObs.id, "sensor_frame");
                    
                    geometry_msgs::Pose2D gP;
                    gP.x = nextObs.globalPose.p.x; gP.y = nextObs.globalPose.p.y; gP.theta = nextObs.globalPose.theta;
                    gPosePub.publish(gP);

                    // stall at expected rate
                    ros::spinOnce();
                    pub_rate.sleep();
                }


                // Save data to files
                if (isLogging) {
                    writeObservationToFile(outputLocation+"/global_obs/"+std::to_string(nextObs.id), nextObs.globalTreePositions);
                    writeObservationToFile(outputLocation+"/local_obs/"+std::to_string(nextObs.id), nextObs.localTreePositions);

                    std::ofstream gposeOut(outputLocation+"/!gp.txt", std::ios_base::app);
                    gposeOut << nextObs.globalPose.printPose() << " " << nextObs.globalPose.linearV << " " << nextObs.globalPose.angularV << std::endl;
                    gposeOut.close();

                    std::ofstream odomOut(outputLocation+"/!odom.txt", std::ios_base::app);
                    odomOut << nextObs.odomPose.printPose() << " " << nextObs.globalPose.linearV << " " << nextObs.globalPose.angularV << std::endl;
                    odomOut.close();
                }

                // Prepare for next iteration
                isFinished = nextObs.isLast;
            }

            if (isOffline) {
                doneMsg.data = outputLocation;
                donePub.publish(doneMsg);
            }

        } else {
            if (isDebug) std::cout << "Could not find a valid starting position (random initialization attempts exceeded limit)." << std::endl;
            --numRuns;
        }
        if (isDebug) std::cout << "+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++" << std::endl;
        robotPath->resetPath();
    }

    // Signal downstream processes to shutdown
    if (isOffline) {
        doneMsg.data = "shutdown";
        donePub.publish(doneMsg);
    }

    ros::Duration(5).sleep(); // sleep for a little to make sure all published messages leave
    delete robotPath;
    if (isDebug) std::cout << "Successfully simulated " << totalRuns << " run(s)." << std::endl;
    
    return 0;
}