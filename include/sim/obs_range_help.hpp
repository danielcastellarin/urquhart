#include <geometry.hpp>
#include <ros/ros.h>
#include <ros/package.h>
#include <memory>
#include <random>
#include <unordered_map>
#include <queue>
#include <unordered_set>
#include <map>
#include <algorithm>
#include <fstream>

std::vector<Tree> readForestFile(std::string path) {
    // Assume each line in the file describes a point in 2D (space-separated)
    std::vector<Tree> forest;
    std::ifstream infile(path);
    double xPosition,yPosition,radius;

    std::string line;
    while (std::getline(infile, line))
    {
        std::istringstream iss(line);
        if (iss >> xPosition >> yPosition >> radius) { // only collect the tree data
            forest.push_back(Tree(xPosition, yPosition, radius));
        }
    }

    infile.close();
    return forest;
}

struct SimConfig
{
    std::string absolutePackagePath, outputDirPath, polygonsPath, polyPolyPath, polyTriPath, polyHierPath, polyDescPath, 
        localPointsPath, 
        matchesPath, matchPolyPath, matchTriPath, matchPointsPath;
    // Required Parameters
    std::vector<Tree> forest;
    std::string forestFile;
    std::vector<int> obsRanges;
    double collisionRadius, 
    // Optional Parameters
    successfulObservationProbability, treePositionStandardDeviation, treeRadiusStandardDeviation;
    int initializationAttempts, randomSeed;
    bool givenStartPose = false;
    Pose initialPose;
    std::string outputDirName;

    // Each config file is designed to be a static run config
    SimConfig(const ros::NodeHandle &nh) {

        absolutePackagePath = ros::package::getPath("urquhart") + "/";

        // Required parameters
        nh.param<std::string>("forestFile", forestFile, "coolForest.txt");
        forest = readForestFile(absolutePackagePath + forestFile);
        obsRanges = nh.param("observationRanges", std::vector<int>{10,20,40,80});   // Note "Radius" and "Range"! (naming changes from paramter to code)
        collisionRadius = nh.param("collisionRadius", 0.3);

        // Optional parameters
        successfulObservationProbability = nh.param("successfulObservationProbability", 1);
        treePositionStandardDeviation = nh.param("treePositionStandardDeviation", 0);
        treeRadiusStandardDeviation = nh.param("treeRadiusStandardDeviation", 0);
        if (nh.hasParam("startPoseX") && nh.hasParam("startPoseY") && nh.hasParam("startPoseTheta")) {
            double x, y, theta;
            nh.getParam("startPoseX", x), nh.getParam("startPoseY", y), nh.getParam("startPoseTheta", theta);
            initialPose = Pose(x, y, theta);
            givenStartPose = true;
        }
        initializationAttempts = nh.param("initializationAttempts", 10000);
        randomSeed = nh.param("randomSeed", -1);
        nh.param<std::string>("outputDirName", outputDirName, "testOutput");

        outputDirPath = absolutePackagePath+"output/"+outputDirName;
        localPointsPath = outputDirPath+"/local_obs/";

        polygonsPath = outputDirPath+"/poly/";
        polyPolyPath = polygonsPath+"p/";
        polyTriPath = polygonsPath+"t/";
        polyHierPath = polygonsPath+"h/";
        polyDescPath = polygonsPath+"d/";

        matchesPath = outputDirPath+"/match/";
        matchPolyPath = matchesPath+"polygonIDs/";
        matchTriPath = matchesPath+"triangleIDs/";
        matchPointsPath = matchesPath+"points/";
    }
    void outputConfig(std::ostream& out) { 
        out << "Forest file used: " << forestFile << std::endl;
        out << "Observation ranges: "; for (auto i: obsRanges) out << i << " ";
        out << std::endl << "Collision width: " << collisionRadius << std::endl;
        out << "Detection noise: " << successfulObservationProbability << std::endl;
        out << "Position noise: " << treePositionStandardDeviation << std::endl;
        out << "Tree radius noise: " << treeRadiusStandardDeviation << std::endl;
        out << "Maximum initialization attempts: " << initializationAttempts << std::endl;
        out << "Random seed: " << (randomSeed >= 0 ? std::to_string(randomSeed) : "None") << std::endl;
        out << "Starting pose input: " << (givenStartPose ? initialPose.printPose() : "None") << std::endl;
    }
};

struct Obs {
    int obsRange;
    Pose globalPose;
    std::vector<Tree> treePositions;
    Obs(int r, Pose pose) : obsRange(r), globalPose(pose) {}
};


class Robot {
    bool givenInitialPose;
    int initAttempts;
    std::vector<int> myObsRanges, myObsRangesSq;

    double successfulObservationProbability;
    double treePositionStandardDeviation;
    double treeRadiusStandardDeviation;

    std::random_device randomDevice;
    std::mt19937 rng;
    std::uniform_real_distribution<double> randX;
    std::uniform_real_distribution<double> randY;
    std::uniform_real_distribution<double> randTheta;
    std::uniform_real_distribution<double> detectionNoise;
    std::normal_distribution<double> positionNoise, treeRadiusNoise;

    // Assume an observation occurs every second
    // linear+angular velocity (magnitude and direction) will depend on path type
    // distBetweenObs can be repurposed for rotating at corners of routes

    public:
        std::vector<Tree> env;
        float collisionWidth;
        Pose globalPose;
        int currentObsIdx = 0;

        Robot(SimConfig cfg);
        Point findValidPoint();
        bool validateStartingPose();
        Obs observe();

        // default detection noise simply is a percentage chance asking "is tree detected?"
        // Potential improvement: detection noise can be proportional to gaussian (tree less likely to be seen further away from obs)
};