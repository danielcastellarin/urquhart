#include "geometry.hpp"
#include <vector>
#include <unordered_map>
#include <fstream>
#include <sstream>

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

// Code should error if invalid run configuration provided
struct SimConfig
{
    // Required Parameters
    std::vector<Tree> forest;
    std::string forestFile, pathType;
    double distanceToTravel, distanceBetweenObservations, observationRadius, 
    collisionRadius, landmarkAssociationThreshold, treeAssociationThreshold, 
    // Optional Parameters
    successfulObservationProbability = 1, treePositionStandardDeviation = 0, treeRadiusStandardDeviation = 0;
    int initializationAttempts = 10000, numObservationsForValidLandmark = 5, randomSeed = -1;
    bool givenStartPose = false;
    Pose initialPose;
    std::string outputDirName;

    // Each config file is designed to be a static run config
    SimConfig(std::string path)
    {
        // Assume each line is describes a key-value pair (space-separated)
        std::ifstream infile(path);
        std::string key,value;
        std::unordered_map<std::string, std::string> options;
        while (infile >> key >> value) options[key] = value;
        infile.close();

        // Required parameters
        forestFile = options["forestFile"];
        forest = readForestFile(forestFile);
        pathType = options["pathType"];
        distanceToTravel = atof(options["distanceToTravel"].c_str());
        distanceBetweenObservations = atof(options["distanceBetweenObservations"].c_str());
        observationRadius = atof(options["observationRange"].c_str()); // Note "Radius" and "Range"!
        collisionRadius = atof(options["collisionRadius"].c_str());
        landmarkAssociationThreshold = atof(options["landmarkAssociationThreshold"].c_str());
        treeAssociationThreshold = atof(options["treeAssociationThreshold"].c_str());

        // Optional parameters
        if (options.find("successfulObservationProbability") != options.end()) 
            successfulObservationProbability = atof(options["successfulObservationProbability"].c_str());
        if (options.find("treePositionStandardDeviation") != options.end())
            treePositionStandardDeviation = atof(options["treePositionStandardDeviation"].c_str());
        if (options.find("treeRadiusStandardDeviation") != options.end())
            treeRadiusStandardDeviation = atof(options["treeRadiusStandardDeviation"].c_str());
        if (options.find("startPoseX") != options.end() && options.find("startPoseY") != options.end() && options.find("startPoseTheta") != options.end()) {
            initialPose = Pose(atof(options["startPoseX"].c_str()), atof(options["startPoseY"].c_str()), atof(options["startPoseTheta"].c_str()));
            givenStartPose = true;
        }
        if (options.find("initializationAttempts") != options.end())
            initializationAttempts = atoi(options["initializationAttempts"].c_str());
        if (options.find("numObservationsForValidLandmark") != options.end())
            numObservationsForValidLandmark = atoi(options["numObservationsForValidLandmark"].c_str());
        if (options.find("randomSeed") != options.end())
            randomSeed = atoi(options["randomSeed"].c_str());
        if (options.find("outputDirName") != options.end() && options["outputDirName"].find_first_not_of("abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ01234567890_-") == std::string::npos)
            outputDirName = options["outputDirName"];
        else {
            int idx = forestFile.find("/");
            outputDirName = pathType + "-" + forestFile.substr(idx >= 0 ? idx+1 : 0, forestFile.find(".")-idx-1);
        }

    }
    bool isValid() { // TODO
        // successfulObservationProbability should be [0,1]
        // initialPose should be within bounding box of forest (requires forest file parsed though, also forest bounding box needs to be stored in its file, which it currently is not)
        // path type is a valid option
        // all other numeric inputs are non-negative
        return true;
    }
    void outputConfig(std::ostream& out) { 
        out << "Forest file used: " << forestFile << std::endl;
        out << "Path type: " << pathType << std::endl;
        out << "Distance to travel: " << distanceToTravel << std::endl;
        out << "Distance between observations: " << distanceBetweenObservations << std::endl;
        out << "Observation radius: " << observationRadius << std::endl;
        out << "Collision width: " << collisionRadius << std::endl;
        out << "Landmark association threshold: " << landmarkAssociationThreshold << std::endl;
        out << "Point association threshold: " << treeAssociationThreshold << std::endl;
        out << "Detection noise: " << successfulObservationProbability << std::endl;
        out << "Position noise: " << treePositionStandardDeviation << std::endl;
        out << "Tree radius noise: " << treeRadiusStandardDeviation << std::endl;
        out << "Maximum initialization attempts: " << initializationAttempts << std::endl;
        out << "Number of observations to validate a landmark: " << numObservationsForValidLandmark << std::endl;
        out << "Starting pose input: " << (givenStartPose ? initialPose.printPose() : "None") << std::endl;
    }
};