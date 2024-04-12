#include <matching.hpp>
#include <logging.hpp>
#include <memory>
#include <random>
#include <iostream>
#include <unordered_map>
#include <queue>
#include <unordered_set>
#include <map>
#include <algorithm>
#include <iostream>

#include <vector>

// Code should error if invalid run configuration provided
struct OldSimConfig
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
    OldSimConfig(std::string path)
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

// IO stuff

std::string buildFilePath(std::string dirName, std::string fileName) { return (dirName.empty() ? "" : dirName+"/") + fileName; }
std::string buildDirectoryPath(std::string parentDir, std::string childDir) { return parentDir+(childDir.empty() ? "" : "/"+childDir); }

void writeHierarchyFiles(std::string fileName, const urquhart::Observation& trees, std::string dirName = "") {
    std::system(("mkdir -p " + dirName+"/p").c_str());
    std::system(("mkdir -p " + dirName+"/t").c_str());
    std::system(("mkdir -p " + dirName+"/h").c_str());
    std::system(("mkdir -p " + dirName+"/d").c_str());
    std::ofstream plyOut(buildFilePath(dirName+"/p", fileName)), triOut(buildFilePath(dirName+"/t", fileName)),
                    hieOut(buildFilePath(dirName+"/h", fileName)), dscOut(buildFilePath(dirName+"/d", fileName));

    logging::writeHierarchyToFile(trees, plyOut, triOut, hieOut, dscOut);

    hieOut.close();
    plyOut.close();
    triOut.close();
    dscOut.close();
}

void writeMatchesToDirectory(std::string dirName, 
                            std::map<int, std::vector<std::pair<size_t, size_t>>> polyMatches, 
                            std::map<int, std::vector<std::pair<size_t, size_t>>> triangleMatches, 
                            std::map<int, std::vector<std::pair<PtLoc, PtLoc>>> pointMatches) {
    std::system(("mkdir -p " + dirName+"/polygonIDs").c_str());
    std::system(("mkdir -p " + dirName+"/triangleIDs").c_str());
    std::system(("mkdir -p " + dirName+"/points").c_str());

    auto polyIt = polyMatches.begin(), triIt = triangleMatches.begin();
    auto pointIt = pointMatches.begin();

    while (polyIt != polyMatches.end()) {
        std::ofstream pMOut(buildFilePath(dirName+"/polygonIDs", std::to_string(polyIt->first)+".txt"));
        for (auto& pMatches : polyIt->second) pMOut << pMatches.first << "|" << pMatches.second << std::endl;
        pMOut.close();
        ++polyIt;
    }

    while (triIt != triangleMatches.end()) {
        std::ofstream tMOut(buildFilePath(dirName+"/triangleIDs", std::to_string(triIt->first)+".txt"));
        for (auto& tMatches : triIt->second) tMOut << tMatches.first << "|" << tMatches.second << std::endl;
        tMOut.close();
        ++triIt;
    }

    while (pointIt != pointMatches.end()) {
        std::ofstream ptMOut(buildFilePath(dirName+"/points", std::to_string(pointIt->first)+".txt"));
        for (auto& ptMatches : pointIt->second) ptMOut << ptMatches.first[0] << " " << ptMatches.first[1] << "|" << ptMatches.second[0] << " " << ptMatches.second[1] << std::endl;
        ptMOut.close();
        ++pointIt;
    }
}

void writePosesToFile(std::string fileName, const std::vector<Pose>& poses, std::string dirName = "") {
    std::ofstream out(buildFilePath(dirName, fileName));
    for (const Pose& pose : poses) out << pose.printPose() << " " << pose.linearV << " " << pose.angularV << std::endl;
    out.close();
}

void writeObservationsToDirectory(std::string dirName, const std::vector<std::vector<Tree>>& observations) {
    int count = 0;
    std::system(("mkdir -p " + dirName).c_str());
    for (const std::vector<Tree>& obs : observations)
        logging::writeObservationToFile(buildFilePath(dirName, std::to_string(count++)+".txt"), obs);
}

void writePolyObservationsToDirectory(std::string dirName, const std::vector<urquhart::Observation>& observations) {
    int count = 0;
    std::system(("mkdir -p " + dirName).c_str());
    for (const urquhart::Observation& obs : observations) writeHierarchyFiles(std::to_string(count++)+".txt", obs, dirName);
}

struct RobotHistory
{
    std::vector<std::vector<Tree>> globalFrameTreeHistory;
    std::vector<std::vector<Tree>> localFrameTreeHistory;
    std::vector<std::vector<Tree>> odomFrameTreeHistory;

    std::vector<urquhart::Observation> observationHistory;
    std::map<int, std::vector<std::pair<size_t, size_t>>> consecutivePolyMatches;
    std::map<int, std::vector<std::pair<size_t, size_t>>> consecutiveTriMatches;
    std::map<int, std::vector<std::pair<PtLoc, PtLoc>>> consecutivePointMatches;

    std::vector<Pose> globalFrameRobotPose;
    std::vector<Pose> odomFrameRobotPose;

    RobotHistory() {}
    void saveGlobalNearbyTrees(std::vector<Tree> trees) {globalFrameTreeHistory.push_back(trees);}
    void saveLocalObservation(std::vector<Tree> obs) {localFrameTreeHistory.push_back(obs);}
    void saveOdomObservation(std::vector<Tree> obs) {odomFrameTreeHistory.push_back(obs);}

    void savePolygonObservation(Points& trees, int obsIdx) {
        observationHistory.push_back(urquhart::Observation(trees));
        if (obsIdx) {
            // Polygon Matching
            auto& currentObs = observationHistory.end()[-1], prevObs = observationHistory.end()[-2];

            std::vector<std::pair<size_t, size_t>> polygonMatches, triangleMatches;

            // Polygon Matching (Level 2)
            matching::polygonMatching(currentObs, currentObs.hier->getChildrenIds(0), prevObs, prevObs.hier->getChildrenIds(0), 5, 3, 0, polygonMatches);
            consecutivePolyMatches[obsIdx] = polygonMatches;

            // Triangle Matching (Level 1)
            for (const auto& [refPoly, targPoly] : polygonMatches) {
                // TODO: ADD CHECK IF % OF TRIANGLES THAT MACTHED IS LARGER THAN 1/2
                matching::polygonMatching(currentObs, currentObs.hier->getChildrenIds(refPoly), prevObs, prevObs.hier->getChildrenIds(targPoly), 5, 3, 0.5, triangleMatches);
            }
            consecutiveTriMatches[obsIdx] = triangleMatches;

            // Vertex Matching (Level 0)
            std::vector<std::pair<PtLoc, PtLoc>> pointMatches; 
            for (const auto& [refIdx, targIdx] : matching::pointIndexMatching(currentObs, prevObs, triangleMatches)) {
                pointMatches.push_back({currentObs.landmarks.col(refIdx), prevObs.landmarks.col(targIdx)});
            }
            consecutivePointMatches[obsIdx] = pointMatches;
        }
    }

    void saveGlobalPose(Pose pose) {globalFrameRobotPose.push_back(pose);}
    void saveOdomPose(Pose pose) {odomFrameRobotPose.push_back(pose);}

    void recordHistory(std::string relativeDirectory = "") {
        writeObservationsToDirectory(buildDirectoryPath(relativeDirectory, "global_obs"), globalFrameTreeHistory);
        writeObservationsToDirectory(buildDirectoryPath(relativeDirectory, "local_obs"), localFrameTreeHistory);
        writeObservationsToDirectory(buildDirectoryPath(relativeDirectory, "odom_obs"), odomFrameTreeHistory);

        writePolyObservationsToDirectory(buildDirectoryPath(relativeDirectory, "poly"), observationHistory);
        writeMatchesToDirectory(buildDirectoryPath(relativeDirectory, "match"), consecutivePolyMatches, consecutiveTriMatches, consecutivePointMatches);

        writePosesToFile("!gp.txt", globalFrameRobotPose, relativeDirectory);
        writePosesToFile("!odom.txt", odomFrameRobotPose, relativeDirectory);
    }
};


// Path Definitions

class Path {
    float obsRadius;
    float obsRadiusSq;
    float ldmkAssocThresh;
    float ptAssocThresh;

    bool givenInitialPose;
    int initAttempts;
    int landmarkWindowSize;

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
        float length;
        float distBetweenObs;
        float collisionWidth;
        Pose globalPose;
        Pose odomPose;

        Path(OldSimConfig cfg) : rng(randomDevice()) {
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

            successfulObservationProbability = cfg.successfulObservationProbability;
            treePositionStandardDeviation = cfg.treePositionStandardDeviation;
            treeRadiusStandardDeviation = cfg.treeRadiusStandardDeviation;

            // Define randomizers for starting pose initialization in case valid pose not given
            // TODO use bounds provided by forest file
            if (cfg.randomSeed >= 0) rng.seed(cfg.randomSeed);
            double minX, maxX, minY, maxY;
            minX = minY = INT16_MAX; maxX = maxY = INT16_MIN;
            for (Tree t : env) {
                if (t.p.x < minX) minX = t.p.x;
                else if (t.p.x > maxX) maxX = t.p.x;
                if (t.p.y < minY) minY = t.p.y;
                else if (t.p.y > maxY) maxY = t.p.y;
            }
            randX = std::uniform_real_distribution<double>{minX, maxX};
            randY = std::uniform_real_distribution<double>{minY, maxY};
            randTheta = std::uniform_real_distribution<double>{-PI, PI};
        }

        // isValid() and getNextPoseOnPath() are implemented by children
        virtual bool isValid() { return true; }
        virtual double getNextPoseOnPath(double distanceTravelled) { return 0; }    // return value is the linear distance travelled

        Point findValidPoint() {    // TODO maybe exit early if cannot find valid point after certain threshold
            Point randomPoint;
            do { randomPoint = Point(randX(rng), randY(rng)); }
            while (!std::all_of(env.begin(), env.end(), Avoids(randomPoint, collisionWidth)));
            return randomPoint;
        }

        bool validateStartingPose() {
            bool isPoseValid = givenInitialPose && isValid();
            for (int i = 0; i < initAttempts && !isPoseValid; ++i, isPoseValid = isValid()) {
                globalPose = Pose(findValidPoint(), randTheta(rng));
                std::cout << "Pose: " << globalPose.printPose() << "\n";
            }
            return isPoseValid;
        }

        // default detection noise simply is a percentage chance asking "is tree detected?"
        // Potential improvement: detection noise can be proportional to gaussian (tree less likely to be seen further away from obs)
        
        // NOTE I am not randomly rotating the observations by [0,90] degrees like the original experiment
        RobotHistory observeAndAssociate() {
            RobotHistory robotHistory;

            // Define randomizers that represent detection and position noise
            std::uniform_real_distribution<double> detectionNoise(0, 1);
            std::normal_distribution<double> positionNoise(0, treePositionStandardDeviation), treeRadiusNoise(0, treeRadiusStandardDeviation);
            int currentObsIdx = 0;
            for (float distance = 0; distance < length; distance = getNextPoseOnPath(distance), ++currentObsIdx) {
                
                // Record the robot's global position and pose estimate
                robotHistory.saveGlobalPose(globalPose);
                robotHistory.saveOdomPose(odomPose);
                
                // Initialize new vectors to store observed trees in reference to the robot and global frames
                std::vector<Tree> globalFrameTrees;
                std::vector<Tree> localFrameTreeObservations;
                // Points localTrees(2, 1);
                Points localTrees; 
                std::vector<Tree> odomFrameTreeObservations;

                // The robot's base orientation is in +y direction
                // Rotate global points points by negative theta to align with robot's current frame
                double globSin = sin(-globalPose.theta), globCos = cos(globalPose.theta);
                // To rotate points into odometry frame -> must apply positive theta
                double odomSin = sin(odomPose.theta), odomCos = cos(-odomPose.theta);
                double globalVisRange = obsRadius*1.5;

                // ===============
                // Observing Trees
                // ===============

                // Observe trees within the robot's vicinity
                int localTreeCount=0;
                for (const Tree& t : env) {
                    // Calculate the tree's position relative to the robot
                    // Record the true position of the tree relative to the global frame (for visualization purposes)
                    double x = t.p.x - globalPose.p.x, y = t.p.y - globalPose.p.y, treeRadius = t.radius;
                    if (std::abs(x) < globalVisRange && std::abs(y) < globalVisRange) globalFrameTrees.push_back(t);

                    // If detection noise present, only a given percentage of potentially observable trees will actually be seen
                    if (detectionNoise(rng) > successfulObservationProbability) continue;
                    
                    // If position noise present, the position of each tree will be perturbed prior to observation (same with their radii)
                    x += positionNoise(rng), y += positionNoise(rng), treeRadius += treeRadiusNoise(rng);
                    if (x*x + y*y < obsRadiusSq) {
                        // Rotate the position of the tree so that it is oriented correctly (relative to the robot's current frame)
                        Tree localTree(x*globCos - y*globSin, x*globSin + y*globCos, treeRadius);
                        localFrameTreeObservations.push_back(localTree);
                        localTrees.conservativeResize(2, ++localTreeCount);
                        localTrees.col(localTreeCount-1) = Eigen::Vector2d{localTree.p.x, localTree.p.y};

                        // Translate the tree's position again to be relative to the robot's odometry
                        // In other words, find where this tree should lie on the robot's map
                        odomFrameTreeObservations.push_back(Tree(odomPose.p.x + (localTree.p.x*odomCos - localTree.p.y*odomSin), 
                                                                odomPose.p.y + (localTree.p.x*odomSin + localTree.p.y*odomCos),
                                                                treeRadius));
                        
                    }
                }

                // Add recordings for this observation to the robot history
                robotHistory.saveGlobalNearbyTrees(globalFrameTrees);
                robotHistory.saveLocalObservation(localFrameTreeObservations);
                robotHistory.saveOdomObservation(odomFrameTreeObservations);
                robotHistory.savePolygonObservation(localTrees, currentObsIdx);
            }
            return robotHistory;
        }
};

class CirclePath : public Path {
    Point globalCenter, localCenter;
    double radius;
    double thetaForHop;

    public:
        CirclePath(OldSimConfig cfg) : Path(cfg) {
            radius = cfg.distanceToTravel/(PI*2);
            thetaForHop = cfg.distanceBetweenObservations / radius;
            localCenter = Point(-radius, 0);
        }

        bool isValid() {
            // Define two circles (with same center): path radius +/- (robot width + tree radius)
            double innerPathRadius = radius - collisionWidth, outerPathRadius = radius + collisionWidth;

            // Test all points in env: return false if any point lies in big circle and not small circle
            globalCenter = Point(globalPose.p, -radius*cos(globalPose.theta), -radius*sin(globalPose.theta));
            for (const Tree& t : env) {
                double x = globalCenter.x - t.p.x, y = globalCenter.y - t.p.y, treeCenter = x*x + y*y,
                    innerBoundRadius = innerPathRadius-t.radius, outerBoundRadius = outerPathRadius+t.radius;
                if (treeCenter < outerBoundRadius*outerBoundRadius && treeCenter > innerBoundRadius*innerBoundRadius) return false;
            }
            return true;
        }
        
        double getNextPoseOnPath(double distanceTravelled) {
            // Global position update
            double newGlobalTheta = thetaForHop + globalPose.theta;
            globalPose.p.x = globalCenter.x + radius * cos(newGlobalTheta);
            globalPose.p.y = globalCenter.y + radius * sin(newGlobalTheta);
            globalPose.theta = newGlobalTheta;   // Pose theta should be theta of interior circle

            globalPose.linearV = distBetweenObs;
            globalPose.angularV = thetaForHop;
            
            // Local odometry update
            double newLocalTheta = thetaForHop + odomPose.theta;
            odomPose.p.x = localCenter.x + radius * cos(newLocalTheta); // Position should be relative to the origin...
            odomPose.p.y = localCenter.y + radius * sin(newLocalTheta);
            odomPose.theta = newLocalTheta;   // Pose theta should be theta of interior circle

            odomPose.linearV = distBetweenObs;
            odomPose.angularV = thetaForHop;

            // Return total distance travelled so far
            return distanceTravelled + distBetweenObs;
        }
};

class LinePath : public Path {
    double globalDeltaXPerObs, globalDeltaYPerObs;
    public:
        LinePath(OldSimConfig cfg) : Path(cfg) {}
        bool isValid() {
            globalDeltaXPerObs = distBetweenObs * cos(globalPose.theta+HALFPI), globalDeltaYPerObs = distBetweenObs * sin(globalPose.theta+HALFPI);
            return std::all_of(env.begin(), env.end(), NoStraightPathCollisions(globalPose, length, collisionWidth));
        }
        double getNextPoseOnPath(double distanceTravelled) { // theta will be constant as the robot moves forward
            // Global position update
            globalPose.p.translate(globalDeltaXPerObs, globalDeltaYPerObs);
            globalPose.linearV = distBetweenObs;
            
            // Local odometry update
            odomPose.p.y += distBetweenObs; // moving straight ahead; no turns -> no change in X
            odomPose.linearV = distBetweenObs;
            
            // Return total distance travelled so far
            return distanceTravelled + distBetweenObs;
        }
};

class SquarePath : public Path {
    double sideLen;
    Pose globalPositions[5];
    Pose localPositions[5];
    int stage = 0;
    bool isTurning = false;

    public:
        SquarePath(OldSimConfig cfg) : Path(cfg), sideLen(cfg.distanceToTravel/4) {
            localPositions[0] = odomPose; // Reminder that odom starts with pose (0,0,0)
            double nextAngle;
            // FIXME, nextangle should be computed after next position, current implementation works accidentally
            for (int i = 0; i<4; ++i) {
                nextAngle = localPositions[i].theta+HALFPI;
                localPositions[i+1] = Pose(Point(localPositions[i].p, sideLen * cos(nextAngle), sideLen * sin(nextAngle)), nextAngle);
            }
        }

        bool isValid() {
            // Define 4 lines, each endpoint at startpoint of next segment rotated 90 counter-clockwise
            double nextAngle;
            globalPositions[0] = globalPose;
            // std::cout << "Pose 1: " << globalPositions[0].printPose() << "\n";
            for (int i = 0; i<4; i++) {
                if (!std::all_of(env.begin(), env.end(), NoStraightPathCollisions(globalPositions[i], sideLen, collisionWidth))) return false;    
                nextAngle = globalPositions[i].theta+HALFPI;
                globalPositions[i+1] = Pose(Point(globalPositions[i].p, sideLen * cos(nextAngle), sideLen * sin(nextAngle)), nextAngle);
                // std::cout << "Pose "<< i+2 << ": " << globalPositions[i+1].printPose() << "\n";
            }
            return true;
        }

        double forward(float distanceTravelled, float remainingDistance) {
            if (distanceTravelled+remainingDistance > sideLen*(1+stage)) { // start turning
                isTurning = true;
                double extra = remainingDistance + distanceTravelled - sideLen*(1+stage);

                // Global position update
                globalPose.p = Point(globalPositions[stage+1].p);
                globalPose.linearV += remainingDistance - extra;

                // Local odometry update
                odomPose.p = Point(localPositions[stage+1].p);
                odomPose.linearV += remainingDistance - extra;
                
                return turn(sideLen*(1+stage), extra);
            }
            // Global position update
            double globalAngle = globalPose.theta+HALFPI;
            globalPose.p.translate(remainingDistance * cos(globalAngle), remainingDistance * sin(globalAngle));
            globalPose.linearV = remainingDistance;

            // Local odometry update
            double localAngle = odomPose.theta+HALFPI;
            odomPose.p.translate(remainingDistance * cos(localAngle), remainingDistance * sin(localAngle));
            odomPose.linearV = remainingDistance;

            return distanceTravelled + remainingDistance;
        }

        double turn(float distanceTravelled, float remainingDistance) {
            if (globalPose.theta+remainingDistance > (globalPositions[stage+1].theta)) { // start moving
                isTurning = false, ++stage;
                double extra = remainingDistance + globalPose.theta - globalPositions[stage].theta;
                // extra~remainder -> should be identical in global & local perspectives 

                // Global position update
                globalPose.theta = globalPositions[stage].theta;
                globalPose.angularV += remainingDistance - extra;

                // Local odometry update
                odomPose.theta = localPositions[stage].theta;
                odomPose.angularV += remainingDistance - extra;

                return forward(sideLen*stage, extra);
            }
            // Global position update
            globalPose.theta += remainingDistance;
            globalPose.angularV += remainingDistance;

            // Local odometry update
            odomPose.theta += remainingDistance;
            odomPose.angularV += remainingDistance;

            return distanceTravelled;
        }
        
        double getNextPoseOnPath(double distanceTravelled) {
            // Reset local and global velocity
            globalPose.linearV = 0, globalPose.angularV = 0;
            odomPose.linearV = 0, odomPose.angularV = 0;
            return isTurning ? turn(distanceTravelled, distBetweenObs) : forward(distanceTravelled, distBetweenObs);
        }
};

class TrianglePath : public Path {
    double sideLen;
    Pose globalPositions[4];
    Pose localPositions[4];
    int stage = 0;
    double turnAmount = 2*PI/3;
    bool isTurning = false;

    public:
        TrianglePath(OldSimConfig cfg) : Path(cfg), sideLen(cfg.distanceToTravel/3) {
            localPositions[0] = odomPose; // Reminder that odom starts with pose (0,0,0)
            double nextAngle, prevAngleAdj = HALFPI;
            for (int i = 0; i<3; ++i) {
                nextAngle = localPositions[i].theta+turnAmount;
                localPositions[i+1] = Pose(Point(localPositions[i].p, sideLen * cos(prevAngleAdj), sideLen * sin(prevAngleAdj)), nextAngle);
                prevAngleAdj += turnAmount;
            }
        }

        bool isValid() {
            // Define 3 lines, each endpoint at startpoint of next segment rotated 120 counter-clockwise
            globalPositions[0] = globalPose;
            double nextAngle, prevAngleAdj = globalPositions[0].theta+HALFPI;

            for (int i = 0; i<3; i++) { // Ensure no points lie within each line segment
                if (!std::all_of(env.begin(), env.end(), NoStraightPathCollisions(globalPositions[i], sideLen, collisionWidth))) return false;    
                nextAngle = globalPositions[i].theta+turnAmount;
                globalPositions[i+1] = Pose(Point(globalPositions[i].p, sideLen * cos(prevAngleAdj), sideLen * sin(prevAngleAdj)), nextAngle);
                prevAngleAdj += turnAmount;
            }
            return true;
        }

        double forward(float distanceTravelled, float remainingDistance) {
            if (distanceTravelled+remainingDistance > sideLen*(1+stage)) { // start turning
                isTurning = true;
                double extra = remainingDistance + distanceTravelled - sideLen*(1+stage);

                // Global position update
                globalPose.p = Point(globalPositions[stage+1].p);
                globalPose.linearV += remainingDistance - extra;

                // Local odometry update
                odomPose.p = Point(localPositions[stage+1].p);
                odomPose.linearV += remainingDistance - extra;
                
                return turn(sideLen*(1+stage), extra);
            }
            // Global position update
            double globalAngle = globalPose.theta+HALFPI;
            globalPose.p.translate(remainingDistance * cos(globalAngle), remainingDistance * sin(globalAngle));
            globalPose.linearV = remainingDistance;

            // Local odometry update
            double localAngle = odomPose.theta+HALFPI;
            odomPose.p.translate(remainingDistance * cos(localAngle), remainingDistance * sin(localAngle));
            odomPose.linearV = remainingDistance;

            return distanceTravelled + remainingDistance;
        }

        double turn(float distanceTravelled, float remainingDistance) {
            if (globalPose.theta+remainingDistance > (globalPositions[stage+1].theta)) { // start moving
                isTurning = false, ++stage;
                double extra = remainingDistance + globalPose.theta - globalPositions[stage].theta;
                // extra~remainder -> should be identical in global & local perspectives 

                // Global position update
                globalPose.theta = globalPositions[stage].theta;
                globalPose.angularV += remainingDistance - extra;

                // Local odometry update
                odomPose.theta = localPositions[stage].theta;
                odomPose.angularV += remainingDistance - extra;

                return forward(sideLen*stage, extra);
            }
            // Global position update
            globalPose.theta += remainingDistance;
            globalPose.angularV += remainingDistance;

            // Local odometry update
            odomPose.theta += remainingDistance;
            odomPose.angularV += remainingDistance;

            return distanceTravelled;
        }
        
        double getNextPoseOnPath(double distanceTravelled) {
            // Reset local and global velocity
            globalPose.linearV = 0, globalPose.angularV = 0;
            odomPose.linearV = 0, odomPose.angularV = 0;
            return isTurning ? turn(distanceTravelled, distBetweenObs) : forward(distanceTravelled, distBetweenObs);
        }
};

class Figure8Path : public Path {
    Pose initGlobalPose;
    Point globalLeftCenter, globalRightCenter, localLeftCenter, localRightCenter;
    double radius, singleCircleCircumference, thetaForHop;
    int stage = 0;

    public:
        Figure8Path(OldSimConfig cfg) : Path(cfg) {
            singleCircleCircumference = cfg.distanceToTravel/2; // 2 circles, half the distance to travel per circle
            radius = singleCircleCircumference/(TWOPI); 
            thetaForHop = cfg.distanceBetweenObservations / radius;
            localLeftCenter = Point(-radius, 0), localRightCenter = Point(radius, 0);
        }

        bool isValid() {
            // Define two circles (with same center): path radius +/- (robot width + tree radius)
            double innerPathRadius = radius - collisionWidth, outerPathRadius = radius + collisionWidth;

            // Test all points in env: return false if any point lies in (left or right) big circle and not small circle
            globalLeftCenter = Point(globalPose.p, -radius*cos(globalPose.theta), -radius*sin(globalPose.theta));
            globalRightCenter = Point(globalPose.p, radius*cos(globalPose.theta), radius*sin(globalPose.theta));
            for (const Tree& t : env) {
                double lX = globalLeftCenter.x - t.p.x, lY = globalLeftCenter.y - t.p.y, distFromLeftCenter = lX*lX + lY*lY;
                double rX = globalRightCenter.x - t.p.x, rY = globalRightCenter.y - t.p.y, distFromRightCenter = rX*rX + rY*rY;
                double innerBoundRadius = innerPathRadius-t.radius, outerBoundRadius = outerPathRadius+t.radius;
                if ((distFromLeftCenter < outerBoundRadius*outerBoundRadius && distFromLeftCenter > innerBoundRadius*innerBoundRadius) || 
                (distFromRightCenter < outerBoundRadius*outerBoundRadius && distFromRightCenter > innerBoundRadius*innerBoundRadius)) return false;
            }
            initGlobalPose = globalPose;
            return true;
        }
        
        double travel(float distanceTravelled, float remainingDistance) {
            if (distanceTravelled+remainingDistance > singleCircleCircumference*(1+stage)) { // begin travelling on other circle

                // Reset global and odom position
                globalPose = initGlobalPose, odomPose = Pose();
                
                double extra = remainingDistance + distanceTravelled - singleCircleCircumference*(++stage);
                return travel(singleCircleCircumference*stage, extra);
            }
            // Update global and odom position
            // applyPoseUpdate(globalPose, stage ? globalRightCenter : globalLeftCenter);
            // applyPoseUpdate(odomPose, stage ? localRightCenter : localLeftCenter);
            Point globalCenter = stage ? globalRightCenter : globalLeftCenter;
            double newGlobalTheta = globalPose.theta + thetaForHop*(stage?-1:1);
            globalPose.p.x = globalCenter.x + radius * cos(newGlobalTheta) * (stage?-1:1);
            globalPose.p.y = globalCenter.y + radius * sin(newGlobalTheta) * (stage?-1:1);
            globalPose.theta = newGlobalTheta;   // Pose theta should be theta of interior circle

            Point localCenter = stage ? localRightCenter : localLeftCenter;
            double newLocalTheta = odomPose.theta + thetaForHop*(stage?-1:1);
            odomPose.p.x = localCenter.x + radius * cos(newLocalTheta) * (stage?-1:1); // Position should be relative to the origin...
            odomPose.p.y = localCenter.y + radius * sin(newLocalTheta) * (stage?-1:1);
            odomPose.theta = newLocalTheta;   // Pose theta should be theta of interior circle

            return distanceTravelled + remainingDistance;
        }

        void applyPoseUpdate(Pose &pose, Point &circleCenter) {
            double newTheta = thetaForHop + pose.theta;
            pose.p.x = circleCenter.x + radius * cos(newTheta);
            pose.p.y = circleCenter.y + radius * sin(newTheta);
            pose.theta = newTheta;   // Pose theta should be theta of interior circle

            pose.linearV = distBetweenObs;
            pose.angularV = thetaForHop;
        }

        double getNextPoseOnPath(double distanceTravelled) {
            // Reset local and global velocity
            globalPose.linearV = globalPose.angularV = distBetweenObs;
            odomPose.linearV = odomPose.angularV = thetaForHop;
            return travel(distanceTravelled, distBetweenObs);
        }
};

// main exec
int main(int argc, char* argv[]) {
    if (argc > 1) {

        // Parse config file and display this run's configs
        OldSimConfig config(argv[1]);
        config.outputConfig(std::cout);

        // Initialize the path through the forest 
        Path* robotPath;
        if (config.pathType == "circle") robotPath = new CirclePath(config);
        else if (config.pathType == "square") robotPath = new SquarePath(config);
        else if (config.pathType == "triangle") robotPath = new TrianglePath(config);
        else if (config.pathType == "line") robotPath = new LinePath(config);
        else if (config.pathType == "figure8") robotPath = new Figure8Path(config);
        else return -1;

        // Find a starting location and observe the environment
        if (robotPath->validateStartingPose()) {
            std::cout << "Robot will follow a " << config.pathType << " path starting at pose (x y theta): " << robotPath->globalPose.printPose() << "\n";
            RobotHistory history = robotPath->observeAndAssociate();

            // Create a directory to store the simulation data
            std::system(("rm -r output/"+config.outputDirName).c_str());
            std::system(("mkdir -p output/"+config.outputDirName).c_str());
            std::cout << "Created output directory: " << config.outputDirName << "\n";

            // Write observations and odometry to files
            history.recordHistory("output/"+config.outputDirName);

            // Also save this run's configuration
            std::ofstream configOut("output/"+buildFilePath(config.outputDirName, "!config.txt"));
            config.outputConfig(configOut);
            configOut << "Total observations: " << history.globalFrameRobotPose.size() << std::endl;
            configOut << "\nRUN COMMAND:";
            for (int i = 0; i < argc; i++) configOut << " " << argv[i];
            configOut.close();

        } else {
            std::cout << "Could not find a valid starting position (random initialization attempts exceeded limit)." << std::endl;    
        }

        delete robotPath;
        
    } else {
        std::cout << "Please provide a configuration file with run parameters for the simulation." << std::endl;
    }
}