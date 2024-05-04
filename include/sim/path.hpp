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

struct SimConfig
{
    std::string absolutePackagePath, outputDirPath, globalPointsPath, localPointsPath;
    // Required Parameters
    std::vector<Tree> forest;
    std::string forestFile, pathType;
    double distanceToTravel, distanceBetweenObservations, observationRadius, collisionRadius, 
    // Optional Parameters
    successfulObservationProbability, treePositionStandardDeviation, treeRadiusStandardDeviation,
    pubRate;
    int initializationAttempts, randomSeed;
    bool givenStartPose = false, isLogging = false, favorableSpawns = false;
    Pose initialPose;
    std::string outputDirName;

    // Each config file is designed to be a static run config
    SimConfig(const ros::NodeHandle &nh) {

        absolutePackagePath = ros::package::getPath("urquhart") + "/";

        // Required parameters
        nh.param<std::string>("forestFile", forestFile, "coolForest.txt");
        forest = readForestFile(absolutePackagePath + "forests/" + forestFile);
        nh.param<std::string>("pathType", pathType, "circle");
        distanceToTravel = nh.param("distanceToTravel", 100);
        distanceBetweenObservations = nh.param("distanceBetweenObservations", 0.2);
        observationRadius = nh.param("observationRange", 15);   // Note "Radius" and "Range"! (naming changes from paramter to code)
        collisionRadius = nh.param("collisionRadius", 0.3);

        // Optional parameters
        successfulObservationProbability = nh.param("successfulObservationProbability", 1.0);
        treePositionStandardDeviation = nh.param("treePositionStandardDeviation", 0.0);
        treeRadiusStandardDeviation = nh.param("treeRadiusStandardDeviation", 0.0);
        if (nh.hasParam("startPoseX") && nh.hasParam("startPoseY") && nh.hasParam("startPoseTheta")) {
            double x, y, theta;
            nh.getParam("startPoseX", x), nh.getParam("startPoseY", y), nh.getParam("startPoseTheta", theta);
            initialPose = Pose(x, y, theta);
            givenStartPose = true;
        }
        pubRate = nh.param("pubRate", 10.0); // Hz
        initializationAttempts = nh.param("initializationAttempts", 10000);
        randomSeed = nh.param("randomSeed", -1);
        favorableSpawns = nh.param("favorableSpawns", false);

        // Setup logging (if necessary)
        isLogging = nh.param("/logging", false);
        nh.param<std::string>("/outputDirName", outputDirName, "testOutput");
        outputDirPath = absolutePackagePath+"output/"+outputDirName;
        globalPointsPath = outputDirPath+"/global_obs/";
        localPointsPath = outputDirPath+"/local_obs/";
    }
    void outputConfig(std::ostream& out) { 
        out << "Forest file used: " << forestFile << std::endl;
        out << "Path type: " << pathType << std::endl;
        out << "Distance to travel: " << distanceToTravel << std::endl;
        out << "Distance between observations: " << distanceBetweenObservations << std::endl;
        out << "Observation radius: " << observationRadius << std::endl;
        out << "Collision width: " << collisionRadius << std::endl;
        out << "Detection noise: " << successfulObservationProbability << std::endl;
        out << "Position noise: " << treePositionStandardDeviation << std::endl;
        out << "Tree radius noise: " << treeRadiusStandardDeviation << std::endl;
        out << "Maximum initialization attempts: " << initializationAttempts << std::endl;
        out << "Random seed: " << (randomSeed >= 0 ? std::to_string(randomSeed) : "None") << std::endl;
    }
};

struct SimKeyframe {
    int id;
    Pose globalPose, odomPose;
    std::vector<Tree> globalTreePositions, globalTreePositionsForVis, localTreePositions;
    bool isLast = false;
    SimKeyframe(int obsId, Pose gPose, Pose oPose) : id(obsId), globalPose(gPose), odomPose(oPose) {}
    void setLastFrame() { isLast = true; }
};


class Path {
    float obsRadius;
    float obsRadiusSq;

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

    int currentObsIdx = 0;
    float distance = 0;

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

        Path(SimConfig cfg);

        // isValid() and getNextPoseOnPath() are implemented by children
        // virtual bool isValid() { return true; }
        // virtual double getNextPoseOnPath(double distanceTravelled) { return 0; }    // return value is the linear distance travelled
        virtual bool isValid()=0;
        virtual double getNextPoseOnPath(double distanceTravelled)=0;    // return value is the linear distance travelled
        Point findValidPoint();
        bool validateStartingPose(bool isDebug);
        SimKeyframe observe();
        void resetPath();

        // default detection noise simply is a percentage chance asking "is tree detected?"
        // Potential improvement: detection noise can be proportional to gaussian (tree less likely to be seen further away from obs)
};

class CirclePath : public Path {
    Point globalCenter, localCenter;
    double radius;
    double thetaForHop;

    public:
        CirclePath(SimConfig cfg) : Path(cfg) {
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
        LinePath(SimConfig cfg) : Path(cfg) {}
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
        SquarePath(SimConfig cfg) : Path(cfg), sideLen(cfg.distanceToTravel/4) {
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
        TrianglePath(SimConfig cfg) : Path(cfg), sideLen(cfg.distanceToTravel/3) {
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
        Figure8Path(SimConfig cfg) : Path(cfg) {
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
