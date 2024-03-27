#include <obs_range_help.hpp>
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

// IO stuff

void writeObservationToFile(std::string filePath, const std::vector<Tree>& trees) {
    std::ofstream out(filePath+".txt");
    for (const Tree& t : trees) out << t.toString() << std::endl;
    out.close();
}

void writeHierarchyToFile(SimConfig cfg, const urquhart::Observation& trees, std::string fileName) {
    std::ofstream plyOut(cfg.polyPolyPath+fileName), triOut(cfg.polyTriPath+fileName), 
                    hieOut(cfg.polyHierPath+fileName), dscOut(cfg.polyDescPath+fileName);
    write_graphviz(hieOut, trees.H->graph);
    hieOut.close();
    
    // Iterate over the indices of the Polygons in the hierarchy
    for(auto pIdx : trees.H->get_children(trees.H->root)) {
        for (int i = 0; i < trees.H->graph[pIdx].landmarkRefs.size(); ++i) {
            auto myPoint =  trees.landmarks.col(trees.H->graph[pIdx].landmarkRefs(i));
            plyOut << pIdx << " " << myPoint[0] << " " << myPoint[1] << "|";
        }
        plyOut << std::endl;
        for(auto d : trees.H->graph[pIdx].descriptor) dscOut << d << " ";
        dscOut << std::endl;
        
        // Iterate over the indices of the Triangles that compose this Polygon
        for(auto tIdx : trees.H->traverse(pIdx)) {
            // Retain only the Polygon objects that have three sides
            if (trees.H->graph[tIdx].n == 3) {
                for (int i = 0; i < trees.H->graph[tIdx].landmarkRefs.size(); ++i) {
                    auto myPoint = trees.landmarks.col(trees.H->graph[tIdx].landmarkRefs(i));
                    triOut << tIdx << " " << myPoint[0] << " " << myPoint[1] << "|";
                }
                triOut << std::endl;
            }
        }
    }
    plyOut.close();
    triOut.close();
    dscOut.close();
}

void matchObs(const urquhart::Observation &ref, const urquhart::Observation &targ, double polyMatchThresh, double validPointMatchThresh,
            std::vector<std::pair<size_t, size_t>> &polygonMatches, std::vector<std::pair<size_t, size_t>> &triangleMatches, std::vector<std::pair<PtLoc, PtLoc>> &vertexMatches) {

    // Polygon Matching (Level 2)
    matching::polygonMatching(ref, ref.H->get_children(0), targ, targ.H->get_children(0), polyMatchThresh, polygonMatches);

    // Triangle Matching (Level 1)
    for (auto pMatch : polygonMatches) {
        // TODO: ADD CHECK IF % OF TRIANGLES THAT MACTHED IS LARGER THAN 1/2
        matching::polygonMatching(ref, ref.H->get_children(pMatch.first), targ, targ.H->get_children(pMatch.second), polyMatchThresh, triangleMatches);
    }

    // Vertex Matching (Level 0)
    vertexMatches = matching::pointMatching(ref, targ, triangleMatches);
}


// Path Definitions

Robot::Robot(SimConfig cfg) : rng(randomDevice()) {
    env = cfg.forest;
    myObsRanges = cfg.obsRanges;
    for (auto i : myObsRanges) myObsRangesSq.push_back(i*i);
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

Point Robot::findValidPoint() {    // TODO maybe exit early if cannot find valid point after certain threshold
    Point randomPoint;
    do { randomPoint = Point(randX(rng), randY(rng)); }
    while (!std::all_of(env.begin(), env.end(), Avoids(randomPoint, collisionWidth)));
    return randomPoint;
}

bool Robot::validateStartingPose() {
    if (!(givenInitialPose && std::all_of(env.begin(), env.end(), Avoids(globalPose.p, collisionWidth))))
        globalPose = Pose(findValidPoint(), randTheta(rng));
    return true;
}

Obs Robot::observe() {
    Obs keyframe(myObsRanges[currentObsIdx], globalPose);

    // The robot's base orientation is in +y direction
    // Rotate global points points by negative theta to align with robot's current frame
    double globSin = sin(-globalPose.theta), globCos = cos(globalPose.theta);

    // Observe trees within the robot's vicinity
    for (const Tree& t : env) {
        // Calculate the tree's position relative to the robot
        double x = t.p.x - globalPose.p.x, y = t.p.y - globalPose.p.y, treeRadius = t.radius;

        // If detection noise present, only a given percentage of potentially observable trees will actually be seen
        if (detectionNoise(rng) > successfulObservationProbability) continue;
        
        // If position noise present, the position of each tree will be perturbed prior to observation (same with their radii)
        x += positionNoise(rng), y += positionNoise(rng), treeRadius += treeRadiusNoise(rng);

        // If the tree is within the robot's observation range, rotate its position to its correct orientation (relative to the robot's current frame) 
        if (x*x + y*y < myObsRangesSq[currentObsIdx]) keyframe.treePositions.push_back(Tree(x*globCos - y*globSin, x*globSin + y*globCos, treeRadius));
    }
    
    // Prepare for the next observation
    return keyframe;
}


// main exec
int main(int argc, char* argv[]) {
    if (argc > 1) {

        // Parse config file and display this run's configs
        ros::init(argc, argv, "obs_ranger");
        ros::NodeHandle n("~");
        SimConfig cfg(n);
        cfg.outputConfig(std::cout);
        int numObs = cfg.obsRanges.size();
        std::cout << "Observation count: " << cfg.obsRanges.size() << std::endl;
        ros::Rate pub_rate(100);

        // Initialize the path through the forest 
        // Robot* r = new Robot(cfg);
        Robot r(cfg);   // no need for heap
        urquhart::Observation* myObs;
        std::vector<urquhart::Observation*> geoHiers;

        // Create a directory to store the simulation data
        std::filesystem::remove_all(cfg.outputDirPath);
        pub_rate.sleep();
        std::filesystem::create_directories(cfg.localPointsPath);
        pub_rate.sleep();
        std::filesystem::create_directories(cfg.polyPolyPath);
        std::filesystem::create_directory(cfg.polyTriPath);
        std::filesystem::create_directory(cfg.polyHierPath);
        std::filesystem::create_directory(cfg.polyDescPath);
        pub_rate.sleep();
        std::filesystem::create_directories(cfg.matchPolyPath);
        std::filesystem::create_directory(cfg.matchTriPath);
        std::filesystem::create_directory(cfg.matchPointsPath);
        std::cout << "Created output directory: " << cfg.outputDirName << std::endl;

        std::vector<int> myRanges;

        // Find a starting location and observe the environment
        if (r.validateStartingPose()) {
            std::cout << "Robot observe trees from stationary pose (x y theta): " << r.globalPose.printPose() << std::endl << std::endl;

            // Also save this run's configuration
            std::ofstream configOut(cfg.outputDirPath+"/!config.txt");
            cfg.givenStartPose = true;  // hack
            cfg.initialPose = r.globalPose;
            cfg.outputConfig(configOut);
            configOut.close();

            // Iterate until we've observed whatever ranges were given as input
            while (ros::ok() && r.currentObsIdx < numObs) {
                Obs nextObs = r.observe();
                std::cout << ++r.currentObsIdx << ": Observed environment at sensor range " << nextObs.obsRange << std::endl;

                // Define geometric hierarchy
                Points treeXYs(2, nextObs.treePositions.size());
                int idx = 0;
                for (const auto& t : nextObs.treePositions) treeXYs.col(idx++) = Eigen::Vector2d{t.p.x, t.p.y};
                myObs = new urquhart::Observation(treeXYs);

                // Save hierarchy data to files
                writeObservationToFile(cfg.localPointsPath+std::to_string(nextObs.obsRange), nextObs.treePositions);
                std::cout << r.currentObsIdx << ": Written observation to file" << std::endl;
                writeHierarchyToFile(cfg, *myObs, std::to_string(nextObs.obsRange)+".txt");
                std::cout << r.currentObsIdx << ": Written geometric hierarchy to file" << std::endl;

                // Try matching previous observations with this one, save the output in files 
                auto rIter = myRanges.begin();
                for (auto ghIter = geoHiers.begin(); ghIter != geoHiers.end(); ++rIter, ++ghIter) {
                    std::vector<std::pair<size_t, size_t>> pM, tM;
                    std::vector<std::pair<PtLoc, PtLoc>> ptM;
                    std::cout << r.currentObsIdx << ": Attempting to match observations at range " << *rIter << " with the one at " << nextObs.obsRange << std::endl;
                    matchObs(**ghIter, *myObs, 5, 5, pM, tM, ptM);
                    std::string mapKey = std::to_string(*rIter) +"-"+ std::to_string(nextObs.obsRange)+".txt";

                    std::ofstream pMOut(cfg.matchPolyPath+mapKey);
                    for (auto& [refPoly, targPoly] : pM) pMOut << refPoly << "|" << targPoly << std::endl;
                    pMOut.close();
                    std::cout << r.currentObsIdx << ": (" << *rIter << "->" << nextObs.obsRange << ") Written polygon matches to file" << std::endl;

                    std::ofstream tMOut(cfg.matchTriPath+mapKey);
                    for (auto& [refTri, targTri] : tM) tMOut << refTri << "|" << targTri << std::endl;
                    tMOut.close();
                    std::cout << r.currentObsIdx << ": (" << *rIter << "->" << nextObs.obsRange << ") Written triangle matches to file" << std::endl;

                    std::ofstream ptMOut(cfg.matchPointsPath+mapKey);
                    for (auto& [refPoint, targPoint] : ptM) ptMOut << refPoint[0] << " " << refPoint[1] << "|" << targPoint[0] << " " << targPoint[1] << std::endl;
                    ptMOut.close();
                    std::cout << r.currentObsIdx << ": (" << *rIter << "->" << nextObs.obsRange << ") Written point matches to file" << std::endl;
                }

                // Save this observation for comparison against successive ones
                geoHiers.push_back(myObs), myRanges.push_back(nextObs.obsRange);
                ros::spinOnce();
                pub_rate.sleep();
                std::cout << "====================================================" << std::endl;
            }

        } else {
            std::cout << "Could not find a valid starting position (random initialization attempts exceeded limit)." << std::endl;    
        }

        // delete r;
        delete myObs;
        
    } else {
        std::cout << "Please provide a configuration file with run parameters for the simulation." << std::endl;
    }
}