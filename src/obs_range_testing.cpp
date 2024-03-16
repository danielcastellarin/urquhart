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
        for(auto ch : trees.H->graph[pIdx].points) plyOut << pIdx << " " << ch[0] << " " << ch[1] << "|";
        plyOut << std::endl;
        for(auto d : trees.H->graph[pIdx].descriptor) dscOut << d << " ";
        dscOut << std::endl;
        
        // Iterate over the indices of the Triangles that compose this Polygon
        for(auto tIdx : trees.H->traverse(pIdx)) {
            // Retain only the Polygon objects that have three sides
            if (trees.H->graph[tIdx].points.size() == 3) {
                for(auto ch : trees.H->graph[tIdx].points) triOut << tIdx << " " << ch[0] << " " << ch[1] << "|";
                triOut << std::endl;
            }
        }
    }
    plyOut.close();
    triOut.close();
    dscOut.close();
}

// Substituted for original for type compatibility with my code
void myPolygonMatching(
    const urquhart::Observation &ref, std::vector<size_t> refIds,
    const urquhart::Observation &targ, std::vector<size_t> targIds, double thresh,
    std::vector<std::pair<size_t, size_t>> &polygonMatches) {
    std::set<size_t> matched;
    for (auto rIdx : refIds) {
        size_t bestMatch = 0, bestDist = 100000;
        urquhart::Polygon rp = ref.H->get_vertex(rIdx);
        for (auto tIdx : targIds) {
            urquhart::Polygon tp = targ.H->get_vertex(tIdx);
            // if tIdx was not matched before and the difference of number of points is not larger than 5
            if (matched.find(tIdx) == matched.end() &&
                std::abs(int(rp.points.size() - tp.points.size())) <= 3)
            {
                double d = euclideanDistance(rp.descriptor, tp.descriptor);
                if (d < bestDist) {
                    bestDist = d;
                    bestMatch = tIdx;
                }
            }
        }

        if (bestDist < thresh) {
            matched.insert(bestMatch);
            polygonMatches.push_back({rIdx, bestMatch});
        }
    }
}

void matchObs(const urquhart::Observation &ref, const urquhart::Observation &targ, double polyMatchThresh, double validPointMatchThresh,
            std::vector<std::pair<size_t, size_t>> &polygonMatches, std::vector<std::pair<size_t, size_t>> &triangleMatches, std::vector<std::pair<vecPtT, vecPtT>> &vertexMatches) {

    // Polygon Matching (Level 2)
    // std::vector<size_t> refIds = ref.H->get_children(0), targIds = targ.H->get_children(0);
    // matching::polygonMatching(ref, ref.H->get_children(0), targ, targ.H->get_children(0), thresh, polygonMatches);
    myPolygonMatching(ref, ref.H->get_children(0), targ, targ.H->get_children(0), polyMatchThresh, polygonMatches);

    // Triangle Matching (Level 1)
    for (auto pMatch : polygonMatches) {
        // refIds = ref.H->get_children(pMatch.first), targIds = targ.H->get_children(pMatch.second);
        // TODO: ADD CHECK IF % OF TRIANGLES THAT MACTHED IS LARGER THAN 1/2
        // matching::polygonMatching(ref, ref.H->get_children(pMatch.first), targ, targ.H->get_children(pMatch.second), thresh, triangleMatches);
        myPolygonMatching(ref, ref.H->get_children(pMatch.first), targ, targ.H->get_children(pMatch.second), polyMatchThresh, triangleMatches);
    }

    // Vertex Matching (Level 0)
    std::set<size_t> uniqueMatches;
    for (auto tMatch : triangleMatches) {   // FIXME? make the loop explicitly over constant references?
        urquhart::Polygon refTriangle = ref.H->get_vertex(tMatch.first), targTriangle = targ.H->get_vertex(tMatch.second);
        std::vector<size_t> chi = {0, 1, 2}, bestPermutation;

        // TODO change the edgeLengths to do squared distance instead of euclidean distance (unnecessary square root)

        // Permute the edges to find the best match between the triangles
        double bestDist = 1000000;
        do {
            double d = euclideanDistance(refTriangle.edgeLengths, std::vector<double>{targTriangle.edgeLengths[chi[0]], targTriangle.edgeLengths[chi[1]], targTriangle.edgeLengths[chi[2]]});
            if (d < bestDist) {
                bestDist = d;
                bestPermutation = chi;
            }
        } while (std::next_permutation(chi.begin(), chi.end()));

        for (size_t i = 0; i < 3; ++i) {
            int refIdx = (i+2)%3, targIdx = (bestPermutation[i]+2)%3;
            size_t uid = cantorPairing(refTriangle.edges[refIdx].first, targTriangle.edges[targIdx].first);
            if (uniqueMatches.find(uid) == uniqueMatches.end()) {
                vertexMatches.push_back({refTriangle.points[refIdx], targTriangle.points[targIdx]});
                uniqueMatches.insert(uid);
            }
        }
    }
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
                PointVector vectorOfTrees;
                for (const auto& t : nextObs.treePositions) vectorOfTrees.push_back(std::vector<double>{t.p.x, t.p.y});
                myObs = new urquhart::Observation(vectorOfTrees);

                // Save hierarchy data to files
                writeObservationToFile(cfg.localPointsPath+std::to_string(nextObs.obsRange), nextObs.treePositions);
                std::cout << r.currentObsIdx << ": Written observation to file" << std::endl;
                writeHierarchyToFile(cfg, *myObs, std::to_string(nextObs.obsRange)+".txt");
                std::cout << r.currentObsIdx << ": Written geometric hierarchy to file" << std::endl;

                // Try matching previous observations with this one, save the output in files 
                auto rIter = myRanges.begin();
                for (auto ghIter = geoHiers.begin(); ghIter != geoHiers.end(); ++rIter, ++ghIter) {
                    std::vector<std::pair<size_t, size_t>> pM, tM;
                    std::vector<std::pair<vecPtT, vecPtT>> ptM;
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