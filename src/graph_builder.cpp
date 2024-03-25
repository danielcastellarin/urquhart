#include <ros/ros.h>
#include <observation.hpp>
#include <matching.hpp>
#include <graph_geometry.hpp>
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

std::vector<std::pair<Eigen::Vector2f, Eigen::Vector2f>> matchObs(urquhart::Observation &ref, urquhart::Observation &targ, double polyMatchThresh, double validPointMatchThresh) {
    std::vector<std::pair<size_t, size_t>> polygonMatches, triangleMatches;
    std::vector<std::pair<Eigen::Vector2f, Eigen::Vector2f>> vertexMatches;
    std::set<size_t> uniqueMatches;

    // Polygon Matching (Level 2)
    // std::vector<size_t> refIds = ref.H->get_children(0), targIds = targ.H->get_children(0);
    matching::polygonMatching(ref, ref.H->get_children(0), targ, targ.H->get_children(0), polyMatchThresh, polygonMatches);

    // Triangle Matching (Level 1)
    for (auto pMatch : polygonMatches) {
        // refIds = ref.H->get_children(pMatch.first), targIds = targ.H->get_children(pMatch.second);
        // TODO: ADD CHECK IF % OF TRIANGLES THAT MACTHED IS LARGER THAN 1/2
        matching::polygonMatching(ref, ref.H->get_children(pMatch.first), targ, targ.H->get_children(pMatch.second), polyMatchThresh, triangleMatches);
    }

    // Vertex Matching (Level 0)
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
                // Eigen::Vector2f refPos(refTriangle.points[refIdx].data()), targPos(targTriangle.points[targIdx].data());
                // vertexMatches.push_back({refPos, targPos});
                vertexMatches.push_back({Eigen::Vector2f{refTriangle.points[refIdx].data()}, Eigen::Vector2f{targTriangle.points[targIdx].data()}});
                uniqueMatches.insert(uid);
            }
        }
    }

    // Post-process: double-check matches, remove any where pairs are not certain distance from each other
    // for (auto iter = vertexMatches.begin(); iter != vertexMatches.end(); ++iter) {
    //     // TODO determine if eigen can parallelize this operation (without my goofy logic)
    //     if (std::abs(iter->first(0) - iter->second(0)) > validPointMatchThresh || std::abs(iter->first(1) - iter->second(1)) > validPointMatchThresh)
    //         vertexMatches.erase(iter);
    // }
    // arrogance check: if matched points across differential observations are not very close, then the match is probably wrong 

    return vertexMatches;
}

Eigen::Matrix3f computeRigid2DEuclidTf(std::vector<std::pair<Eigen::Vector2f, Eigen::Vector2f>> pointPairs) {
    Eigen::MatrixXf A(pointPairs.size()+pointPairs.size(), 4);
    Eigen::VectorXf b(pointPairs.size()+pointPairs.size());

    // https://math.stackexchange.com/questions/77462/finding-transformation-matrix-between-two-2d-coordinate-frames-pixel-plane-to
    int startRow = 0;
    for (auto& [refP, targP] : pointPairs) {
        A.row(startRow)   = (Eigen::Vector4f() << refP(0), -refP(1), 1, 0).finished().transpose();
        A.row(startRow+1) = (Eigen::Vector4f() << refP(1),  refP(0), 0, 1).finished().transpose();
        b[startRow]   = targP(0);
        b[startRow+1] = targP(1);
        startRow += 2;
    }

    // https://www.cs.cmu.edu/~16385/s17/Slides/10.1_2D_Alignment__LLS.pdf <-- slide 24
    // x = (A^T A)^-1 A^T b
    Eigen::Vector4f x = (A.transpose() * A).inverse() * A.transpose() * b;

    // Return the 3x3 matrix to transform frames: reference --> target
    Eigen::Matrix3f tf;
    tf << x[0], -x[1], x[2],
          x[1],  x[0], x[3],
             0,     0,    1;
    return tf;
}
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

// Graph: robot pose nodes, landmark nodes, pose-pose edges, pose-landmark edges, graph state vector (Eigen), look up table (nodes -> state vector position)  

// struct PoseNode { // Pose derived from matching local shapes with global representation
//     int id;
//     Pose pose;
//     PoseNode() : id(-1), pose() {}
//     PoseNode(int n, Pose p) : id(n), pose(p) {}
//     bool operator==(const Point &other) const{ return x == other.x && y == other.y; }
//     void transform(double xDiff, double yDiff, double thetaDiff) {pose.p.x+=xDiff, pose.+=b;}
//     std::string toString() const { return std::to_string(x) + " " + std::to_string(y); }
// };

struct PoseNode { // Pose derived from matching local shapes with global representation
    int id, stateVectorIdx;
    Eigen::Vector3f pose;
    PoseNode() : id(0) {}
    PoseNode(int i, double a, double b, double c) : id(i), pose({a,b,c}) {}
    PoseNode(int i, Eigen::Vector3f p) : id(i), pose(p) {}
    PoseNode(int i, double a, double b, double c, int svPos) : id(i), pose({a,b,c}), stateVectorIdx(svPos) {}
    PoseNode(int i, Eigen::Vector3f p, int svPos) : id(i), pose(p), stateVectorIdx(svPos) {}
    bool operator==(const PoseNode &other) const{ return id == other.id; }
    // std::string toString() const { return std::to_string(id) + ":(" + std::to_string(pose(0)) + "," + std::to_string(pose(1)) + "," + std::to_string(pose(2)) + ")"; }
};

struct LandmarkNode { // Point location derived from matching local shapes with global representation
    int id, stateVectorIdx;
    Eigen::Vector2f position;
    LandmarkNode() : id(0) {}
    LandmarkNode(int i, double a, double b) : id(i), position({a,b}) {}
    LandmarkNode(int i, Eigen::Vector2f p) : id(i), position(p) {}
    LandmarkNode(int i, double a, double b, int svPos) : id(i), position({a,b}), stateVectorIdx(svPos) {}
    LandmarkNode(int i, Eigen::Vector2f p, int svPos) : id(i), position(p), stateVectorIdx(svPos) {}
    bool operator==(const LandmarkNode &other) const{ return id == other.id; }
    // std::string toString() const { return std::to_string(id) + ":(" + std::to_string(position(0)) + "," + std::to_string(position(1)) + ")"; }
};
// TODO double check if nodes should have location data in them or just represent objects over time


// Seems like common information matrix is setting 100s along the diagonal
// https://robotics.stackexchange.com/questions/22451/calculate-information-matrix-for-graph-slam

struct PPEdge { // Point location derived from matching local shapes with global representation
    PoseNode *src, *dst; // pointers to the source and destination nodes for this edge
    Eigen::Vector3f distance;   // difference in position between the poses on either side of this edge
    Eigen::Matrix3f info;       // inverse covariance matrix for 'distance'
    PPEdge() {}
    PPEdge(PoseNode* source, PoseNode* destination, Eigen::Vector3f d) : src(source), dst(destination), distance(d) {
        info << 100,  0,   0,
                0,  100,   0,
                0,    0, 100;
    }
    // bool operator==(const LandmarkNode &other) const{ return id == other.id; }
    // void transform(double xDiff, double yDiff) {p.translate(xDiff, yDiff);}
    // std::string toString() const { return std::to_string(id) + ":(" + p.toString() + ")";}
};

struct PLEdge { // Point location derived from matching local shapes with global representation
    PoseNode *src;
    LandmarkNode *dst; // pointers to the source and destination nodes for this edge
    Eigen::Vector2f distance;   // difference in position between the observing robot and the landmark
    Eigen::Matrix2f info;       // inverse covariance matrix for 'distance'
    PLEdge() {}
    PLEdge(PoseNode* source, LandmarkNode* destination, Eigen::Vector2f d) : src(source), dst(destination), distance(d) {
        info << 100,  0,
                0,  100;
    }
    // bool operator==(const LandmarkNode &other) const{ return id == other.id; }
    // void transform(double xDiff, double yDiff) {p.translate(xDiff, yDiff);}
    // std::string toString() const { return std::to_string(id) + ":(" + p.toString() + ")";}
};



struct LamePPEdge { // Point location derived from matching local shapes with global representation
    int src, dst;
    Eigen::Vector3f distance;   // difference in position between the poses on either side of this edge
    Eigen::Matrix3f info;       // inverse covariance matrix for 'distance'
    LamePPEdge() {}
    LamePPEdge(int source, int destination, Eigen::Vector3f d) : src(source), dst(destination), distance(d) {
        info << 100,  0,   0,
                0,  100,   0,
                0,    0, 100;
    }
};

struct LamePLEdge { // Point location derived from matching local shapes with global representation
    int src, dst;
    Eigen::Vector2f distance;   // difference in position between the observing robot and the landmark
    Eigen::Matrix2f info;       // inverse covariance matrix for 'distance'
    LamePLEdge() {}
    LamePLEdge(int source, int destination, Eigen::Vector2f d) : src(source), dst(destination), distance(d) {
        info << 100,  0,
                0,  100;
    }
};

// TODO struct for Graph
// TODO method to convert nodes/edges to state vector

Eigen::Matrix3f v2t(Eigen::Vector3f vec) {
    float c = cos(vec(2)), s = sin(vec(2));
    // Eigen::Matrix3f tf {c, -s, vec(0), s,  c, vec(1), 0, 0, 1};
    // Eigen::Matrix3f tf {
    //     {c, -s, vec(0)},
    //     {s,  c, vec(1)},
    //     {0.0,  0.0,      1.0}
    // };
    Eigen::Matrix3f tf;
    tf << c, -s, vec(0),
          s,  c, vec(1),
          0,  0,    1;
    return tf;
}

Eigen::Vector3f t2v(Eigen::Matrix3f tf) { 
    Eigen::Vector3f vec {tf(0,2), tf(1,2), atan2(tf(1,0), tf(0,0))};
    return vec;
}


                // TODO remove this if landmark id stored
// template <>
// struct hash<Point> {
//     std::size_t operator()(const Point& c) const {
//         std::size_t result = 0;
//         boost::hash_combine(result, c.x);
//         boost::hash_combine(result, c.y);
//         return result;
//     }
// };
struct Landmark
{
    int id;
    Eigen::Vector2f p;
    Landmark() : id(0) {}
    Landmark(double a, double b) : p({a,b}), id(0) {}
    Landmark(Eigen::Vector2f position) : p(position), id(0) {}
    Landmark(Eigen::Vector2f position, int i) : p(position), id(i) {}
    // Landmark(Landmark l, double a, double b) : x(l.x + a), y(l.y + b), id(l.id) {}
    bool operator==(const Landmark &other) const{ return p(0) == other.p(0) && p(1) == other.p(1); }
    // void translate(double a, double b) {x+=a, y+=b;}
    std::string toString() const { return std::to_string(p(0)) + "," + std::to_string(p(1)); }
};

// template <>
// struct vectorHasher{
// std::size_t operator()(const Eigen::Vector2f& in) const
// {
//     using boost::hash_value;
//     using boost::hash_combine;
//     // Start with a hash value of 0
//     std::size_t seed = 0;
//     hash_combine(seed, hash_value(in(0)));
//     hash_combine(seed, hash_value(in(1)));
//     return seed;
// }
// };
// typedef std::unordered_map< Eigen::Vector2f, int, vectorHasher > PointMap;





// Hash function for Eigen matrix and vector.
// The code is from `hash_combine` function of the Boost library. See
// http://www.boost.org/doc/libs/1_55_0/doc/html/hash/reference.html#boost.hash_combine .
template<typename T>
struct matrix_hash : std::unary_function<T, size_t> {
  std::size_t operator()(T const& matrix) const {
    // Note that it is oblivious to the storage order of Eigen matrix (column- or
    // row-major). It will give you the same hash value for two different matrices if they
    // are the transpose of each other in different storage order.
    size_t seed = 0;
    for (size_t i = 0; i < matrix.size(); ++i) {
      auto elem = *(matrix.data() + i);
      seed ^= std::hash<typename T::Scalar>()(elem) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
    }
    return seed;
  }
};






struct SLAMGraph {
    int prevPoseId = 0, prevLdmkId = 0;
    urquhart::Observation* geoHier = NULL;
    Eigen::VectorXf stateVector; // stored as eigen vector for faster computations??

    // If node postions in state vector are stored in external dictionary...
    std::unordered_map<int, int> nodePositionsInVector;
    // std::unordered_map<int, Pose> poseNodes;
    std::unordered_map<int, Eigen::Vector3f> poseNodeMap;
    // std::unordered_map<int, Point> landmarkNodes;
    // PointMap landmarkIds;
    std::unordered_map<Eigen::Vector2f, int, matrix_hash<Eigen::Vector2f>> landmarkIdMap;
    std::unordered_map<int, Eigen::Vector2f> landmarkNodeMap;
    // std::unordered_map<int, Landmark> landmarkNodeMap;
    std::vector<LamePPEdge> lameppEdges;
    std::vector<LamePLEdge> lameplEdges;

    // If node postions in state vector are stored in nodes...
    std::vector<PoseNode> poseNodeList;
    std::vector<LandmarkNode> landmarkNodeList;
    std::vector<PPEdge> ppEdges;
    std::vector<PLEdge> plEdges;

    SLAMGraph() {}
    
    // void addPoseNode(Pose& p) {
        // TODO determine whether Pose should just be Vector3f
    void addLamePoseNode(Eigen::Vector3f pose) {
        poseNodeMap[++prevPoseId] = pose;
        // the start of this node's data will be at the end of the current state vector
        nodePositionsInVector[prevPoseId] = stateVector.size();
        stateVector.conservativeResize(stateVector.size()+3);   // Append (x,y,theta) to the end of state vector
        stateVector.tail(3) = pose; // TODO: VERIFY THIS IS VALID!?!?!?!?
    }
    void addLameLandmarkNode(Eigen::Vector2f& position) {
        // landmarkNodeMap[--prevLdmkId] = Landmark(position, prevLdmkId);
        landmarkNodeMap[--prevLdmkId] = position;
        // the start of this node's data will be at the end of the current state vector
        nodePositionsInVector[prevLdmkId] = stateVector.size();
        stateVector.conservativeResize(stateVector.size()+2);   // Append (x,y) to the end of state vector 
        stateVector.tail(2) = position; // TODO: VERIFY THIS IS VALID!?!?!?!?
    }

    PoseNode* addPoseNode(Eigen::Vector3f& pose) {
        PoseNode pn(++prevPoseId, pose, stateVector.size());
        poseNodeList.push_back(pn); // List is only kept for storage, it should never be traversed directly 
        
        // Append (x,y,theta) to the end of state vector
        stateVector.conservativeResize(stateVector.size()+3);   // Append (x,y,theta) to the end of state vector
        stateVector.tail(3) = pose; // TODO: VERIFY THIS IS VALID!?!?!?!?
        return &pn;
    }
    LandmarkNode* addLandmarkNode(Eigen::Vector2f& position) {
        LandmarkNode ln(--prevLdmkId, position, stateVector.size());
        landmarkNodeList.push_back(ln);

        // Append (x,y) to the end of state vector
        stateVector.conservativeResize(stateVector.size()+2);
        stateVector.tail(2) = position; // TODO: VERIFY THIS IS VALID!?!?!?!?
        return &ln;
    }

    void addLamePPEdge() {
        lameppEdges.push_back(LamePPEdge(prevPoseId-1, prevPoseId, poseNodeMap[prevPoseId-1] - poseNodeMap[prevPoseId]));
    }
    void addLamePPEdge(int srcId, int dstId) {
        lameppEdges.push_back(LamePPEdge(srcId, dstId, poseNodeMap[srcId] - poseNodeMap[dstId]));
    }

    void addLamePLEdge(int srcId, int dstId, Eigen::Vector2f localTreePosition) {
        lameplEdges.push_back(LamePLEdge(srcId, dstId, localTreePosition));
    }
    void addLamePLEdge(int dstId, Eigen::Vector2f localTreePosition) {
        lameplEdges.push_back(LamePLEdge(prevPoseId, dstId, localTreePosition));
    }
    void addLamePLEdge(Eigen::Vector2f localTreePosition) {
        lameplEdges.push_back(LamePLEdge(prevPoseId, landmarkIdMap[localTreePosition], localTreePosition));
    }

    PoseNode* addPPEdge(Eigen::Vector3f currentPose) {
        PoseNode* newPoseNode(addPoseNode(currentPose));
        // This only gets called once per keyframe
        // newNodePose should be given using t2v() on the tf to global frame computed after successful landmark association
        ppEdges.push_back(PPEdge(&poseNodeList.back(), newPoseNode, Eigen::Vector3f{currentPose - poseNodeList.back().pose}));
        return newPoseNode;
    }
    void addPLEdge(LandmarkNode* existingLandmarkNode, Eigen::Vector2f landmarkLocalPosition) {
        // May be called with landmark nodes that existed before current keyframe or nodes that were just created
        // Regardless, this function expects that node to have been defined already
        // landmarkLocalPosition is taken directly from the landmarks position in this keyframe
        plEdges.push_back(PLEdge(&poseNodeList.back(), existingLandmarkNode, landmarkLocalPosition));
    }



    void updateLameNodesFromStateVector() {
        for (auto&[nodeId, stateVectorPosition] : nodePositionsInVector) {
            if (nodeId > 0) {
                poseNodeMap[nodeId] = stateVector.segment(stateVectorPosition, 3);
            } else {
                landmarkNodeMap[nodeId] = stateVector.segment(stateVectorPosition, 2);
            }
        }
    }

    void updateNodesFromStateVector() {
        // I only really care about the last pose and all the landmarks
        poseNodeList.back().pose = stateVector.segment(poseNodeList.back().stateVectorIdx, 3);
        for (auto& node : landmarkNodeList) {
            node.position = stateVector.segment(node.stateVectorIdx, 2);
        }
    }
};


/* NOTE THE ARCANE KNOWLEDGE OF SOLVING LINEAR SYSTEMS WITH EIGEN 

// Solve Ax = b. Result stored in x. Matlab: x = A \ b.
x = A.ldlt().solve(b));  // A sym. p.s.d.    #include <Eigen/Cholesky>
x = A.llt() .solve(b));  // A sym. p.d.      #include <Eigen/Cholesky>
x = A.lu()  .solve(b));  // Stable and fast. #include <Eigen/LU>
x = A.qr()  .solve(b));  // No pivoting.     #include <Eigen/QR>
x = A.svd() .solve(b));  // Stable, slowest. #include <Eigen/SVD>
// .ldlt() -> .matrixL() and .matrixD()
// .llt()  -> .matrixL()
// .lu()   -> .matrixL() and .matrixU()
// .qr()   -> .matrixQ() and .matrixR()
// .svd()  -> .matrixU(), .singularValues(), and .matrixV()
*/

SLAMGraph g;


float computeGlobalError() {
    float globalError = 0;

    for (auto& edge : g.lameppEdges) {
        int iStart = g.nodePositionsInVector[edge.src], jStart = g.nodePositionsInVector[edge.dst];
        Eigen::Vector3f xI(g.stateVector.segment(iStart, 3)), xJ(g.stateVector.segment(jStart, 3)), eIJ;
        Eigen::Matrix3f XI = v2t(xI), XJ = v2t(xJ), ZIJ = v2t(edge.distance);
        Eigen::Matrix3f XI_inv = XI.inverse(), ZIJ_inv = ZIJ.inverse();

        eIJ = t2v(ZIJ_inv * (XI_inv * XJ)); // NOTE previous 5 lines are verbatim from eIJ calc when optimizing graph; make function
        globalError += eIJ.transpose() * edge.info * eIJ;
    }

    for (auto& edge : g.lameplEdges) {
        int iStart = g.nodePositionsInVector[edge.src], jStart = g.nodePositionsInVector[edge.dst];
        Eigen::Vector3f x(g.stateVector.segment(iStart, 3));
        Eigen::Vector2f l(g.stateVector.segment(jStart, 2)), eIJ;
        float s = sin(x(2)), c = cos(x(2));
        Eigen::Matrix2f R_transposed;
        // Eigen::Matrix2f R_transposed {c, s, -s, c}; // TODO check if valid init?
        R_transposed << c, s,-s, c;
        eIJ = (R_transposed * (l - x.head(2))) - edge.distance; // 2x1
        globalError += eIJ.transpose() * edge.info * eIJ;
    }

    return globalError;
}

void constructGraph(const sensor_msgs::PointCloud2ConstPtr& cloudMsg) {
    // Take the points from the PointCloud2 message and create a geometric hierarchy from it
    pcl::PointCloud<pcl::PointXY> localCloud;
    pcl::fromROSMsg(*cloudMsg, localCloud);
    PointVector vectorOfTrees;
    for(const auto& p : localCloud) vectorOfTrees.push_back(std::vector<double>{p.x, p.y});
    urquhart::Observation localObs(vectorOfTrees);

    // TODO do different things depending on whether the global map is available
    if (g.geoHier == NULL) {
        // TODO determine what gets done when the global map is unavailable (might also occur when keyframe does not match with anything on the map)
        //      maybe I could modify the matching procedure a bit to iteratively loosen the matching constraints if not enough matches are found?
    } else {

        // When global map is present:
        auto matchingPoints = matchObs(localObs, *g.geoHier, 5, 1.5); // TODO use last param for association pruning
        // Matching speed should not be a bottleneck when querying global representation
        // If faster speed desired, only elect global polygons that are a certain distance away from the previous pose  
        // if not enough matches are made, maybe drop the frame? or expand global polygon election range

        // TODO Use RANSAC to estimate the current position of the robot with respect to the global reference frame
        // At the very least, return the inliear associations from matchingPoints
        // If not confident enough in potential match, exit early

        // Use the inliear point pairs to find the robot's current position wrt global frame
        Eigen::Matrix3f currentGlobalRobotPoseTf(computeRigid2DEuclidTf(matchingPoints));

        // Define a new node and edge for the robot's estimated odometry
        // Keep the reference to the current PoseNode to use when creating landmark constraints
        // PoseNode* currentPoseNode = g.addPPEdge(t2v(currentGlobalRobotPoseTf));
        // Alternative external map version:
        g.addLamePoseNode(t2v(currentGlobalRobotPoseTf));
        g.addLamePPEdge();

        // TODO (maybe) enable geo hierarchy to reference LandmarkNodes (Polygon object stores LandmarkNodes in vector corr. every element in "points")
        //      --> when match occurs, pass back the target's LandmarkNode corr. to the point
        std::unordered_set<Eigen::Vector2f, matrix_hash<Eigen::Vector2f>> localMatchedPoints;
        for (auto& [refP, targP] : matchingPoints) {
            // Add constraints for all local points that matched with existing landmarks
            g.addLamePLEdge(targP);
            localMatchedPoints.insert(refP);
        }

        // Try to associate other local points in the global frame  
        bool hasNewLandmarks = false;
        for (auto& pt : vectorOfTrees) {
            // Only process points that were not EXPLICITLY matched with existing landmarks
            Eigen::Vector2f myPoint(pt.data());
            if (localMatchedPoints.find(myPoint) != localMatchedPoints.end()) {

                Eigen::Matrix3f localPointTf;
                localPointTf << 1, 0, myPoint(0),
                                0, 1, myPoint(1),
                                0,  0,         1;
                
                // Obtain this point's observed position relative to the global frame
                // TODO ensure correct mult order
                Eigen::Matrix3f globalLandmarkPosition = currentGlobalRobotPoseTf * localPointTf;
                float nearestPointX = 0.5, nearestPointY = 0.5;
                int nearestPointId = 0;

                // Find the nearest existing landmark below the nearness threshold for the given point in the global frame
                for (auto& [id, globPt] : g.landmarkNodeMap) {
                    double diffX = std::abs(globalLandmarkPosition(0,2) - globPt(0)), diffY = std::abs(globalLandmarkPosition(1,2) - globPt(1));
                    // TODO make this more robust
                    if (diffX < nearestPointX && diffY < nearestPointY) {
                        nearestPointX = diffX, nearestPointY = diffY, nearestPointId = id;
                    }
                }

                // If we didn't find a point close enough to this one, make a node for it
                if (!nearestPointId) {
                    Eigen::Vector2f globalXY{globalLandmarkPosition(0,2), globalLandmarkPosition(1,2)};
                    g.addLameLandmarkNode(globalXY);
                    nearestPointId = g.prevLdmkId;
                    hasNewLandmarks = true;
                }

                // Create a constraint for this landmark
                g.addLamePLEdge(nearestPointId, myPoint);
            }
        }

        // TODO optimize graph
        // Initialize sparse system H and coefficient vector b
        Eigen::MatrixXf H(g.stateVector.size(), g.stateVector.size());
        Eigen::VectorXf b(g.stateVector.size());
        bool isFirstThereforeAddPrior = true;

        // Resolve pose-pose constraints
        for (auto& edge : g.lameppEdges) {
            // Obtain node states from the stete vector
            // TODO figure out how to perserve the "slice" (segment and block parameters) so that it doesn't need to be rewritten so many times
            int iStart = g.nodePositionsInVector[edge.src], jStart = g.nodePositionsInVector[edge.dst];
            Eigen::Vector3f xI(g.stateVector.segment(iStart, 3)), xJ(g.stateVector.segment(jStart, 3));

            // Use xI, xJ, and dist (aka z) to compute error eIJ and jacobians A,B
            Eigen::Vector3f eIJ;
            Eigen::Matrix3f A, B;
            { // Math block
                Eigen::Matrix3f XI = v2t(xI), XJ = v2t(xJ), ZIJ = v2t(edge.distance);
                Eigen::Matrix3f XI_inv = XI.inverse(), ZIJ_inv = ZIJ.inverse();

                eIJ = t2v(ZIJ_inv * (XI_inv * XJ)); // TODO make sure eigen is doing mult correctly

                Eigen::Matrix3f RotXI = XI.block(0,0,2,2), RotZIJ = ZIJ.block(0,0,2,2); // TODO make sure this is grabbing the rotation matrix
                Eigen::Vector2f TransXI = xI.head(2), TransXJ = xJ.head(2); // the lazy way

                Eigen::Matrix2f Aii = -RotZIJ.transpose() * RotXI.transpose(), RotXI_derived;
                RotXI_derived << XI(0,1), XI(0,0),
                                -XI(0,0), XI(0,1);
                Eigen::Vector2f Aij = (RotZIJ * RotXI_derived) * (TransXJ - TransXI), Bij{0,0};
                Eigen::RowVector2f Aji{0,0}, Bji{0,0};
                A << Aii, Aij,
                     Aji,  -1;
                B << -Aii, Aij,
                      Aji,   1;
            }

            // Update H and b accordingly
            b.segment(iStart, 3) += eIJ.transpose() * edge.info * A;
            b.segment(jStart, 3) += eIJ.transpose() * edge.info * B;
            H.block(iStart, iStart, 3, 3) += A.transpose() * edge.info * A;
            H.block(iStart, jStart, 3, 3) += A.transpose() * edge.info * B;
            // H.block(jStart, iStart, 3, 3) += B.transpose() * edge.info * A;
            H.block(jStart, iStart, 3, 3) += H.block(iStart, jStart, 3, 3).transpose();
            H.block(jStart, jStart, 3, 3) += B.transpose() * edge.info * B;

            // Add a prior only to the first pose node of the graph
            // This will fix that node to remain at its current location (fixed reference frame)
            //   all the other measurements are relative to this one
            if (isFirstThereforeAddPrior) {
                H.block(iStart, iStart, 3, 3) += Eigen::Matrix3f::Identity(); // I think scalar lambda can be applied here too?
                isFirstThereforeAddPrior = false;
            }
        }


        // Resolve pose-landmark constraints
        for (auto& edge : g.lameplEdges) {
            // Obtain node states from the stete vector
            // TODO figure out how to perserve the "slice" (segment and block parameters) so that it doesn't need to be rewritten so many times
            int iStart = g.nodePositionsInVector[edge.src], jStart = g.nodePositionsInVector[edge.dst];
            Eigen::Vector3f x(g.stateVector.segment(iStart, 3));
            Eigen::Vector2f l(g.stateVector.segment(jStart, 2));

            // Use xI, xJ, and dist (aka z) to compute error eIJ and jacobians A,B
            Eigen::Vector2f eIJ;
            Eigen::Matrix<float, 2, 3> A;
            Eigen::Matrix2f B;
            { // Math block
                float s = sin(x(2)), c = cos(x(2));
                B << c, s, // B is just the rotation matrix of x transposed
                    -s, c;
                Eigen::Matrix2f RotX_derived;
                RotX_derived << -s,  c,
                                -c, -s;
                Eigen::Vector2f TransXIdiff = l - x.head(2), A2 = RotX_derived * TransXIdiff;

                eIJ = (B * TransXIdiff) - edge.distance; // 2x1
                A << -B, A2; // 2x3
            }

            // Update H and b accordingly
            b.segment(iStart, 3) += eIJ.transpose() * edge.info * A;
            b.segment(jStart, 2) += eIJ.transpose() * edge.info * B;
            H.block(iStart, iStart, 3, 3) += A.transpose() * edge.info * A;
            H.block(iStart, jStart, 3, 2) += A.transpose() * edge.info * B;
            // H.block(jStart, iStart, 2, 3) += B.transpose() * edge.info * A;
            H.block(jStart, iStart, 2, 3) += H.block(iStart, jStart, 3, 2).transpose();
            H.block(jStart, jStart, 2, 2) += B.transpose() * edge.info * B;
        }

        // Update the state vector; according to the result of the system of equations
        g.stateVector += H.llt().solve(b);
        /* NOTE THE ARCANE KNOWLEDGE OF SOLVING LINEAR SYSTEMS WITH EIGEN 

        // Solve Ax = b. Result stored in x. Matlab: x = A \ b.
        x = A.ldlt().solve(b));  // A sym. p.s.d.    #include <Eigen/Cholesky>  <-- this version omits sqrt, is it needed?
        x = A.llt() .solve(b));  // A sym. p.d.      #include <Eigen/Cholesky>
        x = A.lu()  .solve(b));  // Stable and fast. #include <Eigen/LU>
        x = A.qr()  .solve(b));  // No pivoting.     #include <Eigen/QR>
        x = A.svd() .solve(b));  // Stable, slowest. #include <Eigen/SVD>
        // .ldlt() -> .matrixL() and .matrixD()
        // .llt()  -> .matrixL()
        // .lu()   -> .matrixL() and .matrixU()
        // .qr()   -> .matrixQ() and .matrixR()
        // .svd()  -> .matrixU(), .singularValues(), and .matrixV()
        */
        
        
        // Extract the data from the state vector and update the semantic nodes
        g.updateLameNodesFromStateVector();
        // g.updateNodesFromStateVector();
        // NOTE g.geoHier polygons will be handled below
        //      If we assume that the landmark nodes hold references to the points in the geometric hierarchy,
        //      then maybe updating edgeLengths and descriptor will be simpler after? 

        // TODO Spooky aftermath update of geometric hierarchy 
        if (hasNewLandmarks) { // these points SHOULD be on the outer boundary of the global map
            // TODO need to store references to these separately

            // traverse up the local tree for these points, find which triangles and polygons they were part of
            // obtain set of points from all polygons/triangles identified
            // -----------------
            // Compute the geometric hierarchy for those points
            // Magically slap that onto the existing hierarchy, bing bang boom
            //    Replace triangulation
            //    for each poly - replace it from the new hierarchy (if possible)
            // BE CAREFUL if polygons have merged/broken between the hierarchies
            // === OR ===
            // Not sure lol, figure something out that has more specific details than the option above
            // -----------------
        }
        
        // TODO in addition, adjust all point positions within the hierarchy, hopefully globObs.H->points will store pointers to globObs.landmarks 
        // decide whether polygon "edgeLengths" and "descriptor" should be updated too  

    }


}


int main(int argc, char **argv) {
    // Initialize Node and read in private parameters
    ros::init(argc, argv, "graph_builder");
    ros::NodeHandle n("~");
    // SimConfig cfg(n);
    // cfg.outputConfig(std::cout);
    // ros::Rate pub_rate(cfg.pubRate);    // 10Hz by default

    // pcl::PointCloud<pcl::PointXY> localPC;
    ros::Subscriber sub = n.subscribe("/keyframe_maker/keyframe", 10, constructGraph);

    ros::spin();


    return 0;
}