#include "libqhullcpp/RboxPoints.h"
#include "libqhullcpp/QhullError.h"
#include "libqhullcpp/QhullQh.h"
#include "libqhullcpp/QhullFacet.h"
#include "libqhullcpp/QhullFacetList.h"
#include "libqhullcpp/QhullFacetSet.h"
#include "libqhullcpp/QhullLinkedList.h"
#include "libqhullcpp/QhullPoint.h"
#include "libqhullcpp/QhullVertex.h"
#include "libqhullcpp/QhullVertexSet.h"
#include "libqhullcpp/Qhull.h"
// #include <Eigen/Dense>
#include "eigen3/Eigen/Dense"
#include <map>
#include <unordered_set>
#include <set>
#include <vector>
#include <filesystem>
#include <fstream>
#include <sstream>


#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

// Eigen::Matrix2Xd myCloud {-5.939245, 17.134218,
// -17.593330, 3.824696, 
// -2.408555, 19.517879 ,
// -7.875935, 16.795897 ,
// -14.539388, -4.718498,
// 16.804339, -3.308797 ,
// 5.369431, 18.622539 ,
// 3.992928, 8.402482 ,
// -13.842193, -12.220115,
// 9.583480, -0.073340 ,
// 9.659916, -3.133269 ,
// -19.504037, 0.800060 ,
// 7.672071, -10.206556 ,
// -2.087953, 4.057330 ,
// 5.404409, 0.070249 ,
// -4.331035, -15.384193,
// -13.342168, -7.453444,
// 18.190234, -6.869290};



int main(int argc, char **argv) {

    Eigen::Matrix2Xd myCloud(2, 18);
    myCloud << -5.939245,-17.593330,-2.408555,-7.875935,-14.539388,16.804339,5.369431,3.992928,-13.842193,9.583480,9.659916,-19.504037,7.672071,-2.087953,5.404409,-4.331035, -13.342168, 18.190234, 
                17.134218,3.824696, 19.517879, 16.795897,-4.718498,-3.308797 ,18.622539,8.402482,-12.220115,-0.073340,-3.133269,0.800060,-10.206556,4.057330,0.070249,-15.384193,-7.453444,-6.869290;
    

    // for (int iter = 0; iter < myCloud.cols(); ++iter) {
    //     // if (std::abs(ref.ldmkX(iter->first) - targ.ldmkX(iter->second)) > validPointMatchThresh || std::abs(ref.ldmkY(iter->first) - targ.ldmkY(iter->second)) > validPointMatchThresh)
    //     // Eigen::Matrix2d tata {{myCloud(0, iter), myCloud(0, (iter+1)%18)}, {myCloud(1, iter), myCloud(1, (iter+1)%18)}};
    //     // std::cout << tata << std::endl;
    //     std::cout << iter << std::endl;
    //     if (((myCloud.col(iter) - myCloud.col((iter+1)%18)).array().abs() > 3).any())
    //         std::cout << "Bad pair" << std::endl;
    // }
    
    // std::unordered_map<int, std::unordered_set<int>> myCoolThing, otherCoolThing;
    // std::unordered_set<int> a({1,3,6}), b({3,4,67}), c({6,5,8}), d({1,5,78}), e({4,5,6,7,8});
    // myCoolThing[1] = a;
    // myCoolThing[2] = b;
    // myCoolThing[3] = c;
    // myCoolThing[4] = d;
    // otherCoolThing = myCoolThing;

    // myCoolThing[5] = e;
    // myCoolThing[1].insert(5);
    // myCoolThing[1].insert(7);

    // for (const auto& [k,v] : otherCoolThing) {
    //     std::cout << k << ":";
    //     for (const auto& val : v) std::cout << " " << val;
    //     std::cout << std::endl;
    // }

    // for (const auto& [k,v] : myCoolThing) {
    //     std::cout << k << ":";
    //     for (const auto& val : v) std::cout << " " << val;
    //     std::cout << std::endl;
    // }

    

    // // TODO insert data directly from Eigen Matrix, not some goofy vector
    // std::vector<double> qhull_points_data(6);
    // for (size_t pidx = 0; pidx < 3; ++pidx) {
    //     qhull_points_data[pidx * 2 + 0] = (double)pidx;
    //     qhull_points_data[pidx * 2 + 1] = (double)pidx+(pidx%2);
    // }
    // std::cout << "meep";

    // // Read tree positions into QHull
    // orgQhull::PointCoordinates qhull_points(2, "");
    // // qhull_points.append(landmarks.cols(), landmarks.data());
    // qhull_points.append(qhull_points_data);
    // std::cout << "moop";

    // // Compute Delaunay Triangulation
    // orgQhull::Qhull q;
    // q.runQhull(qhull_points.comment().c_str(), qhull_points.dimension(),
    //                qhull_points.count(), qhull_points.coordinates(), "Qt Qbb Qc Qz Q12 d");

    // std::cout << "gaaa";

    // // Facet ids are confusing; we want to map the good facets in order of appearance
    // size_t fIdx = 0;                // <---- This value will equate to the number of Delaunay triangles
    // std::map<size_t, size_t> id_map;
    // for (const auto& e : q.facetList()) {
    //     if (e.isGood()) id_map[e.id()] = ++fIdx;
    // }
    
    // std::cout << "bingo";



    // Eigen::Index closestIdx;
    // Eigen::Matrix2Xd pointMatrix(2, myCloud.cols());
    // pointMatrix.colwise() = Eigen::Vector2d{-2.408555, 19.517879};
    // (pointMatrix - myCloud).array().square().colwise().sum().minCoeff(&closestIdx);
    
    // Eigen::Matrix2Xd pointMatrix = myCloud.colwise() - Eigen::Vector2d{-2.408555, 19.517879};
    // std::cout << myCloud << std::endl << std::endl;
    // // pointMatrix.colwise() -= Eigen::Vector2d{-2.408555, 19.517879};
    // std::cout << myCloud << std::endl << std::endl;
    // std::cout << pointMatrix << std::endl << std::endl;
    // pointMatrix.array().square().colwise().sum().minCoeff(&closestIdx);
    
    // (myCloud.colwise() - Eigen::Vector2d{-2.408555, 19.517879}).array().square().colwise().sum().minCoeff(&closestIdx);
    // std::cout << "closest index expected (2) actual = " << closestIdx << std::endl;
    // Eigen::Index bingo = (myCloud.colwise() - Eigen::Vector2d{-2.408555, 19.517879}).array().square().colwise().sum().minCoeff();
    // std::cout << "closest index expected (2) actual = " << bingo << std::endl;

    // Eigen::VectorXi minDADAD(myCloud.rows());
    // int i=0;
    // for (const auto& row : myCloud.rowwise()) {
    //     row.minCoeff(&minDADAD(i++));
    // } 
    // std::cout << "smallest val indices for each row\n" << minDADAD << std::endl;


    // Eigen::VectorXd minthings = myCloud.rowwise().minCoeff();
    // myCloud.rowwise().minCoeff
    // (myCloud.colwise() - Eigen::Vector2d{-2.408555, 19.517879}).array().square().colwise().sum().minCoeff(&closestIdx);
    // std::cout << "smallest val indices for each row\n" << minthings << std::endl;

    // Eigen::VectorXd bingo = (myCloud.colwise() - Eigen::Vector2d{-2.408555, 19.517879}).array().square().colwise().sum();
    // std::cout << bingo << std::endl << std::endl;
    // std::cout << (bingo.array() < 2) << std::endl << std::endl;
    
    // Eigen::Array<bool, -1, 1> mask = (myCloud.colwise() - Eigen::Vector2d{-2.408555, 19.517879}).array().square().colwise().sum().array() < 2;
    // Eigen::ArrayXi mask_idcs(mask.count(), 1);
    // for (int z = 0, z_idx = 0; z < mask.rows(); ++z) {
    //     if (mask(z)) mask_idcs(z_idx++) = z;
    // }
    // std::cout << mask_idcs << std::endl << std::endl;





    // // Read tree positions into QHull
    // orgQhull::PointCoordinates qhull_points(2, "");
    // qhull_points.append(myCloud.cols(), myCloud.data());

    // // Compute Delaunay Triangulation
    // orgQhull::Qhull q;
    // q.enableOutputStream();
    // q.runQhull(qhull_points.comment().c_str(), qhull_points.dimension(),
    //                 qhull_points.count(), qhull_points.coordinates(), "Qt Qbb Qc Qz Q12 d");
    // // TODO the option "Fx" computes the convex hull, too; would be useful to preallocate space for the edges and edgeLengths


    // // Delaunay regions as a vector of vectors
    // orgQhull::QhullFacetListIterator k(q.facetList());

    // // the facet ids are confusing, we want to map the good facets to order of appearance
    // size_t fIdx = 0, bad = 0;
    // std::map<size_t, size_t> id_map;
    // for (const auto& e : q.facetList()) {
    //     if (e.isGood()) id_map[e.id()] = ++fIdx;
    //     else ++bad;
        
        
        
    //     // TODO: COUNT BAD FACETS (SHOULD BE ON BOUNDARY --> SHOULD MATCH # POINTS ON CONVEX HULL?)




    // }
    // int a = 11, b = 13;
    // size_t aux = a;
    // if (b < a) {
    //     a = b;
    //     b = aux;
    // }
    // int commonVKey = (a + b) * (a + b + 1) / 2 + a;
    

    // // std::cout << bad << std::endl; 
    // // Eigen::Vector4d pV = {10, 11, 13, 14}, pE = {2, 3, 4, 1};
    // // Eigen::Vector3d nV = {12, 13, 11}, nE = {6, 3, 5};
    // // Eigen::Vector3d nV = {11, 13, 12}, nE = {5, 3, 6};


    
    // Eigen::Vector4d pV = {10, 11, 13, 14}, pE = {241, 311, 391, 310};
    // Eigen::Vector3d nV = {12, 13, 11}, nE = {337, 311, 287};
    // // Eigen::Vector3d nV = {11, 13, 12}, nE = {287, 311, 337};
    // bool isOpposingDirs = nV(0) == 12;
    // std::cout << pV.transpose() << std::endl << pE.transpose() << "\n$$$$$$$$$$$$\n";
    // std::cout << nV.transpose() << std::endl << nE.transpose() << "\n============\n";
    // // std::cout << nV.transpose() << " $ " << nE.transpose() << std::endl << std::endl;

    // if (isOpposingDirs) {
    //     nV.reverseInPlace(), nE.reverseInPlace();
    // }

    // int commonEKey = 3;

    // int pVI = -1, pEI = -1;
    // for (int i = 0, j = pV.size()-1; i < pV.size(); j=i++) {
    // // for (int i = 0, j = 1; i < pV.size(); j=(++i+1)%pV.size()) {
    //     if (pE(i) == commonEKey) pEI = i;
    //     if (pV(i) < pV(j) && commonVKey == (pV(i) + pV(j)) * (pV(i) + pV(j) + 1) / 2 + pV(i)) pVI = i;
    //     if (pV(j) < pV(i) && commonVKey == (pV(j) + pV(i)) * (pV(j) + pV(i) + 1) / 2 + pV(j)) pVI = i;
        
    //     // if (pV(i) < pV(j)) std::cout << "(j,i)=(" << pV(j) << "," << pV(i) << ") --> " << (pV(i) + pV(j)) * (pV(i) + pV(j) + 1) / 2 + pV(i) << std::endl;
    //     // if (pV(j) < pV(i)) std::cout << "(j,i)=(" << pV(j) << "," << pV(i) << ") --> " << (pV(j) + pV(i)) * (pV(j) + pV(i) + 1) / 2 + pV(j) << std::endl;
    // }
    // int nVI = -1, nEI = -1;
    // // for (int i = 0, j = (isOpposingDirs ? 1 : nV.size()-1); i < nV.size(); j=(isOpposingDirs ? (++i+1)%nV.size() : i++)) {
    // // for (int i = 0, j = nV.size()-1; i < nV.size(); j=i++) {
    // //     if (nE(i) == commonEKey) nEI = i;
    // //     if (nV(i) < nV(j) && commonVKey == (nV(i) + nV(j)) * (nV(i) + nV(j) + 1) / 2 + nV(i)) nVI = i;
    // //     if (nV(j) < nV(i) && commonVKey == (nV(j) + nV(i)) * (nV(j) + nV(i) + 1) / 2 + nV(j)) nVI = i;
    // // }
    // for (int i = 0, j = nV.size()-1; i < nV.size(); j=i++) {
    //     if (nE(i) == commonEKey) nEI = i;
    //     if (nV(i) < nV(j) && commonVKey == (nV(i) + nV(j)) * (nV(i) + nV(j) + 1) / 2 + nV(i)) nVI = i;
    //     if (nV(j) < nV(i) && commonVKey == (nV(j) + nV(i)) * (nV(j) + nV(i) + 1) / 2 + nV(j)) nVI = i;

    //     // if (nV(i) < nV(j)) std::cout << "(j,i)=(" << nV(j) << "," << nV(i) << ") --> " << (nV(i) + nV(j)) * (nV(i) + nV(j) + 1) / 2 + nV(i) << std::endl;
    //     // if (nV(j) < nV(i)) std::cout << "(j,i)=(" << nV(j) << "," << nV(i) << ") --> " << (nV(j) + nV(i)) * (nV(j) + nV(i) + 1) / 2 + nV(j) << std::endl;
    // }
    // // if (pVI != -1) ++pVI;
    // // if (pEI != -1) ++pEI;
    // std::cout << "pVI: " << pVI << ", pEI: " << pEI << std::endl;
    // std::cout << "nVI: " << nVI << ", nEI: " << nEI << "\n============\n";


    // // Eigen::Transpositions<3, 3> bongo(Eigen::Vector3i{1,2,0});
    // // Eigen::Transpositions<-1, -1> transBongo(bingo.size());
    // // std::cout << bingo.transpose() << std::endl;
    // // int bingoIdx = 4, boomboomIdx = 1;

    // // Eigen::VectorXd::LinSpaced hoot(3,0,2); 

    // Eigen::PermutationMatrix<-1, -1> permP(pV.size()), permN(nV.size());
    // Eigen::VectorXi permPIndices(pV.size()), permNIndices(nV.size());
    // int myPVI = permPIndices.size()-pVI;
    // for(int j = 0; j < permPIndices.size(); ++j) {
    //     permPIndices[j] = (myPVI+j)%permPIndices.size();
    //     // permPIndices[j] = (pVI+j)%permPIndices.size();
    // }
    // permP.indices() = permPIndices;
    
    // int myNVI = permNIndices.size()-nVI;
    // for(int j = 0; j < permNIndices.size(); ++j) {
    //     permNIndices[j] = (myNVI+j)%permNIndices.size();
    // }
    // permN.indices() = permNIndices;

    // // bingo = transBongo * bingo;
    // pV = permP * pV;
    // pE = permP * pE;
    // nV = permN * nV;
    // nE = permN * nE;
    // std::cout << pV.transpose() << std::endl << pE.transpose() << "\n$$$$$$$$$$$$\n";
    // std::cout << nV.transpose() << std::endl << nE.transpose() << "\n============\n";


    // Eigen::Matrix<int, 3, 6> opts;
    // // opts << 0, 1, 2,
    // //         0, 2, 1,
    // //         1, 2, 0,
    // //         1, 0, 2,
    // //         2, 1, 0,
    // //         2, 0, 1;
    // opts << 0, 0, 1, 1, 2, 2,
    //         1, 2, 2, 0, 1, 0,
    //         2, 1, 0, 2, 0, 1;

    // // opts << 0, 1, 0, 1, 2, 2,
    // //         1, 2, 2, 0, 1, 0,
    // //         2, 0, 1, 2, 0, 1;
    // // opts << 0, 0, 0, 0, 0, 0,
    // //         1, 0, 1, 1, 1, 1,
    // //         0, 0, 2, 2, 2, 2;

    // Eigen::Matrix<int, 3, 2> coolOpts;
    // coolOpts << 0, 1,
    //             2, 2,
    //             1, 0;

    // // std::cout << opts << std::endl;
    // Eigen::PermutationMatrix<3, 3> perm;
    // Eigen::Transpositions<3, 3> trans;
    // Eigen::Vector3d myLens = {3, 6, 9};

    // for (int c = 0; c < 6; ++c) {
    //     // trans.indices() = opts.col(c);
    //     // std::cout << "Trans " << c << ": " << (trans * myLens).transpose() << std::endl;
    //     perm.indices() = opts.col(c);
    //     // std::cout << "Perm  " << c << ": " << (perm  * myLens).transpose() << std::endl;
    //     std::cout << "Perm  " << c << ": " << (myLens.transpose()  * perm) << std::endl;
    // }

    // Eigen::PermutationMatrix<3, 3> dopePerm(Eigen::Vector3i{0,1,2});
    // Eigen::Vector3d myNewLens = {3, 6, 9};
    // // myNewLens.normalize();
    // std::cout << myNewLens.normalized() << std::endl;

    // for (int c = 0; c < 6; dopePerm.indices() = coolOpts.col(c++ % 2)) {
    //     // perm.indices() = opts.col(c);
    //     // std::cout << "Trans " << c << ": " << (trans * myLens).transpose() << std::endl;
    //     std::cout << "Perm  " << c << ": " << (dopePerm  * myNewLens).transpose() << std::endl;
    // }
    









    // Eigen::Array4i bingo = {0, 1, 2, 3};
    // Eigen::Array4i bango = bingo.shiftRight<3>();
    // std::cout << bango.transpose() << std::endl;

    // Eigen::Vector2i ldmkIndices({1,3});

    // Eigen::Matrix<double, 2, 4> A {
    //     {1, 2, 3, 4},
    //     {5, 6, 7, 8}
    // };
    // std::cout << A << std::endl;


    // // std::cout << A(Eigen::placeholders::all, ldmkIndices).rowwise().sum() << std::endl;
    // // std::cout << A(Eigen::placeholders::all, ldmkIndices) << std::endl;


    // Eigen::Vector2d centroid = A(Eigen::placeholders::all, ldmkIndices).rowwise().sum() / 2;
    // std::cout << centroid << std::endl;
    // // std::cout << centroid / 2 << std::endl;

    // // std::cout << A(Eigen::placeholders::all, 1) << std::endl;
    // // std::cout << A(Eigen::indexing::all, 1) << std::endl;
    // // Eigen::ArrayXXi B = A.shiftRight<2>();
    // // std::cout << A << std::endl;
    // std::cout << std::endl << std::endl << std::endl;




    // Eigen::Vector4d pV = {10, 11, 13, 14}, pE = {241, 311, 391, 310};
    // Eigen::Vector3d nV = {12, 13, 11}, nE = {337, 311, 287};

    // std::cout << pV.middleRows(1, pV.size()-2);
    // std::cout << pV.middleRows(1, 2);








    // int a = 11, b = 13;
    // size_t aux = a;
    // if (b < a) {
    //     a = b;
    //     b = aux;
    // }
    // int commonVKey = (a + b) * (a + b + 1) / 2 + a;

    
    // Eigen::Vector4d pV = {10, 11, 13, 14}, pE = {241, 311, 391, 310};
    // Eigen::Vector3d nV = {12, 13, 11}, nE = {337, 311, 287};
    // // Eigen::Vector3d nV = {11, 13, 12}, nE = {287, 311, 337};
    // bool isOpposingDirs = nV(0) == 12;
    // std::cout << pV.transpose() << std::endl << pE.transpose() << "\n$$$$$$$$$$$$\n";
    // std::cout << nV.transpose() << std::endl << nE.transpose() << "\n============\n";

    // if (isOpposingDirs) {
    //     nV.reverseInPlace(), nE.reverseInPlace();
    // }

    // // FUNCTION Q
    // int pVI = -1;
    // for (int i = 0; i < pV.size(); ++i) {
    //     if (pE(i) == commonVKey) {
    //         pVI = i; continue;
    //     }
    // }
    // // FUNCTION Q
    // int nVI = -1;
    // for (int i = 0; i < nV.size(); ++i) {
    //     if (nE(i) == commonVKey) {
    //         nVI = i; continue;
    //     }
    // }
    // std::cout << "pVI: " << ++pVI << std::endl; // NOTE remember pVI must be incremented
    // std::cout << "nVI: " << nVI << "\n============\n";


    // // Function W
    // Eigen::PermutationMatrix<-1, -1> permP(pV.size()), permN(nV.size());
    // Eigen::VectorXi permPIndices(pV.size()), permNIndices(nV.size());
    // int myPVI = permPIndices.size()-pVI;
    // for(int j = 0; j < permPIndices.size(); ++j) {
    //     permPIndices[j] = (myPVI+j)%permPIndices.size();
    // }
    // permP.indices() = permPIndices;
    // pV = permP * pV;
    // pE = permP * pE;
    
    // // Function W
    // int myNVI = permNIndices.size()-nVI;
    // for(int j = 0; j < permNIndices.size(); ++j) {
    //     permNIndices[j] = (myNVI+j)%permNIndices.size();
    // }
    // permN.indices() = permNIndices;
    // nV = permN * nV;
    // nE = permN * nE;

    // std::cout << pV.transpose() << std::endl << pE.transpose() << "\n$$$$$$$$$$$$\n";
    // std::cout << nV.transpose() << std::endl << nE.transpose() << "\n============\n";

    // Eigen::VectorXd combinedVRef(4 + 3 - 2), combinedERef(4 + 3 - 2);
    // combinedVRef << pV.head(pV.size()-1), nV.tail(nV.size()-1);
    // combinedERef << pE.head(pE.size()-1), nE.tail(nE.size()-1);

    // std::cout << combinedVRef.transpose() << "\n$$$$$$$$$$$$\n" << combinedERef.transpose() << std::endl;


    return 0;

}



//// the dungeon
// inline std::vector<std::pair<Eigen::Index, Eigen::Index>> coolRANSAC(const Points& localLandmarks, const Points& globalLandmarks,
//         const std::vector<std::pair<Eigen::Index, Eigen::Index>>& allMatches,
//         std::vector<std::pair<Eigen::Index, Eigen::Index>>& inliers,
//         double validDistAfterTf,
//         double matchRatio,
//         int maxIter) {
//             std::random_device randomDevice;
//             std::mt19937 rng(randomDevice);
//             // IDEA: using some subset of matches, that subset will be valid if most of the points will be close to nearby existing points after tf 

//             // Init indices to access the matches
//             std::vector<int> indices(allMatches.size());
//             std::iota(indices.begin(), indices.end(), 0);

//             int expectedNumMatchedLandmarks = localLandmarks.cols() * matchRatio;

            

//             for (int i = 0; i < maxIter; ++i) {
//                 std::vector<std::pair<Eigen::Index, Eigen::Index>> goodMatchesIFound;
//                 std::shuffle(indices.begin(), indices.end(), rng);

//                 // use first 4 matches for now
//                 Eigen::Matrix3d potentialTf = tfFromSubsetMatches(localLandmarks, globalLandmarks, allMatches, indices, 4);


//                 std::unordered_set<Eigen::Index> globalMatchedPointIndices;
//                 for (int lIdx = 0; lIdx < localLandmarks.cols(); ++lIdx) {
//                     // Define this tree's local position as a homogeneous matrix
//                     Eigen::Matrix3d localPointTf {
//                         {1, 0, localLandmarks(0, lIdx)},
//                         {0, 1, localLandmarks(1, lIdx)},
//                         {0, 0, 1},
//                     };
//                     // Obtain this point's observed position relative to the global frame
//                     PtLoc globalXY{(potentialTf * localPointTf)(Eigen::seq(0,1), 2)};
                    
//                     // Determine whether any global landmark is close enough to this point after it has been transformed 
//                     double nearestSqDistToPoint = 1;  // meters
//                     int nearestPointId = -1;
//                     for (int gIdx = 0; gIdx < globalLandmarks.cols(); ++gIdx) {
//                         if (globalMatchedPointIndices.find(gIdx) != globalMatchedPointIndices.end()) continue;
//                         double sqDiff = squaredDistance2D(globalXY, globalLandmarks.col(gIdx));
//                         if (sqDiff < nearestSqDistToPoint) nearestSqDistToPoint = sqDiff, nearestPointId = gIdx;
//                     }
//                     if (nearestPointId != -1) {
//                         goodMatchesIFound.push_back({lIdx, nearestPointId});
//                         globalMatchedPointIndices.insert(nearestPointId);
//                     }
//                 }

//                 if (goodMatchesIFound.size() >= expectedNumMatchedLandmarks) {
//                     // valid tf

//                 }


//             }

//             // IDEA: if we see a certain number of TFs that are pretty close to each other, use one of those
//         }


// std::vector<Eigen::Index> myRANSAC(const Points& localLandmarks, const Points& globalLandmarks,
//         const std::vector<std::pair<Eigen::Index, Eigen::Index>>& allMatches,
//         std::vector<std::pair<Eigen::Index, Eigen::Index>>& inliers,
//         double validDistAfterTf,
//         double matchRatio,
//         int maxIter) {
//             std::random_device randomDevice;
//             std::mt19937 rng(randomDevice);
//             // IDEA: using some subset of matches, that subset will be valid if most of the points will be close to nearby existing points after tf 

//             // Init indices to access the matches
//             std::vector<int> indices(allMatches.size());
//             std::iota(indices.begin(), indices.end(), 0);

//             int expectedNumMatchedLandmarks = localLandmarks.cols() * matchRatio;

//             // use cantor pair here
//             // std::unordered_map<std::pair<Eigen::Index, Eigen::Index>, int> associationVotes; 
//             // std::unordered_map<int, int> associationVotes;

//             // rows -> local ldmk ids | cols -> global ldmk ids
//             Eigen::Matrix<int, -1, -1> associationVotes = Eigen::Matrix<int,-1,-1>::Zero(localLandmarks.cols(), globalLandmarks.cols());

            
//             // maybe just have maxIter be 50
//             // for (int i = 0; i < maxIter && inliers.size() < expectedNumMatchedLandmarks; ++i) {
//             for (int i = 0; i < maxIter; ++i) {
//                 std::vector<std::pair<Eigen::Index, Eigen::Index>> goodMatchesIFound;
//                 std::shuffle(indices.begin(), indices.end(), rng);

//                 // use first 4 matches for now
//                 Eigen::Matrix3d potentialTf = tfFromSubsetMatches(localLandmarks, globalLandmarks, allMatches, indices, 4);


//                 std::unordered_set<Eigen::Index> globalMatchedPointIndices;
//                 for (int lIdx = 0; lIdx < localLandmarks.cols(); ++lIdx) {
//                     // Define this tree's local position as a homogeneous matrix
//                     Eigen::Matrix3d localPointTf {
//                         {1, 0, localLandmarks(0, lIdx)},
//                         {0, 1, localLandmarks(1, lIdx)},
//                         {0, 0, 1},
//                     };
//                     // Obtain this point's observed position relative to the global frame
//                     PtLoc globalXY{(potentialTf * localPointTf)(Eigen::seq(0,1), 2)};
                    
//                     // Determine whether any global landmark is close enough to this point after it has been transformed 
//                     double nearestSqDistToPoint = validDistAfterTf;  // meters
//                     int nearestPointId = -1;
//                     for (int gIdx = 0; gIdx < globalLandmarks.cols(); ++gIdx) {
//                         if (globalMatchedPointIndices.find(gIdx) != globalMatchedPointIndices.end()) continue;
//                         double sqDiff = squaredDistance2D(globalXY, globalLandmarks.col(gIdx));
//                         if (sqDiff < nearestSqDistToPoint) nearestSqDistToPoint = sqDiff, nearestPointId = gIdx;
//                     }
//                     if (nearestPointId != -1) {
//                         goodMatchesIFound.push_back({lIdx, nearestPointId});
//                         globalMatchedPointIndices.insert(nearestPointId);
//                     }
//                 }

//                 if (goodMatchesIFound.size() >= expectedNumMatchedLandmarks) {
//                     // valid tf

//                 }


//             }

//             // IDEA: if we see a certain number of TFs that are pretty close to each other, use one of those
//         }