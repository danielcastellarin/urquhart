#include <matching.hpp>
#include <fstream>
#include <sstream>
#include <string>

using namespace std;


void writeHierarchyToFile(const urquhart::Observation& trees, std::string fileName) {
    std::ofstream plyOut(fileName+"-p.txt"), triOut(fileName+"-t.txt"), 
                    hieOut(fileName+"-h.txt"), dscOut(fileName+"-d.txt");
    trees.hier->viewTree(hieOut);
    hieOut.close();
    
    // Iterate over the indices of the Polygons in the hierarchy
    for(auto pIdx : trees.hier->getChildrenIds(0)) {
        for (int i = 0; i < trees.hier->getPolygon(pIdx).landmarkRefs.size(); ++i) {
            auto myPoint =  trees.landmarks.col(trees.hier->getPolygon(pIdx).landmarkRefs(i));
            plyOut << pIdx << " " << myPoint[0] << " " << myPoint[1] << "|";
        }
        plyOut << std::endl;
        for(auto d : trees.hier->getPolygon(pIdx).descriptor) dscOut << d << " ";
        dscOut << std::endl;
        
        // Iterate over the indices of the Triangles that compose this Polygon
        for(auto tIdx : trees.hier->getChildrenIds(pIdx)) {
            // Retain only the Polygon objects that have three sides
            if (trees.hier->getPolygon(tIdx).n == 3) {
                for (int i = 0; i < trees.hier->getPolygon(tIdx).landmarkRefs.size(); ++i) {
                    auto myPoint = trees.landmarks.col(trees.hier->getPolygon(tIdx).landmarkRefs(i));
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

Points readCoolFile(string path) {
    ifstream infile(path);
    std::vector<Eigen::Vector2d> intake;

    while (infile) {
        string s;
        if (!getline(infile, s)) break;

        istringstream ss(s);
        PtLoc record;
        int idx = -1;
        while (ss) {
            string s;
            if (!getline(ss, s, ',')) break;
            record(++idx) = stod(s);
        }
        intake.push_back(record);
    }
    if (!infile.eof()) {
        cerr << "Fooey!\n";
    }

    Points landmarks(2, intake.size());
    for (int i = 0; i < intake.size(); ++i) {
        landmarks.col(i) = intake[i];
    }

    return landmarks;
}

int main(int argc, char* argv[]) {
    if (argc > 3) {
        std::string num = string(argv[3]);

        Points points1 = readCoolFile(string(argv[1]));
        urquhart::Observation obs1(points1);
        std::cout << "Obs 1 Polygons: " << std::endl;
        obs1.view();
        writeHierarchyToFile(obs1, "1"+num);

        std::ofstream pts1Out("1"+num+"-pts.txt");
        for (int i = 0; i < obs1.landmarks.cols(); ++i) {
            pts1Out << obs1.landmarks(0, i) << " " << obs1.landmarks(1, i) << std::endl; 
        }
        pts1Out.close();

        Points points2 = readCoolFile(string(argv[2]));
        urquhart::Observation obs2(points2);
        std::cout << "Obs 2 Polygons: " << std::endl;
        obs2.view();
        writeHierarchyToFile(obs2, "2"+num);

        std::ofstream pts2Out("2"+num+"-pts.txt");
        for (int i = 0; i < obs2.landmarks.cols(); ++i) {
            pts2Out << obs2.landmarks(0, i) << " " << obs2.landmarks(1, i) << std::endl; 
        }
        pts2Out.close();

        std::ofstream matOut(num+"-m.txt");
        std::cout << "Matching Time: " << std::endl;
        auto matches = matching::hierarchyMatching(obs1, obs2, 5);
        for (const auto& [refPoint, targPoint] : matches) {
            std::cout << refPoint(0) << "," << refPoint(1) << "|";
            std::cout << targPoint(0) << "," << targPoint(1) << std::endl;
            matOut << refPoint(0) << "," << refPoint(1) << "|";
            matOut << targPoint(0) << "," << targPoint(1) << std::endl;
        }
        matOut.close();
        return 0;

    } else {
        std::cout << "Please pass the paths of both landmark position files as argument" << std::endl;
    }
}
