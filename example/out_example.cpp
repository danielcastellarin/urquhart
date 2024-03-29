#include <matching.hpp>
#include <logging.hpp>
#include <fstream>
#include <sstream>
#include <string>

using namespace std;


void writeHierarchyFiles(const urquhart::Observation& trees, std::string fileName) {
    std::ofstream plyOut(fileName+"-p.txt"), triOut(fileName+"-t.txt"), 
                    hieOut(fileName+"-h.txt"), dscOut(fileName+"-d.txt");
    
    logging::writeHierarchyToFile(trees, plyOut, triOut, hieOut, dscOut);
    
    hieOut.close();
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
        writeHierarchyFiles(obs1, "1"+num);

        std::ofstream pts1Out("1"+num+"-pts.txt");
        for (int i = 0; i < obs1.landmarks.cols(); ++i) {
            pts1Out << obs1.landmarks(0, i) << " " << obs1.landmarks(1, i) << std::endl; 
        }
        pts1Out.close();

        Points points2 = readCoolFile(string(argv[2]));
        urquhart::Observation obs2(points2);
        std::cout << "Obs 2 Polygons: " << std::endl;
        obs2.view();
        writeHierarchyFiles(obs2, "2"+num);

        std::ofstream pts2Out("2"+num+"-pts.txt");
        for (int i = 0; i < obs2.landmarks.cols(); ++i) {
            pts2Out << obs2.landmarks(0, i) << " " << obs2.landmarks(1, i) << std::endl; 
        }
        pts2Out.close();

        std::cout << "Matching Time: " << std::endl;
        auto matches = matching::hierarchyIndexMatching(obs1, obs2, 5);
        std::ofstream matOut(num+"-m.txt");
        for (const auto& [refIdx, targIdx] : matches) {
            std::cout << obs1.ldmkX(refIdx) << "," << obs1.ldmkY(refIdx) << "|";
            std::cout << obs2.ldmkX(targIdx) << "," << obs2.ldmkY(targIdx) << std::endl;
            matOut << obs1.ldmkX(refIdx) << "," << obs1.ldmkY(refIdx) << "|";
            matOut << obs2.ldmkX(targIdx) << "," << obs2.ldmkY(targIdx) << std::endl;
        }
        matOut.close();
        return 0;

    } else {
        std::cout << "Please pass the paths of both landmark position files as argument" << std::endl;
    }
}
