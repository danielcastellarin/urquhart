#include <matching.hpp>
#include <fstream>
#include <sstream>
#include <string>

using namespace std;

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
    if (argc > 2) {
        Points points1 = readCoolFile(string(argv[1]));
        urquhart::Observation obs1(points1);
        std::cout << "Obs 1 Polygons: " << std::endl;
        obs1.view();

        Points points2 = readCoolFile(string(argv[2]));
        urquhart::Observation obs2(points2);
        std::cout << "Obs 2 Polygons: " << std::endl;
        obs2.view();

        std::cout << "Matching Time: " << std::endl;
        // auto matches = matching::hierarchyIndexMatching(obs1, obs2, 5);
        auto matches = matching::nonGreedyHierarchyIndexMatching(obs1, obs2, 5, 3, 0.5);
        for (const auto& [refIdx, targIdx] : matches) {
            std::cout << obs1.ldmkX(refIdx) << "," << obs1.ldmkY(refIdx) << "|";
            std::cout << obs2.ldmkX(targIdx) << "," << obs2.ldmkY(targIdx) << std::endl;
        }
        return 0;

    } else {
        std::cout << "Please pass the paths of both landmark position files as argument" << std::endl;
    }
}
