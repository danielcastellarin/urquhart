#include <hype_matching.hpp>
#include <hype_observation.hpp>
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
        // std::cout << points1 << std::endl;
        // std::cout << "bing bong" << std::endl;
        hype_urq::Observation obs1(points1);
        std::cout << "Obs 1 Polygons: " << std::endl;
        obs1.view();
        // abort();

        Points points2 = readCoolFile(string(argv[2]));
        hype_urq::Observation obs2(points2);
        std::cout << "Obs 2 Polygons: " << std::endl;
        obs2.view();

        std::cout << "Matching Time: " << std::endl;
        auto matches = hype_matching::hierarchyMatching(obs1, obs2, 5);
        for (const auto& [refPoint, targPoint] : matches) {
            std::cout << refPoint(0) << "," << refPoint(1) << "|";
            std::cout << targPoint(0) << "," << targPoint(1) << std::endl;
        }
        return 0;

        // TODO COMPARE RESULTS WITH OG BACKEND EXAMPLE
        // potential diff caused by different triangulation --> i.e. points read in different config to QHull
        // Can test by looking at landmarks from both versions of the code and seeing if they match

    } else {
        std::cout << "Please pass the paths of both landmark position files as argument" << std::endl;
    }
}
