#include <matching.hpp>
#include <fstream>
#include <sstream>
#include <string>

using namespace std;

int getNumPoints(string path) {
    int number_of_lines = 0;
    std::ifstream myfile(path);
    std::string line;

    // Get number of lines in file
    while (std::getline(myfile, line)) ++number_of_lines;
    myfile.close();
    return number_of_lines;
}

void readCoolFile(string path, Points& landmarks) { // from my sim outputs
    std::ifstream infile(path);
    std::string line;
    double xPosition,yPosition,radius;
    int i = 0;
    while (std::getline(infile, line)) {
        std::istringstream iss(line); // only collect the tree data
        if (iss >> xPosition >> yPosition >> radius) landmarks.col(i++) = PtLoc{xPosition, yPosition};
    }

    infile.close();
}

int main(int argc, char* argv[]) {
    if (argc > 2) {
        Points points1(2, getNumPoints(string(argv[1])));
        readCoolFile(string(argv[1]), points1);
        urquhart::Observation obs1(points1);
        std::cout << "Obs 1 Polygons: " << obs1.H->get_children(0).size() << std::endl;
        // obs1.view();

        Points points2(2, getNumPoints(string(argv[2])));
        readCoolFile(string(argv[2]), points2);
        urquhart::Observation obs2(points2);
        std::cout << "Obs 2 Polygons: " << obs2.H->get_children(0).size() << std::endl;
        // obs2.view();

        std::cout << "Matching Time: " << std::endl;
        auto matches = matching::hierarchyMatching(obs1, obs2, 5);
        for (const auto& [refPoint, targPoint] : matches) {
            std::cout << refPoint(0) << "," << refPoint(1) << "|";
            std::cout << targPoint(0) << "," << targPoint(1) << std::endl;
        }
        return 0;

    } else {
        std::cout << "Please pass the paths of both landmark position files as argument" << std::endl;
    }
}
