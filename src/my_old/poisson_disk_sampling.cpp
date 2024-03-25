#include <vector>
#include <random>
#include <cstdlib>
#include <string>
#include <algorithm>
#include <unordered_map>

#include <iostream>
#include <fstream>

/*  Design decision note: (Randomization of tree radii)

Initially, it felt awkward that I was sampling a uniform distribution to determine the radius of each tree.
My gut feeling was that in actual forests, tree width will not be uniformly random in a local area, but some other distribution.
For now, sampling a uniform distribution will work.
*/


constexpr double PI = 3.14159265358;

// Code should error if invalid run configuration provided
struct ForestConfig
{
    // Required Parameters
    float forestHeight, forestWidth, forestSize, treeSpacing,
    // Optional Parameters
    treeRadiusMin = 0, treeRadiusMax = 0,
    // treeRadiusMean = 0, treeRadiusStdDev = 0,
    noiseMean = 0, noiseStdDev = 0;
    int samplingAttempts = 30, randomSeed = -1;
    std::string outputFileName;

    // Each config file is designed to be a static run config
    ForestConfig(std::string path)
    {
        // Assume each line is describes a key-value pair (space-separated)
        std::ifstream infile(path);
        std::string key,value;
        std::unordered_map<std::string, std::string> options;
        while (infile >> key >> value) options[key] = value;
        infile.close();

        // Required parameters
        if (options.find("forestSize") != options.end()) {
            forestSize = atof(options["forestSize"].c_str());
            forestHeight = forestSize, forestWidth = forestSize;
        } else {
            forestHeight = atof(options["forestHeight"].c_str()), forestWidth = atof(options["forestWidth"].c_str());
            forestSize = forestHeight*forestWidth;
        }
        treeSpacing = atof(options["treeSpacing"].c_str());

        // Optional parameters
        if (options.find("treeRadiusMin") != options.end()) treeRadiusMin = atof(options["treeRadiusMin"].c_str());
        if (options.find("treeRadiusMax") != options.end()) treeRadiusMax = atof(options["treeRadiusMax"].c_str());
        // if (options.find("treeRadiusMean") != options.end()) treeRadiusMean = atof(options["treeRadiusMean"].c_str());
        // if (options.find("treeRadiusStdDev") != options.end()) treeRadiusStdDev = atof(options["treeRadiusStdDev"].c_str());
        if (options.find("samplingAttempts") != options.end()) samplingAttempts = atoi(options["samplingAttempts"].c_str());
        if (options.find("noiseMean") != options.end()) noiseMean = atof(options["noiseMean"].c_str());
        if (options.find("noiseStdDev") != options.end()) noiseStdDev = atof(options["noiseStdDev"].c_str());
        if (options.find("randomSeed") != options.end()) randomSeed = atoi(options["randomSeed"].c_str());
        if (options.find("outputFileName") != options.end() && options["outputFileName"].find_first_not_of("abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ01234567890_-") == std::string::npos)
            outputFileName = options["outputFileName"];
        else
            outputFileName = (noiseMean || noiseStdDev ? "noisy-" : std::string("")) + 
                (forestSize >= 10000 ? "big" : (forestSize > 100 ? "medium" : "small")) + std::string("-") +
                (treeSpacing >= 10 ? "sparse" : (treeSpacing > 5 ? "avg" : "dense")) + std::string("-forest.txt");
    }
    bool isValid() { // TODO
        // treespacing << forestSize, and both these values are present
        return treeSpacing && forestSize > treeSpacing + treeRadiusMax + treeRadiusMax;
    }
    void outputConfig(std::ostream& out) { 
        out << "Forest height: " << forestHeight << std::endl;
        out << "Forest width: " << forestWidth << std::endl;
        out << "Tree spacing: " << treeSpacing << std::endl;
        out << "Minimum tree radius: " << treeRadiusMin << std::endl;
        out << "Maximum tree radius: " << treeRadiusMax << std::endl;
        // out << "Mean of gaussian sampled when determining tree radius: " << treeRadiusMean << std::endl;
        // out << "Standard Deviation of gaussian sampled when determining tree radius: " << treeRadiusStdDev << std::endl;
        out << "Sampling attempts per tree: " << samplingAttempts << std::endl;
        out << "Mean of gaussian sampled when applying noise to tree positions: " << noiseMean << std::endl;
        out << "Standard Deviation of gaussian sampled when applying noise to tree positions: " << noiseStdDev << std::endl;
        out << "Random seed used: " << (randomSeed == -1 ? "None" : std::to_string(randomSeed)); // << std::endl;
    }
};

struct Tree {
    double x, y, radius;
    Tree() : x(-1), y(-1), radius(-1) {}
    Tree(double a, double b, double r) : x(a), y(b), radius(r) {}
    bool isEmpty() { return this->x == this->y == this->radius == -1; }
    void applyNoise(double xNoise, double yNoise) { x += xNoise; y += yNoise; }
    std::string toString() { return std::to_string(x) + " " + std::to_string(y) + " " + std::to_string(radius); }
};

bool insertIfValidTree(Tree grid[], float cellSize, int numCols, int gwidth, int gheight, Tree t, float treeSpacing, std::vector<Tree>& forest, std::vector<Tree>& treesToSample) {
    // If this tree is out-of-bounds --> exit early
    if (t.x - t.radius < 0 || t.x + t.radius > gwidth || t.y - t.radius < 0 || t.y + t.radius > gheight) return false;
    
    // Find the cell that the given tree will be placed in
    int xIdx = floor(t.x / cellSize), yIdx = floor(t.y / cellSize);

    // Get the indices for the cells immediately neighboring this tree's cell
    int smallest_row = std::max(xIdx - 1, 0);
    int largest_row = std::min(xIdx + 1, gheight - 1);
    int smallest_col = std::max(yIdx - 1, 0);
    int largest_col = std::min(yIdx + 1, gwidth - 1);

    // Loop over these nine cells, check if they contain trees that are closer than the minimum allowed distance to this tree
    for (int row = smallest_row; row <= largest_row; row++) {
        for (int col = smallest_col; col <= largest_col; col++) {
            Tree adjTree = grid[row*numCols+col];   // Get the tree in this cell
            if (!adjTree.isEmpty()) {
                // distance should account for tree radius!
                // d^2 > xDiff^2 + yDiff^2
                // d = a + r1 + r2 ----> (a + r1 + r2)^2 > xDiff^2 + yDiff^2
                double xDiff = adjTree.x - t.x, yDiff = adjTree.y - t.y;
                double combinedTreeWidthAndSpacing = t.radius + adjTree.radius + treeSpacing;
                if (xDiff*xDiff + yDiff*yDiff < combinedTreeWidthAndSpacing*combinedTreeWidthAndSpacing) return false;
            }
        }
    }

    // Record the new tree
    forest.push_back(t);
    treesToSample.push_back(t);
    grid[xIdx*numCols+yIdx] = t;    // hack
    // skip to row "xidx", get column # "yidx"
    
    return true;
}

std::vector<Tree> poissonDiskSamplingWithRadius(ForestConfig cfg) {
    int N = 2; // This indicates the number of dimensions in space we will be sampling from
    std::vector<Tree> forest, treesToSample;
    double minTreeSpacing = cfg.treeSpacing+cfg.treeRadiusMin+cfg.treeRadiusMin;

    // Initialize RNG and first point
    std::random_device rd;
    std::mt19937 rng(rd());
    if (cfg.randomSeed >= 0) rng.seed(cfg.randomSeed);
    std::uniform_real_distribution<double> radians(0, 2*PI);
    std::uniform_real_distribution<double> getTreeDist(minTreeSpacing, 2*minTreeSpacing);
    std::uniform_real_distribution<double> randTreeRadius(cfg.treeRadiusMin, cfg.treeRadiusMax);
    // std::normal_distribution<double> randTreeRadius(cfg.treeRadiusMean, cfg.treeRadiusStdDev);

    // Grid initialization
    float cellSize = minTreeSpacing/sqrt(N);
    int numRows = ceil(cfg.forestHeight/cellSize)+1;
    int numCols = ceil(cfg.forestWidth/cellSize)+1;
    Tree* grid = new Tree[numRows*numCols];
    std::cout << "Cell Size: " << cellSize << std::endl;
    std::cout << "Grid Size: rows=" << numRows << ", cols=" << numCols << std::endl;

    // Insert first tree (note that height and width are flipped, since height analogous with rows and width analogous with columns in the grid)
    Tree tree = Tree(std::uniform_real_distribution<double>{cfg.treeRadiusMax, cfg.forestWidth-cfg.treeRadiusMax}(rng), std::uniform_real_distribution<double>{cfg.treeRadiusMax, cfg.forestHeight-cfg.treeRadiusMax}(rng), randTreeRadius(rng));
    insertIfValidTree(grid, cellSize, numCols, cfg.forestWidth, cfg.forestHeight, tree, cfg.treeSpacing, forest, treesToSample);

    // Continue while we have trees that are available for sampling
    while (!treesToSample.empty()) {
        // Randomly choose a tree to base the search for other trees
        int idx = std::uniform_int_distribution<int>{0, treesToSample.size()-1}(rng);
        tree = treesToSample[idx];

        // Iteratively try to find other valid trees nearby this tree
        bool isFound = false;
        for (int tries = 0; !isFound && tries < cfg.samplingAttempts; ++tries) {
            // Sample a new potential tree
            double theta = radians(rng), dist = getTreeDist(rng);
            Tree newTree = Tree(tree.x + dist * cos(theta), tree.y + dist * sin(theta), randTreeRadius(rng));

            // Insert the new tree into the grid if it is valid
            isFound = insertIfValidTree(grid, cellSize, numCols, cfg.forestWidth, cfg.forestHeight, newTree, cfg.treeSpacing, forest, treesToSample);
        }

        // Remove this tree from our list if we are not likely to sample a valid tree from it again
        if (!isFound) treesToSample.erase(treesToSample.begin()+idx);
    }
    if (forest.size()) std::cout << "Number of trees generated: " << forest.size() << std::endl;
    else std::cout << "Failed to generate forest; initial tree definition was: " << tree.toString() << std::endl;

    // Apply noise to every tree; we don't care if the trees overlap...
    if (cfg.noiseStdDev || cfg.noiseMean) {
        std::normal_distribution<double> gauss(cfg.noiseMean, cfg.noiseStdDev);
        for (auto& t : forest) t.applyNoise(gauss(rng), gauss(rng));
        std::cout << "Noise applied to trees in forest." << std::endl;
    }

    // Cleanup and return
    delete[] grid;
    return forest;
}

/**
 * We simulate a 1 km2 or approximately 247 acres forest. To
ensure a consistent density of trees across the map, the set of
2-D landmarks is generated by Poisson-Disc sampling through
Bridson’s algorithm [25] with a minimum distance between
points of 7 m.
 * 
*/

/**
 *  To account for this, each
point in the set is perturbed with Gaussian noise with 0 mean
and 3 m standard deviation.
*/

// units are meters
int main(int argc, char* argv[]){
    if (argc > 1) {
        // Parse config file and display this run's configs
        ForestConfig cfg(argv[1]);
        cfg.outputConfig(std::cout);
        std::cout << std::endl;

        if (!cfg.isValid()) return -1;
        std::vector<Tree> forest = poissonDiskSamplingWithRadius(cfg);

        // Save this forest environment
        std::system("mkdir forests");
        std::ofstream forestOut("forests/"+ cfg.outputFileName, std::ios::trunc);
        cfg.outputConfig(forestOut);
        forestOut << std::endl << "Number of trees in forest: " << forest.size() << std::endl;
        forestOut << "\nRUN COMMAND:";
        for (int i = 0; i < argc; i++) forestOut << " " << argv[i];
        forestOut << std::endl << std::endl;
        for (const Tree& t : forest) forestOut << t.x << " " << t.y << " " << t.radius << std::endl;
        forestOut.close();

    } else {
        std::cout << "Please provide a configuration file with run parameters for the simulation." << std::endl;
    }
}

