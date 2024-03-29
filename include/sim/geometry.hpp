#include <string>
#include <cmath>
#include <vector>
#include <fstream>
#include <sstream>

constexpr double PI = 3.14159265358;
constexpr double HALFPI = PI/2;
constexpr double TWOPI = 2*PI;

// Geometric Primitives

struct Point
{
    double x, y;
    Point() : x(0), y(0) {}
    Point(double a, double b) : x(a), y(b) {}
    Point(Point p, double a, double b) : x(p.x + a), y(p.y + b) {}
    bool operator==(const Point &other) const{ return x == other.x && y == other.y; }
    void translate(double a, double b) {x+=a, y+=b;}
    std::string toString() const { return std::to_string(x) + " " + std::to_string(y); }
};

struct Pose
{
    Point p;
    double theta, linearV = 0, angularV = 0; // angles in radians
    // currently assuming no movement when defining pose
    Pose() : p(Point()), theta(0) {}
    Pose(double a, double b, double c) : p(Point(a,b)), theta(c) {}
    Pose(Point point, double t) : p(point), theta(t) {}
    std::string printPose() const { return p.toString() + " " + std::to_string(theta); }
};

struct Tree {
    Point p;
    double radius;
    Tree() : p(Point()), radius(-1) {}
    Tree(double a, double b, double r) : p(Point(a,b)), radius(r) {}
    Tree(Tree t, double a, double b, double r) : p(Point(t.p,a,b)), radius(r) {}
    bool operator==(const Tree &other) const{ return p == other.p && radius == other.radius; }
    bool isEmpty() { return radius == -1; }
    void applyNoise(double xNoise, double yNoise) { p.translate(xNoise, yNoise); }
    std::string toString() const { return p.toString() + " " + std::to_string(radius); }
};

// Geometric Validation Methods

struct Avoids {
    const Point rP; const double cW;
    Avoids(Point randomPoint, double collisionWidth) : rP(randomPoint), cW(collisionWidth) {}
    bool operator()(Tree t) const {
        double xDiff = t.p.x - rP.x, yDiff = t.p.y - rP.y, berth = t.radius + cW;
        return xDiff*xDiff + yDiff*yDiff > berth*berth;
    }
};

struct NoStraightPathCollisions {
    const Point startPoint; const float pathLength; const double robotWidth; const double pathAngle;
    const double sinPathAngle; const double cosPathAngle; const double sinPerpAngle; const double cosPerpAngle; 
    NoStraightPathCollisions(Pose sP, float len, double rW) : startPoint(sP.p), pathLength(len), robotWidth(rW), pathAngle(sP.theta+HALFPI),
        sinPathAngle(sin(pathAngle)), cosPathAngle(cos(pathAngle)), sinPerpAngle(sin(sP.theta)), cosPerpAngle(cos(sP.theta)) {}
    
    bool operator()(Tree t) const {
        // Return true if the width of the robot and tree is either:
        //  - smaller than the unsigned distance to the line of the robot's path
        //  - smaller than the signed distance to the line perpendicular to the starting point of the path
        //  - smaller than the signed distance to the line perpendicular to the ending point of the path
        double width = robotWidth + t.radius, xDiff = startPoint.x - t.p.x, yDiff = startPoint.y - t.p.y;
        return std::abs(cosPathAngle*yDiff - sinPathAngle*xDiff) > width ||
            width <= cosPerpAngle*yDiff - sinPerpAngle*xDiff ||     // Trust the C++ code optimizer
            cosPerpAngle*yDiff - sinPerpAngle*xDiff < -pathLength - width;

        // For the perpendicular line at the starting point of the path:
        // Signed Dist  {                (-)            |   (+)
        // Pathway      {                End   <----- Start
        //              {       |---------|=============|---------|
        // Measurements { ______|__width__|___pathLen___|__width__|______
        // NoCollision? {  TRUE |              FALSE              | TRUE
    }
};


std::vector<Tree> readForestFile(std::string path) {
    // Assume each line in the file describes a point in 2D (space-separated)
    std::vector<Tree> forest;
    std::ifstream infile(path);
    double xPosition,yPosition,radius;

    std::string line;
    while (std::getline(infile, line))
    {
        std::istringstream iss(line);
        if (iss >> xPosition >> yPosition >> radius) { // only collect the tree data
            forest.push_back(Tree(xPosition, yPosition, radius));
        }
    }

    infile.close();
    return forest;
}