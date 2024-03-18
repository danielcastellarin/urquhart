#include <string>
#include <cmath>

constexpr double PI = 3.14159265358;
constexpr double HALFPI = PI/2;
constexpr double TWOPI = 2*PI;

// Geometric Primitives

// struct Landmark
// {
//     int id;
//     double x, y;
//     Landmark() : x(0), y(0), id(0) {}
//     Landmark(double a, double b) : x(a), y(b), id(0) {}
//     Landmark(Landmark l, double a, double b) : x(l.x + a), y(l.y + b), id(l.id) {}
//     bool operator==(const Landmark &other) const{ return x == other.x && y == other.y; }
//     void translate(double a, double b) {x+=a, y+=b;}
//     std::string toString() const { return std::to_string(x) + "," + std::to_string(y); }
// };

struct Point
{
    double x, y;
    Point() : x(0), y(0) {}
    Point(double a, double b) : x(a), y(b) {}
    Point(Point p, double a, double b) : x(p.x + a), y(p.y + b) {}
    bool operator==(const Point &other) const{ return x == other.x && y == other.y; }
    void translate(double a, double b) {x+=a, y+=b;}
    std::string toString() const { return std::to_string(x) + "," + std::to_string(y); }
    // size_t hash() const {
    //     std::size_t result = 0;
    //     boost::hash_combine(result, x);
    //     boost::hash_combine(result, y);
    //     return result;
    // }
    // std::size_t operator()(const Point& p) const {
    //     using std::size_t;
    //     using std::hash;

    //     return hash<double>()(p.x) ^ (hash<double>()(p.y) << 1);
    // }
};

struct Pose
{
    Point p;
    double theta; // angles in radians
    // currently assuming no movement when defining pose
    Pose() : p(Point()), theta(0) {}
    Pose(double a, double b, double c) : p(a,b), theta(c) {}
    Pose(Point point, double t) : p(point), theta(t) {}
    std::string printPose() const { return p.toString() + " " + std::to_string(theta); }
};

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




// struct Tree {
//     Point p;
//     double radius;
//     Tree() : p(Point()), radius(-1) {}
//     Tree(double a, double b, double r) : p(Point(a,b)), radius(r) {}
//     Tree(Tree t, double a, double b, double r) : p(Point(t.p,a,b)), radius(r) {}
//     bool operator==(const Tree &other) const{ return p == other.p && radius == other.radius; }
//     bool isEmpty() { return radius == -1; }
//     void applyNoise(double xNoise, double yNoise) { p.translate(xNoise, yNoise); }
//     std::string toString() const { return p.toString() + " " + std::to_string(radius); }
// };

// // Geometric Validation Methods

// struct Avoids {
//     const Point rP; const double cW;
//     Avoids(Point randomPoint, double collisionWidth) : rP(randomPoint), cW(collisionWidth) {}
//     bool operator()(Tree t) const {
//         double xDiff = t.p.x - rP.x, yDiff = t.p.y - rP.y, berth = t.radius + cW;
//         return xDiff*xDiff + yDiff*yDiff > berth*berth;
//     }
// };

// struct NoStraightPathCollisions {
//     const Point startPoint; const float pathLength; const double robotWidth; const double pathAngle;
//     const double sinPathAngle; const double cosPathAngle; const double sinPerpAngle; const double cosPerpAngle; 
//     NoStraightPathCollisions(Pose sP, float len, double rW) : startPoint(sP.p), pathLength(len), robotWidth(rW), pathAngle(sP.theta+HALFPI),
//         sinPathAngle(sin(pathAngle)), cosPathAngle(cos(pathAngle)), sinPerpAngle(sin(sP.theta)), cosPerpAngle(cos(sP.theta)) {}
    
//     bool operator()(Tree t) const {
//         // Return true if the width of the robot and tree is either:
//         //  - smaller than the unsigned distance to the line of the robot's path
//         //  - smaller than the signed distance to the line perpendicular to the starting point of the path
//         //  - smaller than the signed distance to the line perpendicular to the ending point of the path
//         double width = robotWidth + t.radius, xDiff = startPoint.x - t.p.x, yDiff = startPoint.y - t.p.y;
//         return std::abs(cosPathAngle*yDiff - sinPathAngle*xDiff) > width ||
//             width <= cosPerpAngle*yDiff - sinPerpAngle*xDiff ||     // Trust the C++ code optimizer
//             cosPerpAngle*yDiff - sinPerpAngle*xDiff < -pathLength - width;

//         // For the perpendicular line at the starting point of the path:
//         // Signed Dist  {                (-)            |   (+)
//         // Pathway      {                End   <----- Start
//         //              {       |---------|=============|---------|
//         // Measurements { ______|__width__|___pathLen___|__width__|______
//         // NoCollision? {  TRUE |              FALSE              | TRUE
//     }
// };