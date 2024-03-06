#include <iostream>
#include <vector>
#include <cmath>
#include <Eigen/Dense>

using namespace std;
using namespace Eigen;

MatrixXd callHomography(vector<vector<pair<double, double>>> correspondences)
{
    assert(correspondences.size() >= 4);

    double meanX1 = 0, meanY1 = 0, meanX2 = 0, meanY2 = 0;
    for (auto point : correspondences)
    {
        meanX1 += point[0].first;
        meanY1 += point[0].second;
        meanX2 += point[1].first;
        meanY2 += point[1].second;
    }

    meanX1 /= correspondences.size();
    meanY1 /= correspondences.size();
    meanX2 /= correspondences.size();
    meanY2 /= correspondences.size();

    double s1 = 0, s2 = 0;
    for (auto point : correspondences)
    {
        s1 += sqrt(pow(point[0].first - meanX1, 2) + pow(point[0].second - meanY1, 2));
        s2 += sqrt(pow(point[1].first - meanX2, 2) + pow(point[1].second - meanY2, 2));
    }

    s1 = sqrt(2) * correspondences.size() / s1;
    s2 = sqrt(2) * correspondences.size() / s2;

    Matrix3d T1, T2;
    T1 << 1,  0, -meanX1,
          0, s1, -meanY1,
          0,  0,    1/s1;
    T2 << 1,  0, -meanX2,
          0,  1, -meanY2,
          0,  0,    1/s2;
    T1 = T1 * s1;
    T2 = T2 * s2;
    
    MatrixXd A(correspondences.size() * 2, 9);
    for (int i = 0; i < correspondences.size(); i++)
    {
        Vector3d norm1(correspondences[i][0].first, correspondences[i][0].second, 1);
        Vector3d norm2(correspondences[i][1].first, correspondences[i][1].second, 1);

        Vector3d p1 = T1 * norm1;
        Vector3d p2 = T2 * norm2;

        double x1 = p1(0), y1 = p1(1), x2 = p2(0), y2 = p2(1);

        A.row(i * 2) << 0, 0, 0, -x2, -y2, -1, x2 * y1, y2 * y1, y1;
        A.row(i * 2 + 1) << x2, y2, 1, 0, 0, 0, -x2 * x1, -y2 * x1, -x1;
    }
    
    MatrixXd B = A.transpose() * A;
    
    JacobiSVD<MatrixXd> svd(B, ComputeThinU | ComputeThinV);
    
	MatrixXd H(3, 3);
    // H = svd.matrixV().transpose().row(8).reshaped(3, 3).transpose();
    H = MatrixXd::Map(svd.matrixV().transpose().row(8).data(), 3, 3).transpose();
	
    H = T1.inverse() * H * T2;
    
    return H;
}

int main()
{
    vector<vector<pair<double, double>>> ranCorr = 
    {
    	{{0, 0}, {554, 548}},
    	{{7, 0}, {1074, 537}},
    	{{7, 13.2}, {1178, 969}},
    	{{0.2, 13.2}, {192, 993}}
	};
	MatrixXd ph = callHomography(ranCorr);
	cout << "Homography:\n" << ph << endl;
	
	Vector3d test(1178, 969, 1);
	Vector3d output = ph * test;
	cout << "\nOutput for input [7 13.2]:\n" << output/output[2] << endl;
	
	// Normalization
	MatrixXd normal = ph / ph(2, 2);
	cout << "\nNormalized Homography:\n" << normal << endl;

	// Define the test array and perform the matrix multiplication
	Vector3d testn = normal * test;
	cout << "\nOutput for input [7 13.2] after normalization:\n" 
         << testn / testn[2] << endl;
}