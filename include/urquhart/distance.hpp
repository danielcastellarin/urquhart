#pragma once

#include <iostream>
#include <vector>
#include <algorithm>
#include <numeric>
#include <tuple>
#include <utility>
#include <math.h>
#include <eigen3/Eigen/Dense> // References new version of Eigen (3.4.90)

using EdgeSet = Eigen::Matrix2Xi;
using Points = Eigen::Matrix2Xd;
using PtLoc = Eigen::Vector2d;

// Alternative method for vector distance?
// Eigen::VectorXd diff = A - B;
// return diff.transpose().dot(diff);

inline double descriptorDistance(const Eigen::VectorXd& A, const Eigen::VectorXd& B) {
  return std::sqrt((A - B).array().square().sum());
}

inline double descriptorDistanceAgain(const Eigen::VectorXd& A, Eigen::VectorXd B) {
  return std::sqrt((A - B).array().square().sum());
}

// EuclideanDistance2D is used for polygon descriptor definition
inline double euclideanDistance2D(const PtLoc& A, const PtLoc& B) {
  return std::sqrt((A - B).array().square().sum());
}

// Get the squared distance of every sampled point to a centroid
inline Eigen::VectorXd centroidDistance(const Points& samples, const Eigen::Vector2d& centroid) {
  Points centroidMatrix(2, samples.cols());
  centroidMatrix.colwise() = centroid;
  return (centroidMatrix - samples).array().square().colwise().sum().sqrt();
}


inline size_t cantorPairing(size_t a, size_t b) {
  // a is always the smallest number
  size_t aux = a;
  if (b < a) {
    a = b;
    b = aux;
  }
  return (a + b) * (a + b + 1) / 2 + a;
}
