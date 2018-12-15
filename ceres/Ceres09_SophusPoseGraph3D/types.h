/**
 * Define the type used in 3D pose graph SLAM formulation. Each vertex of the graph has a unique integer ID with a
 * position and rotation represented by quaterniond. There are delta transformation and rotation constraints between two
 * vertices
 */

#pragma once
#include <iostream>
#include <map>
#include <string>
#include "Eigen/Core"
#include "Eigen/Geometry"
#include "sophus/so3.hpp"

// the state for each vertex in the pose graph
struct Pose3d {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    const double* data() const { return p.data(); }
    double* data() { return p.data(); }

    Eigen::Vector3d p;
    Sophus::SO3d r;

    // the name of the data type in g2o file format
    static const std::string name() { return "VERTEX_SE3:QUAT"; }
};

// the constraint between two vertices in the pose graph, ie, the transform form vertex id_begin to vertex it_end
struct Constraint3d {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    int idBegin;
    int idEnd;

    // the transformation represents the pose of the end frame E w.r.t the begin frame B. In other words, it transforms
    // a vector in the E frame to the B frame
    Pose3d t_be;

    // the inverse of the covariance matrix for the measurement, the order are x, y, z, delta orientation
    Eigen::Matrix<double, 6, 6> information;

    // the name of the data type in the g2o file format
    static const std::string name() { return "EDGE_SE3:QUAT"; }
};

using MapOfPoses = std::map<int, Pose3d, std::less<int>, Eigen::aligned_allocator<std::pair<const int, Pose3d>>>;
using VectorOfConstaints = std::vector<Constraint3d, Eigen::aligned_allocator<Constraint3d>>;

// read for Pose2d
std::istream& operator>>(std::istream& is, Pose3d& pose) {
    Eigen::Quaterniond q;
    is >> pose.p.x() >> pose.p.y() >> pose.p.z() >> q.x() >> q.y() >> q.z() >> q.w();

    // normalize the quaternion to account for precision loss due to serialization
    q.normalize();

    pose.r = Sophus::SO3d(q);
    return is;
}

// read for Constraint2d
std::istream& operator>>(std::istream& is, Constraint3d& constraint) {
    is >> constraint.idBegin >> constraint.idEnd >> constraint.t_be;

    for (int i = 0; i < 6 && is.good(); ++i) {
        for (int j = i; j < 6 && is.good(); ++j) {
            is >> constraint.information(i, j);
            if (i != j) {
                constraint.information(j, i) = constraint.information(i, j);
            }
        }
    }

    return is;
}