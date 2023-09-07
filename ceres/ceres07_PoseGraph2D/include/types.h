/**
 * Define the type used in 2D pose graph SLAM formulation. Each vertex of the graph has a unique integer ID with a
 * position and orientation. There are delta transformation constraints between two vertices
 */

#pragma once
#include <Eigen/Core>
#include <iostream>
#include <string>
#include "NormalizeAngle.h"

// the state for each vertex in the pose graph
struct Pose2d {
    double x;
    double y;
    double yaw;  // unit: rad

    static const std::string name() { return "VERTEX_SE2"; }
};

// the constraint between two vertices in the pose graph, ie, the transform form vertex id_begin to vertex it_end
struct Constraint2d {
    int idBegin;
    int idEnd;
    double x;
    double y;
    double yaw;  // unit: rad

    Eigen::Matrix3d information;  // the inverse of the covariance matrix for the measurement. the order are x, y, yaw
    static const std::string name() { return "EDGE_SE2"; }
};

// read for Pose2d
std::istream& operator>>(std::istream& is, Pose2d& pose) {
    is >> pose.x >> pose.y >> pose.yaw;
    // normalize the angle to [-pi, pi)
    pose.yaw = normalizeAngle(pose.yaw);
    return is;
}

// read for Constraint2d
std::istream& operator>>(std::istream& is, Constraint2d& constraint) {
    is >> constraint.idBegin >> constraint.idEnd >> constraint.x >> constraint.y >> constraint.yaw >>
        constraint.information(0, 0) >> constraint.information(0, 1) >> constraint.information(0, 2) >>
        constraint.information(1, 1) >> constraint.information(1, 2) >> constraint.information(2, 2);

    // set the lower triangular part the information matrix
    constraint.information(1, 0) = constraint.information(0, 1);
    constraint.information(2, 0) = constraint.information(0, 2);
    constraint.information(2, 1) = constraint.information(1, 2);

    // normalize angle to [-pi, pi)
    constraint.yaw = normalizeAngle(constraint.yaw);
    return is;
}