#include <fmt/format.h>
#include <Eigen/Core>
#include <boost/geometry.hpp>
#include <boost/geometry/geometries/multi_point.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometries/polygon.hpp>
#include <boost/geometry/geometries/register/multi_point.hpp>
#include <boost/geometry/geometries/register/point.hpp>
#include <common/common.hpp>
#include <iostream>
#include <vector>

namespace bg = boost::geometry;

// 使用Boost.Geometry定义的2D点类型
using MyPoint = bg::model::d2::point_xy<double>;
using MyPolygon = bg::model::polygon<MyPoint>;
BOOST_GEOMETRY_REGISTER_MULTI_POINT(std::vector<MyPoint>);

// 特化 boost::geometry::traits
namespace boost::geometry::traits {

template <>
struct tag<Eigen::Vector2d> {
    typedef point_tag type;
};

template <>
struct coordinate_type<Eigen::Vector2d> {
    typedef double type;
};

template <>
struct coordinate_system<Eigen::Vector2d> {
    typedef cs::cartesian type;
};

template <>
struct dimension<Eigen::Vector2d> : boost::mpl::int_<2> {};

template <>
struct access<Eigen::Vector2d, 0> {
    static double get(const Eigen::Vector2d& p) { return p[0]; }
    static void set(Eigen::Vector2d& p, double value) { p[0] = value; }
};

template <>
struct access<Eigen::Vector2d, 1> {
    static double get(const Eigen::Vector2d& p) { return p[1]; }
    static void set(Eigen::Vector2d& p, double value) { p[1] = value; }
};

}  // namespace boost::geometry::traits

BOOST_GEOMETRY_REGISTER_MULTI_POINT(std::vector<Eigen::Vector2d>);

/**
 * @brief Calculate the 2D convex hull of a set of points
 *
 */
void simple2dHull() {
    // 定义输入点集
    // bg::model::multi_point<MyPoint> points;
    std::vector<MyPoint> points;
    points.emplace_back(MyPoint(2, 1.3));
    points.emplace_back(MyPoint(2.4, 1.7));
    points.emplace_back(MyPoint(2.8, 1.8));
    points.emplace_back(MyPoint(3.4, 1.2));
    points.emplace_back(MyPoint(3.7, 1.6));
    points.emplace_back(MyPoint(3.4, 2));
    points.emplace_back(MyPoint(4.1, 3));
    points.emplace_back(MyPoint(5.3, 2.6));
    points.emplace_back(MyPoint(5.4, 1.2));
    points.emplace_back(MyPoint(4.9, 0.8));
    points.emplace_back(MyPoint(2.9, 0.7));
    points.emplace_back(MyPoint(2, 1.3));

    // 定义存储凸包结果的多边形
    MyPolygon polygon;

    // 计算凸包
    bg::convex_hull(points, polygon);

    // 输出凸包的顶点
    LOG(INFO) << "convex hull points:";

    for (auto it = polygon.outer().begin(); it != polygon.outer().end(); ++it) {
        LOG(INFO) << fmt::format("({:.3f}, {:.3f})", it->get<0>(), it->get<1>());
    }
}

void useDefinedClass() {
    LOG(INFO) << "use defined points";
    // 定义输入点集
    std::vector<Eigen::Vector2d> points;
    points.emplace_back(Eigen::Vector2d(2, 1.3));
    points.emplace_back(Eigen::Vector2d(2.4, 1.7));
    points.emplace_back(Eigen::Vector2d(2.8, 1.8));
    points.emplace_back(Eigen::Vector2d(3.4, 1.2));
    points.emplace_back(Eigen::Vector2d(3.7, 1.6));
    points.emplace_back(Eigen::Vector2d(3.4, 2));
    points.emplace_back(Eigen::Vector2d(4.1, 3));
    points.emplace_back(Eigen::Vector2d(5.3, 2.6));
    points.emplace_back(Eigen::Vector2d(5.4, 1.2));
    points.emplace_back(Eigen::Vector2d(4.9, 0.8));
    points.emplace_back(Eigen::Vector2d(2.9, 0.7));
    points.emplace_back(Eigen::Vector2d(2, 1.3));

    // 定义存储凸包结果的多边形
    bg::model::polygon<Eigen::Vector2d> polygon;

    // 计算凸包
    bg::convex_hull(points, polygon);

    // 输出凸包的顶点
    LOG(INFO) << "convex hull points:";

    for (auto it = polygon.outer().begin(); it != polygon.outer().end(); ++it) {
        LOG(INFO) << fmt::format("({:.3f}, {:.3f})", it->x(), it->y());
    }
}

int main(int argc, const char* argv[]) {
    common::initLog(argc, argv);

    simple2dHull();
    useDefinedClass();

    common::closeLog();
    return 0;
}