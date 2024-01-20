/**
 * @brief Some extension to Sophus, include:
 *  1. Print using `operator<<` for SO2, SE2, SO3, SE3
 *  2. Serialization using json
 */

#pragma once
#include <fmt/format.h>
#include <nlohmann/json.hpp>
#include <ostream>
#include <sophus/se2.hpp>
#include <sophus/se3.hpp>

namespace Sophus {

/**
 * @brief Overload operator<< for print SO2
 *
 * @tparam T    Typename, double or float
 * @param os    Output stream
 * @param r     Rotation in SO2
 * @return  Output stream
 */
template <typename T>
std::ostream& operator<<(std::ostream& os, const SO2<T>& r) {
    os << fmt::format("{:.5f} deg", r.log() * 180. / M_PI);
    return os;
}

/**
 * @brief Overload operator<< for print SE2
 *
 * @tparam T    Typename, double or float
 * @param os    Output stream
 * @param t     Rotation in SE2
 * @return  Output stream
 */
template <typename T>
std::ostream& operator<<(std::ostream& os, const SE2<T>& t) {
    os << fmt::format("[{:.5f}, {:.5f}, {:.5f}] deg,m", t.so2().log() * 180. / M_PI, t.translation()[0],
                      t.translation()[1]);
    return os;
}

/**
 * @brief Convert SO2<T> to json, just save as vector<T>
 * @tparam T    Typename, double or float
 * @param j     Json
 * @param r     Rotation in SO2
 */
template <typename T>
void to_json(nlohmann::json& j, const SO2<T>& r) {
    j = r.log();
}

/**
 * @brief Convert json to SO2<T>
 * @tparam T    Typename, double or float
 * @param j     Json
 * @param r     Rotation in SO2
 */
template <typename T>
void from_json(const nlohmann::json& j, SO2<T>& r) {
    r = Sophus::SO2<T>::exp(j.get<T>());
}

/**
 * @brief Convert SE2<T> to json
 * @tparam T    Typename, double or float
 * @param j     Json
 * @param t     Transformation in SE2
 */
template <typename T>
void to_json(nlohmann::json& j, const SE2<T>& t) {
    std::vector<T> data = {t.so2().log(), t.translation()[0], t.translation()[1]};
    j = data;
}

/**
 * @brief Convert json to SE2<T>
 * @tparam T    Typename, double or float
 * @param j     Json
 * @param t     Transformation in SE2
 */
template <typename T>
void from_json(const nlohmann::json& j, SE2<T>& t) {
    std::vector<T> data = j.get<std::vector<T>>();
    t.so2() = Sophus::SO2<T>::exp(data[0]);
    t.translation() = Eigen::Map<Eigen::Matrix<T, 2, 1>>(data.data() + 1);
}

/**
 * @brief Overload operator<< for print SO3
 *
 * @tparam T    Typename, double or float
 * @param os    Output stream
 * @param r     Rotation in SO3
 * @return  Output stream
 */
template <typename T>
std::ostream& operator<<(std::ostream& os, const SO3<T>& r) {
    Eigen::Matrix<T, 3, 1> angle = r.log() * 180. / M_PI;
    os << fmt::format("[{:.5f}, {:.5f}, {:.5f}] deg", angle[0], angle[1], angle[2]);
    return os;
}

/**
 * @brief Overload operator<< for print SE3
 *
 * @tparam T    Typename, double or float
 * @param os    Output stream
 * @param t     Rotation in SE3
 * @return  Output stream
 */
template <typename T>
std::ostream& operator<<(std::ostream& os, const SE3<T>& t) {
    Eigen::Matrix<T, 3, 1> angle = t.so3().log() * 180. / M_PI;
    auto& trans = t.translation();
    os << fmt::format("[{:.5f}, {:.5f}, {:.5f}, {:.5f}, {:.5f}, {:.5f}] deg,m", angle[0], angle[1], angle[2], trans[0],
                      trans[1], trans[2]);
    return os;
}

/**
 * @brief Convert SO3<T> to json, just save as vector<T>
 * @tparam T    Typename, double or float
 * @param j     Json
 * @param r     Rotation in SO3
 */
template <typename T>
void to_json(nlohmann::json& j, const SO3<T>& r) {
    Eigen::Matrix<T, 3, 1> angle = r.log();
    j = std::vector<T>(angle.data(), angle.data() + 3);
}

/**
 * @brief Convert json to SO3<T>
 * @tparam T    Typename, double or float
 * @param j     Json
 * @param r     Rotation in SO3
 */
template <typename T>
void from_json(const nlohmann::json& j, SO3<T>& r) {
    std::vector<T> angle = j.get<std::vector<T>>();
    r = Sophus::SO3<T>::exp(Eigen::Map<Eigen::Matrix<T, 3, 1>>(angle.data()));
}

/**
 * @brief Convert SE3<T> to json
 * @tparam T    Typename, double or float
 * @param j     Json
 * @param t     Transformation in SE3
 */
template <typename T>
void to_json(nlohmann::json& j, const SE3<T>& t) {
    Eigen::Matrix<T, 3, 1> angle = t.so3().log();
    std::vector<T> data = {angle[0], angle[1], angle[2], t.translation()[0], t.translation()[1], t.translation()[2]};
    j = data;
}

/**
 * @brief Convert json to SE3<T>
 * @tparam T    Typename, double or float
 * @param j     Json
 * @param t     Transformation in SE3
 */
template <typename T>
void from_json(const nlohmann::json& j, SE3<T>& t) {
    std::vector<T> data = j.get<std::vector<T>>();
    t.so3() = Sophus::SO3<T>::exp(Eigen::Map<Eigen::Matrix<T, 3, 1>>(data.data()));
    t.translation() = Eigen::Map<Eigen::Matrix<T, 3, 1>>(data.data() + 3);
}

}  // namespace Sophus