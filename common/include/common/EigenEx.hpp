/**
 * @brief Some extension for Eigen, include:
 *  1. Template to check Eigen matrix is fix size vector or matrix, return type, etc.
 *  2. Format Eigen::Vector or Eigen::Matrix
 *  3. Serialization using json
 */

#pragma once
#include <fmt/format.h>
#include <fmt/ostream.h>
#include <fmt/ranges.h>
#include <Eigen/Core>
#include <nlohmann/json.hpp>

namespace common {

/**
 * @brief Check whether the type is Eigen::Matrix
 *
 * @tparam T    Type for checking
 */
template <class T,
          typename = std::enable_if_t<std::is_base_of<Eigen::MatrixBase<std::decay_t<T>>, std::decay_t<T>>::value>>
struct IsMatrix : std::true_type {};

/**
 * @brief Check whether the Matrix is Vector or RowVector
 *
 * @tparam Matrix   To be checked Eigen matrix
 */
template <typename Matrix,
          typename = std::enable_if_t<Matrix::RowsAtCompileTime == 1 || Matrix::ColsAtCompileTime == 1>>
struct IsVector : std::true_type {};

/**
 * @brief Check whether the Vector is fixed size with input dimension
 *
 * @tparam Vector   To be checked Eigen matrix
 * @tparam DimNum   The dimension number
 */
template <typename Vector, int DimNum,
          typename = std::enable_if_t<Vector::RowsAtCompileTime == DimNum && Vector::ColsAtCompileTime == 1>>
struct IsFixedSizeVector : std::true_type {};

/**
 * @brief Check whether the Matrix is fixed size with input dimension
 *
 * @tparam Matrix   To be checked Eigen matrix
 * @tparam RowsNum  The dimension number in row
 * @tparam ColsNum  The dimension number in col
 */
template <typename Matrix, int RowsNum, int ColsNum,
          typename = std::enable_if_t<Matrix::RowsAtCompileTime == RowsNum && Matrix::ColsAtCompileTime == ColsNum>>
struct IsFixedSizeMatrix : std::true_type {};

/**
 * @brief Check whether the Matrix is fixed size with input dimension, or with dynamic size
 *
 * @tparam Matrix   To be checked Eigen matrix
 * @tparam RowsNum  The dimension number in row
 * @tparam ColsNum  The dimension number in col
 */
template <typename Matrix, int RowsNum, int ColsNum,
          typename = std::enable_if_t<(Matrix::RowsAtCompileTime == RowsNum && Matrix::ColsAtCompileTime == ColsNum) ||
                                      (Matrix::RowsAtCompileTime == Eigen::Dynamic &&
                                       Matrix::ColsAtCompileTime == Eigen::Dynamic)>>
struct IsFixedSizeOrDynamicMatrix : std::true_type {};

/**
 * @brief Check whether the Matrix is fixed rows with input dimension
 *
 * @tparam Matrix   To be checked Eigen matrix
 * @tparam RowsNum  The dimension number in row
 */
template <typename Matrix, int RowsNum, typename = std::enable_if_t<Matrix::RowsAtCompileTime == RowsNum>>
struct IsFixedRowsMatrix : std::true_type {};

/**
 * @brief Check whether the Matrix is fixed cols with input dimension
 *
 * @tparam Matrix   To be checked Eigen matrix
 * @tparam ColsNum  The dimension number in col
 */
template <typename Matrix, int ColsNum, typename = std::enable_if_t<Matrix::ColsAtCompileTime == ColsNum>>
struct IsFixedColsMatrix : std::true_type {};

/**
 * @brief Get the Scalar type for the binary operation for two members, one is type of Scalar type and another is Eigen
 * matrix
 *
 * @tparam Scalar   Scalar type for one member
 * @tparam Derived  Eigen matrix for another member
 * @return The returned binary operation type
 */
template <typename Scalar, typename Derived>
using ReturnScalar = typename Eigen::ScalarBinaryOpTraits<Scalar, typename Derived::Scalar>::ReturnType;

}  // namespace common

/**
 * @brief Disable format Eigen::Vector using fmt/ranges candidate
 *
 */
template <typename Matrix, typename Char>
struct fmt::range_format_kind<
    Matrix, Char,
    std::enable_if_t<common::IsVector<Matrix>::value && std::is_base_of<Eigen::MatrixBase<Matrix>, Matrix>::value>>
    : std::false_type {};

/**
 * @brief Disable format Eigen::Matrix using fmt/ranges candidate
 *
 */
template <typename Matrix, typename Char>
struct fmt::range_format_kind<Matrix, Char,
                              std::enable_if_t<common::IsMatrix<Matrix>::value &&
                                               !(Matrix::RowsAtCompileTime == 1 || Matrix::ColsAtCompileTime == 1)>>
    : std::false_type {};

/**
 * @brief Add specialization to format Eigen::Vector
 *
 * @tparam Vector Vector or RowVector
 */
template <typename Matrix, typename Char>
struct fmt::formatter<
    Matrix, Char,
    std::enable_if_t<common::IsVector<Matrix>::value && std::is_base_of<Eigen::MatrixBase<Matrix>, Matrix>::value>> {
  public:
    using Scalar = typename Matrix::Scalar;

    template <typename ParseContext>
    constexpr auto parse(ParseContext& ctx) -> decltype(ctx.begin()) {
        auto it = ctx.begin();
        if (it != ctx.end() && *it != '}') {
            if (*it != ':') {
                throw format_error("invalid format specifier");
            }
            ++it;
        } else {
            detail::maybe_set_debug_format(underlying_, true);
        }
        ctx.advance_to(it);
        return underlying_.parse(ctx);
    }

    template <typename FormatContext>
    auto format(const Matrix& mat, FormatContext& ctx) -> decltype(ctx.out()) {
        detail::range_mapper<buffer_context<Char>> mapper;
        auto out = ctx.out();
        out = detail::copy_str<Char>(opening_bracket_, out);
        for (int i = 0; i < mat.size(); ++i) {
            if (i > 0) {
                out = detail::copy_str<Char>(separator_, out);
            }
            ctx.advance_to(out);
            out = underlying_.format(mapper.map(mat[i]), ctx);
        }
        out = detail::copy_str<Char>(closing_bracket_, out);
        return out;
    }

  private:
    detail::range_formatter_type<Char, Scalar> underlying_;
    basic_string_view<Char> separator_ = detail::string_literal<Char, ',', ' '>{};
    basic_string_view<Char> opening_bracket_ = detail::string_literal<Char, '['>{};
    basic_string_view<Char> closing_bracket_ = detail::string_literal<Char, ']'>{};
};

/**
 * @brief Add specialization to format Eigen::Matrix, or dynamic Eigen::Vector
 *
 * @tparam Matrix Matrix
 */
template <typename Matrix, typename Char>
struct fmt::formatter<Matrix, Char,
                      std::enable_if_t<common::IsMatrix<Matrix>::value &&
                                       !(Matrix::RowsAtCompileTime == 1 || Matrix::ColsAtCompileTime == 1)>> {
  public:
    using Scalar = typename Matrix::Scalar;

    template <typename ParseContext>
    constexpr auto parse(ParseContext& ctx) -> decltype(ctx.begin()) {
        auto it = ctx.begin();
        if (it != ctx.end() && *it != '}') {
            if (*it != ':') {
                throw format_error("invalid format specifier");
            }
            ++it;
        } else {
            detail::maybe_set_debug_format(underlying_, true);
        }
        ctx.advance_to(it);
        return underlying_.parse(ctx);
    }

    template <typename FormatContext>
    auto format(const Matrix& mat, FormatContext& ctx) -> decltype(ctx.out()) {
        detail::range_mapper<buffer_context<Char>> mapper;
        int rows = mat.rows();
        int cols = mat.cols();
        auto out = ctx.out();
        out = detail::copy_str<Char>(opening_bracket_, out);
        if (rows == 1 || cols == 1) {
            for (int i = 0; i < mat.size(); ++i) {
                if (i > 0) {
                    out = detail::copy_str<Char>(separator_, out);
                }
                ctx.advance_to(out);
                out = underlying_.format(mapper.map(mat(i)), ctx);
            }
        } else {
            for (int i = 0; i < rows; ++i) {
                if (i != 0) {
                    out = detail::copy_str<Char>(separator_, out);
                }
                out = detail::copy_str<Char>(opening_bracket_, out);
                for (int j = 0; j < cols; ++j) {
                    if (j != 0) {
                        out = detail::copy_str<Char>(separator_, out);
                    }
                    ctx.advance_to(out);
                    out = underlying_.format(mapper.map(mat(i, j)), ctx);
                }
                out = detail::copy_str<Char>(closing_bracket_, out);
            }
        }
        out = detail::copy_str<Char>(closing_bracket_, out);
        return out;
    }

  private:
    detail::range_formatter_type<Char, Scalar> underlying_;
    basic_string_view<Char> separator_ = detail::string_literal<Char, ',', ' '>{};
    basic_string_view<Char> opening_bracket_ = detail::string_literal<Char, '['>{};
    basic_string_view<Char> closing_bracket_ = detail::string_literal<Char, ']'>{};
};

namespace Eigen {

/**
 * @brief Convert Eigen Vector to json, just save it as vector<T>
 */
template <typename _Scalar, int _Rows, int _Options, int _MaxRows, int _MaxCols>
void to_json(nlohmann::json& j, const Matrix<_Scalar, _Rows, 1, _Options, _MaxRows, _MaxCols>& m) {
    j = std::vector<_Scalar>(m.data(), m.data() + m.size());
}

/**
 * @brief Convert json to Eigen Vector, just load it as vector<T>
 */
template <typename _Scalar, int _Rows, int _Options, int _MaxRows, int _MaxCols>
void from_json(const nlohmann::json& j, Matrix<_Scalar, _Rows, 1, _Options, _MaxRows, _MaxCols>& m) {
    std::vector<_Scalar> data = j.get<std::vector<_Scalar>>();
    m = Map<Matrix<_Scalar, _Rows, 1, _Options, _MaxRows, _MaxCols>>(data.data());
}

/**
 * @brief Convert Eigen RowVector to json, just save it as vector<T>
 */
template <typename _Scalar, int _Cols, int _Options, int _MaxRows, int _MaxCols>
void to_json(nlohmann::json& j, const Matrix<_Scalar, 1, _Cols, _Options, _MaxRows, _MaxCols>& m) {
    j = std::vector<_Scalar>(m.data(), m.data() + m.size());
}

/**
 * @brief Convert json to Eigen RowVector, just load it as vector<T>
 */
template <typename _Scalar, int _Cols, int _Options, int _MaxRows, int _MaxCols>
void from_json(const nlohmann::json& j, Matrix<_Scalar, 1, _Cols, _Options, _MaxRows, _MaxCols>& m) {
    std::vector<_Scalar> data = j.get<std::vector<_Scalar>>();
    m = Map<Matrix<_Scalar, 1, _Cols, _Options, _MaxRows, _MaxCols>>(data.data());
}

/**
 * @brief Convert Eigen Matrix to json
 *
 * @note The data order saved in json will be the same as read, i.e., the transpose of m. For example, for 2x2 matrix
 * [[a, b], [c, d]], the data order in Eigen will be [a,c,d,b], the data order in json will be [a,b,c,d]
 */
template <typename _Scalar, int _Rows, int _Cols, int _Options, int _MaxRows, int _MaxCols>
void to_json(nlohmann::json& j, const Matrix<_Scalar, _Rows, _Cols, _Options, _MaxRows, _MaxCols>& m) {
    Matrix<_Scalar, _Cols, _Rows, _Options, _MaxCols, _MaxRows> mT = m.transpose();
    j = nlohmann::json{
        {"rows", m.rows()}, {"cols", m.cols()}, {"data", std::vector<_Scalar>(mT.data(), mT.data() + mT.size())}};
}

/**
 * @brief Convert json to Eigen Matrix
 */
template <typename _Scalar, int _Rows, int _Cols, int _Options, int _MaxRows, int _MaxCols>
void from_json(const nlohmann::json& j, Matrix<_Scalar, _Rows, _Cols, _Options, _MaxRows, _MaxCols>& m) {
    int rows = j.at("rows").get<int>();
    int cols = j.at("cols").get<int>();
    std::vector<_Scalar> data = j.at("data").get<std::vector<_Scalar>>();
    Matrix<_Scalar, _Cols, _Rows, _Options, _MaxCols, _MaxRows> mT =
        Map<Matrix<_Scalar, _Cols, _Rows, _Options, _MaxCols, _MaxRows>>(data.data(), cols, rows);
    m = mT.transpose();
}

}  // namespace Eigen