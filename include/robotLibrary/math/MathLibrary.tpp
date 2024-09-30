// MathLibrary.tpp
#ifndef MATH_LIBRARY_TPP
#define MATH_LIBRARY_TPP

#include "MathLibrary.h" // Ensure this includes the class declarations
#include <cmath>         // For trigonometric functions
#include <algorithm>     // For std::copy
#include <stdexcept>     // For exceptions

namespace MathLib {

// --------------------- Vector Implementation ---------------------

// Default constructor initializes to zero
template <std::size_t N>
constexpr Vector<N>::Vector() : data{} {}

// Constructor from std::array
template <std::size_t N>
constexpr Vector<N>::Vector(const std::array<double, N>& arr) : data(arr) {}

// Constructor from initializer list
template <std::size_t N>
Vector<N>::Vector(const std::initializer_list<double>& list) {
    if (list.size() != N) {
        throw std::invalid_argument("Initializer list size must match Vector size.");
    }
    std::copy(list.begin(), list.end(), data.begin());
}

// Vector addition
template <std::size_t N>
constexpr Vector<N> Vector<N>::operator+(const Vector<N>& other) const {
    Vector<N> result;
    for (std::size_t i = 0; i < N; ++i) {
        result.data[i] = this->data[i] + other.data[i];
    }
    return result;
}

// Vector subtraction
template <std::size_t N>
constexpr Vector<N> Vector<N>::operator-(const Vector<N>& other) const {
    Vector<N> result;
    for (std::size_t i = 0; i < N; ++i) {
        result.data[i] = this->data[i] - other.data[i];
    }
    return result;
}

// Scalar multiplication
template <std::size_t N>
constexpr Vector<N> Vector<N>::operator*(double scalar) const {
    Vector<N> result;
    for (std::size_t i = 0; i < N; ++i) {
        result.data[i] = this->data[i] * scalar;
    }
    return result;
}

// Scalar division
template <std::size_t N>
constexpr Vector<N> Vector<N>::operator/(double scalar) const {
    Vector<N> result;
    for (std::size_t i = 0; i < N; ++i) {
        result.data[i] = this->data[i] / scalar;
    }
    return result;
}

// Dot product
template <std::size_t N>
constexpr double Vector<N>::dot(const Vector<N>& other) const {
    double result = 0.0;
    for (std::size_t i = 0; i < N; ++i) {
        result += this->data[i] * other.data[i];
    }
    return result;
}

// Norm (magnitude) of the vector
template <std::size_t N>
constexpr double Vector<N>::norm() const {
    return std::sqrt(this->dot(*this));
}

// Normalized vector
template <std::size_t N>
constexpr Vector<N> Vector<N>::normalized() const {
    double n = this->norm();
    if (n == 0.0) {
        throw std::runtime_error("Cannot normalize a zero vector.");
    }
    return (*this) / n;
}

// Transpose of the vector (returns a 1xN matrix)
template <std::size_t N>
constexpr Matrix<1, N> Vector<N>::transpose() const {
    Matrix<1, N> mat;
    for (std::size_t i = 0; i < N; ++i) {
        mat.data[0][i] = this->data[i];
    }
    return mat;
}

// Cross product (only for 3D vectors)
template <std::size_t N>
Vector<3> Vector<N>::cross(const Vector<3>& other) const {
    static_assert(N == 3, "Cross product is only defined for 3D vectors.");
    Vector<3> result;
    result.data[0] = this->data[1] * other.data[2] - this->data[2] * other.data[1];
    result.data[1] = this->data[2] * other.data[0] - this->data[0] * other.data[2];
    result.data[2] = this->data[0] * other.data[1] - this->data[1] * other.data[0];
    return result;
}

// --------------------- Operator<< Implementation ---------------------

/**
 * @brief Overload of the output stream operator for Vector<N>.
 * 
 * Prints the vector in the format: [x, y, z, ...]
 * 
 * @tparam N Dimension of the vector.
 * @param os Output stream.
 * @param vec Vector to be printed.
 * @return std::ostream& Reference to the output stream.
 */
template <std::size_t N>
std::ostream& operator<<(std::ostream& os, const Vector<N>& vec) {
    os << "[";
    for (std::size_t i = 0; i < N; ++i) {
        os << vec.data[i];
        if (i != N - 1)
            os << ", ";
    }
    os << "]";
    return os;
}

// --------------------- Matrix Implementation ---------------------

// Default constructor initializes to zero
template <std::size_t ROWS, std::size_t COLS>
constexpr Matrix<ROWS, COLS>::Matrix() : data{} {}

// Constructor from std::array
template <std::size_t ROWS, std::size_t COLS>
constexpr Matrix<ROWS, COLS>::Matrix(const std::array<std::array<double, COLS>, ROWS>& arr) : data(arr) {}

// Constructor from initializer list
template <std::size_t ROWS, std::size_t COLS>
Matrix<ROWS, COLS>::Matrix(const std::initializer_list<std::initializer_list<double>>& list) {
    if (list.size() != ROWS) {
        throw std::invalid_argument("Initializer list row size must match Matrix rows.");
    }
    auto row_it = list.begin();
    for (std::size_t i = 0; i < ROWS; ++i, ++row_it) {
        if (row_it->size() != COLS) {
            throw std::invalid_argument("Initializer list column size must match Matrix columns.");
        }
        std::copy(row_it->begin(), row_it->end(), data[i].begin());
    }
}

// Matrix addition
template <std::size_t ROWS, std::size_t COLS>
constexpr Matrix<ROWS, COLS> Matrix<ROWS, COLS>::operator+(const Matrix<ROWS, COLS>& other) const {
    Matrix<ROWS, COLS> result;
    for (std::size_t i = 0; i < ROWS; ++i)
        for (std::size_t j = 0; j < COLS; ++j)
            result.data[i][j] = this->data[i][j] + other.data[i][j];
    return result;
}

// Matrix subtraction
template <std::size_t ROWS, std::size_t COLS>
constexpr Matrix<ROWS, COLS> Matrix<ROWS, COLS>::operator-(const Matrix<ROWS, COLS>& other) const {
    Matrix<ROWS, COLS> result;
    for (std::size_t i = 0; i < ROWS; ++i)
        for (std::size_t j = 0; j < COLS; ++j)
            result.data[i][j] = this->data[i][j] - other.data[i][j];
    return result;
}

// Scalar multiplication
template <std::size_t ROWS, std::size_t COLS>
constexpr Matrix<ROWS, COLS> Matrix<ROWS, COLS>::operator*(double scalar) const {
    Matrix<ROWS, COLS> result;
    for (std::size_t i = 0; i < ROWS; ++i)
        for (std::size_t j = 0; j < COLS; ++j)
            result.data[i][j] = this->data[i][j] * scalar;
    return result;
}

// Scalar division
template <std::size_t ROWS, std::size_t COLS>
constexpr Matrix<ROWS, COLS> Matrix<ROWS, COLS>::operator/(double scalar) const {
    Matrix<ROWS, COLS> result;
    for (std::size_t i = 0; i < ROWS; ++i)
        for (std::size_t j = 0; j < COLS; ++j)
            result.data[i][j] = this->data[i][j] / scalar;
    return result;
}

// Matrix multiplication
template <std::size_t ROWS, std::size_t COLS>
template <std::size_t OTHER_COLS>
constexpr Matrix<ROWS, OTHER_COLS> Matrix<ROWS, COLS>::operator*(const Matrix<COLS, OTHER_COLS>& other) const {
    Matrix<ROWS, OTHER_COLS> result;
    for (std::size_t i = 0; i < ROWS; ++i) {
        for (std::size_t j = 0; j < OTHER_COLS; ++j) {
            double sum = 0.0;
            for (std::size_t k = 0; k < COLS; ++k) {
                sum += this->data[i][k] * other.data[k][j];
            }
            result.data[i][j] = sum;
        }
    }
    return result;
}

// Matrix-vector multiplication
template <std::size_t ROWS, std::size_t COLS>
constexpr Vector<ROWS> Matrix<ROWS, COLS>::operator*(const Vector<COLS>& vec) const {
    Vector<ROWS> result;
    for (std::size_t i = 0; i < ROWS; ++i) {
        double sum = 0.0;
        for (std::size_t j = 0; j < COLS; ++j) {
            sum += this->data[i][j] * vec.data[j];
        }
        result.data[i] = sum;
    }
    return result;
}

// Transpose of the matrix
template <std::size_t ROWS, std::size_t COLS>
constexpr Matrix<COLS, ROWS> Matrix<ROWS, COLS>::transpose() const {
    Matrix<COLS, ROWS> result;
    for (std::size_t i = 0; i < ROWS; ++i)
        for (std::size_t j = 0; j < COLS; ++j)
            result.data[j][i] = this->data[i][j];
    return result;
}

// Set all elements to zero
template <std::size_t ROWS, std::size_t COLS>
constexpr void Matrix<ROWS, COLS>::setZero() {
    for (std::size_t i = 0; i < ROWS; ++i)
        for (std::size_t j = 0; j < COLS; ++j)
            data[i][j] = 0.0;
}

// Set the matrix to identity (only for square matrices)
template <std::size_t ROWS, std::size_t COLS>
constexpr void Matrix<ROWS, COLS>::setIdentity() {
    static_assert(ROWS == COLS, "Identity matrix must be square.");
    for (std::size_t i = 0; i < ROWS; ++i)
        for (std::size_t j = 0; j < COLS; ++j)
            data[i][j] = (i == j) ? 1.0 : 0.0;
}

// --------------------- Free Functions ---------------------

/**
 * @brief Creates a Denavit-Hartenberg (DH) transformation matrix.
 * 
 * @param alpha Link twist angle (radians).
 * @param a Link length.
 * @param d Link offset.
 * @param theta Joint angle (radians).
 * @return Matrix<4, 4> The DH transformation matrix.
 */
inline Matrix<4, 4> fromDH(double alpha, double a, double d, double theta) {
    Matrix<4, 4> T;

    double cos_theta = std::cos(theta);
    double sin_theta = std::sin(theta);
    double cos_alpha = std::cos(alpha);
    double sin_alpha = std::sin(alpha);

    T.data[0][0] = cos_theta;
    T.data[0][1] = -sin_theta * cos_alpha;
    T.data[0][2] = sin_theta * sin_alpha;
    T.data[0][3] = a * cos_theta;

    T.data[1][0] = sin_theta;
    T.data[1][1] = cos_theta * cos_alpha;
    T.data[1][2] = -cos_theta * sin_alpha;
    T.data[1][3] = a * sin_theta;

    T.data[2][0] = 0.0;
    T.data[2][1] = sin_alpha;
    T.data[2][2] = cos_alpha;
    T.data[2][3] = d;

    T.data[3][0] = 0.0;
    T.data[3][1] = 0.0;
    T.data[3][2] = 0.0;
    T.data[3][3] = 1.0;

    return T;
}

/**
 * @brief Creates a cross product of two 3D vectors.
 * 
 * @param a First vector.
 * @param b Second vector.
 * @return Vector<3> The resulting cross product vector.
 */
inline Vector<3> cross(const Vector<3>& a, const Vector<3>& b) {
    Vector<3> result;
    result.data[0] = a.data[1] * b.data[2] - a.data[2] * b.data[1];
    result.data[1] = a.data[2] * b.data[0] - a.data[0] * b.data[2];
    result.data[2] = a.data[0] * b.data[1] - a.data[1] * b.data[0];
    return result;
}

} // namespace MathLib

#endif // MATH_LIBRARY_TPP
