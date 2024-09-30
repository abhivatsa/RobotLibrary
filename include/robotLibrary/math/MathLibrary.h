// MathLibrary.h
#ifndef MATH_LIBRARY_H
#define MATH_LIBRARY_H

#include <array>
#include <initializer_list>
#include <cstddef> // For std::size_t
#include <ostream> // **Added to support operator<< for Vector<N>**

namespace MathLib {

/**
 * @brief Template class representing a mathematical vector.
 * 
 * @tparam N Dimension of the vector.
 */
template <std::size_t ROWS, std::size_t COLS> class Matrix;

template <std::size_t N>
class Vector {
public:
    std::array<double, N> data;

    // Constructors
    constexpr Vector();
    constexpr Vector(const std::array<double, N>& arr);
    Vector(const std::initializer_list<double>& list);

    // Operator Overloads
    constexpr Vector<N> operator+(const Vector<N>& other) const;
    constexpr Vector<N> operator-(const Vector<N>& other) const;
    constexpr Vector<N> operator*(double scalar) const;
    constexpr Vector<N> operator/(double scalar) const;

    // Dot product
    constexpr double dot(const Vector<N>& other) const;

    // Norm and Normalization
    constexpr double norm() const;
    constexpr Vector<N> normalized() const;

    // Transpose
    constexpr class Matrix<1, N> transpose() const;

    // Cross product (only for 3D vectors)
    Vector<3> cross(const Vector<3>& other) const;
};

/**
 * @brief Overload of the output stream operator for Vector<N>.
 * 
 * Allows printing Vector<N> objects using std::ostream.
 * 
 * @tparam N Dimension of the vector.
 * @param os Output stream.
 * @param vec Vector to be printed.
 * @return std::ostream& Reference to the output stream.
 */
template <std::size_t N>
std::ostream& operator<<(std::ostream& os, const Vector<N>& vec);

/**
 * @brief Template class representing a mathematical matrix.
 * 
 * @tparam ROWS Number of rows.
 * @tparam COLS Number of columns.
 */
template <std::size_t ROWS, std::size_t COLS>
class Matrix {
public:
    std::array<std::array<double, COLS>, ROWS> data;

    // Constructors
    constexpr Matrix();
    constexpr Matrix(const std::array<std::array<double, COLS>, ROWS>& arr);
    Matrix(const std::initializer_list<std::initializer_list<double>>& list);

    // Operator Overloads
    constexpr Matrix<ROWS, COLS> operator+(const Matrix<ROWS, COLS>& other) const;
    constexpr Matrix<ROWS, COLS> operator-(const Matrix<ROWS, COLS>& other) const;
    constexpr Matrix<ROWS, COLS> operator*(double scalar) const;
    constexpr Matrix<ROWS, COLS> operator/(double scalar) const;

    // Matrix Multiplication
    template <std::size_t OTHER_COLS>
    constexpr Matrix<ROWS, OTHER_COLS> operator*(const Matrix<COLS, OTHER_COLS>& other) const;

    // Matrix-Vector Multiplication
    constexpr Vector<ROWS> operator*(const Vector<COLS>& vec) const;

    // Transpose
    constexpr Matrix<COLS, ROWS> transpose() const;

    // Utility Functions
    constexpr void setZero();
    constexpr void setIdentity();
};

// Free Function: Create a Denavit-Hartenberg (DH) Transformation Matrix
inline Matrix<4, 4> fromDH(double alpha, double a, double d, double theta);

// Free Function: Cross Product for 3D Vectors
inline Vector<3> cross(const Vector<3>& a, const Vector<3>& b);

} // namespace MathLib

// Include the implementation
#include "MathLibrary.tpp"

#endif // MATH_LIBRARY_H
