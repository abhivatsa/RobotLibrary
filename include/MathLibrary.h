#ifndef MATH_LIBRARY_H
#define MATH_LIBRARY_H

#include <array>
#include <cassert>
#include <cstddef>
#include <iostream>
#include <algorithm>
#include <cmath> // For sin, cos
#include <stdexcept>

namespace MathLib {

// Forward declaration of Matrix template for use in Vector::transpose
template <std::size_t ROWS, std::size_t COLS>
struct Matrix;

// Vector Template
template <std::size_t N>
struct Vector {
    std::array<double, N> data;

    // Default constructor initializes all elements to zero
    constexpr Vector() : data{} {}

    // Constructor from std::array
    constexpr Vector(const std::array<double, N>& arr) : data(arr) {}

    // Constructor from initializer list
    Vector(const std::initializer_list<double>& list) {
        if (list.size() != N) {
            throw std::invalid_argument("Initializer list size must match vector size.");
        }
        std::copy(list.begin(), list.end(), data.begin());
    }

    // Addition
    constexpr Vector<N> operator+(const Vector<N>& other) const {
        Vector<N> result;
        for (std::size_t i = 0; i < N; ++i) {
            result.data[i] = this->data[i] + other.data[i];
        }
        return result;
    }

    // Element-wise Multiplication
    constexpr Vector<N> operator*(const Vector<N>& other) const {
        Vector<N> result;
        for (std::size_t i = 0; i < N; ++i) {
            result.data[i] = this->data[i] * other.data[i];
        }
        return result;
    }

    // Scalar Multiplication
    constexpr Vector<N> operator*(double scalar) const {
        Vector<N> result;
        for (std::size_t i = 0; i < N; ++i) {
            result.data[i] = this->data[i] * scalar;
        }
        return result;
    }

    // Dot Product
    double dot(const Vector<N>& other) const {
        double result = 0.0;
        for (std::size_t i = 0; i < N; ++i) {
            result += this->data[i] * other.data[i];
        }
        return result;
    }

    // Norm (Euclidean)
    double norm() const {
        return std::sqrt(this->dot(*this));
    }

    // Normalize
    Vector<N> normalized() const {
        double n = norm();
        if (n == 0.0) {
            throw std::runtime_error("Cannot normalize a zero vector.");
        }
        return (*this) * (1.0 / n);
    }

    // Transpose: Returns a Matrix<1, N>
    constexpr Matrix<1, N> transpose() const;

    // Output Stream Overload for Debugging
    friend std::ostream& operator<<(std::ostream& os, const Vector<N>& vec) {
        os << "Vector" << N << "(";
        for (std::size_t i = 0; i < N; ++i) {
            os << vec.data[i];
            if (i < N - 1) os << ", ";
        }
        os << ")";
        return os;
    }
};

// Matrix Template
template <std::size_t ROWS, std::size_t COLS>
struct Matrix {
    std::array<std::array<double, COLS>, ROWS> data;

    // Default constructor initializes all elements to zero
    constexpr Matrix() : data{} {}

    // Constructor from nested std::array
    constexpr Matrix(const std::array<std::array<double, COLS>, ROWS>& arr) : data(arr) {}

    // Constructor from initializer list
    Matrix(const std::initializer_list<std::initializer_list<double>>& list) {
        if (list.size() != ROWS) {
            throw std::invalid_argument("Initializer list rows must match matrix rows.");
        }
        auto row_it = list.begin();
        for (std::size_t i = 0; i < ROWS; ++i, ++row_it) {
            if (row_it->size() != COLS) {
                throw std::invalid_argument("Initializer list columns must match matrix columns.");
            }
            std::copy(row_it->begin(), row_it->end(), data[i].begin());
        }
    }

    // Addition
    constexpr Matrix<ROWS, COLS> operator+(const Matrix<ROWS, COLS>& other) const {
        Matrix<ROWS, COLS> result;
        for (std::size_t i = 0; i < ROWS; ++i)
            for (std::size_t j = 0; j < COLS; ++j)
                result.data[i][j] = this->data[i][j] + other.data[i][j];
        return result;
    }

    // Scalar Multiplication
    constexpr Matrix<ROWS, COLS> operator*(double scalar) const {
        Matrix<ROWS, COLS> result;
        for (std::size_t i = 0; i < ROWS; ++i)
            for (std::size_t j = 0; j < COLS; ++j)
                result.data[i][j] = this->data[i][j] * scalar;
        return result;
    }

    // Matrix Multiplication
    template <std::size_t OTHER_COLS>
    constexpr Matrix<ROWS, OTHER_COLS> operator*(const Matrix<COLS, OTHER_COLS>& other) const {
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

    // Matrix-Vector Multiplication
    template <std::size_t N>
    Vector<ROWS> operator*(const Vector<N>& vec) const {
        static_assert(COLS == N, "Matrix columns must match vector size.");
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

    // Transpose: Returns a Matrix<COLS, ROWS>
    constexpr Matrix<COLS, ROWS> transpose() const {
        Matrix<COLS, ROWS> result;
        for (std::size_t i = 0; i < ROWS; ++i)
            for (std::size_t j = 0; j < COLS; ++j)
                result.data[j][i] = this->data[i][j];
        return result;
    }

    // Static function to create an identity matrix
    static constexpr Matrix<ROWS, COLS> identity() {
        static_assert(ROWS == COLS, "Identity matrix requires ROWS == COLS.");
        Matrix<ROWS, COLS> id;
        for (std::size_t i = 0; i < ROWS; ++i) {
            for (std::size_t j = 0; j < COLS; ++j) {
                id.data[i][j] = (i == j) ? 1.0 : 0.0;
            }
        }
        return id;
    }

    // Output Stream Overload for Debugging
    friend std::ostream& operator<<(std::ostream& os, const Matrix<ROWS, COLS>& mat) {
        os << "Matrix" << ROWS << "x" << COLS << "(\n";
        for (std::size_t i = 0; i < ROWS; ++i) {
            os << "  [ ";
            for (std::size_t j = 0; j < COLS; ++j) {
                os << mat.data[i][j];
                if (j < COLS - 1) os << ", ";
            }
            os << " ]\n";
        }
        os << ")";
        return os;
    }
};

// Free function to create DH transformation matrix (only for Matrix<4,4>)
inline Matrix<4,4> fromDH(double alpha, double a, double d, double theta) {
    Matrix<4,4> T;

    T.data[0][0] = std::cos(theta);
    T.data[0][1] = -std::sin(theta) * std::cos(alpha);
    T.data[0][2] = std::sin(theta) * std::sin(alpha);
    T.data[0][3] = a * std::cos(theta);

    T.data[1][0] = std::sin(theta);
    T.data[1][1] = std::cos(theta) * std::cos(alpha);
    T.data[1][2] = -std::cos(theta) * std::sin(alpha);
    T.data[1][3] = a * std::sin(theta);

    T.data[2][0] = 0.0;
    T.data[2][1] = std::sin(alpha);
    T.data[2][2] = std::cos(alpha);
    T.data[2][3] = d;

    T.data[3][0] = 0.0;
    T.data[3][1] = 0.0;
    T.data[3][2] = 0.0;
    T.data[3][3] = 1.0;

    return T;
}

// Vector's transpose implementation
template <std::size_t N>
constexpr Matrix<1, N> Vector<N>::transpose() const {
    Matrix<1, N> result;
    for (std::size_t i = 0; i < N; ++i) {
        result.data[0][i] = this->data[i];
    }
    return result;
}

} // namespace MathLib

#endif // MATH_LIBRARY_H

