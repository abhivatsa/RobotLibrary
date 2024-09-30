// RobotParameters.h
#ifndef ROBOT_PARAMETERS_H
#define ROBOT_PARAMETERS_H

#include "LinkParameters.h"
#include "../math/MathLibrary.h"
#include <array>
#include <cstddef>      // For std::size_t
#include <stdexcept>    // For exceptions
#include <iostream>     // For std::ostream

namespace RoboticsLibrary {

/**
 * @brief Template structure representing joint angles for the robot.
 * 
 * @tparam N Number of joints in the robot.
 */
template <std::size_t N>
struct JointAngles {
    std::array<double, N> angles; ///< Array of joint angles in radians.

    /**
     * @brief Default constructor initializes all angles to zero.
     */
    JointAngles() : angles{} {}

    /**
     * @brief Constructor with initializer list.
     * 
     * @param list Initializer list containing joint angles.
     */
    JointAngles(const std::initializer_list<double>& list) {
        if (list.size() != N) {
            throw std::invalid_argument("Initializer list size must match number of joints.");
        }
        std::copy(list.begin(), list.end(), angles.begin());
    }

    /**
     * @brief Overload output operator for printing joint angles.
     */
    friend std::ostream& operator<<(std::ostream& os, const JointAngles<N>& ja) {
        os << "[";
        for (std::size_t i = 0; i < N; ++i) {
            os << ja.angles[i];
            if (i != N - 1) os << ", ";
        }
        os << "]";
        return os;
    }
};

/**
 * @brief Template class representing the parameters of a robot with N links.
 * 
 * @tparam N Number of links in the robot.
 */
template <std::size_t N>
struct RobotParameters {
    std::array<LinkParameters, N> links; ///< Array containing parameters for each link.

    /**
     * @brief Constructor for RobotParameters.
     * 
     * @param link_params Array of LinkParameters.
     */
    RobotParameters(const std::array<LinkParameters, N>& link_params)
        : links(link_params) {}
};

} // namespace RoboticsLibrary

#endif // ROBOT_PARAMETERS_H
