// RobotParameters.h
#ifndef ROBOT_PARAMETERS_H
#define ROBOT_PARAMETERS_H

#include "JointParameters.h"
#include <array>
#include <cstddef> // For std::size_t

namespace RoboticsLibrary {

/**
 * @brief Structure to hold all parameters for the robot.
 * 
 * @tparam N Number of joints in the robot.
 */
template <std::size_t N>
struct RobotParameters {
    std::array<JointParameters, N> joints; ///< Array of joint parameters.

    /**
     * @brief Default constructor.
     */
    RobotParameters() = default;

    /**
     * @brief Constructor accepting an initializer list for joints.
     * 
     * @param joints_ Array containing parameters for each joint.
     */
    RobotParameters(const std::array<JointParameters, N>& joints_) : joints(joints_) {}
};

} // namespace RoboticsLibrary

#endif // ROBOT_PARAMETERS_H
