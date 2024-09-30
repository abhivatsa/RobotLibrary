// RecursiveNewtonEuler.h
#ifndef RECURSIVE_NEWTON_EULER_H
#define RECURSIVE_NEWTON_EULER_H

#include "../common/RobotParameters.h" // Includes RobotParameters<N> and JointAngles<N>
#include "../math/MathLibrary.h"      // Custom math library with Vector and Matrix templates
#include <array>
#include <cstddef>                     // For std::size_t
#include <stdexcept>                   // For exceptions

namespace RoboticsLibrary {
namespace dynamics {

/**
 * @brief Template class for computing dynamics using the Recursive Newton-Euler (RNE) algorithm.
 * 
 * This class calculates the joint torques required for gravity compensation based on the robot's
 * current joint angles and physical parameters.
 * 
 * @tparam N Number of degrees of freedom (joints) in the robot.
 */
template <std::size_t N>
class RecursiveNewtonEuler {
public:
    /**
     * @brief Constructs a RecursiveNewtonEuler instance with the provided robot parameters.
     * 
     * @param robotParams A reference to the robot's parameters, including link configurations.
     */
    explicit RecursiveNewtonEuler(const RobotParameters<N>& robotParams);

    /**
     * @brief Computes the joint torques required for gravity compensation.
     * 
     * This method calculates the torques needed at each joint to counteract gravitational forces,
     * enabling smooth hand-guiding of the robot.
     * 
     * @param joint_angles The current joint angles in radians.
     * @return std::array<double, N> The required joint torques for gravity compensation.
     */
    std::array<double, N> computeGravityCompensation(const JointAngles<N>& joint_angles) const;

private:
    RobotParameters<N> robotParams_; ///< Robot parameters containing link configurations.

    /**
     * @brief Helper function to compute the homogeneous transformation matrix for each link.
     * 
     * @param position Position vector (x, y, z) of the link in meters.
     * @param orientation Orientation vector (roll, pitch, yaw) in radians.
     * @return MathLib::Matrix<4,4> The homogeneous transformation matrix.
     */
    MathLib::Matrix<4, 4> computeTransformationMatrix(const MathLib::Vector<3>& position,
                                                      const MathLib::Vector<3>& orientation) const;

    /**
     * @brief Helper function to convert Euler angles (Roll-Pitch-Yaw) to a rotation matrix.
     * 
     * @param rpy Roll-Pitch-Yaw angles in radians.
     * @return MathLib::Matrix<3,3> The corresponding rotation matrix.
     */
    MathLib::Matrix<3, 3> eulerToRotationMatrix(const MathLib::Vector<3>& rpy) const;

    /**
     * @brief Computes the cross product of two 3D vectors.
     * 
     * @param a First vector.
     * @param b Second vector.
     * @return MathLib::Vector<3> The resulting cross product vector.
     */
    //MathLib::Vector<3> crossProduct(const MathLib::Vector<3>& a, const MathLib::Vector<3>& b) const;
};

} // namespace dynamics
} // namespace RoboticsLibrary

#include "RecursiveNewtonEuler.tpp" // Include the template implementation

#endif // RECURSIVE_NEWTON_EULER_H

