// ForwardKinematics.h
#ifndef FORWARD_KINEMATICS_H
#define FORWARD_KINEMATICS_H

#include "../common/RobotParameters.h" // Includes RobotParameters<N> and JointAngles<N>
#include "../math/MathLibrary.h"      // Custom math library with Vector and Matrix templates
#include <array>
#include <cstddef>           // For std::size_t
#include <stdexcept>         // For exceptions

namespace RoboticsLibrary {
namespace kinematics {

/**
 * @brief Template class for computing Forward Kinematics of an N-DoF Robot.
 * 
 * This class calculates the end-effector's transformation matrix and the Jacobian matrix
 * based on the current joint angles. It utilizes the position and orientation (Euler angles)
 * provided within RobotParameters.
 * 
 * @tparam N Number of degrees of freedom (joints) in the robot.
 */
template <std::size_t N>
class ForwardKinematics {
public:
    /**
     * @brief Constructs a ForwardKinematics instance with the provided robot parameters.
     * 
     * @param robotParams A reference to the robot's parameters, including joint configurations.
     */
    explicit ForwardKinematics(const RobotParameters<N>& robotParams);

    /**
     * @brief Destructor for ForwardKinematics.
     */
    ~ForwardKinematics() = default;

    /**
     * @brief Computes the end-effector's homogeneous transformation matrix based on joint angles.
     * 
     * @param joint_angles An instance of JointAngles<N> containing current joint angles in radians.
     * @return MathLib::Matrix<4,4> The end-effector's transformation matrix.
     */
    MathLib::Matrix<4,4> computeFK(const JointAngles<N>& joint_angles) const;

    /**
     * @brief Computes the Jacobian matrix based on current joint angles.
     * 
     * @param joint_angles An instance of JointAngles<N> containing current joint angles in radians.
     * @return MathLib::Matrix<6, N> The Jacobian matrix mapping joint velocities to end-effector velocities.
     */
    MathLib::Matrix<6, N> computeJacobian(const JointAngles<N>& joint_angles) const;

private:
    RobotParameters<N> robotParams_; ///< Robot parameters containing joint configurations.

    /**
     * @brief Computes the rotation matrix from Euler angles (Roll-Pitch-Yaw).
     * 
     * @param rpy Roll-Pitch-Yaw angles in radians.
     * @return MathLib::Matrix<3,3> The corresponding rotation matrix.
     */
    MathLib::Matrix<3,3> eulerToRotationMatrix(const MathLib::Vector<3>& rpy) const;

    /**
     * @brief Computes the homogeneous transformation matrix from rotation and translation.
     * 
     * @param R Rotation matrix.
     * @param t Translation vector.
     * @return MathLib::Matrix<4,4> Homogeneous transformation matrix.
     */
    MathLib::Matrix<4,4> computeTransformationMatrix(const MathLib::Matrix<3,3>& R,
                                                    const MathLib::Vector<3>& t) const;
};

} // namespace kinematics
} // namespace RoboticsLibrary

// Include the template implementation
#include "ForwardKinematics.tpp" // Ensure this file exists and is correctly named

#endif // FORWARD_KINEMATICS_H
