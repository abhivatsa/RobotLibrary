#ifndef FORWARD_KINEMATICS_H
#define FORWARD_KINEMATICS_H

#include "RobotParameters.h" // Includes RobotParameters<N>
#include "MathLibrary.h"     // Custom math library with Vector and Matrix templates
#include <array>
#include <cstddef>           // For std::size_t

namespace RoboticsLibrary {
namespace kinematics {

/**
 * @brief Template class for computing Forward Kinematics of an N-DoF Robot.
 * 
 * This class calculates the end-effector's transformation matrix based on the current joint angles.
 * It utilizes the Denavit-Hartenberg (DH) parameters encapsulated within RobotParameters.
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
     * @brief Computes Forward Kinematics and returns the transformation matrix.
     * 
     * @param joint_angles An array containing the current angles of all joints.
     * @param trans_mat Reference to a 4x4 matrix where the resulting transformation matrix will be stored.
     * @return int Status code (0 for success, non-zero for failure).
     */
    int computeFK(const std::array<double, N>& joint_angles,
                 MathLib::Matrix<4,4>& trans_mat) const noexcept;

    /**
     * @brief Computes Forward Kinematics and returns end-effector position and orientation.
     * 
     * @param joint_angles An array containing the current angles of all joints.
     * @param eef_pos A 3-element array to store the end-effector's position (x, y, z).
     * @param eef_rpy A 3-element array to store the end-effector's orientation in Roll-Pitch-Yaw angles.
     * @return int Status code (0 for success, non-zero for failure).
     */
    int computeFK(const std::array<double, N>& joint_angles,
                 double eef_pos[3], double eef_rpy[3]) const noexcept;

private:
    RobotParameters<N> robotParams_; ///< Robot parameters containing joint configurations.

    /**
     * @brief Computes the Denavit-Hartenberg (DH) transformation matrix for a given joint.
     * 
     * @param joint The parameters of the joint for which to compute the transformation.
     * @return A 4x4 transformation matrix representing the joint's transformation.
     */
    MathLib::Matrix<4, 4> computeTransform(const JointParameters& joint) const noexcept;

    /**
     * @brief Converts a quaternion to Euler angles (Roll, Pitch, Yaw).
     * 
     * @param quat A 4-element array representing the quaternion (w, x, y, z).
     * @param rpy A 3-element array to store the resulting Euler angles.
     */
    void quaternionToEuler(const double quat[4], double rpy[3]) const noexcept;

    /**
     * @brief Converts Euler angles (Roll, Pitch, Yaw) to a quaternion.
     * 
     * @param rpy A 3-element array representing the Euler angles.
     * @param quat A 4-element array to store the resulting quaternion.
     */
    void eulerToQuaternion(const double rpy[3], double quat[4]) const noexcept;
};

#include "ForwardKinematics.tpp" // Include the template implementation

} // namespace kinematics
} // namespace RoboticsLibrary

#endif // FORWARD_KINEMATICS_H
