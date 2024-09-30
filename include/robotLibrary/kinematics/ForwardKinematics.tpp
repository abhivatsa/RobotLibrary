// ForwardKinematics.tpp
#ifndef FORWARD_KINEMATICS_TPP
#define FORWARD_KINEMATICS_TPP

#include "ForwardKinematics.h"
#include <cmath>    // For trigonometric functions

namespace RoboticsLibrary {
namespace kinematics {

// Constructor
template <std::size_t N>
ForwardKinematics<N>::ForwardKinematics(const RobotParameters<N>& robotParams)
    : robotParams_(robotParams) {}

// Convert Euler angles to Rotation Matrix
template <std::size_t N>
MathLib::Matrix<3,3> ForwardKinematics<N>::eulerToRotationMatrix(const MathLib::Vector<3>& rpy) const {
    double roll = rpy.data[0];
    double pitch = rpy.data[1];
    double yaw = rpy.data[2];

    double cr = std::cos(roll);
    double sr = std::sin(roll);
    double cp = std::cos(pitch);
    double sp = std::sin(pitch);
    double cy = std::cos(yaw);
    double sy = std::sin(yaw);

    MathLib::Matrix<3,3> R;
    R.data = {{
        { cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr },
        { sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr },
        { -sp,     cp * sr,                cp * cr               }
    }};

    return R;
}

// Compute Homogeneous Transformation Matrix
template <std::size_t N>
MathLib::Matrix<4,4> ForwardKinematics<N>::computeTransformationMatrix(const MathLib::Matrix<3,3>& R,
                                                                      const MathLib::Vector<3>& t) const {
    MathLib::Matrix<4,4> T;
    T.setIdentity(); // Initialize to identity

    for (std::size_t i = 0; i < 3; ++i) {
        for (std::size_t j = 0; j < 3; ++j) {
            T.data[i][j] = R.data[i][j];
        }
        T.data[i][3] = t.data[i];
    }
    return T;
}

// Compute Forward Kinematics
template <std::size_t N>
MathLib::Matrix<4,4> ForwardKinematics<N>::computeFK(const JointAngles<N>& joint_angles) const {
    MathLib::Matrix<4,4> T = MathLib::Matrix<4,4>(); // Initialized to zero
    T.setIdentity(); // Set to identity

    for (std::size_t i = 0; i < N; ++i) {
        // Get link parameters
        const LinkParameters& link = robotParams_.links[i];

        // Current joint angle
        double theta = joint_angles.angles[i];

        // Compute rotation matrix from Euler angles
        MathLib::Matrix<3,3> R = eulerToRotationMatrix(link.orientation);

        // Compute translation vector
        MathLib::Vector<3> t = link.position;

        // Compute homogeneous transformation for the current link
        MathLib::Matrix<4,4> T_link = computeTransformationMatrix(R, t);

        // Apply joint rotation about Z-axis (assuming revolute joints rotate around Z)
        MathLib::Matrix<4,4> T_joint;
        T_joint.setIdentity();
        double cos_theta_joint = std::cos(theta);
        double sin_theta_joint = std::sin(theta);
        T_joint.data[0][0] = cos_theta_joint;
        T_joint.data[0][1] = -sin_theta_joint;
        T_joint.data[1][0] = sin_theta_joint;
        T_joint.data[1][1] = cos_theta_joint;
        // Z-axis rotation remains unchanged

        // Update the overall transformation
        T = T * T_joint * T_link;
    }

    return T;
}

// Compute Jacobian
template <std::size_t N>
MathLib::Matrix<6, N> ForwardKinematics<N>::computeJacobian(const JointAngles<N>& joint_angles) const {
    MathLib::Matrix<6, N> J;
    J.setZero(); // Initialize Jacobian to zero

    // Compute forward kinematics to get end-effector position
    MathLib::Matrix<4,4> T_end = computeFK(joint_angles);
    MathLib::Vector<3> p_end = { T_end.data[0][3], T_end.data[1][3], T_end.data[2][3] };

    // Initialize transformation matrix
    MathLib::Matrix<4,4> T = MathLib::Matrix<4,4>();
    T.setIdentity(); // Set to identity

    for (std::size_t i = 0; i < N; ++i) {
        // Get link parameters
        const LinkParameters& link = robotParams_.links[i];

        // Current joint angle
        double theta = joint_angles.angles[i];

        // Compute rotation matrix from Euler angles
        MathLib::Matrix<3,3> R = eulerToRotationMatrix(link.orientation);

        // Compute translation vector
        MathLib::Vector<3> t = link.position;

        // Compute homogeneous transformation for the current link
        MathLib::Matrix<4,4> T_link = computeTransformationMatrix(R, t);

        // Apply joint rotation about Z-axis
        MathLib::Matrix<4,4> T_joint;
        T_joint.setIdentity();
        double cos_theta_joint = std::cos(theta);
        double sin_theta_joint = std::sin(theta);
        T_joint.data[0][0] = cos_theta_joint;
        T_joint.data[0][1] = -sin_theta_joint;
        T_joint.data[1][0] = sin_theta_joint;
        T_joint.data[1][1] = cos_theta_joint;
        // Z-axis rotation remains unchanged

        // Update the overall transformation
        T = T * T_joint * T_link;

        // Current joint axis (assumed to be Z-axis in the joint frame)
        MathLib::Vector<3> z_axis = { T.data[0][2], T.data[1][2], T.data[2][2] };

        // Current joint origin
        MathLib::Vector<3> p_joint = { T.data[0][3], T.data[1][3], T.data[2][3] };

        // Compute linear velocity Jacobian column using MathLibrary's free cross product function
        MathLib::Vector<3> Jv = MathLib::cross(z_axis, (p_end - p_joint));

        // Compute angular velocity Jacobian column
        MathLib::Vector<3> Jw = z_axis;

        // Assign to Jacobian matrix
        for (std::size_t j = 0; j < 3; ++j) {
            J.data[j][i] = Jv.data[j];
            J.data[j+3][i] = Jw.data[j];
        }
    }

    return J;
}

} // namespace kinematics
} // namespace RoboticsLibrary

#endif // FORWARD_KINEMATICS_TPP
