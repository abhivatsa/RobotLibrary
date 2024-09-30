#ifndef RECURSIVE_NEWTON_EULER_TPP
#define RECURSIVE_NEWTON_EULER_TPP

#include "RecursiveNewtonEuler.h"

namespace RoboticsLibrary {
namespace dynamics {

// Constructor
template <std::size_t N>
RecursiveNewtonEuler<N>::RecursiveNewtonEuler(const RobotParameters<N>& robotParams)
    : robotParams_(robotParams) {}

// Convert Euler angles to Rotation Matrix
template <std::size_t N>
MathLib::Matrix<3, 3> RecursiveNewtonEuler<N>::eulerToRotationMatrix(const MathLib::Vector<3>& rpy) const {
    double roll = rpy.data[0];
    double pitch = rpy.data[1];
    double yaw = rpy.data[2];

    double cr = std::cos(roll);
    double sr = std::sin(roll);
    double cp = std::cos(pitch);
    double sp = std::sin(pitch);
    double cy = std::cos(yaw);
    double sy = std::sin(yaw);

    MathLib::Matrix<3, 3> R;
    R.data = {{
        { cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr },
        { sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr },
        { -sp,     cp * sr,                cp * cr               }
    }};

    return R;
}

// Compute Transformation Matrix
template <std::size_t N>
MathLib::Matrix<4, 4> RecursiveNewtonEuler<N>::computeTransformationMatrix(
    const MathLib::Vector<3>& position,
    const MathLib::Vector<3>& orientation) const {

    MathLib::Matrix<3, 3> R = eulerToRotationMatrix(orientation);

    // Use setIidentity() method from MathLib
    MathLib::Matrix<4, 4> T ;
    T.setIdentity();

    for (std::size_t i = 0; i < 3; ++i) {
        for (std::size_t j = 0; j < 3; ++j) {
            T.data[i][j] = R.data[i][j];
        }
        T.data[i][3] = position.data[i];
    }

    return T;
}

// Compute Gravity Compensation
template <std::size_t N>
std::array<double, N> RecursiveNewtonEuler<N>::computeGravityCompensation(const JointAngles<N>& joint_angles) const {
    // Initialize transformation matrices
    std::array<MathLib::Matrix<4, 4>, N + 1> T; // T[0] is base
    T[0].setIdentity();

    // Compute transformation matrices for each link
    for (std::size_t i = 0; i < N; ++i) {
        const LinkParameters& link = robotParams_.links[i];
        MathLib::Matrix<4, 4> T_link = computeTransformationMatrix(link.position, link.orientation);
        T[i + 1] = T[i] * T_link;
    }

    // Compute global center of mass positions
    std::array<MathLib::Vector<3>, N> com_global;
    for (std::size_t i = 0; i < N; ++i) {
        const LinkParameters& link = robotParams_.links[i];
        MathLib::Vector<4> com_local_homogeneous = { link.com.data[0], link.com.data[1], link.com.data[2], 1.0 };
        MathLib::Vector<4> com_global_homogeneous = T[i + 1] * com_local_homogeneous;
        com_global[i] = { com_global_homogeneous.data[0],
                          com_global_homogeneous.data[1],
                          com_global_homogeneous.data[2] };
    }

    // Compute gravitational forces
    std::array<MathLib::Vector<3>, N> F;
    for (std::size_t i = 0; i < N; ++i) {
        const LinkParameters& link = robotParams_.links[i];
        F[i] = { 0.0, 0.0, -link.mass * 9.81 }; // Gravity in -Z direction
    }

    // Initialize torque accumulator
    std::array<MathLib::Vector<3>, N> torques = {};

    // Backward recursion for torque computation
    for (int i = static_cast<int>(N) - 1; i >= 0; --i) {
        // Position of the joint axis (assuming Z-axis rotation)
        MathLib::Vector<3> p_joint = { T[i + 1].data[0][3], T[i + 1].data[1][3], T[i + 1].data[2][3] };

        // Torque due to the current link's weight
        MathLib::Vector<3> r = com_global[i] - p_joint;
        
        // Use MathLib's cross product
        MathLib::Vector<3> torque = MathLib::cross(r, F[i]);

        // Add torque from child links
        if (i < static_cast<int>(N) - 1) {
            torque = { torque.data[0] + torques[i + 1].data[0],
                       torque.data[1] + torques[i + 1].data[1],
                       torque.data[2] + torques[i + 1].data[2] };
        }

        torques[i] = torque;
    }

    // Project torques onto joint axes (assuming Z-axis)
    std::array<double, N> joint_torques = {};
    for (std::size_t i = 0; i < N; ++i) {
        // Joint axis in global frame (Z-axis)
        MathLib::Vector<3> joint_axis = { T[i + 1].data[0][2], T[i + 1].data[1][2], T[i + 1].data[2][2] };
        joint_torques[i] = joint_axis.dot(torques[i]);
    }

    return joint_torques;
}

} // namespace dynamics
} // namespace RoboticsLibrary

#endif // RECURSIVE_NEWTON_EULER_TPP
