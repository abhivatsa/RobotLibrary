// RobotModel.tpp
#ifndef ROBOT_MODEL_TPP
#define ROBOT_MODEL_TPP

#include "RobotModel.h"

namespace RoboticsLibrary {

// Helper function to compute rotation matrix around Z-axis
inline MathLib::Matrix<4, 4> rotationZ(double theta) {
    MathLib::Matrix<4, 4> R;
    R.setIdentity();
    double cos_theta = std::cos(theta);
    double sin_theta = std::sin(theta);
    R.data[0][0] = cos_theta;
    R.data[0][1] = -sin_theta;
    R.data[1][0] = sin_theta;
    R.data[1][1] = cos_theta;
    return R;
}

// Compute Rotation Matrix from Euler Angles (ZYX Order)
inline MathLib::Matrix<3, 3> computeEulerRotationMatrix(const MathLib::Vector<3>& euler_angles) {
    double roll = euler_angles.data[0];
    double pitch = euler_angles.data[1];
    double yaw = euler_angles.data[2];

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

// Constructor
template <std::size_t N>
RobotModel<N>::RobotModel(const RobotParameters<N>& robotParams)
    : robotParams_(robotParams) {}

// Compute Transformation Matrix
template <std::size_t N>
MathLib::Matrix<4, 4> RobotModel<N>::computeTransformationMatrix(const MathLib::Vector<3>& position,
                                                                  const MathLib::Vector<3>& orientation) const {
    // Compute rotation matrix from Euler angles
    MathLib::Matrix<3, 3> R = computeEulerRotationMatrix(orientation);

    // Create homogeneous transformation matrix
    MathLib::Matrix<4, 4> T;
    T.setIdentity();

    for (std::size_t i = 0; i < 3; ++i) {
        for (std::size_t j = 0; j < 3; ++j) {
            T.data[i][j] = R.data[i][j];
        }
        T.data[i][3] = position.data[i];
    }

    return T;
}

// Compute Global Centers of Mass
template <std::size_t N>
std::array<MathLib::Vector<3>, N> RobotModel<N>::computeGlobalCoMs(const JointAngles<N>& joint_angles) const {
    std::array<MathLib::Matrix<4,4>, N + 1> T; // T[0] is base
    T[0].setIdentity();

    // Compute transformation matrices for each link
    for (std::size_t i = 0; i < N; ++i) {
        const LinkParameters& link = robotParams_.links[i];
        double joint_angle = joint_angles.angles[i]; // Current joint angle

        // Compute joint rotation (assumed to be around Z-axis)
        MathLib::Matrix<4, 4> R_z = rotationZ(joint_angle);

        // Compute link transformation
        MathLib::Matrix<4, 4> T_link = computeTransformationMatrix(link.position, link.orientation);

        // Global transformation: T_prev * R_z * T_link
        T[i + 1] = T[i] * R_z * T_link;
    }

    // Compute global center of mass positions
    std::array<MathLib::Vector<3>, N> com_global;
    for (std::size_t i = 0; i < N; ++i) {
        const LinkParameters& link = robotParams_.links[i];
        // Local CoM in homogeneous coordinates
        MathLib::Vector<4> com_local_homogeneous = { link.com.data[0], link.com.data[1], link.com.data[2], 1.0 };
        // Global CoM: T[i + 1] * com_local_homogeneous
        MathLib::Vector<4> com_global_homogeneous = T[i + 1] * com_local_homogeneous;
        // Extract X, Y, Z
        com_global[i] = { com_global_homogeneous.data[0],
                          com_global_homogeneous.data[1],
                          com_global_homogeneous.data[2] };
    }

    return com_global;
}

// Check if two links collide based on their CoMs and radii
template <std::size_t N>
bool RobotModel<N>::checkLinkCollision(const MathLib::Vector<3>& com1, const MathLib::Vector<3>& com2,
                                       double radius1, double radius2) const {
    // Compute the Euclidean distance between the two CoMs
    double dx = com1.data[0] - com2.data[0];
    double dy = com1.data[1] - com2.data[1];
    double dz = com1.data[2] - com2.data[2];
    double distance = std::sqrt(dx * dx + dy * dy + dz * dz);

    // Check if distance is less than or equal to the sum of radii
    return (distance <= (radius1 + radius2));
}

// Detect self-collision based on current joint angles
template <std::size_t N>
bool RobotModel<N>::detectSelfCollision(const JointAngles<N>& joint_angles) const {
    // Step 1: Compute global CoM positions
    std::array<MathLib::Vector<3>, N> com_global = computeGlobalCoMs(joint_angles);

    // Step 2: Iterate over each unique pair of links to check for collisions
    for (std::size_t i = 0; i < N - 1; ++i) {
        for (std::size_t j = i + 1; j < N; ++j) {
            const LinkParameters& link1 = robotParams_.links[i];
            const LinkParameters& link2 = robotParams_.links[j];

            // Retrieve collision radii
            double radius1 = link1.collision_geometry.radius;
            double radius2 = link2.collision_geometry.radius;

            // Check collision between link i and link j
            if (checkLinkCollision(com_global[i], com_global[j], radius1, radius2)) {
                return true; // Collision detected
            }
        }
    }

    return false; // No collisions detected
}

} // namespace RoboticsLibrary

#endif // ROBOT_MODEL_TPP
