#include "ForwardKinematics.h"
#include <cmath>
#include <iostream>

namespace motion_planning {

// ForwardKinematics constructor
ForwardKinematics::ForwardKinematics(const std::string& name, const std::vector<double>& jointParameters)
    : Robot(name, jointParameters)
{
    // Validate jointParameters size
    if (jointParameters.size() != 6) {
        throw std::invalid_argument("Joint parameters must be of size 6.");
    }

    // Initialize DH parameters for Inline configuration
    dh_params = {{
        // Joint 1
        DHParameters(0.0, 0.0, 0.0, 0.0),
        // Joint 2
        DHParameters(-M_PI_2, 0.0, 0.0, -M_PI_2),
        // Joint 3
        DHParameters(0.0, 0.6, 0.0, M_PI_2),
        // Joint 4
        DHParameters(M_PI_2, 0.0, 0.4, 0.0),
        // Joint 5
        DHParameters(-M_PI_2, 0.0, 0.0, 0.0),
        // Joint 6
        DHParameters(M_PI_2, 0.0, 0.0, 0.0)
    }};
}

int ForwardKinematics::computeFK(const std::array<double, 6>& joint_angles, MathLib::Matrix<4,4>& trans_mat) const {
    // Initialize T_glo as identity matrix using the math library
    MathLib::Matrix<4,4> T_glo = MathLib::Matrix<4,4>::identity();

    // Iterate over each joint to compute the overall transformation matrix
    for (size_t ctr = 0; ctr < dh_params.size(); ctr++) {
        // Current joint's theta
        double joint_theta = dh_params[ctr].theta + joint_angles[ctr];

        // Create the local transformation matrix using DH parameters
        MathLib::Matrix<4,4> T_loc = MathLib::fromDH(
            dh_params[ctr].alpha,
            dh_params[ctr].a,
            dh_params[ctr].d,
            joint_theta
        );

        // Update the global transformation matrix
        T_glo = T_glo * T_loc;
    }

    // Assign the computed global transformation matrix to the output parameter
    trans_mat = T_glo;
    return 0;
}

int ForwardKinematics::computeFK(const std::array<double, 6>& joint_angles, double eef_pos[3], double eef_rpy[3]) const {
    // Compute the transformation matrix
    MathLib::Matrix<4,4> trans_mat;
    computeFK(joint_angles, trans_mat);

    // Extract end-effector position from the transformation matrix
    eef_pos[0] = trans_mat.data[0][3];
    eef_pos[1] = trans_mat.data[1][3];
    eef_pos[2] = trans_mat.data[2][3];

    // Extract rotation matrix (3x3) from the transformation matrix
    double R[3][3] = {
        {trans_mat.data[0][0], trans_mat.data[0][1], trans_mat.data[0][2]},
        {trans_mat.data[1][0], trans_mat.data[1][1], trans_mat.data[1][2]},
        {trans_mat.data[2][0], trans_mat.data[2][1], trans_mat.data[2][2]}
    };

    // Convert rotation matrix to quaternion
    double quat[4]; // [w, x, y, z]
    double trace = R[0][0] + R[1][1] + R[2][2];
    if (trace > 0.0) {
        double s = std::sqrt(trace + 1.0) * 2.0; // s=4*qw
        quat[0] = 0.25 * s;
        quat[1] = (R[2][1] - R[1][2]) / s;
        quat[2] = (R[0][2] - R[2][0]) / s;
        quat[3] = (R[1][0] - R[0][1]) / s;
    }
    else {
        if (R[0][0] > R[1][1] && R[0][0] > R[2][2]) {
            double s = std::sqrt(1.0 + R[0][0] - R[1][1] - R[2][2]) * 2.0; // s=4*qx
            quat[0] = (R[2][1] - R[1][2]) / s;
            quat[1] = 0.25 * s;
            quat[2] = (R[0][1] + R[1][0]) / s;
            quat[3] = (R[0][2] + R[2][0]) / s;
        }
        else if (R[1][1] > R[2][2]) {
            double s = std::sqrt(1.0 + R[1][1] - R[0][0] - R[2][2]) * 2.0; // s=4*qy
            quat[0] = (R[0][2] - R[2][0]) / s;
            quat[1] = (R[0][1] + R[1][0]) / s;
            quat[2] = 0.25 * s;
            quat[3] = (R[1][2] + R[2][1]) / s;
        }
        else {
            double s = std::sqrt(1.0 + R[2][2] - R[0][0] - R[1][1]) * 2.0; // s=4*qz
            quat[0] = (R[1][0] - R[0][1]) / s;
            quat[1] = (R[0][2] + R[2][0]) / s;
            quat[2] = (R[1][2] + R[2][1]) / s;
            quat[3] = 0.25 * s;
        }
    }

    // Convert quaternion to Euler angles (roll, pitch, yaw)
    quaternionToEuler(quat, eef_rpy);

    return 0;
}

void ForwardKinematics::quaternionToEuler(const double quat[4], double rpy[3]) const {
    // Assuming the quaternion is in the form [w, x, y, z]
    double w = quat[0];
    double x = quat[1];
    double y = quat[2];
    double z = quat[3];

    // Roll (x-axis rotation)
    double sinr_cosp = 2.0 * (w * x + y * z);
    double cosr_cosp = 1.0 - 2.0 * (x * x + y * y);
    rpy[0] = std::atan2(sinr_cosp, cosr_cosp);

    // Pitch (y-axis rotation)
    double sinp = 2.0 * (w * y - z * x);
    if (std::abs(sinp) >= 1)
        rpy[1] = std::copysign(M_PI / 2, sinp); // Use 90 degrees if out of range
    else
        rpy[1] = std::asin(sinp);

    // Yaw (z-axis rotation)
    double siny_cosp = 2.0 * (w * z + x * y);
    double cosy_cosp = 1.0 - 2.0 * (y * y + z * z);
    rpy[2] = std::atan2(siny_cosp, cosy_cosp);
}

void ForwardKinematics::eulerToQuaternion(const double rpy[3], double quat[4]) const {
    // Assuming rpy = {roll, pitch, yaw}
    double cr = std::cos(rpy[0] * 0.5);
    double sr = std::sin(rpy[0] * 0.5);
    double cp = std::cos(rpy[1] * 0.5);
    double sp = std::sin(rpy[1] * 0.5);
    double cy = std::cos(rpy[2] * 0.5);
    double sy = std::sin(rpy[2] * 0.5);

    quat[0] = cr * cp * cy + sr * sp * sy; // w
    quat[1] = sr * cp * cy - cr * sp * sy; // x
    quat[2] = cr * sp * cy + sr * cp * sy; // y
    quat[3] = cr * cp * sy - sr * sp * cy; // z
}

} // namespace motion_planning

