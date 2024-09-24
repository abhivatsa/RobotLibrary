#ifndef FORWARD_KINEMATICS_H
#define FORWARD_KINEMATICS_H

#include "../Robot.h" // Adjust the path based on your directory structure
#include "MathLibrary.h" // Include your custom math library
#include <array>

namespace motion_planning {

// Struct for Denavit-Hartenberg Parameters
struct DHParameters {
    double alpha;
    double a;
    double d;
    double theta;

    // Constructor to initialize DH parameters
    DHParameters(double _alpha = 0.0, double _a = 0.0, double _d = 0.0, double _theta = 0.0)
        : alpha(_alpha), a(_a), d(_d), theta(_theta) {}
};

class ForwardKinematics : public Robot {
public:
    ForwardKinematics(const std::string& name, const std::vector<double>& jointParameters);
    ~ForwardKinematics() override = default;

    // Compute Forward Kinematics and return the transformation matrix
    int computeFK(const std::array<double, 6>& joint_angles, MathLib::Matrix<4,4>& trans_mat) const;

    // Compute Forward Kinematics and return end-effector position and orientation (Euler angles)
    int computeFK(const std::array<double, 6>& joint_angles, double eef_pos[3], double eef_rpy[3]) const;

private:
    std::array<DHParameters, 6> dh_params; // DH parameters for each joint

    // Utility functions for quaternion and Euler conversions
    void quaternionToEuler(const double quat[4], double rpy[3]) const;
    void eulerToQuaternion(const double rpy[3], double quat[4]) const;
};

} // namespace motion_planning

#endif // FORWARD_KINEMATICS_H

