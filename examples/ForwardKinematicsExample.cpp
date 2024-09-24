#include "Kinematics/ForwardKinematics.h"
#include <iostream>
#include <array>

using namespace motion_planning;
using namespace MathLib;

int main() {
    try {
        // Define robot name and joint parameters
        std::string robot_name = "ExampleRobot";
        std::vector<double> joint_params = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

        // Instantiate ForwardKinematics
        ForwardKinematics fk(robot_name, joint_params);

        // Define joint angles (in radians)
        std::array<double, 6> joint_angles = {0.0, M_PI_4, M_PI_2, 0.0, 0.0, 0.0};

        // Compute transformation matrix
        Matrix<4,4> trans_mat;
        if (fk.computeFK(joint_angles, trans_mat) == 0) {
            std::cout << "Transformation Matrix:\n" << trans_mat << std::endl;
        } else {
            std::cerr << "Failed to compute Forward Kinematics." << std::endl;
        }

        // Compute end-effector position and orientation
        double eef_pos[3];
        double eef_rpy[3];
        if (fk.computeFK(joint_angles, eef_pos, eef_rpy) == 0) {
            std::cout << "End-Effector Position: (" 
                      << eef_pos[0] << ", " 
                      << eef_pos[1] << ", " 
                      << eef_pos[2] << ")\n";
            std::cout << "End-Effector Orientation (RPY): (" 
                      << eef_rpy[0] << ", " 
                      << eef_rpy[1] << ", " 
                      << eef_rpy[2] << ")\n";
        } else {
            std::cerr << "Failed to compute End-Effector Pose." << std::endl;
        }

    } catch (const std::exception& ex) {
        std::cerr << "Exception: " << ex.what() << std::endl;
    }

    return 0;
}

