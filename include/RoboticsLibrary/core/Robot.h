// Robot.h
#ifndef ROBOT_H
#define ROBOT_H

#include "RobotParameters.h"
#include "ForwardKinematics.h"
#include "InverseKinematics.h"
#include "RecursiveNewtonEuler.h"
#include <string> // For file paths

namespace RoboticsLibrary {

/**
 * @brief The Robot class encapsulates all functionalities and parameters of the robot.
 */
class Robot {
public:
    /**
     * @brief Constructs a Robot instance with default parameters.
     */
    Robot();

    /**
     * @brief Constructs a Robot instance with externally provided parameters.
     * 
     * @param params The robot parameters.
     */
    Robot(const RobotParameters<6>& params);

    /**
     * @brief Loads robot parameters from a JSON configuration file.
     * 
     * @param configFilePath Path to the JSON configuration file.
     */
    void loadFromConfig(const std::string& configFilePath);

    /**
     * @brief Performs forward kinematics to compute end-effector pose.
     * 
     * @return The end-effector pose as a transformation matrix.
     */
    MathLib::Matrix<4, 4> computeForwardKinematics();

    /**
     * @brief Computes the Jacobian matrix for the current robot configuration.
     * 
     * @return The Jacobian matrix.
     */
    MathLib::Matrix<6, 6> computeJacobian();

    /**
     * @brief Solves inverse kinematics for a desired end-effector pose.
     * 
     * @param desiredPose The desired end-effector pose.
     * @return RobotParameters containing the joint angles.
     */
    RobotParameters<6> computeInverseKinematics(const MathLib::Matrix<4, 4>& desiredPose);

    /**
     * @brief Performs dynamic computations using the Recursive Newton-Euler algorithm.
     */
    void computeDynamics();

private:
    RobotParameters<6> parameters; // Assuming a 6-DOF robot
    ForwardKinematics fk;
    InverseKinematics ik;
    RecursiveNewtonEuler rne;

    /**
     * @brief Initializes robot parameters internally with default values.
     */
    void initializeDefaultParameters();
};

} // namespace RoboticsLibrary

#endif // ROBOT_H
