// Robot.h
#ifndef ROBOT_H
#define ROBOT_H

#include "../common/RobotParameters.h"
#include "../kinematics/ForwardKinematics.h"
#include "../kinematics/InverseKinematics.h"
#include "../dynamics/RecursiveNewtonEuler.h"
#include "../TrajectoryPlanning/TrajectoryPlanner.h"
#include "../TrajectoryPlanning/Trajectory.h"
#include "../collisiondetection/RobotModel.h"
#include <string>        // For file paths
#include <array>
#include <optional>      // For std::optional
#include <cassert>       // For assert
#include <iostream>      // For std::ostream
#include <nlohmann/json.hpp> // Include JSON library

namespace RoboticsLibrary {

/**
 * @brief The Robot class encapsulates all functionalities and parameters of the robot.
 * 
 * @tparam N Number of links/joints in the robot.
 */
template <std::size_t N>
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
    explicit Robot(const RobotParameters<N>& params);

    /**
     * @brief Loads robot parameters from a JSON configuration file.
     * 
     * @param configFilePath Path to the JSON configuration file.
     * @return true If loading was successful.
     * @return false If loading failed.
     */
    bool loadFromConfig(const std::string& configFilePath);

    /**
     * @brief Performs forward kinematics to compute end-effector pose based on given joint angles.
     * 
     * @param joint_angles The joint angles for which to compute forward kinematics.
     * @return MathLib::Matrix<4, 4> The end-effector pose as a transformation matrix.
     */
    MathLib::Matrix<4, 4> computeForwardKinematics(const JointAngles<N>& joint_angles) const;

    /**
     * @brief Computes the Jacobian matrix for the given joint angles.
     * 
     * @param joint_angles The joint angles for which to compute the Jacobian.
     * @return MathLib::Matrix<6, N> The Jacobian matrix.
     */
    MathLib::Matrix<6, N> computeJacobian(const JointAngles<N>& joint_angles) const;

    /**
     * @brief Solves inverse kinematics for a desired end-effector pose.
     * 
     * @param desiredPose The desired end-effector pose.
     * @return std::optional<JointAngles<N>> The computed joint angles, or std::nullopt if no solution found.
     */
    std::optional<JointAngles<N>> computeInverseKinematics(const MathLib::Matrix<4, 4>& desiredPose) const;

    /**
     * @brief Performs dynamic computations using the Recursive Newton-Euler algorithm.
     * 
     * @param joint_angles The current joint angles.
     * @param joint_velocities The current joint velocities.
     * @param joint_accelerations The current joint accelerations.
     * @return std::array<MathLib::Vector<3>, N> The computed forces and torques for each joint.
     */
    std::array<MathLib::Vector<3>, N> computeDynamics(const JointAngles<N>& joint_angles,
                                                     const std::array<double, N>& joint_velocities,
                                                     const std::array<double, N>& joint_accelerations) const;

    /**
     * @brief Detects self-collisions based on the given joint angles.
     * 
     * @param joint_angles The joint angles to evaluate for collisions.
     * @return true If a self-collision is detected.
     * @return false If no self-collisions are detected.
     */
    bool detectSelfCollision(const JointAngles<N>& joint_angles) const;

    /**
     * @brief Plans a trajectory from the current joint angles to the target joint angles.
     * 
     * @param start_angles The starting joint angles.
     * @param target_angles The target joint angles.
     * @param duration Duration of the trajectory in seconds.
     * @param steps Number of interpolation steps.
     * @return std::optional<Trajectory<N>> The planned trajectory, or std::nullopt if planning failed.
     */
    std::optional<Trajectory<N>> planTrajectory(const JointAngles<N>& start_angles,
                                               const JointAngles<N>& target_angles,
                                               double duration,
                                               std::size_t steps) const;

    /**
     * @brief Executes a given trajectory.
     *        (Placeholder for actual execution implementation)
     * 
     * @param traj The trajectory to execute.
     */
    void executeTrajectory(const Trajectory<N>& traj) const;

private:
    RobotParameters<N> parameters_; ///< Robot parameters.
    kinematics::ForwardKinematics<N> fk_;       ///< Forward kinematics handler.
    kinematics::InverseKinematics<N> ik_;       ///< Inverse kinematics handler.
    dynamics::RecursiveNewtonEuler<N> rne_;   ///< Dynamics handler.
    RobotModel<N> robot_model_;     ///< Robot model for collision detection.
    TrajectoryPlanner<N> planner_;  ///< Trajectory planner.

    /**
     * @brief Initializes robot parameters internally with default values.
     *        This method can be expanded to set up default configurations.
     */
    void initializeDefaultParameters();
};

} // namespace RoboticsLibrary

#include "Robot.tpp"

#endif // ROBOT_H
