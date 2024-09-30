// Robot.tpp
#ifndef ROBOT_TPP
#define ROBOT_TPP

#include <nlohmann/json.hpp> // For JSON parsing
#include <fstream>           // For file operations

namespace RoboticsLibrary {

// Alias for JSON library
using json = nlohmann::json;

// --------------------- Robot Implementation ---------------------

// Default constructor
template <std::size_t N>
Robot<N>::Robot() {
    initializeDefaultParameters();
}

// Constructor with parameters
template <std::size_t N>
Robot<N>::Robot(const RobotParameters<N>& params)
    : parameters_(params),
      fk_(params),
      ik_(params),
      rne_(params),
      robot_model_(params),
      planner_() {}

// Initialize default parameters (can be customized)
template <std::size_t N>
void Robot<N>::initializeDefaultParameters() {
    // Example: Initialize all links with default values
    std::array<LinkParameters, N> default_links = {};
    for (std::size_t i = 0; i < N; ++i) {
        MathLib::Vector<3> position = {0.0, 0.0, 0.0};
        MathLib::Vector<3> orientation = {0.0, 0.0, 0.0};
        double mass = 1.0;
        MathLib::Vector<3> com = {0.0, 0.0, 0.0};
        MathLib::Matrix<3, 3> inertia;
        inertia.setIdentity();
        inertia = inertia * 0.01;

        CollisionGeometry collision_geom(0.1, 0.5); // Example values

        default_links[i] = LinkParameters(position, orientation, mass, com, inertia, collision_geom);
    }
    parameters_ = RobotParameters<N>(default_links);
    fk_ = ForwardKinematics<N>(parameters_);
    ik_ = InverseKinematics<N>(parameters_);
    rne_ = RecursiveNewtonEuler<N>(parameters_);
    robot_model_ = RobotModel<N>(parameters_);
    // TrajectoryPlanner does not require initialization beyond default
}

// Load robot parameters from JSON config file
template <std::size_t N>
bool Robot<N>::loadFromConfig(const std::string& configFilePath) {
    std::ifstream file(configFilePath);
    if (!file.is_open()) {
        std::cerr << "Failed to open config file: " << configFilePath << "\n";
        return false;
    }

    json j;
    try {
        file >> j;
    } catch (json::parse_error& e) {
        std::cerr << "JSON parse error: " << e.what() << "\n";
        return false;
    }

    if (!j.contains("links") || !j["links"].is_array() || j["links"].size() != N) {
        std::cerr << "Invalid or mismatched number of links in config file.\n";
        return false;
    }

    std::array<LinkParameters, N> links;
    for (std::size_t i = 0; i < N; ++i) {
        auto& link_json = j["links"][i];
        // Extract position
        MathLib::Vector<3> position;
        for (std::size_t dim = 0; dim < 3; ++dim) {
            position.data[dim] = link_json["position"][dim].get<double>();
        }

        // Extract orientation
        MathLib::Vector<3> orientation;
        for (std::size_t dim = 0; dim < 3; ++dim) {
            orientation.data[dim] = link_json["orientation"][dim].get<double>();
        }

        // Extract mass
        double mass = link_json["mass"].get<double>();

        // Extract center of mass
        MathLib::Vector<3> com;
        for (std::size_t dim = 0; dim < 3; ++dim) {
            com.data[dim] = link_json["com"][dim].get<double>();
        }

        // Extract inertia tensor
        MathLib::Matrix<3, 3> inertia;
        for (std::size_t row = 0; row < 3; ++row) {
            for (std::size_t col = 0; col < 3; ++col) {
                inertia.data[row][col] = link_json["inertia_tensor"][row][col].get<double>();
            }
        }

        // Extract collision geometry (cylinders)
        double radius = link_json["collision_geometry"]["radius"].get<double>();
        double height = link_json["collision_geometry"]["height"].get<double>();
        CollisionGeometry collision_geom(radius, height);

        // Create LinkParameters
        links[i] = LinkParameters(position, orientation, mass, com, inertia, collision_geom);
    }

    // Update parameters and associated components
    parameters_ = RobotParameters<N>(links);
    fk_ = ForwardKinematics<N>(parameters_);
    ik_ = InverseKinematics<N>(parameters_);
    rne_ = RecursiveNewtonEuler<N>(parameters_);
    robot_model_ = RobotModel<N>(parameters_);
    // TrajectoryPlanner does not require re-initialization

    return true;
}

// Compute Forward Kinematics
template <std::size_t N>
MathLib::Matrix<4, 4> Robot<N>::computeForwardKinematics(const JointAngles<N>& joint_angles) const {
    return fk_.compute(joint_angles);
}

// Compute Jacobian
template <std::size_t N>
MathLib::Matrix<6, N> Robot<N>::computeJacobian(const JointAngles<N>& joint_angles) const {
    return fk_.computeJacobian(joint_angles);
}

// Compute Inverse Kinematics
template <std::size_t N>
std::optional<JointAngles<N>> Robot<N>::computeInverseKinematics(const MathLib::Matrix<4, 4>& desiredPose) const {
    return ik_.solve(desiredPose);
}

// Compute Dynamics
template <std::size_t N>
std::array<MathLib::Vector<3>, N> Robot<N>::computeDynamics(const JointAngles<N>& joint_angles,
                                                             const std::array<double, N>& joint_velocities,
                                                             const std::array<double, N>& joint_accelerations) const {
    return rne_.compute(joint_angles, joint_velocities, joint_accelerations);
}

// Detect Self-Collision
template <std::size_t N>
bool Robot<N>::detectSelfCollision(const JointAngles<N>& joint_angles) const {
    return robot_model_.detectSelfCollision(joint_angles);
}

// Plan Trajectory
template <std::size_t N>
std::optional<Trajectory<N>> Robot<N>::planTrajectory(const JointAngles<N>& start_angles,
                                               const JointAngles<N>& target_angles,
                                               double duration,
                                               std::size_t steps) const {
    // Generate the trajectory
    Trajectory<N> traj = planner_.generateCubicSplineTrajectory(start_angles.angles, target_angles.angles, duration, steps);

    // Optional: Validate the trajectory (e.g., collision-free)
    for (const auto& wp : traj.waypoints) {
        JointAngles<N> angles;
        angles.angles = wp.joint_angles;
        if (detectSelfCollision(angles)) {
            std::cerr << "Collision detected at time " << wp.time << "s during trajectory planning.\n";
            return std::nullopt; // Trajectory is invalid due to collision
        }
    }

    return traj;
}

// Execute Trajectory
template <std::size_t N>
void Robot<N>::executeTrajectory(const Trajectory<N>& traj) const {
    // Placeholder for actual execution logic
    // This could involve sending joint angle commands to actuators with appropriate timing
    for (const auto& wp : traj.waypoints) {
        // Example: Print joint angles at each waypoint
        std::cout << "Executing Trajectory at time " << wp.time << "s: " << wp.joint_angles << "\n";
        // Implement actual actuator command here
    }
}

} // namespace RoboticsLibrary

#endif // ROBOT_TPP
