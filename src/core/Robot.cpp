// Robot.cpp (Full Implementation)
#include "Robot.h"
#include <fstream>
#include <nlohmann/json.hpp>
#include <stdexcept>
#include <iostream> // For debugging (optional)

namespace RoboticsLibrary {

using json = nlohmann::json;

// Default Constructor: Initializes with default parameters
Robot::Robot() : fk(parameters), ik(parameters), rne(parameters) {
    initializeDefaultParameters();
}

// Parameterized Constructor: Initializes with external parameters
Robot::Robot(const RobotParameters<6>& params) : parameters(params), fk(parameters), ik(parameters), rne(parameters) {}

// Initializes with hardcoded default parameters
void Robot::initializeDefaultParameters() {
    // Define default joint parameters
    // For illustration, initializing with arbitrary values
    parameters.joints = {
        JointParameters(/*alpha=*/0.0, /*a=*/0.5, /*d=*/0.2, /*theta=*/0.0, /*mass=*/10.0,
                        MathLib::Vector<3>{0.0, 0.0, 0.1},
                        MathLib::Matrix<3, 3>{
                            {0.1, 0.0, 0.0},
                            {0.0, 0.1, 0.0},
                            {0.0, 0.0, 0.1}
                        }),
        // Initialize remaining joints...
        JointParameters(/*alpha=*/-1.5708, /*a=*/0.0, /*d=*/0.0, /*theta=*/0.0, /*mass=*/8.0,
                        MathLib::Vector<3>{0.0, 0.0, 0.1},
                        MathLib::Matrix<3, 3>{
                            {0.08, 0.0, 0.0},
                            {0.0, 0.08, 0.0},
                            {0.0, 0.0, 0.08}
                        }),
        // ... Add other joints as needed
    };
}

// Loads robot parameters from a JSON configuration file
void Robot::loadFromConfig(const std::string& configFilePath) {
    std::ifstream configFile(configFilePath);
    if (!configFile.is_open()) {
        throw std::runtime_error("Failed to open robot configuration file: " + configFilePath);
    }

    json config;
    try {
        configFile >> config;
    } catch (const json::parse_error& e) {
        throw std::runtime_error("JSON parsing error: " + std::string(e.what()));
    }

    // Validate JSON structure
    if (!config.contains("robot") || !config["robot"].contains("joints")) {
        throw std::invalid_argument("Invalid configuration file: Missing 'robot.joints' section.");
    }

    const auto& jointsConfig = config["robot"]["joints"];

    // Validate number of joints
    if (jointsConfig.size() != parameters.joints.size()) {
        throw std::invalid_argument("Configuration file joint count (" + std::to_string(jointsConfig.size()) +
                                    ") does not match expected count (" + std::to_string(parameters.joints.size()) + ").");
    }

    // Iterate and populate joint parameters
    for (std::size_t i = 0; i < jointsConfig.size(); ++i) {
        const auto& joint = jointsConfig[i];

        // Validate required fields
        std::vector<std::string> requiredFields = { "alpha", "a", "d", "theta", "mass", "com", "inertia" };
        for (const auto& field : requiredFields) {
            if (!joint.contains(field)) {
                throw std::invalid_argument("Joint " + std::to_string(i) + " is missing required field: '" + field + "'.");
            }
        }

        // Extract and assign values
        double alpha = joint["alpha"];
        double a = joint["a"];
        double d = joint["d"];
        double theta = joint["theta"];
        double mass = joint["mass"];

        // Extract center of mass
        const auto& comArray = joint["com"];
        if (!comArray.is_array() || comArray.size() != 3) {
            throw std::invalid_argument("Joint " + std::to_string(i) + " has invalid 'com' array size.");
        }
        MathLib::Vector<3> com = { comArray[0].get<double>(), comArray[1].get<double>(), comArray[2].get<double>() };

        // Extract inertia tensor
        const auto& inertiaArray = joint["inertia"];
        if (!inertiaArray.is_array() || inertiaArray.size() != 3) {
            throw std::invalid_argument("Joint " + std::to_string(i) + " has invalid 'inertia' array size.");
        }

        MathLib::Matrix<3, 3> inertia;
        for (std::size_t row = 0; row < 3; ++row) {
            const auto& rowArray = inertiaArray[row];
            if (!rowArray.is_array() || rowArray.size() != 3) {
                throw std::invalid_argument("Joint " + std::to_string(i) + " has invalid 'inertia' row size.");
            }
            for (std::size_t col = 0; col < 3; ++col) {
                inertia.data[row][col] = rowArray[col].get<double>();
            }
        }

        // Assign to parameters
        parameters.joints[i] = JointParameters(alpha, a, d, theta, mass, com, inertia);
    }

    // Reinitialize computational modules with updated parameters
    fk = ForwardKinematics(parameters);
    ik = InverseKinematics(parameters);
    rne = RecursiveNewtonEuler(parameters);
}

MathLib::Matrix<4, 4> Robot::computeForwardKinematics() {
    return fk.calculate();
}

MathLib::Matrix<6, 6> Robot::computeJacobian() {
    // Implement Jacobian computation within ForwardKinematics or a separate module
    // Placeholder
    MathLib::Matrix<6, 6> jacobian;
    // Compute and populate jacobian
    return jacobian;
}

RobotParameters<6> Robot::computeInverseKinematics(const MathLib::Matrix<4, 4>& desiredPose) {
    return ik.solve(desiredPose, parameters);
}

void Robot::computeDynamics() {
    rne.compute(parameters);
}

} // namespace RoboticsLibrary
