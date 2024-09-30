// RobotModel.h
#ifndef ROBOT_MODEL_H
#define ROBOT_MODEL_H

#include "../common/RobotParameters.h"
#include "../math/MathLibrary.h"
#include <array>
#include <cstddef>      // For std::size_t
#include <cmath>        // For std::sqrt
#include <stdexcept>    // For exceptions

namespace RoboticsLibrary {

/**
 * @brief Template class for detecting collisions in an N-link robot.
 * 
 * The RobotModel class handles collision detection for a robot with N links.
 * It checks for self-collision by comparing the cylindrical collision geometries
 * of the links based on their parameters.
 * 
 * @tparam N Number of links in the robot.
 */
template <std::size_t N>
class RobotModel {
public:
    /**
     * @brief Constructs a RobotModel instance with the provided robot parameters.
     * 
     * @param robotParams A reference to the robot's parameters, including joint configurations.
     */
    explicit RobotModel(const RobotParameters<N>& robotParams);

    /**
     * @brief Detects self-collision between the robot links based on current joint angles.
     * 
     * @param joint_angles The current joint angles of the robot.
     * @return True if a self-collision is detected, false otherwise.
     */
    bool detectSelfCollision(const JointAngles<N>& joint_angles) const;

private:
    RobotParameters<N> robotParams_; ///< Robot parameters containing joint configurations.

    /**
     * @brief Computes the homogeneous transformation matrix for each link using position and orientation.
     * 
     * @param position Position vector (x, y, z) of the link in meters.
     * @param orientation Orientation vector (roll, pitch, yaw) in radians.
     * @return MathLib::Matrix<4,4> The homogeneous transformation matrix.
     */
    MathLib::Matrix<4, 4> computeTransformationMatrix(const MathLib::Vector<3>& position,
                                                      const MathLib::Vector<3>& orientation) const;

    /**
     * @brief Computes the global positions of all link centers of mass based on joint angles.
     * 
     * @param joint_angles The current joint angles of the robot.
     * @return std::array<MathLib::Vector<3>, N> Global positions of each link's center of mass.
     */
    std::array<MathLib::Vector<3>, N> computeGlobalCoMs(const JointAngles<N>& joint_angles) const;

    /**
     * @brief Checks for collision between two cylindrical links.
     * 
     * @param com1 Global center of mass position of the first link.
     * @param com2 Global center of mass position of the second link.
     * @param radius1 Collision radius of the first link.
     * @param radius2 Collision radius of the second link.
     * @return True if the two links collide, false otherwise.
     */
    bool checkLinkCollision(const MathLib::Vector<3>& com1, const MathLib::Vector<3>& com2,
                            double radius1, double radius2) const;
};

} // namespace RoboticsLibrary

#include "RobotModel.tpp"

#endif // ROBOT_MODEL_H
