// Waypoint.h
#ifndef WAYPOINT_H
#define WAYPOINT_H

#include "../math/MathLibrary.h"
#include <array>

namespace RoboticsLibrary {

/**
 * @brief Structure representing a single waypoint in a trajectory.
 * 
 * @tparam N Number of joints in the robot.
 */
template <std::size_t N>
struct Waypoint {
    MathLib::Vector<N> joint_angles; ///< Joint angles in radians.
    double time;                     ///< Timestamp in seconds.

    /**
     * @brief Constructor for Waypoint.
     * 
     * @param angles Joint angles.
     * @param t Timestamp.
     */
    Waypoint(const MathLib::Vector<N>& angles_ = MathLib::Vector<N>(), double t_ = 0.0)
        : joint_angles(angles_), time(t_) {}
};

} // namespace RoboticsLibrary

#endif // WAYPOINT_H
