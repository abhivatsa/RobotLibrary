// Trajectory.h
#ifndef TRAJECTORY_H
#define TRAJECTORY_H

#include "Waypoint.h"
#include <vector>
#include <ostream> // Added to support std::ostream in operator<<

namespace RoboticsLibrary {

/**
 * @brief Structure representing a trajectory composed of multiple waypoints.
 * 
 * @tparam N Number of joints in the robot.
 */
template <std::size_t N>
struct Trajectory {
    std::vector<Waypoint<N>> waypoints; ///< Sequence of waypoints.

    /**
     * @brief Adds a waypoint to the trajectory.
     * 
     * @param wp The waypoint to add.
     */
    void addWaypoint(const Waypoint<N>& wp) {
        waypoints.push_back(wp);
    }

    /**
     * @brief Clears all waypoints from the trajectory.
     */
    void clear() {
        waypoints.clear();
    }

    /**
     * @brief Retrieves the number of waypoints in the trajectory.
     * 
     * @return std::size_t Number of waypoints.
     */
    std::size_t size() const {
        return waypoints.size();
    }

    /**
     * @brief Overload output operator for printing the trajectory.
     */
    friend std::ostream& operator<<(std::ostream& os, const Trajectory<N>& traj) {
        os << "Trajectory:\n";
        for (const auto& wp : traj.waypoints) {
            os << "  Time " << wp.time << "s: " << wp.joint_angles << "\n";
        }
        return os;
    }
};

} // namespace RoboticsLibrary

#endif // TRAJECTORY_H
