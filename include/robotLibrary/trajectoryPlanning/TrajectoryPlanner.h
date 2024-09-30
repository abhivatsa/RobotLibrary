// TrajectoryPlanner.h
#ifndef TRAJECTORY_PLANNER_H
#define TRAJECTORY_PLANNER_H

#include "Trajectory.h"
#include "../common/RobotParameters.h" // Include for JointAngles<N>
#include <vector>
#include <algorithm>
#include <cmath>

namespace RoboticsLibrary {

/**
 * @brief Class responsible for generating trajectories.
 * 
 * @tparam N Number of joints in the robot.
 */
template <std::size_t N>
class TrajectoryPlanner {
public:
    /**
     * @brief Constructs a TrajectoryPlanner instance.
     */
    TrajectoryPlanner() = default;

    /**
     * @brief Generates a trajectory using cubic spline interpolation between start and end waypoints.
     * 
     * @param start The starting joint angles as an array.
     * @param end The ending joint angles as an array.
     * @param duration Duration of the trajectory in seconds.
     * @param steps Number of interpolation steps.
     * @return Trajectory<N> The generated trajectory.
     */
    Trajectory<N> generateCubicSplineTrajectory(const std::array<double, N>& start,
                                                const std::array<double, N>& end,
                                                double duration,
                                                std::size_t steps) const;

private:
    /**
     * @brief Cubic interpolation function.
     * 
     * @param t Current time.
     * @param t0 Start time.
     * @param t1 End time.
     * @param p0 Start position.
     * @param p1 End position.
     * @param v0 Start velocity.
     * @param v1 End velocity.
     * @return double Interpolated position.
     */
    double cubicInterpolate(double t, double t0, double t1, double p0, double p1, double v0, double v1) const;
};

} // namespace RoboticsLibrary

#include "TrajectoryPlanner.tpp"

#endif // TRAJECTORY_PLANNER_H
