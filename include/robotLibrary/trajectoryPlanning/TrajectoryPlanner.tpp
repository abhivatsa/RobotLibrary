// TrajectoryPlanner.tpp
#ifndef TRAJECTORY_PLANNER_TPP
#define TRAJECTORY_PLANNER_TPP

#include "TrajectoryPlanner.h"

namespace RoboticsLibrary {

template <std::size_t N>
Trajectory<N> TrajectoryPlanner<N>::generateCubicSplineTrajectory(const std::array<double, N>& start,
                                                                   const std::array<double, N>& end,
                                                                   double duration,
                                                                   std::size_t steps) const {
    Trajectory<N> traj;
    traj.clear();

    double dt = duration / static_cast<double>(steps - 1);

    // Assume zero initial and final velocities for simplicity
    std::array<double, N> v0 = {}; // Initialized to zero
    std::array<double, N> v1 = {}; // Initialized to zero

    for (std::size_t step = 0; step < steps; ++step) {
        double current_time = step * dt;
        MathLib::Vector<N> current_angles;

        for (std::size_t joint = 0; joint < N; ++joint) {
            current_angles.data[joint] = cubicInterpolate(current_time, 0.0, duration,
                                                         start[joint],
                                                         end[joint],
                                                         v0[joint],
                                                         v1[joint]);
        }

        traj.addWaypoint(Waypoint<N>(current_angles, current_time));
    }

    return traj;
}

template <std::size_t N>
double TrajectoryPlanner<N>::cubicInterpolate(double t, double t0, double t1, double p0, double p1, double v0, double v1) const {
    double tau = (t - t0) / (t1 - t0); // Normalize time to [0,1]

    // Cubic Hermite spline basis functions
    double h00 = 2 * std::pow(tau, 3) - 3 * std::pow(tau, 2) + 1;
    double h10 = std::pow(tau, 3) - 2 * std::pow(tau, 2) + tau;
    double h01 = -2 * std::pow(tau, 3) + 3 * std::pow(tau, 2);
    double h11 = std::pow(tau, 3) - std::pow(tau, 2);

    double result = h00 * p0 + h10 * (t1 - t0) * v0 + h01 * p1 + h11 * (t1 - t0) * v1;
    return result;
}

} // namespace RoboticsLibrary

#endif // TRAJECTORY_PLANNER_TPP
