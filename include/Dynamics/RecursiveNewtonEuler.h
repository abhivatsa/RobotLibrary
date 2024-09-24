#ifndef RECURSIVE_NEWTON_EULER_H
#define RECURSIVE_NEWTON_EULER_H

#include "../Robot.h"
#include "../MathLibrary.h"
#include <array>
#include <stdexcept>

namespace motion_planning {

// Forward declaration of computeTransformationMat
template <std::size_t N>
class RecursiveNewtonEuler;

// Specialization if needed (not necessary here)

template <std::size_t N>
class RecursiveNewtonEuler : public Robot {
public:
    // Constructor: Initializes Robot base class and dynamic parameters
    RecursiveNewtonEuler(const std::string& name,
                         const std::vector<double>& jointParameters,
                         const std::array<double, N>& alpha_init,
                         const std::array<double, N>& a_init,
                         const std::array<double, N>& d_init,
                         const std::array<double, N>& theta_init,
                         const std::array<double, N>& mass_init,
                         const std::array<MathLib::Vector<3>, N>& pos_com_init,
                         const std::array<MathLib::Matrix<3, 3>, N>& inertia_com_init)
        : Robot(name, jointParameters),
          alpha(alpha_init),
          a(a_init),
          d(d_init),
          theta(theta_init),
          m(mass_init),
          pos_com(pos_com_init),
          inertia_com(inertia_com_init)
    {
        // Validate joint parameters
        if (jointParameters.size() != N) {
            throw std::invalid_argument("Joint parameters size must match number of joints.");
        }
    }

    // Destructor
    ~RecursiveNewtonEuler() override = default;

    /**
     * @brief Computes the torque for N joints using the Recursive Newton-Euler algorithm.
     * 
     * @param joint_pos Current joint positions (angles) in radians.
     * @param joint_vel Current joint velocities in radians per second.
     * @param joint_acc Current joint accelerations in radians per second squared.
     * @param joint_torque Output array to store computed joint torques.
     * @return int Status code (0 for success, non-zero for failure).
     */
    int computeTorque(const std::array<double, N>& joint_pos,
                      const std::array<double, N>& joint_vel,
                      const std::array<double, N>& joint_acc,
                      std::array<double, N>& joint_torque) const;

private:
    /**
     * @brief Computes the transformation matrix for a given joint.
     * 
     * @param joint_pos Joint position (angle) in radians.
     * @param joint_num Joint index.
     * @param rotation_mat Output rotation matrix (3x3).
     * @param pos_vec Output position vector.
     * @return int Status code (0 for success, non-zero for failure).
     */
    int computeTransformationMat(double joint_pos, int joint_num,
                                 MathLib::Matrix<3, 3>& rotation_mat,
                                 MathLib::Vector<3>& pos_vec) const;

    // Dynamic parameters
    std::array<double, N> alpha;        // Link twist angles
    std::array<double, N> a;            // Link lengths
    std::array<double, N> d;            // Link offsets
    std::array<double, N> theta;        // Joint angles
    std::array<double, N> m;            // Masses

    std::array<MathLib::Vector<3>, N> pos_com;          // Center of mass positions
    std::array<MathLib::Matrix<3, 3>, N> inertia_com;   // Inertia tensors

    // Force and torque arrays for the algorithm (fixed size for real-time compliance)
    std::array<MathLib::Vector<3>, N + 1> force_com;   // Forces
    std::array<MathLib::Vector<3>, N + 1> torque_com;  // Torques
};

// Implementation of RecursiveNewtonEuler

template <std::size_t N>
int RecursiveNewtonEuler<N>::computeTorque(const std::array<double, N>& joint_pos,
                                          const std::array<double, N>& joint_vel,
                                          const std::array<double, N>& joint_acc,
                                          std::array<double, N>& joint_torque) const
{
    MathLib::Matrix<3, 3> rotation_mat;
    MathLib::Vector<3> pos_vec;
    MathLib::Vector<3> z_dir = {0.0, 0.0, 1.0};

    joint_torque.fill(0.0);          // Initialize all joint torques to 0
    force_com.fill(MathLib::Vector<3>());  // Initialize forces to zero
    torque_com.fill(MathLib::Vector<3>()); // Initialize torques to zero

    MathLib::Vector<3> omega = {0.0, 0.0, 0.0};
    MathLib::Vector<3> omega_dot = {0.0, 0.0, 0.0};
    MathLib::Vector<3> vel_dot = {0.0, 0.0, -9.81};  // Gravity in m/s²

    // Forward recursion: Compute velocities and accelerations
    for (std::size_t joint_ctr = 0; joint_ctr < N; ++joint_ctr) {
        int solve_chk = computeTransformationMat(joint_pos[joint_ctr], joint_ctr, rotation_mat, pos_vec);
        if (solve_chk != 0) {
            return solve_chk;  // Propagate error
        }

        // Update angular velocity and acceleration
        MathLib::Vector<3> omega_next = rotation_mat * omega + joint_vel[joint_ctr] * z_dir;
        MathLib::Vector<3> omega_dot_next = rotation_mat * omega_dot + rotation_mat * (omega * (joint_vel[joint_ctr] * z_dir)) + joint_acc[joint_ctr] * z_dir;

        // Update linear acceleration
        MathLib::Vector<3> vel_dot_next = rotation_mat * (omega_dot * pos_vec + omega * (omega * pos_vec)) + vel_dot;

        // Compute acceleration of the center of mass
        MathLib::Vector<3> vel_dot_com = omega_dot_next * pos_com[joint_ctr] + omega_next * (omega_next * pos_com[joint_ctr]) + vel_dot_next;

        // Compute force and torque at the center of mass
        force_com[joint_ctr + 1] = MathLib::Vector<3>(m[joint_ctr] * vel_dot_com);
        torque_com[joint_ctr + 1] = inertia_com[joint_ctr] * omega_dot_next + omega_next * (inertia_com[joint_ctr] * omega_next);

        // Update state for next joint
        omega = omega_next;
        omega_dot = omega_dot_next;
        vel_dot = vel_dot_next;
    }

    // Initialize previous force and torque for backward recursion
    MathLib::Vector<3> force_prev = {0.0, 0.0, 0.0};
    MathLib::Vector<3> torque_prev = {0.0, 0.0, 0.0};

    // Backward recursion: Compute forces and torques
    for (std::size_t joint_ctr = N; joint_ctr > 0; --joint_ctr) {
        int solve_chk = computeTransformationMat(joint_pos[joint_ctr - 1], joint_ctr - 1, rotation_mat, pos_vec);
        if (solve_chk != 0) {
            return solve_chk;  // Propagate error
        }

        // Compute force and torque
        MathLib::Vector<3> force = rotation_mat.transpose() * force_prev + force_com[joint_ctr];
        MathLib::Vector<3> torque = torque_com[joint_ctr] + rotation_mat.transpose() * torque_prev +
                                     pos_com[joint_ctr - 1] * force_com[joint_ctr] +
                                     pos_vec * (rotation_mat * force_prev);

        // Update previous force and torque for the next iteration
        force_prev = force;
        torque_prev = torque;

        // Extract Z-axis torque component as the joint torque
        joint_torque[joint_ctr - 1] = torque.data[2]; // Assuming joint rotation about Z-axis
    }

    return 0; // Success
}

template <std::size_t N>
int RecursiveNewtonEuler<N>::computeTransformationMat(double joint_pos, int joint_num,
                                                      MathLib::Matrix<3, 3>& rotation_mat,
                                                      MathLib::Vector<3>& pos_vec) const
{
    double total_theta = joint_pos + theta[joint_num];
    double c = std::cos(total_theta);
    double s = std::sin(total_theta);
    double ca = std::cos(alpha[joint_num]);
    double sa = std::sin(alpha[joint_num]);

    // Define rotation matrix using standard DH parameters
    MathLib::Matrix<3, 3> rot({
        std::array<double, 3>{c, -s * ca, s * sa},
        std::array<double, 3>{s, c * ca, -c * sa},
        std::array<double, 3>{0.0, sa, ca}
    });

    rotation_mat = rot;

    // Define position vector using standard DH parameters
    MathLib::Vector<3> pos({
        a[joint_num],
        -sa * d[joint_num],
        ca * d[joint_num]
    });

    pos_vec = pos;

    return 0; // Success
}

} // namespace motion_planning

#endif // RECURSIVE_NEWTON_EULER_H

