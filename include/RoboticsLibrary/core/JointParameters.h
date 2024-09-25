// JointParameters.h
#ifndef JOINT_PARAMETERS_H
#define JOINT_PARAMETERS_H

#include "MathLibrary.h"
#include <array>
#include <cstddef> // For std::size_t

namespace RoboticsLibrary {

/**
 * @brief Structure to hold all parameters for a single joint.
 */
struct JointParameters {
    // DH Parameters
    double alpha; ///< Twist angle (radians)
    double a;     ///< Link length
    double d;     ///< Link offset
    double theta; ///< Joint angle (can be variable for revolute joints)

    // Dynamic Parameters
    double mass;                                  ///< Mass of the link
    MathLib::Vector<3> com;                       ///< Center of mass (in link frame)
    MathLib::Matrix<3, 3> inertia;               ///< Inertia tensor (in link frame)

    /**
     * @brief Constructor for JointParameters.
     * 
     * @param alpha_ Twist angle.
     * @param a_ Link length.
     * @param d_ Link offset.
     * @param theta_ Joint angle.
     * @param mass_ Mass of the link.
     * @param com_ Center of mass vector.
     * @param inertia_ Inertia tensor matrix.
     */
    JointParameters(double alpha_, double a_, double d_, double theta_,
                   double mass_,
                   const MathLib::Vector<3>& com_,
                   const MathLib::Matrix<3, 3>& inertia_)
        : alpha(alpha_), a(a_), d(d_), theta(theta_),
          mass(mass_), com(com_), inertia(inertia_) {}
};

} // namespace RoboticsLibrary

#endif // JOINT_PARAMETERS_H
