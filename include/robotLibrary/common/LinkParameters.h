// LinkParameters.h
#ifndef LINK_PARAMETERS_H
#define LINK_PARAMETERS_H

#include "../math/MathLibrary.h"
#include <array>
#include <cstddef>      // For std::size_t
#include <stdexcept>    // For exceptions

namespace RoboticsLibrary {

/**
 * @brief Struct containing cylindrical collision geometry parameters for a link.
 */
struct CollisionGeometry {
    double radius;    ///< Radius of the cylinder in meters.
    double height;    ///< Height of the cylinder in meters.
    
    /**
     * @brief Constructor for CollisionGeometry.
     * 
     * @param r Radius of the cylinder.
     * @param h Height of the cylinder.
     */
    CollisionGeometry(double r = 0.0, double h = 0.0)
        : radius(r), height(h) {
        if (radius < 0.0 || height < 0.0) {
            throw std::invalid_argument("Radius and height must be non-negative.");
        }
    }
};

/**
 * @brief Struct representing the parameters of a single robot link.
 */
struct LinkParameters {
    MathLib::Vector<3> position;           ///< Position vector (x, y, z) in meters.
    MathLib::Vector<3> orientation;        ///< Orientation vector (roll, pitch, yaw) in radians.
    double mass;                           ///< Mass of the link in kilograms.
    MathLib::Vector<3> com;                ///< Center of mass vector (x, y, z) in meters.
    MathLib::Matrix<3, 3> inertia_tensor; ///< Inertia tensor matrix (kg·m²).
    CollisionGeometry collision_geometry;  ///< Cylindrical collision geometry parameters.
    
    /**
     * @brief Constructor for LinkParameters.
     * 
     * @param pos Position vector.
     * @param ori Orientation vector.
     * @param m Mass of the link.
     * @param com_ Center of mass vector.
     * @param inertia Inertia tensor matrix.
     * @param collision_geom Cylindrical collision geometry parameters.
     */
    LinkParameters(const MathLib::Vector<3>& pos,
                  const MathLib::Vector<3>& ori,
                  double m,
                  const MathLib::Vector<3>& com_,
                  const MathLib::Matrix<3, 3>& inertia,
                  const CollisionGeometry& collision_geom)
        : position(pos),
          orientation(ori),
          mass(m),
          com(com_),
          inertia_tensor(inertia),
          collision_geometry(collision_geom) {}
};

} // namespace RoboticsLibrary

#endif // LINK_PARAMETERS_H
