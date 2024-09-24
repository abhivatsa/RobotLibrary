#ifndef ROBOT_H
#define ROBOT_H

#include "MathLibrary.h"
#include <vector>
#include <string>

namespace motion_planning {

class Robot {
public:
    Robot(const std::string& name, const std::vector<double>& jointParameters)
        : name(name), jointParameters(jointParameters) {}

    virtual ~Robot() = default;

    // Accessors
    std::string getName() const { return name; }
    std::vector<double> getJointParameters() const { return jointParameters; }

    // Potentially other common methods

protected:
    std::string name;
    std::vector<double> jointParameters;
    // Other shared attributes
};

} // namespace motion_planning

#endif // ROBOT_H

