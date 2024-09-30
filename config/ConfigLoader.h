// ConfigLoader.h
#ifndef CONFIG_LOADER_H
#define CONFIG_LOADER_H

#include <nlohmann/json.hpp>
#include <fstream>
#include <string>
#include <iostream>

namespace RoboticsLibrary {
namespace config {

class ConfigLoader {
public:
    ConfigLoader() = default;

    /**
     * @brief Loads and parses the config.json file.
     * 
     * @param filepath Path to the config.json file.
     * @return nlohmann::json Parsed JSON object.
     */
    nlohmann::json loadConfig(const std::string& filepath) const {
        std::ifstream config_file(filepath);
        if (!config_file.is_open()) {
            throw std::runtime_error("Failed to open config.json at: " + filepath);
        }

        nlohmann::json config;
        config_file >> config;

        return config;
    }
};

} // namespace config
} // namespace RoboticsLibrary

#endif // CONFIG_LOADER_H
