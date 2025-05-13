#pragma once

#include "rigidbody/RigidBodySystem.h"
#include <string>
#include <vector>

/**
 * @file ScenarioLoader.h
 *
 * @brief Utility for loading scenarios from JSON files.
 */

class ScenarioLoader {
public:
    /**
     * Load a scenario from a JSON file into the rigid body system
     * 
     * @param system The rigid body system to load the scenario into
     * @param filename The JSON file to load
     * @return true if loading was successful, false otherwise
     */
    static bool loadFromFile(RigidBodySystem& system, const std::string& filename);
    
    /**
     * List all available JSON scenario files in the scenarios directory
     * 
     * @return Vector of filenames (without path)
     */
    static std::vector<std::string> listAvailableScenarios();
    
private:
    /**
     * Get path to scenarios directory
     * 
     * @return Path to scenarios directory
     */
    static std::string getScenariosPath();
    
    /**
     * Parse JSON and apply to rigid body system
     * 
     * @param system The rigid body system to load the scenario into
     * @param jsonContent The JSON content as a string
     * @return true if parsing was successful, false otherwise
     */
    static bool parseScenario(RigidBodySystem& system, const std::string& jsonContent);
};