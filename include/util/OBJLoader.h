#pragma once

/**
 * @file OBJLoader.h
 *
 * @brief File loading for OBJ meshes.
 *
 */

#include <Eigen/Dense>

// Class responsible for loading all the meshes included in an OBJ file
class OBJLoader
{
public:

    static bool load(const std::string& filename, Eigen::MatrixXf& meshV, Eigen::MatrixXi& meshF);
    
};
