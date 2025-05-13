#pragma once

#include <Eigen/Dense>

namespace Eigen
{
    // Vector types    
    typedef Eigen::Matrix<float, 6, 1> Vector6f;
	
    // Additional common types for physics simulation
    typedef Eigen::Matrix<float, Dynamic, Dynamic> MatrixXf;
    typedef Eigen::Matrix<float, Dynamic, 1> VectorXf;
    typedef Eigen::Matrix<float, 3, 3> Matrix3f;
    typedef Eigen::Matrix<float, 4, 4> Matrix4f;
    typedef Eigen::Matrix<float, Dynamic, 3> MatrixX3f;
    typedef Eigen::Matrix<float, 3, Dynamic> Matrix3Xf;
}

typedef Eigen::Matrix<float, Eigen::Dynamic, 6, Eigen::RowMajor> JBlock;
typedef Eigen::Matrix<float, 6, Eigen::Dynamic, Eigen::ColMajor> JBlockTranspose;
typedef Eigen::Matrix<float, 6, 6> GBlock;