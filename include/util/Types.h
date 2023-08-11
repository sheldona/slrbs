#pragma once

#include <Eigen/Dense>

// Define some important matrix, vector types.
// This file can also be used to override default scalar types
//   e.g. for autodiff computations or changing fp precision
//
namespace Eigen
{
	// Vector types	
	typedef Eigen::Matrix< float, 6, 1 > Vector6f;
}

typedef Eigen::Matrix<float, Eigen::Dynamic, 6, Eigen::RowMajor> JBlock;
typedef Eigen::Matrix<float, 6, Eigen::Dynamic, Eigen::ColMajor> JBlockTranspose;
