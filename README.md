**Sheldon's Little Rigid-Body Simulator (SLRBS)**

A simple, lightweight rigid body simulation engine. 

I developed this rigid body simulator primarily for the purposes of teaching.  It has been used during my course work at the [École de technologie supérieure](https://www.etsmtl.ca/) for our [SIGGRAPH course](https://siggraphcontact.github.io/) on simulating contact. 

Please feel free to use it for your own projects.


**Getting started**

The simulator can be configured and compiled using CMake. The following steps should work on Windows with Visual Studio 2022 installed:

`mkdir build/`

`cd build/`

`cmake ..\CMakeLists.txt -G "Visual Studio 17 2022"`

`cmake --build .`

**Creating scenarios**

The file *include/rigidbody/Scenarios.h* shows some examples of how to create scenes using the various collision geometries and joint types.


**Third party dependencies**

SLRBS depends on Eigen (https://eigen.tuxfamily.org/) and Polyscope (https://polyscope.run/). Source code for both libraries is provided in the *3rdParty* folder

Eigen 3.x uses the [Mozilla Public License v2](3rdParty/Eigen3/include/eigen3/Eigen/Core).

Polyscope uses the [MIT License](3rdParty/polyscope/LICENSE).

