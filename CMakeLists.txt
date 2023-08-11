cmake_minimum_required(VERSION 3.12)
project(slrbs)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}/cmake")
find_package(OpenGL REQUIRED)

set(collision_HEADERS include/collision/CollisionDetect.h 
                      include/collision/Geometry.h )
set(collision_SOURCE src/collision/CollisionDetect.cpp ) 

set(rigidbody_HEADERS include/rigidbody/RigidBody.h
                      include/rigidbody/RigidBodySystem.h 
                      include/rigidbody/Scenarios.h )
set(rigidbody_SOURCE src/rigidbody/RigidBody.cpp 
                     src/rigidbody/RigidBodySystem.cpp )

set(joint_HEADERS include/joint/Joint.h
				  include/joint/Hinge.h
				  include/joint/Spherical.h )
set(joint_SOURCE src/joint/Joint.cpp
				 src/joint/Hinge.cpp
			     src/joint/Spherical.cpp )

set(contact_HEADERS include/contact/Contact.h )
set(contact_SOURCE src/contact/Contact.cpp )

set(solver_HEADERS include/solvers/Solver.h 
                   include/solvers/SolverBoxPGS.h )
set(solver_SOURCE src/solvers/SolverBoxPGS.cpp )

set(util_HEADERS include/util/OBJLoader.h
                 include/util/MeshAssets.h
				 include/util/Types.h )
set(util_SOURCE src/util/OBJLoader.cpp
                src/util/MeshAssets.cpp )

set(viewer_HEADERS include/viewer/SimViewer.h)
set(viewer_SOURCE src/viewer/SimViewer.cpp)
        
add_executable (slrbs main.cpp
          ${contact_HEADERS} ${contact_SOURCE} 
          ${joint_HEADERS} ${joint_SOURCE}
          ${collision_HEADERS} ${collision_SOURCE}
          ${rigidbody_HEADERS} ${rigidbody_SOURCE}
          ${solver_HEADERS} ${solver_SOURCE}
		  ${util_HEADERS} ${util_SOURCE}
          ${viewer_HEADERS} ${viewer_SOURCE} )

add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/3rdParty/polyscope)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include 3rdParty/Eigen3/include/eigen3 ${COMMON_INCLUDES})
target_link_libraries(slrbs OpenGL::GL polyscope)

# Setup source code groups
#
source_group(src FILES main.cpp)
source_group(src/collision FILES ${collision_SOURCE})
source_group(include/collision FILES ${collision_HEADERS})
source_group(src/joint FILES ${joint_SOURCE})
source_group(include/joint FILES ${joint_HEADERS})
source_group(src/contact FILES ${contact_SOURCE})
source_group(include/contact FILES ${contact_HEADERS})
source_group(src/rigidbody FILES ${rigidbody_SOURCE})
source_group(include/rigidbody FILES ${rigidbody_HEADERS})
source_group(src/solvers FILES ${solver_SOURCE})
source_group(include/solvers FILES ${solver_HEADERS})
source_group(src/util FILES ${util_SOURCE})
source_group(include/util FILES ${util_HEADERS})
source_group(src/viewer FILES ${viewer_SOURCE})
source_group(include/viewer FILES ${viewer_HEADERS})
source_group(src/util FILES ${util_SOURCE})

if(MSVC) 
	set_property(TARGET slrbs PROPERTY VS_DEBUGGER_WORKING_DIRECTORY ${CMAKE_SOURCE_DIR})
endif()