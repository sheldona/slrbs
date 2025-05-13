cmake_minimum_required(VERSION 3.23)
project(slrbs
        VERSION 0.1.0
        DESCRIPTION "SLRBS"
        HOMEPAGE_URL "https://github.com/ETSim/slrbs"
        LANGUAGES CXX)

# ─────────────────────────────────────────────────────────────────────────────
#  User-configurable options
# ─────────────────────────────────────────────────────────────────────────────
option(BUILD_TESTING        "Enable testing"                          ON)

# ---- viewer selection ------------------------------------------------
option(USE_QT               "Enable Qt support"                       OFF)
option(USE_POLYSCOPE        "Enable Polyscope-based viewer"           ON)

# ---- optional libs ---------------------------------------------------
option(USE_SYSTEM_QT        "Use system Qt5/6 installation"           OFF)
option(USE_OPENCV           "Enable OpenCV support"                   OFF)
option(USE_TORCH            "Enable LibTorch support"                 OFF)
option(USE_OPENMP           "Enable OpenMP"                           ON)
option(USE_OPENMP_SIMD      "Enable OpenMP SIMD directives"           ON)
option(USE_SIMD_INTRINSICS  "Enable low‑level SIMD intrinsics"        ON)
option(USE_PYBIND           "Enable Python bindings via pybind11"     OFF)
option(USE_CCACHE           "Enable C/C++ compiler caching with ccache" ON)
option(USE_TCNN             "Enable tiny‑cuda‑nn neural network support" OFF)

# ─────────────────────────────────────────────────────────────────────────────
#  Global options
# ─────────────────────────────────────────────────────────────────────────────
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(POLICY CMP0169)
    cmake_policy(SET CMP0169 NEW)  # Use the new behavior for FetchContent
endif()

# Build options (defaults to Release build)
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()
set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS Debug Release RelWithDebInfo MinSizeRel)

# Enable testing
if(BUILD_TESTING)
    enable_testing()
    include(CTest)
endif()

# Check for MinGW compiler on Windows
if(MINGW)
    message(STATUS "MinGW compiler detected")
endif()

# Friendly warnings
if(MSVC)
    add_compile_options(/permissive- /W4 /Zc:preprocessor /MP)
    # Disable some MSVC warnings
    add_compile_options(/wd4267 /wd4244)
else()
    add_compile_options(-Wall -Wextra -pedantic -Wshadow -Wconversion)
    # Enable faster builds with ccache if available
    if(USE_CCACHE)
        find_program(CCACHE_PROGRAM ccache)
        if(CCACHE_PROGRAM)
            set(CMAKE_CXX_COMPILER_LAUNCHER "${CCACHE_PROGRAM}")
        endif()
    endif()
endif()

# Silence the "OpenGL deprecated" spam on recent macOS
if(APPLE)
    add_compile_definitions(GL_SILENCE_DEPRECATION)
endif()

# ─────────────────────────────────────────────────────────────────────────────
#  OpenGL — one target name for every platform
# ─────────────────────────────────────────────────────────────────────────────
find_package(OpenGL REQUIRED)

if(TARGET OpenGL::GL)             # Linux / most Windows toolchains
    set(OPENGL_LIB OpenGL::GL)
elseif(TARGET OpenGL::OpenGL)     # macOS framework
    set(OPENGL_LIB OpenGL::OpenGL)
elseif(WIN32)                     # really old CMake on Windows
    set(OPENGL_LIB opengl32)
else()
    message(FATAL_ERROR "Could not locate a usable OpenGL target")
endif()

# ─────────────────────────────────────────────────────────────────────────────
#  Qt (Qt 6 preferred, Qt 5 fallback)
# ─────────────────────────────────────────────────────────────────────────────
if(USE_QT)
    if(USE_SYSTEM_QT)
        # Set QT_MINGW flag for MinGW specific handling
        if(MINGW)
            set(QT_MINGW TRUE)
            # Set Qt prefix path if using MinGW
            # You might need to adjust this path to where your MinGW Qt is installed
            if(NOT DEFINED CMAKE_PREFIX_PATH)
                # Try to find Qt in common MinGW installation locations
                if(EXISTS "C:/Qt")
                    file(GLOB QT_VERSIONS "C:/Qt/*")
                    foreach(QT_VERSION ${QT_VERSIONS})
                        if(IS_DIRECTORY "${QT_VERSION}/mingw*")
                            file(GLOB QT_MINGW_DIRS "${QT_VERSION}/mingw*")
                            list(GET QT_MINGW_DIRS 0 QT_MINGW_DIR)
                            if(EXISTS "${QT_MINGW_DIR}")
                                message(STATUS "Using Qt from: ${QT_MINGW_DIR}")
                                set(CMAKE_PREFIX_PATH "${QT_MINGW_DIR}")
                                break()
                            endif()
                        endif()
                    endforeach()
                endif()
            endif()
        endif()

        # Qt6 first
        find_package(Qt6 COMPONENTS Core Widgets Gui OpenGLWidgets QUIET)
        if(Qt6_FOUND)
            set(QT_LIBS Qt6::Core Qt6::Widgets Qt6::Gui Qt6::OpenGLWidgets)
        else()
            # Qt5 fallback
            find_package(Qt5 COMPONENTS Core Widgets Gui QUIET)
            find_package(Qt5 COMPONENTS OpenGL QUIET)       # optional

            if(Qt5_FOUND)
                if(TARGET Qt5::OpenGLWidgets)
                    list(APPEND QT_LIBS Qt5::OpenGLWidgets)
                elseif(TARGET Qt5::OpenGL)
                    list(APPEND QT_LIBS Qt5::OpenGL)
                endif()
                list(APPEND QT_LIBS Qt5::Core Qt5::Widgets Qt5::Gui)
            else()
                # Check if we're using MinGW with incompatible Qt
                if(MINGW AND EXISTS "C:/Users/AP07080/AppData/Local/miniconda3/Library/lib/Qt5Core_conda.lib")
                    message(WARNING "Found Qt libraries in Anaconda, but they are incompatible with MinGW")
                    message(WARNING "Qt libraries in Anaconda are built with MSVC, not MinGW")
                    message(WARNING "Disabling Qt support - please install MinGW-compatible Qt")
                    set(USE_QT OFF CACHE BOOL "Enable Qt support" FORCE)
                else()
                    message(WARNING "Qt5 not found. Disabling Qt support.")
                    set(USE_QT OFF CACHE BOOL "Enable Qt support" FORCE)
                endif()
            endif()
        endif()
    else()
        message(WARNING "USE_SYSTEM_QT is OFF; please install Qt manually.")
        set(USE_QT OFF CACHE BOOL "Enable Qt support" FORCE)
    endif()

    if(USE_QT)
        add_compile_definitions(USE_QT)
    endif()
endif()

# ─────────────────────────────────────────────────────────────────────────────
#  Optional libs (OpenCV, LibTorch)
# ─────────────────────────────────────────────────────────────────────────────
if(USE_OPENCV)
    find_package(OpenCV QUIET)
    if(OpenCV_FOUND)
        add_compile_definitions(USE_OPENCV)
        set(OPENCV_LIBS ${OpenCV_LIBS})
    else()
        message(WARNING "OpenCV requested but not found!")
        set(USE_OPENCV OFF CACHE BOOL "Enable OpenCV support" FORCE)
    endif()
endif()

if(USE_TORCH)
    find_package(Torch QUIET)
    if(Torch_FOUND)
        add_compile_definitions(USE_TORCH)
        set(TORCH_LIBS ${Torch_LIBRARIES})
    else()
        message(WARNING "LibTorch requested but not found!")
        set(USE_TORCH OFF CACHE BOOL "Enable LibTorch support" FORCE)
    endif()
endif()

# ─────────────────────────────────────────────────────────────────────────────
#  OpenMP support
# ─────────────────────────────────────────────────────────────────────────────
if(USE_OPENMP)
    find_package(OpenMP)
    if(OpenMP_CXX_FOUND)
        message(STATUS "Found OpenMP: ${OpenMP_CXX_LIBRARIES}")
        set(OPENMP_LIBS OpenMP::OpenMP_CXX)

        # MinGW-specific OpenMP fixes
        if(MINGW)
            # Add explicit linking to libgomp for MinGW
            set(OPENMP_LIBS ${OPENMP_LIBS} -lgomp)
        endif()

        add_compile_definitions(USE_OPENMP)
        if(USE_OPENMP_SIMD)
            add_compile_definitions(USE_OPENMP_SIMD)
        endif()
    else()
        message(WARNING "OpenMP requested but not found!")
        set(USE_OPENMP OFF CACHE BOOL "Enable OpenMP" FORCE)
        set(USE_OPENMP_SIMD OFF CACHE BOOL "Enable OpenMP SIMD directives" FORCE)
        set(OPENMP_LIBS "")
    endif()
endif()

# ─────────────────────────────────────────────────────────────────────────────
#  Third‑party libraries (fully reproducible with FetchContent)
# ─────────────────────────────────────────────────────────────────────────────
include(FetchContent)

# Polyscope (visualisation)
FetchContent_Declare(
        polyscope
        GIT_REPOSITORY https://github.com/nmwsharp/polyscope.git
        GIT_BRANCH master
        GIT_PROGRESS   YES
)

# Eigen (linear algebra)
FetchContent_Declare(
        eigen
        GIT_REPOSITORY https://gitlab.com/libeigen/eigen.git
        GIT_TAG        3.4
        GIT_SHALLOW    TRUE
        SOURCE_SUBDIR  cmake
)

# FetchContent_MakeAvailable for base libraries
FetchContent_MakeAvailable(polyscope eigen)

# Note: We'll use GLM from Polyscope instead of fetching it again to avoid conflicts

# pybind11 (only when requested)
if(USE_PYBIND)
    FetchContent_Declare(
            pybind11
            GIT_REPOSITORY https://github.com/pybind/pybind11.git
            GIT_TAG        v2.11.1
    )
    FetchContent_MakeAvailable(pybind11)
endif()

# tiny-cuda-nn (only when requested) - needs CUDA
if(USE_TCNN)
    enable_language(CUDA)
    set(CMAKE_CUDA_STANDARD 14)
    FetchContent_Declare(
            tiny_cuda_nn
            GIT_REPOSITORY https://github.com/NVlabs/tiny-cuda-nn.git
            GIT_TAG        v1.7
    )
    FetchContent_MakeAvailable(tiny_cuda_nn)
endif()

# ─────────────────────────────────────────────────────────────────────────────
#  Source files
# ─────────────────────────────────────────────────────────────────────────────
file(GLOB_RECURSE SRC_FILES CONFIGURE_DEPENDS
        src/*.cpp
        include/*.h)

# Filter out python bindings from main executable ALWAYS (not just when USE_PYBIND is OFF)
list(FILTER SRC_FILES EXCLUDE REGEX ".*python_bindings\\.cpp$")
list(APPEND SRC_FILES "${CMAKE_CURRENT_SOURCE_DIR}/main.cpp")

# ─────────────────────────────────────────────────────────────────────────────
#  Conditionally disable Qt viewer if not available
# ─────────────────────────────────────────────────────────────────────────────
if(NOT USE_QT)
    message(STATUS "Qt support disabled - filtering out Qt-related source files")
    list(FILTER SRC_FILES EXCLUDE REGEX ".*QtSimWindow\\.cpp$")
endif()

# ─────────────────────────────────────────────────────────────────────────────
#  Main executable target
# ─────────────────────────────────────────────────────────────────────────────
add_executable(slrbs ${SRC_FILES})

# Use GLM from Polyscope to avoid conflicts
target_include_directories(slrbs PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/include
        ${eigen_SOURCE_DIR}
        ${polyscope_SOURCE_DIR}/deps/json/include
        ${polyscope_SOURCE_DIR}/deps/glm
)

# If we need tiny-cuda-nn
if(USE_TCNN)
    target_include_directories(slrbs PRIVATE ${tiny_cuda_nn_SOURCE_DIR}/include)
    target_compile_definitions(slrbs PRIVATE USE_TCNN)
endif()

# Add OpenMP compile options if found
if(OpenMP_CXX_FOUND)
    target_compile_options(slrbs PRIVATE ${OpenMP_CXX_FLAGS})
endif()

# ─────────────────────────────────────────────────────────────────────────────
#  SIMD intrinsics support
# ─────────────────────────────────────────────────────────────────────────────
if(USE_SIMD_INTRINSICS)
    include(CheckCXXCompilerFlag)
    # Check for AVX2 support
    check_cxx_compiler_flag("-mavx2" HAS_AVX2)
    if(HAS_AVX2)
        message(STATUS "Enabling AVX2 instructions")
        target_compile_options(slrbs PRIVATE -mavx2)
    else()
        # Check for SSE4.2 as fallback
        check_cxx_compiler_flag("-msse4.2" HAS_SSE42)
        if(HAS_SSE42)
            message(STATUS "Enabling SSE4.2 instructions")
            target_compile_options(slrbs PRIVATE -msse4.2)
        else()
            # Default to at least SSE2
            message(STATUS "Enabling SSE2 instructions")
            target_compile_options(slrbs PRIVATE -msse2)
        endif()
    endif()

    # Define a preprocessor symbol so code can check if SIMD is enabled
    target_compile_definitions(slrbs PRIVATE USE_SIMD_INTRINSICS)
endif()

# ─────────────────────────────────────────────────────────────────────────────
#  Link libraries
# ─────────────────────────────────────────────────────────────────────────────
target_link_libraries(slrbs PRIVATE
        ${OPENGL_LIB}
        polyscope
        ${OPENMP_LIBS}
)

# Conditional libraries
if(USE_QT)
    target_link_libraries(slrbs PRIVATE ${QT_LIBS})
endif()

if(USE_OPENCV)
    target_link_libraries(slrbs PRIVATE ${OPENCV_LIBS})
endif()

if(USE_TORCH)
    target_link_libraries(slrbs PRIVATE ${TORCH_LIBS})
endif()

if(USE_TCNN)
    # Create an interface library for tiny-cuda-nn if it doesn't exist
    if(NOT TARGET tiny-cuda-nn::tiny-cuda-nn)
        add_library(tiny-cuda-nn-interface INTERFACE)
        add_library(tiny-cuda-nn::tiny-cuda-nn ALIAS tiny-cuda-nn-interface)

        # Set up the interface library appropriately
        target_include_directories(tiny-cuda-nn-interface INTERFACE
                ${tiny_cuda_nn_SOURCE_DIR}/include
                ${tiny_cuda_nn_SOURCE_DIR}/dependencies
        )

        # Add CUDA link libraries if needed
        target_link_libraries(tiny-cuda-nn-interface INTERFACE cuda cudart)
    endif()

    target_link_libraries(slrbs PRIVATE tiny-cuda-nn::tiny-cuda-nn)
endif()

# Make sure we're building a console app on Windows
if(WIN32 OR MINGW)
    set_target_properties(slrbs PROPERTIES
            WIN32_EXECUTABLE FALSE)
endif()

# Add postfix to Debug builds
set_target_properties(slrbs PROPERTIES DEBUG_POSTFIX "d")

# ─────────────────────────────────────────────────────────────────────────────
#  Python bindings (if enabled)
# ─────────────────────────────────────────────────────────────────────────────
if(USE_PYBIND)
    file(GLOB_RECURSE PYMODULE_SRC_FILES CONFIGURE_DEPENDS src/*.cpp include/*.h)
    list(FILTER PYMODULE_SRC_FILES EXCLUDE REGEX ".*main\\.cpp$")

    # Also exclude QtSimWindow if Qt is not available
    if(NOT USE_QT)
        list(FILTER PYMODULE_SRC_FILES EXCLUDE REGEX ".*QtSimWindow\\.cpp$")
    endif()

    pybind11_add_module(pyslrbs ${PYMODULE_SRC_FILES})

    target_include_directories(pyslrbs PRIVATE
            ${CMAKE_CURRENT_SOURCE_DIR}/include
            ${eigen_SOURCE_DIR}
            ${polyscope_SOURCE_DIR}/deps/json/include
            ${polyscope_SOURCE_DIR}/deps/glm
            ${pybind11_SOURCE_DIR}/include
    )

    if(USE_TCNN)
        target_include_directories(pyslrbs PRIVATE ${tiny_cuda_nn_SOURCE_DIR}/include)
        target_compile_definitions(pyslrbs PRIVATE USE_TCNN)
    endif()

    if(OpenMP_CXX_FOUND)
        target_compile_options(pyslrbs PRIVATE ${OpenMP_CXX_FLAGS})
        target_compile_definitions(pyslrbs PRIVATE USE_OPENMP)
        if(USE_OPENMP_SIMD)
            target_compile_definitions(pyslrbs PRIVATE USE_OPENMP_SIMD)
        endif()
    endif()
    if(USE_SIMD_INTRINSICS)
        target_compile_definitions(pyslrbs PRIVATE USE_SIMD_INTRINSICS)
    endif()

    target_link_libraries(pyslrbs PRIVATE
            ${OPENGL_LIB}
            polyscope
            ${OPENMP_LIBS}
    )

    # Conditional libraries for Python module
    if(USE_QT)
        target_link_libraries(pyslrbs PRIVATE ${QT_LIBS})
    endif()

    if(USE_OPENCV)
        target_link_libraries(pyslrbs PRIVATE ${OPENCV_LIBS})
    endif()

    if(USE_TORCH)
        target_link_libraries(pyslrbs PRIVATE ${TORCH_LIBS})
    endif()

    if(USE_TCNN)
        target_link_libraries(pyslrbs PRIVATE tiny-cuda-nn::tiny-cuda-nn)
    endif()

    install(TARGETS pyslrbs
            LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}/python
            RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}/python)
endif()

# ─────────────────────────────────────────────────────────────────────────────
#  Install targets
# ─────────────────────────────────────────────────────────────────────────────
include(GNUInstallDirs)

install(TARGETS slrbs
        RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
        BUNDLE DESTINATION ${CMAKE_INSTALL_BINDIR})

set_property(GLOBAL PROPERTY USE_FOLDERS ON)
foreach(file IN LISTS SRC_FILES)
    get_filename_component(folder "${file}" PATH)
    string(REPLACE "${CMAKE_CURRENT_SOURCE_DIR}/src/"     "" folder "${folder}")
    string(REPLACE "${CMAKE_CURRENT_SOURCE_DIR}/include/" "" folder "${folder}")
    source_group("${folder}" FILES "${file}")
endforeach()

# Start Visual Studio's debugger in project root
if(MSVC)
    set_property(TARGET slrbs PROPERTY VS_DEBUGGER_WORKING_DIRECTORY "${CMAKE_SOURCE_DIR}")
endif()

# Copy resources after build (if needed)
if(EXISTS "${CMAKE_SOURCE_DIR}/resources")
    add_custom_command(TARGET slrbs POST_BUILD
            COMMAND ${CMAKE_COMMAND} -E copy_directory
            "${CMAKE_SOURCE_DIR}/resources"
            "$<TARGET_FILE_DIR:slrbs>/resources"
    )
endif()