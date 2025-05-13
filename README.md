# Sheldon's Little Rigid-Body Simulator (SLRBS)

[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)
[![CMake](https://img.shields.io/badge/CMake-3.15+-blue.svg)](https://cmake.org/)
[![C++](https://img.shields.io/badge/C++-17-blue.svg)](https://isocpp.org/)
[![Platform](https://img.shields.io/badge/Platform-Windows%20%7C%20Linux%20%7C%20macOS-lightgrey.svg)](https://github.com/yourusername/SLRBS)
[![PRs Welcome](https://img.shields.io/badge/PRs-welcome-brightgreen.svg)](https://github.com/yourusername/SLRBS/pulls)

A simple, lightweight rigid body simulation engine.

Developed primarily for teaching, SLRBS has been used at the [École de technologie supérieure](https://www.etsmtl.ca/) in the [SIGGRAPH contact simulation course](https://siggraphcontact.github.io/). Feel free to use it in your own projects.

---

## Table of Contents

1. [Getting Started](#getting-started)
2. [Dependencies](#dependencies)
3. [Building](#building)

   * [Release Build](#release-build)
   * [Debug Build](#debug-build)
4. [Running the Executable](#running-the-executable)
5. [Creating Scenarios](#creating-scenarios)
6. [License](#license)

---

## Getting Started

Clone the repository and create a build directory:

```bash
git clone https://github.com/ETSim/SLRBS.git
cd SLRBS
mkdir build
cd build
```

## Dependencies

* **Eigen**
* **Polyscope**
* CMake 3.15 or higher

## Building

You can configure and build either a **Release** or **Debug** version.

### Release Build

Optimized for performance.

**Windows (Visual Studio 2022)**

```powershell
cmake .. -G "Visual Studio 17 2022" -DCMAKE_BUILD_TYPE=Release
cmake --build . --config Release
```

**Linux/macOS (Makefile)**

```bash
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j$(nproc)
```

The release executable will be:

* **Windows**: `build/Release/slrbs.exe`
* **Linux/macOS**: `build/slrbs`

### Debug Build

Includes debug symbols and no optimization for easier debugging.

**Windows (Visual Studio 2022)**

```powershell
cmake ..\ -G "Visual Studio 17 2022" -DCMAKE_BUILD_TYPE=Debug
cmake --build . --config Debug
```

**Linux/macOS (Makefile)**

```bash
cmake .. -DCMAKE_BUILD_TYPE=Debug
make -j$(nproc)
```

The debug executable will be:

* **Windows**: `build/Debug/slrbs.exe`
* **Linux/macOS**: `build/slrbs`

## Running the Executable

After building, you can run the simulator from the build directory:

```bash
# On Windows (PowerShell)
./Release/slrbs.exe           # for the release build
# or
./Debug/slrbs.exe             # for the debug build

# On Linux/macOS
./slrbs
```

Use the GUI to select built-in scenarios or load custom JSON scenarios.

## Creating Scenarios

See `include/rigidbody/Scenarios.h` for examples of how to programmatically define scenarios with various geometries and joints.

## License

* **SLRBS** is licensed under the **MIT License**.
* **Eigen** is under the **MPL-2.0** (see `3rdParty/Eigen3`).
* **Polyscope** is under the **MIT License** (see `3rdParty/polyscope/LICENSE`).
