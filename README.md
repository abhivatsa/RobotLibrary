# RoboticsLibrary

![License](https://img.shields.io/badge/license-MIT-blue.svg)
![C++](https://img.shields.io/badge/C%2B%2B-17-blue.svg)
![Build Status](https://img.shields.io/badge/build-passing-brightgreen.svg)

**RoboticsLibrary** is a comprehensive, header-only C++ library designed to facilitate the development of advanced robotics applications. It offers robust tools for mathematical computations, kinematics, dynamics, trajectory planning, and collision detection, enabling developers to create, simulate, and control robotic systems with precision and efficiency.

---

## Table of Contents

- [Features](#features)
- [Installation](#installation)
- [Building RoboticsLibrary](#building-roboticslibrary)
- [Usage](#usage)
  - [Including RoboticsLibrary Directly](#including-roboticslibrary-directly)
  - [Using RoboticsLibrary as a Git Submodule](#using-roboticslibrary-as-a-git-submodule)
  - [Using `find_package` with CMake](#using-find_package-with-cmake)
- [Configuration](#configuration)
  - [Loading `config.json`](#loading-configjson)
- [Documentation](#documentation)
- [License](#license)
- [Contact](#contact)
- [Acknowledgments](#acknowledgments)

---

## Features

- **Mathematical Foundations**:
  - `Vector<N>` and `Matrix<ROWS, COLS>` classes with comprehensive operator overloads.
  - Support for vector and matrix arithmetic, dot product, cross product (for 3D vectors), normalization, and transposition.

- **Kinematics**:
  - Forward kinematics computations to determine end-effector positions based on joint angles.
  - Inverse kinematics solutions for desired end-effector poses.

- **Dynamics**:
  - Implementation of the Recursive Newton-Euler (RNE) algorithm for accurate dynamics calculations.
  - Gravity compensation torque computations for smooth and controlled robotic movements.

- **Trajectory Planning**:
  - Cubic spline interpolation for generating smooth trajectories between waypoints.
  - Flexible waypoint management with timestamps for precise motion sequencing.

- **Collision Detection**:
  - Robot model representation for detecting potential collisions within the workspace.
  - Integration with kinematics and dynamics modules to ensure safe and efficient movements.

- **Template-Based Design**:
  - Highly flexible and scalable, supporting robots with varying degrees of freedom (number of joints).

- **Header-Only Library**:
  - Easy integration into projects without the need for separate compilation.

---

## Installation

RoboticsLibrary is a header-only library, making it straightforward to integrate into your C++ projects. To set up the development environment, follow the steps below.

### Prerequisites

- **C++ Compiler**: Supports C++17 or higher (e.g., GCC, Clang, MSVC).
- **CMake**: Version 3.10 or higher.
- **Git**: For version control and fetching dependencies.

### Steps

1. **Clone the Repository**

   ```bash
   git clone https://github.com/yourusername/roboticslibrary.git
   cd roboticslibrary
