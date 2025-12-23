# Installation Guide

## 1. Supported Platforms
This project is developed and tested **only on Linux systems**.

- Operating System:
  - Ubuntu 20.04 
  - Ubuntu 22.04 

- CPU Architecture:
  - x86_64
  - aarch64

- Unsupported:
  - Windows ❌
  - macOS ❌

## 2. Third-party Dependencies Overview

This project depends on multiple third-party libraries, including
optimization, kinematics, communication, and visualization components.

| Library    | Version | Install Method |
|------------|---------|----------------|
| Eigen      | -       | -              |
| Boost      | 1.78.0  | source         |
| Qt         | 5.x     | -              |
| Pinocchio  | -       | -              |
| NLopt      | -       | source         |
| CasADi     | -       | source         |
| Protobuf   | 21.12   | source         |
| cppzmq     | -       | source         |
| libmodbus  | -       | source         |
| FastDDS    | -       | source         |

## 3. Third-party Libraries Purpose

- **Eigen**  
  A basic matrix and linear algebra library. Used for matrix operations and general mathematical computations.

- **Boost**  
  Provides a wide range of C++ utilities. Mainly using the `boost::json` module to parse and serialize JSON files. This module is available starting from Boost 1.78.

- **Qt**  
  Used for graphical user interface (GUI) development and visualization.

- **Pinocchio**  
  A robotics library for kinematics and dynamics computations. Used to solve robot forward and inverse kinematics, especially for humanoid robot arms.

- **NLopt**  
  A library for nonlinear optimization. Used in early stages for optimization, later replaced by CasADi + IPOPT for more advanced optimization.

- **CasADi**  
  An automatic differentiation library, used to compute gradients efficiently. Mainly applied for solving humanoid robot dual-arm inverse kinematics.

- **Protobuf**  
  A communication protocol library. Used to receive structured data from XR devices.

- **cppzmq**  
  C++ bindings for ZeroMQ. Used to implement communication between local C++ projects and Python projects.

- **libmodbus**  
  Library for Modbus communication. Used to control OYMOTION ROHAND robotic hands and write the corresponding SDK.

- **FastDDS**  
  A DDS (Data Distribution Service) implementation used in ROS2. This project uses FastDDS to communicate between standard C++ projects and ROS2 packages.


## 4. Notes on Source-built Dependencies

### 1. CasADi
When building CasADi with CMake, it is recommended to enable IPOPT:

```bash
cmake -DWITH_PYTHON=ON -DWITH_PYTHON3=ON -DWITH_IPOPT=ON ..
```

You can install IPOPT via apt:

```bash
sudo apt install coinor-libipopt-dev
```

It is recommended to install IPOPT first before building CasADi.
For more details, see: https://github.com/casadi/casadi/wiki/InstallationLinux

### 2. Protobuf
Prebuilt releases are available at: https://github.com/protocolbuffers/protobuf/releases
Additional build hints are in src/README.md.

### 3. Fast DDS

Download and installation instructions can be found in the official documentation(Part 3.1.4. CMake installation):
https://fast-dds.docs.eprosima.com/en/latest/installation/sources/sources_linux.html

### 4. Other source-built libraries (Boost, NLopt, libmodbus, etc.)
Most of these libraries provide official build instructions.
The typical compilation steps that have been used successfully are:

```bash
mkdir build
cd build
cmake ..
make -j4
sudo make install
```
