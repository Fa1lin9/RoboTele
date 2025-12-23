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

## 3. Notes on Source-built Dependencies

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

### 3. Other source-built libraries (Boost, NLopt, libmodbus, FastDDS, etc.)
Most of these libraries provide official build instructions.
The typical compilation steps that have been used successfully are:

```bash
mkdir build
cd build
cmake ..
make -j4
sudo make install
```
