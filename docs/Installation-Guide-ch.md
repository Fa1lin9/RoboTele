# 安装指南

## 1. 支持的平台
本项目仅在 Linux 系统下开发和测试。

- 操作系统：
  - Ubuntu 20.04
  - Ubuntu 22.04

- CPU 架构：
  - x86_64
  - aarch64

- 不支持：
  - Windows ❌
  - macOS ❌

## 2. 第三方依赖概览

本项目依赖多个第三方库，包括优化、运动学、通信和可视化相关组件。

| 库          | 版本   | 安装方式 |
|------------|--------|----------|
| Eigen       | -      | -        |
| Boost       | 1.78.0 | 源码     |
| Qt          | 5.x    | -        |
| Pinocchio   | -      | -        |
| NLopt       | -      | 源码     |
| CasADi      | -      | 源码     |
| Protobuf    | 21.12  | 源码     |
| cppzmq      | -      | 源码     |
| libmodbus   | -      | 源码     |
| FastDDS     | -      | 源码     |

## 3. 第三方库作用说明

- **Eigen**  
  基本的矩阵和线性代数库，用于矩阵运算和通用数学计算。

- **Boost**  
  提供丰富的 C++ 工具集。主要使用 boost::json 模块解析和序列化 JSON 文件，该模块从 Boost 1.78 开始提供。

- **Qt**  
  用于图形用户界面（GUI）开发和可视化。

- **Pinocchio**  
  机器人运动学与动力学库，用于求解机器人的正、逆运动学，特别是人形机器人双臂。

- **NLopt**  
  非线性优化求解库，早期用于优化问题，后期转向使用 CasADi + IPOPT 进行高级优化。

- **CasADi**  
  自动求导库，用于高效计算梯度。主要用于求解人形机器人双臂逆运动学问题。

- **Protobuf**  
  通信协议库，用于接收来自 XR 设备的结构化数据。

- **cppzmq**  
  ZeroMQ 的 C++ 绑定，用于实现本地 C++ 项目与 Python 项目之间的通信。

- **libmodbus**  
  Modbus 通信库，用于控制 oymotion ROHand 灵巧手，并编写相应的 SDK。

- **FastDDS**  
  DDS（数据分发服务）实现，用于 ROS2。本项目使用 FastDDS 实现标准 C++ 项目与 ROS2 包之间的通信。

## 4. 源码编译库说明

### 1. CasADi
使用 CMake 编译 CasADi 时，建议启用 IPOPT：
```bash
cmake -DWITH_PYTHON=ON -DWITH_PYTHON3=ON -DWITH_IPOPT=ON ..
```
可以通过 apt 安装 IPOPT：
```bash
sudo apt install coinor-libipopt-dev
```
建议先安装 IPOPT 再编译 CasADi。
更多信息请参见：
https://github.com/casadi/casadi/wiki/InstallationLinux

### 2. Protobuf
可在以下地址获取预编译版本：
https://github.com/protocolbuffers/protobuf/releases

更多编译提示请参见 src/README.md。

### 3. Fast DDS

下载和安装说明请参考官方文档(Part 3.1.4. CMake installation)：
https://fast-dds.docs.eprosima.com/en/latest/installation/sources/sources_linux.html
 
### 4. 其他源码编译库（Boost、NLopt、libmodbus 等）
大部分库都有官方编译文档。
常用编译步骤如下：
```bash
mkdir build
cd build
cmake ..
make -j4
sudo make install
```