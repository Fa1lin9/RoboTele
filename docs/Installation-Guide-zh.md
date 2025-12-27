# 安装指南

## 1. 支持的平台

本项目仅在 Linux 系统下开发和测试，且由于部分模块依赖于ROS2运行，因此推荐在Ubuntu 22.04下使用。

- **操作系统**：
  - Ubuntu 20.04
  - Ubuntu 22.04(强烈推荐)
- **CPU 架构**：
  - x86_64
  - aarch64
- **不支持**：
  - Windows ❌
  - macOS ❌

---

## 2. 第三方依赖概览

本项目依赖多个第三方库，包括优化、运动学、通信和可视化相关组件。

| 库               | 版本     | 安装方式 |
|-----------------|--------|------|
| Eigen           | -      | -    |
| Boost           | 1.78.0 | 源码   |
| Qt              | 5.x    | -    |
| Pinocchio       | 3.8.0  | 源码   |
| NLopt           | 2.10.0 | 源码   |
| CasADi          | 3.7.0  | 源码   |
| Protobuf        | 21.12  | 源码   |
| cppzmq          | 4.11.0 | 源码   |
| libmodbus       | 3.1.11 | 源码   |
| FastDDS         | 3.2.2  | 源码   |
| urdfdom         | 5.0.4  | 源码   |
| urdfdom_headers | 2.0.2  | 源码   |

## 3. 部分第三方库作用说明

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

### Notes
使用git tags可以查看各个tag版本，并从中选取自己想要的
```bash
git tags
```
使用git checkout tag/vX.X.X切换到对应的版本
```bash
git checkout tag/vX.X.X
```
git clone xxx之后记得更新子模块
```bash
git submodule update --init --recursive
```

###

### Boost
注意需要1.78.0版本的boost
```bash
cd boost_1_78_0
./bootstrap.sh
./b2
sudo ./b2 install
```

### FastDDS
从官网下载版本为3.2.2的源码包，然后执行install.sh
```bash
sudo bash ./install.sh
```

### Protobuf
```bash
cd protobuf
mkdir build && cd build
cmake ..
make -j$(nproc)
sudo make install
```

### NLopt
```bash
cd nlopt
mkdir build && cd build
cmake ..
make -j$(nproc)
sudo make install
```

### Pinocchio
注意编译pinocchio之前先编译urdfdom和urdfdom-headers，并通过CMAKE_PREFIX_PATH指定cmake优先查找库的路径
```bash
cd pinocchio
mkdir build && cd build
cmake .. -DBUILD_EXAMPLES=OFF -DBUILD_TESTING=OFF -DCMAKE_PREFIX_PATH=/usr/local
make -j$(nproc)
sudo make install
```

### CasADi
使用 CMake 编译 CasADi 时，必须启用 IPOPT：
可以通过 apt 安装 IPOPT：
```bash
sudo apt install coinor-libipopt-dev
```

```bash
sudo apt install coinor-libipopt-dev
cd casadi
mkdir build && cd build
cmake -DWITH_PYTHON=ON -DWITH_PYTHON3=ON -DWITH_IPOPT=ON ..
make -j$(nproc)
sudo make install
```

更多信息请参见：
https://github.com/casadi/casadi/wiki/InstallationLinux

### Protobuf
可在以下地址获取预编译版本：
https://github.com/protocolbuffers/protobuf/releases
```bash
cd protobuf
mkdir build && cd build
cmake ..
make -j$(nproc)
sudo make install
```
 
### 其他源码编译库（cppzmq、libmodbus、urdfdom 等）
大部分库都有官方编译文档。
常用编译步骤如下：
```bash
mkdir build && cd build
cmake ..
make -j4
sudo make install
```