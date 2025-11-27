#pragma once
#include <Eigen/Dense>
#include <zmq.hpp>
#include <iostream>
#include <mutex>

// 打印函数信息
#include <FunctionLogger.hpp>

// 该类主要是负责接受VisionPro端解析得到的数据
// 通信方式主要是通过Socket解析python脚本发送过来的数据
class DataCollector
{
public:
    DataCollector();
    DataCollector(const std::string &address_);
    ~DataCollector();

    void Run();

    void Stop();

//    std::vector<Eigen::Matrix4d> GetValue();

    std::vector<Eigen::Matrix4d> GetPoseMatrix();

    std::vector<Eigen::Vector3d> GetLeftHandPositions();

    std::vector<Eigen::Vector3d> GetRightHandPositions();

    bool HasNewData();

    void Init(const std::string &address_);

private:
    const int numJointsHand = 25;

    // some variable for socket
    std::string address;
    zmq::context_t context;
    zmq::socket_t subscriber;

    // some variable for data
    // 头、双臂的位姿矩阵
    Eigen::Matrix4d headPose;
    Eigen::Matrix4d leftArmPose;
    Eigen::Matrix4d rightArmPose;

    // 两只手的点位
    std::vector<Eigen::Vector3d> leftHandPositions;
    std::vector<Eigen::Vector3d> rightHandPositions;

    // lock
    std::mutex mutex;

    bool stopFlag = false;
    std::atomic_bool hasNewData;
};
