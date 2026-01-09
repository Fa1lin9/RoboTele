#pragma once
#include <RobotHardware/RobotHardware.hpp>
#include <CanDriver/CanDriver.h>

#include <stdio.h>
#include <cstring>
#include <stdlib.h>
#include <sstream>
#include <fstream>
#include <cmath>
#include <iostream>
#include <unistd.h>
#include <cstdlib>
#include <Ti5MOVE.h>
#include <Ti5BASIC.h>
#include <Ti5LOGIC.h>
#include <mathfunc.h>
#include <tool.h>
#include <csignal>
#include <mutex>
#include <shared_mutex>
#include <Ti5CAN_Driver.h>
#include <map>
#include <termios.h>
#include <fcntl.h>
#include <algorithm>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <future>

#define RESET "\033[0m"
#define RED "\033[31m"     /* Red */
#define GREEN "\033[32m"   /* Green */
#define YELLOW "\033[33m"  /* Yellow */
#define BLUE "\033[34m"    /* Blue */
#define MAGENTA "\033[35m" /* Magenta */
#define CYAN "\033[36m"    /* Cyan */

enum ArmType{
    LeftArm,
    RightArm,
};

class Ti5RobotHardware
        :public RobotHardware
{
public:
    Ti5RobotHardware(const RobotHardware::BasicConfig &config_);

    ~Ti5RobotHardware();
    /* ---------------- Basic Information ---------------- */

    // Left(device0): 23, 24, 25, 26, 27, 28, 29
    // Right(device0): 16, 17, 18, 19, 14, 21, 22
    // Head(device0): 30, 31, 32
    // Waist(device0) : 13, 14, 15

    // joint name: {can device , can id}
//    {"L_SHOULDER_P",    {0, 23}},
//    {"L_SHOULDER_R",    {0, 24}},
//    {"L_SHOULDER_Y",    {0, 25}},
//    {"L_ELBOW_P",       {0, 26}},
//    {"L_FOREARM_Y",     {0, 27}},
//    {"L_WRIST_P",       {0, 28}},
//    {"L_HAND_R",        {0, 29}},
//    {"R_SHOULDER_P",    {0, 16}},
//    {"R_SHOULDER_R",    {0, 17}},
//    {"R_SHOULDER_Y",    {0, 18}},
//    {"R_ELBOW_P",       {0, 19}},
//    {"R_FOREARM_Y",     {0, 20}},
//    {"R_WRIST_P",       {0, 21}},
//    {"R_HAND_R",        {0, 22}},
//    {"NECK_Y_S",        {0, 30}},  // head y
//    {"NECK_P_S",        {0, 31}},  // head p
//    {"NECK_R_S",        {0, 32}},  // head r
//    {"WAIST_P_S",       {0, 15}},  // waist p
//    {"WAIST_R_S",       {0, 13}},  // waist r
//    {"WAIST_Y_S",       {0, 14}},  // waist y

    /* ---------------- Basic Information ---------------- */

    bool Connect() override;

    bool Disconnect() override;

    bool isConnect() override;

    bool EmergencyStop() override;

    bool BackToZero() override;

    bool BackToInitPose(const RobotHardware::RobotCmd& config_) override;

    bool BackToZero(const RobotHardware::RobotCmd& config_) override;

    void Info() override;

    void GetJointsStatus() override;

    std::vector<double> GetJointsAngle() override;

    Eigen::VectorXd GetJointsAngleEigen() override;

    bool MoveJ(const std::vector<double> &jointsAngle_) override;

    bool MoveL() override;

    bool MoveJ(const RobotHardware::RobotCmd& config_) override;

    bool Init() override;


private:
    bool Initialize(bool verbose);

    bool SendRecvJoints(const std::vector<double>& jointsValue,
                        size_t dof,
                        size_t canDevice,
                        const std::vector<size_t>& canID,
                        const std::string& partName);

    void LoadConfig(const RobotHardware::BasicConfig &config);

    bool connectFlag = false;

    // degree of freedom
    const size_t waistDof = 3;
    const size_t headDof = 3;
    const size_t armDof = 7;

    // Can Device
    size_t leftArmCanDevice = 0;
    size_t rightArmCanDevice = 0;
    size_t headCanDevice = 0;
    size_t waistCanDevice = 0;

    // Can ID
    std::vector<size_t> leftArmCanID = {23, 24, 25, 26, 27, 28, 29};
    std::vector<size_t> rightArmCanID = {16, 17, 18, 19, 20, 21, 22};
    std::vector<size_t> headCanID = {30, 31, 32};
    std::vector<size_t> waistCanID = {13, 14, 15};


};
