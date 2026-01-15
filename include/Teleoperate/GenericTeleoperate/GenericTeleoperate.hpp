#pragma once
#include <RobotTeleoperate/RobotTeleoperate.hpp>

class GenericTeleoperate
        :public RobotTeleoperate
{
public:
    GenericTeleoperate(const RobotTeleoperate::BasicConfig &config);
//    GenericTeleoperate(const std::string &fileName);
    ~GenericTeleoperate();

    bool Init() override;
    bool StartTeleop(bool verbose) override;
    bool StopTeleop() override;
    void Info() override;

private:
    bool CheckDataValid();

    bool CheckSolutionValid(const Eigen::VectorXd& sol,
                            const Eigen::VectorXd& qInit);

    std::string address;
    CsvWriter csvWriter;

    DataCollector dataCollector;
    Ros2Bridge ros2Bridge;

    // some flags
    bool startFlag = false;

    bool saveFlag = true;

//    const RobotBase::RobotType robotType = RobotBase::RobotType::GenericRobot;

    // Speed Limits
    Eigen::VectorXd speedThreshold;
};
