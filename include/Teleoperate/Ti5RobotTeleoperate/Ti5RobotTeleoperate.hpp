#pragma once
#include <RobotTeleoperate/RobotTeleoperate.hpp>

class Ti5RobotTeleoperate
        :public RobotTeleoperate
{
public:
    Ti5RobotTeleoperate(const RobotTeleoperate::BasicConfig &config);
//    Ti5RobotTeleoperate(const std::string &fileName);
    ~Ti5RobotTeleoperate();

    bool Init() override;
    bool StartTeleop(bool verbose) override;
    bool StopTeleop() override;

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

    const RobotBase::RobotType robotType = RobotBase::RobotType::Ti5Robot;

    // Speed Limits
    Eigen::VectorXd speedThreshold;
};
