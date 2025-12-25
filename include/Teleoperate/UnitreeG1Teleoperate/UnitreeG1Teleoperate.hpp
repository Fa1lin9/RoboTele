#pragma once
#include <RobotTeleoperate/RobotTeleoperate.hpp>

class UnitreeG1Teleoperate
        :public RobotTeleoperate
{
public:
    UnitreeG1Teleoperate(const RobotTeleoperate::BasicConfig &config);
//    UnitreeG1Teleoperate(const std::string &fileName);
    ~UnitreeG1Teleoperate();

    bool Init() override;
    bool StartTeleoperate(bool verbose) override;
    bool StopTeleoperate() override;

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

    const RobotBase::RobotType robotType = RobotBase::RobotType::UnitreeG1;

    // Speed Limits
    Eigen::VectorXd speedThreshold;
};
