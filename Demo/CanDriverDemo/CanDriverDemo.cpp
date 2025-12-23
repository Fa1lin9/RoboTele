#include <iostream>

#include <CanDriver/CanDriver.h>
#include <boost/shared_ptr.hpp>
#include <unordered_map>
#include <PhysicalRobot/PhysicalRobot.hpp>
#include <map>
#include <iomanip>
#include <math.h>

using namespace CanDriver;

constexpr size_t dofArm = 7;
constexpr size_t dofHead = 3;
constexpr size_t dofWaist = 3;


const   std::map<std::string, std::tuple<size_t,size_t>> jointsList = {
    // joint name: {can device , can id}
    {"L_SHOULDER_P",    {0, 23}},
    {"L_SHOULDER_R",    {0, 24}},
    {"L_SHOULDER_Y",    {0, 25}},
    {"L_ELBOW_P",       {0, 26}},
    {"L_FOREARM_Y",     {0, 27}},
    {"L_WRIST_P",       {0, 28}},
    {"L_HAND_R",        {0, 29}},
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
};

int main(){
    // Test on 2025.12.23
    // MainDemo can't link to the libcontrolcan.so
    // Set configeration
    PhysicalRobot::BasicConfig config = {
        .robotType = RobotBase::RobotType::Ti5Robot,
    };

    boost::shared_ptr<PhysicalRobot> physicalRobotPtr
            = PhysicalRobot::GetPtr(config);

    physicalRobotPtr->Connect();

    if(!InitCan()){
        std::cout<< " Can't initialize the Can ! " <<std::endl;
        return 0;
    }else{
        std::cout<< " Sucessfully initialize the Can ! " <<std::endl;
    }

    std::cout<< " --------------------------------------------- " <<std::endl;

    double value;
    double position = 1.0;
    for(const auto& joint:jointsList){
        // Get Value
        const std::string name = joint.first;
        const size_t canDevice = std::get<0>(joint.second);
        const size_t canID = std::get<1>(joint.second);
        std::cout<< " name " << name
                 << ", canDevice " << canDevice
                 << ", canID " << canID
                 << std::endl;

//        if(canID == 28){
//            double value = 0;
//            auto result = SendRecvPosition(canDevice, canID, &value);
//            std::cout<<" SendRecvPosition: "<<std::endl;
//            std::cout<<" Can ID: "<<std::get<0>(result.value())
//                     <<" Status: "<<std::get<1>(result.value())
//                     <<std::endl;
//            auto result =
//                    SendRecvMultiPosition(canDevice, canID, &value, bool(canID % 7 != 0));
//            for(auto const& temp:result){
//                std::cout<<" SendRecvPosition: "<<std::endl;
//                std::cout<<" Can ID: "<<std::get<0>(temp)
//                         <<" Status: "<<std::get<1>(temp)
//                         <<std::endl;
//            }
//        }

//        SendPosition(canDevice, canID, 0);

        //
        ConfigMaxVelocity(canDevice, canID, 700);
        RecvMaxVelocity(canDevice, canID, &value);
        std::cout<<" Joint "<<name<<" Max Velocity: "<< std::fixed << std::setprecision(3)<<value <<std::endl;

        ConfigMaxAcceleration(canDevice, canID, 5);
        RecvMaxAcceleration(canDevice, canID, &value);
        std::cout<<" Joint "<<name<<" Max Acceleration: "<< std::fixed << std::setprecision(3)<<value<<std::endl;

        // useless
        RecvPosition(canDevice, canID, &position);
        std::cout<<" Position: "<< std::fixed << std::setprecision(3)<< position <<std::endl;

        std::cout<< " --------------------------------------------- " <<std::endl;
    }


//    PhysicalRobot::config config = {
//        .type = PhysicalRobot::Type::CrpRobot,
//    };

//    boost::shared_ptr<PhysicalRobot> robotPtr
//            = PhysicalRobot::GetPtr(config);

//    robotPtr->Connect();
//    std::vector<double> jointsAngle = robotPtr->GetJointsAngle();

////    Eigen::VectorXd jointsAngleEigen = Eigen::VectorXd::Map(jointsAngle.data(),jointsAngle.size());
//    Eigen::VectorXd jointsAngleEigen = robotPtr->GetJointsAngleEigen();

//    std::cout<<"JointsAngle:\n"<<jointsAngleEigen<<std::endl;
}
