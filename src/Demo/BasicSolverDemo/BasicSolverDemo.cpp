#include <HeadSolver/HeadSolver.hpp>
#include <WaistSolver/WaistSolver.hpp>

int main(){
    HeadSolver head(RobotBase::RobotType::Ti5Robot);

    auto headJointsIndex = head.GetJointsIndex();
    auto headJointsName = head.GetJointsName();
    auto headJointsInfo = head.GetJointsInfo();

    for(size_t i=0;i<headJointsIndex.size();i++){
//        std::cout<<"JointsIndex "<<jointsIndex[i]<<": "<<jointsName[i]<<std::endl;
        std::cout<<"JointsIndex "<<
                   headJointsInfo[i].index<<": "<<
                   headJointsInfo[i].name<<", EulerAxis: "<<
                   headJointsInfo[i].type<<std::endl;
    }

    WaistSolver waist(RobotBase::RobotType::Ti5Robot);

    auto waistJointsIndex = waist.GetJointsIndex();
    auto waistJointsName = waist.GetJointsName();
    auto waistJointsInfo = waist.GetJointsInfo();

    for(size_t i=0;i<waistJointsIndex.size();i++){
//        std::cout<<"JointsIndex "<<jointsIndex[i]<<": "<<jointsName[i]<<std::endl;
        std::cout<<"JointsIndex "<<
                   waistJointsInfo[i].index<<": "<<
                   waistJointsInfo[i].name<<", EulerAxis: "<<
                   waistJointsInfo[i].type<<std::endl;
    }
}
