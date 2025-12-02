#include <HeadSolver/HeadSolver.hpp>
#include <WaistSolver/WaistSolver.hpp>

int main(){
    HeadSolver head;

    auto headJointsIndex = head.GetJointsIndex(RobotType::Ti5Robot);
    auto headJointsName = head.GetJointsName(RobotType::Ti5Robot);
    auto headJointsInfo = head.GetJointsInfo(RobotType::Ti5Robot);

    for(size_t i=0;i<headJointsIndex.size();i++){
//        std::cout<<"JointsIndex "<<jointsIndex[i]<<": "<<jointsName[i]<<std::endl;
        std::cout<<"JointsIndex "<<headJointsInfo[i].index<<": "<<headJointsInfo[i].name<<std::endl;
    }

    WaistSolver waist;

    auto waistJointsIndex = waist.GetJointsIndex(RobotType::Ti5Robot);
    auto waistJointsName = waist.GetJointsName(RobotType::Ti5Robot);
    auto waistJointsInfo = waist.GetJointsInfo(RobotType::Ti5Robot);

    for(size_t i=0;i<waistJointsIndex.size();i++){
//        std::cout<<"JointsIndex "<<jointsIndex[i]<<": "<<jointsName[i]<<std::endl;
        std::cout<<"JointsIndex "<<waistJointsInfo[i].index<<": "<<waistJointsInfo[i].name<<std::endl;
    }
}
