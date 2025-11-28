#include <HeadSolver/HeadSolver.hpp>
#include <WaistSolver/WaistSolver.hpp>

int main(){
    HeadSolver head;

    auto jointsIndex = head.GetJointsIndex(RobotType::Ti5Robot);
    auto jointsName = head.GetJointsName(RobotType::Ti5Robot);
    auto jointsInfo = head.GetJointsInfo(RobotType::Ti5Robot);

    for(size_t i=0;i<jointsIndex.size();i++){
//        std::cout<<"JointsIndex "<<jointsIndex[i]<<": "<<jointsName[i]<<std::endl;
        std::cout<<"JointsIndex "<<jointsInfo[i].index<<": "<<jointsInfo[i].name<<std::endl;
    }
}
