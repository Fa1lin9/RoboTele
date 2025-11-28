#include <DataCollector/DataCollector.hpp>
#include <HandSolver/HandSolver.hpp>
#include <HandController/HandController.hpp>

int main(){
    DataCollector dataCollector("tcp://127.0.0.1:5555");

    HandSolver::BasicConfig handSolverConfig;
    handSolverConfig.dofHand = 6;
    handSolverConfig.type = HandSolver::Type::VisionPro;
    auto handSolver = HandSolver::GetPtr(handSolverConfig);

    // Collector VisionPro's Data
    std::thread dataThread(&DataCollector::Run, &dataCollector);


    while(true){
        HandSolver::Data handData;
        if(dataCollector.HasNewData()){
            std::cout << "Get New Data! " << std::endl;
            handData.leftHandPositions = dataCollector.GetLeftHandPositions();
            handData.rightHandPositions = dataCollector.GetRightHandPositions();
        }else{
            continue;
        }
        auto handAngle = handSolver->Solve(handData);

        std::cout << "-----------------------------------------" << std::endl;
        std::cout << "HandAngle: " << std::endl;
        std::cout << handAngle << std::endl;
    }

    // delete the thread
    dataCollector.Stop();
    dataThread.join();
}
