#include <HandController/HandController.hpp>
#include <ROHandController/ROHandController.hpp>

HandController::HandController(){

}

HandController::~HandController(){

}

boost::shared_ptr<HandController> HandController::GetPtr(const HandController::BasicConfig &config_){
    switch (config_.type) {
        case HandController::Type::ROHand :{
           return boost::make_shared<ROHandController>(config_);
        }
        default:{
            return nullptr;
        }
    }
}

const std::unordered_map<std::string, HandController::Type> HandController::typeMap = {
    {"ROHand", HandController::Type::ROHand}
};

HandController::Type HandController::GetTypeFromStr(const std::string& str){
    auto temp = HandController::typeMap.find(str);
    if(temp != HandController::typeMap.end()){
        return temp->second;
    }

    throw std::invalid_argument("[HandController::GetTypeFromStr] Invalid string");
}
