#include <HandController/HandController.hpp>
#include <ROHandController/ROHandController.hpp>

HandController::HandController(){

}

HandController::~HandController(){

}

std::shared_ptr<HandController> HandController::GetPtr(const HandController::BasicConfig &config_){
    switch (config_.type) {
        case HandBase::HandType::ROHand :{
           return std::make_shared<ROHandController>(config_);
        }
        default:{
            return nullptr;
        }
    }
}

