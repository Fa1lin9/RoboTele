#include <Transform/Transform.hpp>
#include <VisionPro2Ti5RobotTransform/VisionPro2Ti5RobotTransform.hpp>
#include <VisionPro2UnitreeG1Transform/VisionPro2UnitreeG1Transform.hpp>

Transform::Transform(){

}

Transform::~Transform(){

}

boost::shared_ptr<Transform> Transform::GetPtr(const Transform::BasicConfig &config_){
    switch (config_.type) {
        case Transform::Type::VisionPro2Ti5Robot :{
           return boost::make_shared<VisionPro2Ti5RobotTransform>(config_);
        }
    case Transform::Type::VisionPro2UnitreeG1 :{
       return boost::make_shared<VisionPro2UnitreeG1Transform>(config_);
    }
        default:{
            return nullptr;
        }
    }
}

const std::unordered_map<std::string, Transform::Type> Transform::typeMap = {
    {"VisionPro2Ti5Robot", Transform::Type::VisionPro2Ti5Robot},
    {"VisionPro2UnitreeG1", Transform::Type::VisionPro2UnitreeG1}
};

Transform::Type Transform::GetTypeFromStr(const std::string& str){
    auto temp = Transform::typeMap.find(str);
    if(temp != Transform::typeMap.end()){
        return temp->second;
    }

    throw std::invalid_argument("[Transform::GetTypeFromStr] Invalid string");
}
