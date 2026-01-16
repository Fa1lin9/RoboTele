#include <Transform/Transform.hpp>
#include <VisionPro2Ti5RobotTransform/VisionPro2Ti5RobotTransform.hpp>
#include <VisionPro2UnitreeG1Transform/VisionPro2UnitreeG1Transform.hpp>
#include <VisionProTransform/VisionProTransform.hpp>

Transform::Transform(){

}

Transform::~Transform(){

}

std::shared_ptr<Transform> Transform::GetPtr(const Transform::BasicConfig &config_){
    switch (config_.type) {
        case Transform::Type::VisionPro2Ti5Robot :{
           return std::make_shared<VisionPro2Ti5RobotTransform>(config_);
        }
        case Transform::Type::VisionPro2UnitreeG1 :{
           return std::make_shared<VisionPro2UnitreeG1Transform>(config_);
        }
        case Transform::Type::VisionPro :{
           return std::make_shared<VisionProTransform>(config_);
        }
        default:{
            return nullptr;
        }
    }
}

std::shared_ptr<Transform> Transform::GetPtr(const std::string& filePath){
    JsonParser jsonParser(filePath);
    json::object rootObj = jsonParser.GetJsonObject();

    // Transform
    Transform::BasicConfig config = {
        .T_Head2Waist = JsonParser::JsonArray2EigenMatrixXd(rootObj["T_Head2Waist"].as_array()),
        .T_XR2Robot = JsonParser::JsonArray2EigenMatrixXd(rootObj["T_XR2Robot"].as_array()),
        .T_Robot2LeftWrist = JsonParser::JsonArray2EigenMatrixXd(rootObj["T_Robot2LeftWrist"].as_array()),
        .T_Robot2RightWrist = JsonParser::JsonArray2EigenMatrixXd(rootObj["T_Robot2RightWrist"].as_array()),
        .offset = JsonParser::JsonArray2EigenVectorXd(rootObj["Offset"].as_array()),
        .type = Transform::GetTypeFromStr(rootObj["Type"].as_string().c_str()),
    };

    return Transform::GetPtr(config);
}

const std::unordered_map<std::string, Transform::Type> Transform::typeMap = {
    {"VisionPro2Ti5Robot", Transform::Type::VisionPro2Ti5Robot},
    {"VisionPro2UnitreeG1", Transform::Type::VisionPro2UnitreeG1},
    {"VisionPro", Transform::Type::VisionPro}
};

Transform::Type Transform::GetTypeFromStr(const std::string& str){
    auto temp = Transform::typeMap.find(str);
    if(temp != Transform::typeMap.end()){
        return temp->second;
    }

    throw std::invalid_argument("[Transform::GetTypeFromStr] Invalid string");
}
