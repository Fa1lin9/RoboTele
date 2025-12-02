#include <WaistSolver/WaistSolver.hpp>

WaistSolver::WaistSolver(){
    this->Init();
}

WaistSolver::~WaistSolver(){

}

void WaistSolver::Init(){
    this->configPath =
            static_cast<std::string>(SOURCE_FILE_PATH) + "/config/BasicSolver/WaistSolver/WaistSolver.json";
    std::cout << "[WaistSolver::Init] Current path of the configuration is " << std::endl;
    std::cout << this->configPath << std::endl;

    this->jsonParser.Init(this->configPath);
}

Eigen::Vector3d WaistSolver::Solve(const Eigen::Matrix4d &mat){
    this->headPose = mat;

    Eigen::Matrix3d rot = this->headPose.block<3,3>(0,0);
//    this->rpy = rot.eulerAngles(2,1,0);
//    this->rpy = MatrixUtils::RotationToEulerXYZ(rot);
    this->rpy = MatrixUtils::RotationToEulerZYX(rot);

//    // Back to -pi ~ pi
//    MatrixUtils::NormalizeAngle(this->rpy.value());

    this->roll = this->rpy.value()(0);
    this->pitch = this->rpy.value()(1);
    this->yaw = this->rpy.value()(2);


    if(this->rpy.has_value()){
        return this->rpy.value();
    }else{
        throw std::logic_error("[WaistSolver::GetValue] The rpy don't have value");
    }
};

std::vector<int> WaistSolver::GetJointsIndex(const RobotType::Type &type){
    json::object obj = this->jsonParser.GetJsonObject();
    json::object ti5RobotObj = obj["Ti5Robot"].as_object();

    switch (type) {
        case RobotType::Type::Ti5Robot :{
           return JsonParser::JsonArray2StdVecInt(ti5RobotObj["JointsIndex"].as_array());
        }
        default:{
            throw std::logic_error("[WaistSolver::GetJointsIndex] Plz privide legal RobotType");
        }
    }
}

std::vector<std::string> WaistSolver::GetJointsName(const RobotType::Type &type){
    json::object obj = this->jsonParser.GetJsonObject();
    json::object ti5RobotObj = obj["Ti5Robot"].as_object();

    switch (type) {
        case RobotType::Type::Ti5Robot :{
           return JsonParser::JsonArray2StdVecStr(ti5RobotObj["JointsName"].as_array());
        }
        default:{
            throw std::logic_error("[WaistSolver::GetJointsName] Plz privide legal RobotType");
        }
    }
}

std::vector<RobotType::JointInfo> WaistSolver::GetJointsInfo(const RobotType::Type &type){
    json::object obj = this->jsonParser.GetJsonObject();
    json::object ti5RobotObj = obj["Ti5Robot"].as_object();

    auto names = this->GetJointsName(type);
    auto indices = this->GetJointsIndex(type);

    if(names.size() != indices.size()){
        throw std::logic_error("[WaistSolver::GetJointsInfo] Names and indices size mismatch!");
    }

    std::vector<RobotType::JointInfo> joints;
    joints.reserve(names.size());
    for(size_t i=0; i<names.size(); ++i){
        joints.push_back({names[i], indices[i]});
    }

    return joints;
}

