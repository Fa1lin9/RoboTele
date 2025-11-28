#include <HeadSolver/HeadSolver.hpp>

HeadSolver::HeadSolver(){
    this->Init();
}

HeadSolver::~HeadSolver(){

}

void HeadSolver::Init(){
    this->configPath =
            static_cast<std::string>(SOURCE_FILE_PATH) + "/config/BasicSolver/HeadSolver/HeadSolver.json";
    std::cout << "[HeadSolver::Init] Current path of the configuration is " << std::endl;
    std::cout << this->configPath << std::endl;

    this->jsonParser.Init(this->configPath);
}

Eigen::Vector3d HeadSolver::Solve(const Eigen::Matrix4d &mat){
    this->headPose = mat;

    Eigen::Matrix3d rot = this->headPose.block<3,3>(0,0);
//    this->rpy = rot.eulerAngles(2,1,0);
//    this->rpy = RotationToEulerXYZ(rot);
    this->rpy = RotationToEulerZYX(rot);

//    // Back to -pi ~ pi
//    this->NormalizeAngle(this->rpy.value());

    this->roll = this->rpy.value()(0);
    this->pitch = this->rpy.value()(1);
    this->yaw = this->rpy.value()(2);


    if(this->rpy.has_value()){
        return this->rpy.value();
    }else{
        throw std::logic_error("[HeadSolver::GetValue] The rpy don't have value");
    }
};

std::vector<int> HeadSolver::GetJointsIndex(const RobotType::Type &type){
    json::object obj = this->jsonParser.GetJsonObject();
    json::object ti5RobotObj = obj["Ti5Robot"].as_object();

    switch (type) {
        case RobotType::Type::Ti5Robot :{
           return JsonParser::JsonArray2StdVecInt(ti5RobotObj["JointsIndex"].as_array());
        }
        default:{
            throw std::logic_error("[HeadSolver::GetJointsIndex] Plz privide legal RobotType");
        }
    }
}

std::vector<std::string> HeadSolver::GetJointsName(const RobotType::Type &type){
    json::object obj = this->jsonParser.GetJsonObject();
    json::object ti5RobotObj = obj["Ti5Robot"].as_object();

    switch (type) {
        case RobotType::Type::Ti5Robot :{
           return JsonParser::JsonArray2StdVecStr(ti5RobotObj["JointsName"].as_array());
        }
        default:{
            throw std::logic_error("[HeadSolver::GetJointsName] Plz privide legal RobotType");
        }
    }
}

std::vector<RobotType::JointInfo> HeadSolver::GetJointsInfo(const RobotType::Type &type){
    json::object obj = this->jsonParser.GetJsonObject();
    json::object ti5RobotObj = obj["Ti5Robot"].as_object();

    auto names = this->GetJointsName(type);
    auto indices = this->GetJointsIndex(type);

    if(names.size() != indices.size()){
        throw std::logic_error("[HeadSolver::GetJointsInfo] Names and indices size mismatch!");
    }

    std::vector<RobotType::JointInfo> joints;
    joints.reserve(names.size());
    for(size_t i=0; i<names.size(); ++i){
        joints.push_back({names[i], indices[i]});
    }

    return joints;
}

void HeadSolver::NormalizeAngle(Eigen::VectorXd& angle){
    for(int i=0;i<angle.size();i++){
        angle(i) = fmod(angle(i) + M_PI, 2 * M_PI); // 先 +π 再取模
        if (angle(i) < 0) {
            angle(i) += 2 * M_PI; // 确保在 [0, 2π]
        }
        angle(i) -= M_PI; // 回到 [-π, π]
    }
}

Eigen::Vector3d HeadSolver::RotationToEulerXYZ(const Eigen::Matrix3d &R)
{
    Eigen::Vector3d rpy;

    double sy = -R(0,2);
    if (sy > 1.0) sy = 1.0;
    if (sy < -1.0) sy = -1.0;

    rpy[1] = std::asin(sy);  // Y

    if (std::abs(std::cos(rpy[1])) > 1e-6)
    {
        rpy[0] = std::atan2(R(1,2), R(2,2));  // X
        rpy[2] = std::atan2(R(0,1), R(0,0));  // Z
    }
    else
    {
        rpy[0] = 0.0;
        rpy[2] = std::atan2(-R(1,0), R(1,1));
    }

    return rpy;
}

Eigen::Vector3d HeadSolver::RotationToEulerZYX(const Eigen::Matrix3d &R)
{
    Eigen::Vector3d rpy;

    // pitch = asin(-R(2,0))
    double sy = -R(2,0);
    if (sy > 1.0) sy = 1.0;
    if (sy < -1.0) sy = -1.0;

    rpy[1] = std::asin(sy);  // pitch

    // Check singularity: |cos(pitch)| < small
    if (std::abs(std::cos(rpy[1])) > 1e-6)
    {
        rpy[0] = std::atan2(R(2,1), R(2,2));  // roll
        rpy[2] = std::atan2(R(1,0), R(0,0));  // yaw
    }
    else
    {
        // Gimbal lock: pitch = +-90°
        rpy[0] = 0.0;
        rpy[2] = std::atan2(-R(0,1), R(1,1));
    }

    return rpy;  // [roll, pitch, yaw]
}
