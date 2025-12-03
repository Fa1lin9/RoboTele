#include <HeadSolver/HeadSolver.hpp>

HeadSolver::HeadSolver(){
}

HeadSolver::HeadSolver(const RobotType::Type& type){
    this->Init(type);
}

HeadSolver::~HeadSolver(){

}

void HeadSolver::Init(const RobotType::Type& type){
    this->type = type;
    this->configPath =
            static_cast<std::string>(SOURCE_FILE_PATH) + "/config/BasicSolver/HeadSolver/HeadSolver.json";
    std::cout << "[HeadSolver::Init] Current path of the configuration is " << std::endl;
    std::cout << this->configPath << std::endl;

    this->jsonParser.Init(this->configPath);

    // For Bounds
    this->boundsLower = {};
}

Eigen::Vector3d HeadSolver::Solve(const Eigen::Matrix4d &mat){
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

    // Clip the Angle
//    auto jointsInfo = this->GetJointsInfo();

    if(this->rpy.has_value()){
        return this->rpy.value();
    }else{
        throw std::logic_error("[HeadSolver::GetValue] The rpy don't have value");
    }
};

std::vector<int> HeadSolver::GetJointsIndex(){
    json::object obj = this->jsonParser.GetJsonObject();

    const std::string typeStr = RobotType::GetStrFromType(this->type);

    if (!obj.contains(typeStr)) {
        throw std::logic_error("[HeadSolver::GetJointsIndex] JSON does not contain robot '" + typeStr + "'");
    }

    json::object robotObj;
    try {
        robotObj = obj[typeStr].as_object();
    } catch (const std::exception &e) {
        throw std::logic_error("[HeadSolver::GetJointsIndex] '" + typeStr + "' is not a JSON object. " + e.what());
    }

    if (!robotObj.contains("JointsIndex")) {
        throw std::logic_error("[HeadSolver::GetJointsIndex] '" + typeStr + "' does not contain JointsIndex");
    }

    try {
        auto arr = robotObj["JointsIndex"].as_array();
        return JsonParser::JsonArray2StdVecInt(arr);
    } catch (const std::exception &e) {
        throw std::logic_error(
            std::string("[HeadSolver::GetJointsIndex] Failed to parse JointsIndex for '")
            + typeStr + "'. " + e.what()
        );
    }
}


std::vector<std::string> HeadSolver::GetJointsName(){
    json::object obj = this->jsonParser.GetJsonObject();

    const std::string typeStr = RobotType::GetStrFromType(this->type);

    if (!obj.contains(typeStr)) {
        throw std::logic_error("[HeadSolver::GetJointsName] JSON does not contain robot '" + typeStr + "'");
    }

    json::object robotObj;
    try {
        robotObj = obj[typeStr].as_object();
    } catch (const std::exception& e) {
        throw std::logic_error("[HeadSolver::GetJointsName] '" + typeStr + "' is not a JSON object. " + e.what());
    }

    if (!robotObj.contains("JointsName")) {
        throw std::logic_error("[HeadSolver::GetJointsName] '" + typeStr + "' does not contain JointsName");
    }

    try {
        auto arr = robotObj["JointsName"].as_array();
        return JsonParser::JsonArray2StdVecStr(arr);
    } catch (const std::exception &e) {
        throw std::logic_error(
            std::string("[HeadSolver::GetJointsName] Failed to parse JointsName for '")
            + typeStr + "'. " + e.what());
    }
}


std::vector<RobotType::JointInfo> HeadSolver::GetJointsInfo(){
    json::object obj = this->jsonParser.GetJsonObject();

    const std::string typeStr = RobotType::GetStrFromType(this->type);

    if (!obj.contains(typeStr)) {
        throw std::logic_error("[HeadSolver::GetJointsName] JSON does not contain robot '" + typeStr + "'");
    }

    json::object robotObj;
    try {
        robotObj = obj[typeStr].as_object();
    } catch (const std::exception& e) {
        throw std::logic_error("[HeadSolver::GetJointsName] '" + typeStr + "' is not a JSON object. " + e.what());
    }

    auto names = this->GetJointsName();
    auto indices = this->GetJointsIndex();
    auto types = JsonParser::JsonArray2StdVecStr(robotObj["EulerAxis"].as_array());

    if(names.size() != indices.size()){
        throw std::logic_error("[HeadSolver::GetJointsInfo] Names and indices size mismatch!");
    }

    std::vector<RobotType::JointInfo> jointsInfo;
    jointsInfo.reserve(names.size());
    for(size_t i=0; i<names.size(); ++i){
        RobotType::JointInfo item{
            .name = names[i],
            .index = indices[i],
            .type = MatrixUtils::GetEulerAxisFromStr(types[i]),
        };
        jointsInfo.push_back(item);
    }

    return jointsInfo;
}

