#include <HeadSolver/HeadSolver.hpp>

HeadSolver::HeadSolver(){
}

HeadSolver::HeadSolver(const RobotBase::RobotType& type){
    this->Init(type);
}

HeadSolver::~HeadSolver(){

}

void HeadSolver::Init(const RobotBase::RobotType& type){
    this->type = type;

    this->configPath =
            static_cast<std::string>(SOURCE_FILE_PATH) + "/config/BasicSolver/HeadSolver/HeadSolver.json";
    std::cout << "[HeadSolver::Init] Current path of the configuration is " << std::endl;
    std::cout << this->configPath << std::endl;

    this->jsonParser.Init(this->configPath);

    // For Json Object
    this->rootObj = this->jsonParser.GetJsonObject();
    this->typeStr = RobotBase::GetStrFromType(this->type);

    if (!this->rootObj.contains(this->typeStr)) {
        throw std::logic_error("[HeadSolver::Init] JSON does not contain robot '" + this->typeStr + "'");
    }

    try {
        this->robotObj = this->rootObj[this->typeStr].as_object();
    } catch (const std::exception &e) {
        throw std::logic_error("[HeadSolver::Init] '" + this->typeStr + "' is not a JSON object. " + e.what());
    }

    // JointsInfo
    this->jointsInfo = this->GetJointsInfo();

    // For Bounds
    this->upperBound = JsonParser::JsonArray2StdVecDouble(this->robotObj["UpperBound"].as_array());
    this->lowerBound = JsonParser::JsonArray2StdVecDouble(this->robotObj["LowerBound"].as_array());

    if(0){
        for(size_t i=0;i<this->upperBound.size();i++){
            std::cout << "Joint "
                      << this->jointsInfo[i].index
                      << " "
                      << this->jointsInfo[i].name
                      << ": UpperBound "
                      << this->upperBound[i]
                      << ", LowerBound "
                      << this->lowerBound[i]
                      << std::endl;
        }
    }
}

Eigen::Vector3d HeadSolver::Solve(const Eigen::Matrix4d &mat){
    this->headPose = mat;

    Eigen::Matrix3d rot = this->headPose.block<3,3>(0,0);
//    this->rpy = rot.eulerAngles(2,1,0);
//    this->rpy = MatrixUtils::RotationToEulerXYZ(rot);
    this->rpy = MatrixUtils::RotationToEulerZYX(rot);

    // Clip the Angle
    for(size_t i=0;i<this->jointsInfo.size();i++){
        const auto& item = this->jointsInfo[i];

        int axis = -1;
        if(item.type == MatrixUtils::EulerAxis::Roll)  axis = 0;
        if(item.type == MatrixUtils::EulerAxis::Pitch) axis = 1;
        if(item.type == MatrixUtils::EulerAxis::Yaw)   axis = 2;

        if(axis >= 0){
            rpy.value()(axis) =
                std::min(std::max(rpy.value()(axis), this->lowerBound[i]), this->upperBound[i]);
        }
    }

    this->roll = this->rpy.value()(0);
    this->pitch = this->rpy.value()(1);
    this->yaw = this->rpy.value()(2);

    if(this->rpy.has_value()){
        return this->rpy.value();
    }else{
        throw std::logic_error("[HeadSolver::Solve] The rpy don't have value");
    }
};

std::vector<int> HeadSolver::GetJointsIndex(){
   if (!robotObj.contains("JointsIndex")) {
        throw std::logic_error("[HeadSolver::GetJointsIndex] '" + this->typeStr + "' does not contain JointsIndex");
    }

    try {
        auto arr = robotObj["JointsIndex"].as_array();
        return JsonParser::JsonArray2StdVecInt(arr);
    } catch (const std::exception &e) {
        throw std::logic_error(
            std::string("[HeadSolver::GetJointsIndex] Failed to parse JointsIndex for '")
            + this->typeStr + "'. " + e.what()
        );
    }
}


std::vector<std::string> HeadSolver::GetJointsName(){
    if (!robotObj.contains("JointsName")) {
        throw std::logic_error("[HeadSolver::GetJointsName] '" + this->typeStr + "' does not contain JointsName");
    }

    try {
        auto arr = robotObj["JointsName"].as_array();
        return JsonParser::JsonArray2StdVecStr(arr);
    } catch (const std::exception &e) {
        throw std::logic_error(
            std::string("[HeadSolver::GetJointsName] Failed to parse JointsName for '")
            + this->typeStr + "'. " + e.what());
    }
}


std::vector<RobotBase::JointInfo> HeadSolver::GetJointsInfo(){
    if (!this->robotObj.contains("EulerAxis")) {
        throw std::logic_error("[HeadSolver::GetJointsInfo] '" + this->typeStr + "' does not contain EulerAxis");
    }

    auto names = this->GetJointsName();
    auto indices = this->GetJointsIndex();
    auto types = JsonParser::JsonArray2StdVecStr(robotObj["EulerAxis"].as_array());

    if(names.size() != indices.size()){
        throw std::logic_error("[HeadSolver::GetJointsInfo] Names and indices size mismatch!");
    }

    std::vector<RobotBase::JointInfo> jointsInfo;
    jointsInfo.reserve(names.size());
    for(size_t i=0; i<names.size(); ++i){
        RobotBase::JointInfo item{
            .name = names[i],
            .index = indices[i],
            .type = MatrixUtils::GetEulerAxisFromStr(types[i]),
        };
        jointsInfo.push_back(item);
    }

    return jointsInfo;
}

