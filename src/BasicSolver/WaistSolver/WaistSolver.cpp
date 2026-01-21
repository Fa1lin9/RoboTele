#include <WaistSolver/WaistSolver.hpp>

WaistSolver::WaistSolver(){
}

WaistSolver::WaistSolver(const std::string& path){
    this->Init(path);
}

WaistSolver::~WaistSolver(){

}

void WaistSolver::Init(const std::string& path){
//    this->configPath =
//            static_cast<std::string>(SOURCE_FILE_PATH) + "/config/BasicSolver/WaistSolver/WaistSolver.json";

    this->configPath = path;

    std::cout << "[WaistSolver::Init] Current path of the configuration is " << std::endl;
    std::cout << this->configPath << std::endl;

    this->jsonParser.Init(this->configPath);

    // For Json Object
    this->robotObj = this->jsonParser.GetJsonObject();

    // For JointsIndex
    if (!this->robotObj.contains("JointsIndex")) {
         throw std::logic_error("[WaistSolver::Init] Does not contain JointsIndex");
     }

     try {
         auto arr = this->robotObj["JointsIndex"].as_array();
         this->jointsIndex = JsonParser::JsonArray2StdVecInt(arr);
     } catch (const std::exception &e) {
         throw std::logic_error(
             std::string("[WaistSolver::Init] Failed to parse JointsIndex! \n")
             + "Error: " + e.what()
         );
     }

    // For JointsName
    if (!this->robotObj.contains("JointsName")) {
        throw std::logic_error("[WaistSolver::Init] Does not contain JointsName");
    }

    try {
        auto arr = this->robotObj["JointsName"].as_array();
        this->jointsName = JsonParser::JsonArray2StdVecStr(arr);
    } catch (const std::exception &e) {
        throw std::logic_error(
            std::string("[WaistSolver::Init] Failed to parse JointsName! \n")
            + "Error: " + e.what()
        );
    }

    // For Direction
    if (!this->robotObj.contains("Direction")) {
         throw std::logic_error("[WaistSolver::Init] Does not contain Direction");
     }

     try {
         auto arr = this->robotObj["Direction"].as_array();
         this->direction = JsonParser::JsonArray2StdVecInt(arr);
     } catch (const std::exception &e) {
         throw std::logic_error(
            std::string("[WaistSolver::Init] Failed to parse Direction! \n")
            + "Error: " + e.what()
         );
     }

    // For JoinstInfo
    if (!this->robotObj.contains("EulerAxis")) {
        throw std::logic_error("[WaistSolver::Init] Does not contain EulerAxis");
    }

    auto types = JsonParser::JsonArray2StdVecStr(robotObj["EulerAxis"].as_array());

    if(this->jointsName.size() != this->jointsIndex.size()){
        throw std::logic_error("[WaistSolver::Init] Names and indices size mismatch!");
    }

    this->jointsInfo.reserve(this->jointsName.size());
    for(size_t i=0; i<this->jointsName.size(); ++i){
        RobotBase::JointInfo item{
            .name = this->jointsName[i],
            .index = this->jointsIndex[i],
            .type = MatrixUtils::GetEulerAxisFromStr(types[i]),
            .direction = this->direction[i],
        };
        jointsInfo.push_back(item);
    }

    // For Bounds
    this->upperBound = JsonParser::JsonArray2StdVecDouble(this->robotObj["UpperBound"].as_array());
    this->lowerBound = JsonParser::JsonArray2StdVecDouble(this->robotObj["LowerBound"].as_array());

    if(1){
        std::cout << "[WaistSolver::Init] JointsInfo: " << std::endl;
        for(size_t i=0;i<this->upperBound.size();i++){
            std::cout << "Joint "
                      << this->jointsInfo[i].index
                      << " "
                      << this->jointsInfo[i].name
                      << ": UpperBound "
                      << this->upperBound[i]
                      << ", LowerBound "
                      << this->lowerBound[i]
                      << ", Direction "
                      << this->jointsInfo[i].direction
                      << std::endl;
        }
    }

    this->initFlag = true;
}

Eigen::Vector3d WaistSolver::Solve(const Eigen::Matrix4d &mat){
    if(!this->initFlag){
        throw std::logic_error("[WaistSolver::Solve] Plz initialize first! ");
    }

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
        throw std::logic_error("[WaistSolver::Solve] The rpy don't have value");
    }
};

//std::vector<int> WaistSolver::GetJointsIndex(){
//    return this->jointsIndex;
//}


//std::vector<std::string> WaistSolver::GetJointsName(){
//    return this->jointsName;
//}

//std::vector<int> WaistSolver::GetDirection()
//{
//    return this->direction;
//}


std::vector<RobotBase::JointInfo> WaistSolver::GetJointsInfo(){
    if(!this->initFlag){
        throw std::logic_error("[WaistSolver::GetJointsInfo] Plz initialize first! ");
    }

    return this->jointsInfo;
}

