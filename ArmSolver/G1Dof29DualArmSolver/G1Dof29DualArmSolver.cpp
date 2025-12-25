#include <G1Dof29DualArmSolver/G1Dof29DualArmSolver.hpp>

G1Dof29DualArmSolver::G1Dof29DualArmSolver(const ArmSolver::BasicConfig &config_)
    :maxIteration(config_.maxIteration),
     relativeTol(config_.relativeTol)
{
    LOG_FUNCTION;
    // the initialization of the model must be first
    pinocchio::urdf::buildModel(
                this->modelPath,
                this->robotModel);

    this->robotModelSX = this->robotModel.cast<casadi::SX>();

    // As For Unitree G1
    // The size of the baseFrameName should be 1
    // The size of the targetFrameName should be 2, and as the order: left arm , right arm
    if(config_.baseFrameName.size() != 1){
        std::string error = " As for G1Dof29DualArmSolver,the size of the baseFrameName should be 1! ";
        throw std::length_error(error);
    }else{
        this->baseIndex = this->robotModel.getFrameId(config_.baseFrameName[0]);
    }

    if(config_.targetFrameName.size() != 2){
        std::string error = " As for G1Dof29DualArmSolver,the size of the baseFrameName should be 2! And the order is: left arm, right arm! ";
        throw std::length_error(error);
    }else{
        this->leftArmEndIndex = this->robotModel.getFrameId(config_.targetFrameName[0]);
        this->rightArmEndIndex = this->robotModel.getFrameId(config_.targetFrameName[1]);
    }

//    if(config_.baseOffset.size() != 1){
//        std::string error = " As for G1Dof29DualArmSolver,the size of the baseOffset should be 1! ";
//        throw std::length_error(error);
//    }else{
//        this->baseOffset = config_.baseOffset[0];
//        this->baseOffsetSX = this->baseOffset.cast<casadi::SX>();
//    }

    // Init
    // Initialize some basic parameter,like the bounds
    this->InitRobot(config_);

    this->wTranslation = config_.wTranslation;
    this->wRotation = config_.wRotation;
    this->wRegularization = config_.wRegularization;
    this->wSmooth = config_.wSmooth;

    // Init auto-diff
    this->InitOptim(config_);

    std::cout<<" Init Sucessfully! "<<std::endl;
}

G1Dof29DualArmSolver::~G1Dof29DualArmSolver()
{

}

size_t G1Dof29DualArmSolver::GetDofTotal(){
    return this->dofTotal;
}

std::vector<std::string> G1Dof29DualArmSolver::GetJointNames(){
    return this->jointNames;
}


boost::optional<Eigen::VectorXd> G1Dof29DualArmSolver::Solve(
    const std::vector<Eigen::Matrix4d>& targetPose,
    const Eigen::VectorXd& qLast_,
    bool verbose)
{
    // check the target pose
    for(size_t i=0;i<targetPose.size();i++){
        if(!MatrixUtils::IsPoseMatrix(targetPose[i])){
            std::cout<<"targetPose is not a pose matrix"<<std::endl;
            return boost::none;
        }
    }

    // Set Value
    // qInit
    std::vector<double> qInitVec(qLast_.data(), qLast_.data() + qLast_.size());
    this->opti.set_value(this->qLast, casadi::DM(qInitVec));

    // targetPose
    casadi::DM leftTargetPoseDM(4,4);
    casadi::DM rightTargetPoseDM(4,4);
    for(size_t i=0;i<4;i++){
        for(size_t j=0;j<4;j++){
            leftTargetPoseDM(i,j) = targetPose[0](i,j);
            rightTargetPoseDM(i,j) = targetPose[1](i,j);
        }
    }

    this->opti.set_value(this->targetPoseLeft, leftTargetPoseDM);
    this->opti.set_value(this->targetPoseRight, rightTargetPoseDM);

//        std::cout<<"targetPose in Eigen:"<<targetPose[0]<<std::endl;
//        std::cout<<"targetPose in Casadi:"<<targetPoseLeftDM<<std::endl;

    // qVar
    // Set decision variables's initial values
    this->opti.set_initial(this->qVar, casadi::DM(qInitVec));

    auto start = std::chrono::high_resolution_clock::now();

    casadi::OptiSol sol = this->opti.solve();

    auto end = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    std::cout << "Optimization 耗时: " << duration.count() << " ms" << std::endl;

    // -----------------------------
    // 获取求解结果
    // -----------------------------
    casadi::DM qSolution = sol.value(this->qVar);

    // 直接用 Eigen::Map 转为 Eigen::VectorXd
    Eigen::VectorXd qEigen = Eigen::Map<Eigen::VectorXd>(qSolution.ptr(), qSolution.size1());

    // -----------------------------
    // 打印调试信息
    // -----------------------------
    if(verbose){
        // check result
        std::cout<<"------------ Solver Result ------------"<<std::endl;
        std::cout << " Joint Value =\n" << std::fixed << std::setprecision(5) << qEigen << std::endl;
//            std::cout << " Function Value = " << funcValue << std::endl;
        std::cout<<" Left Arm Translation: \n"<<Forward(qEigen)[0].translation()<<std::endl;
        std::cout<<" Left Arm Rotation: \n"<<Forward(qEigen)[0].rotation()<<std::endl;
        std::cout<<" Right Arm Translation: \n"<<Forward(qEigen)[1].translation()<<std::endl;
        std::cout<<" Rigit Arm Rotation: \n"<<Forward(qEigen)[1].rotation()<<std::endl;
        std::cout<<"------------ Solver Result ------------"<<std::endl;
    }

    // 返回 Eigen::VectorXd
    return boost::optional<Eigen::VectorXd>(qEigen);
}

std::vector<pinocchio::SE3> G1Dof29DualArmSolver::Forward(const Eigen::VectorXd& q){
    return std::vector<pinocchio::SE3>{};
}

void G1Dof29DualArmSolver::Info(){
    std::cout << " ----------------------------------------------------- "<< std::endl;

//    std::cout << "Size of joints upper limitaion: " << robotModel.upperPositionLimit.rows() << std::endl;
    std::cout << "Number of joints: " << robotModel.njoints << std::endl;
    std::cout << "Number of DOFs: " << robotModel.nv << std::endl;
    std::cout << "Number of frames: " << robotModel.nframes << std::endl;
//    for(size_t i =0;i<robotModel.nframes;i++){
//        std::cout<<" Frame "<<i<<" : "<<robotModel.frames[i]<<std::endl;
//    }

    for(size_t i =0;i<robotModel.njoints;i++){
//        std::cout<<"Joint "<<i<<" : "<<robotModel.joints[i]<<std::endl;
    }

    std::cout << "Number of q: " << robotModel.nq << std::endl;

    for(pinocchio::JointIndex i=0; i<robotModel.njoints; i++){
        std::cout << "Joint " << i << ": " << robotModel.names[i]
                  << ", type: " << robotModel.joints[i].shortname()
                  << ", parent: " << robotModel.parents[i] << std::endl;
    }

    std::cout << "this->totalLowerBound: " << std::endl;
    for(size_t i=0;i<this->totalLowerBound.size();i++){
        std::cout << this->totalLowerBound[i] << std::endl;
    }
    std::cout << "this->totalUpperBound: " << std::endl;
    for(size_t i=0;i<this->totalUpperBound.size();i++){
        std::cout << this->totalUpperBound[i] << std::endl;
    }


}

// Initialization
void G1Dof29DualArmSolver::InitRobot(const ArmSolver::BasicConfig &config_){
    LOG_FUNCTION;

    // Load JointNames
    for(pinocchio::JointIndex i=1; i<robotModel.njoints; ++i){
        std::cout << "Joint " << i << ": " << robotModel.names[i]
                  << ", type: " << robotModel.joints[i].shortname()
                  << ", parent: " << robotModel.parents[i] << std::endl;
        this->jointNames.push_back(robotModel.names[i]);
    }

    double min,max;
    for(size_t i = 0;i<this->dofArm;i++){
        // left arm
        min = this->robotModel.lowerPositionLimit(this->leftArmID[0] + i - 1);
        max = this->robotModel.upperPositionLimit(this->leftArmID[0] + i - 1);
        this->leftArmLowerBound.push_back(min);
        this->leftArmUpperBound.push_back(max);

        // right arm
        min = this->robotModel.lowerPositionLimit(this->rightArmID[0] + i - 1);
        max = this->robotModel.upperPositionLimit(this->rightArmID[0] + i - 1);
        this->rightArmLowerBound.push_back(min);
        this->rightArmUpperBound.push_back(max);
    }

    // For bound
    this->totalLowerBound.resize(this->dofTotal);
    this->totalUpperBound.resize(this->dofTotal);

    for(size_t i=0;i<this->dofArm;i++){
        // For Left Arm
        this->totalLowerBound[this->leftArmID[i] - 1] = this->leftArmLowerBound[i];
        this->totalUpperBound[this->leftArmID[i] - 1] = this->leftArmUpperBound[i];

        // For Right Arm
        this->totalLowerBound[this->rightArmID[i] - 1] = this->rightArmLowerBound[i];
        this->totalUpperBound[this->rightArmID[i] - 1] = this->rightArmUpperBound[i];

    }

    // according to the config , set limitation to the joint
    // Left Arm
    std::cout<<"The current DOF of the left arm is "<<config_.dofArm[0]<<std::endl;
    for(size_t i=0;i<this->dofArm - config_.dofArm[0];i++){
        size_t temp = this->leftArmID.back() - 1 - i;
        this->totalLowerBound[temp] = 0;
        this->totalUpperBound[temp] = 0;
        std::cout<<"Set limitation to left arm joint"<<temp<<std::endl;
    }
    // Right Arm
    std::cout<<"The current DOF of the right arm is "<<config_.dofArm[1]<<std::endl;
    for(size_t i=0;i<this->dofArm - config_.dofArm[1];i++){
        size_t temp = this->rightArmID.back() - 1 - i;
        this->totalLowerBound[temp] = 0;
        this->totalUpperBound[temp] = 0;
        std::cout<<"Set limitation to right arm joint"<<temp<<std::endl;
    }

    // Initial Pose
    this->initPose = Eigen::VectorXd::Zero(21);

    std::cout<<" The size of the totalBoundsLower is "<<totalLowerBound.size()<<std::endl;
    std::cout<<" The size of the totalBoundsUpper is "<<totalUpperBound.size()<<std::endl;
}

void G1Dof29DualArmSolver::InitOptim(const ArmSolver::BasicConfig &config_){
//    pinocchio::DataTpl<casadi::SX> dataSX(this->robotModelSX);
    this->dataPtrSX = std::make_shared<pinocchio::DataTpl<casadi::SX>>(this->robotModelSX);

    // Creating symbolic variables
    casadi::SX targetPoseLeft_ = casadi::SX::sym("targetPoseLeft_",4,4);
    casadi::SX targetPoseRight_ = casadi::SX::sym("targetPoseRight_",4,4);
    casadi::SX qVar_ = casadi::SX::sym("qVar_",this->dofTotal);
    pinocchio::DataTpl<casadi::SX>::ConfigVectorType q =
            pinocchio::DataTpl<casadi::SX>::ConfigVectorType::Zero(this->dofTotal);
    for(int i =0;i<q.size();i++){
        q(i) = qVar_(i);
    }

    pinocchio::forwardKinematics(robotModelSX, *this->dataPtrSX, q);
    pinocchio::updateFramePlacements(robotModelSX, *this->dataPtrSX);

    // extract matrix
    Eigen::Matrix<casadi::SX,4,4> basePose =
            this->baseOffsetSX * (*this->dataPtrSX).oMf[this->baseIndex].toHomogeneousMatrix();
    Eigen::Matrix<casadi::SX,4,4> leftArmEndPose = (*this->dataPtrSX).oMf[this->leftArmEndIndex].toHomogeneousMatrix();
    Eigen::Matrix<casadi::SX,4,4> rightArmEndPose = (*this->dataPtrSX).oMf[this->rightArmEndIndex].toHomogeneousMatrix();

    Eigen::Matrix<casadi::SX,4,4> leftArmPose = basePose.inverse() * leftArmEndPose;
    Eigen::Matrix<casadi::SX,4,4> rightArmPose = basePose.inverse() * rightArmEndPose;

    casadi::SX basePoseSX = Eigen2SX(basePose);
    casadi::SX leftArmEndPoseSX = Eigen2SX(leftArmEndPose);
    casadi::SX rightArmEndPoseSX = Eigen2SX(rightArmEndPose);

    casadi::SX basePoseInvSX = casadi::SX::inv(basePoseSX);
    casadi::SX leftArmPoseSX = casadi::SX::mtimes(basePoseInvSX, leftArmEndPoseSX);
    casadi::SX rightArmPoseSX = casadi::SX::mtimes(basePoseInvSX, rightArmEndPoseSX);

    // translation error
//    std::cout<<" Translation Error "<<std::endl;
    casadi::SX currentLeftTrans  = leftArmPoseSX(casadi::Slice(0,3), 3);
    casadi::SX currentRightTrans = rightArmPoseSX(casadi::Slice(0,3), 3);
    casadi::SX targetLeftTrans   = targetPoseLeft_(casadi::Slice(0,3), 3);
    casadi::SX targetRightTrans  = targetPoseRight_(casadi::Slice(0,3), 3);

    casadi::SX currentTrans = casadi::SX::vertcat({currentLeftTrans, currentRightTrans});
    casadi::SX targetTrans  = casadi::SX::vertcat({targetLeftTrans,  targetRightTrans});
    casadi::SX transError   = casadi::SX::sumsqr(currentTrans - targetTrans);

    this->translationalError =
            casadi::Function("translationalError",
                             {qVar_, targetPoseLeft_, targetPoseRight_},
                             {transError});

    // rotation error
//    std::cout<<" rotation Error "<<std::endl;
    Eigen::Matrix<casadi::SX,3,3> leftArmError
            = leftArmPose.block<3,3>(0,0) * SX2Eigen(targetPoseLeft_).block<3,3>(0,0).transpose();
    Eigen::Matrix<casadi::SX,3,3> rightArmError
            = rightArmPose.block<3,3>(0,0) * SX2Eigen(targetPoseRight_).block<3,3>(0,0).transpose();

    Eigen::Matrix<casadi::SX,3,1> leftRotaError  = pinocchio::log3(leftArmError).cast<casadi::SX>();
    Eigen::Matrix<casadi::SX,3,1> rightRotaError = pinocchio::log3(rightArmError).cast<casadi::SX>();

    casadi::SX rotaError =
            casadi::SX::sumsqr(casadi::SX::vertcat({
                Eigen2SX(leftRotaError),
                Eigen2SX(rightRotaError)
            }));

    this->rotationalError =
            casadi::Function("rotationalError",
                             {qVar_, targetPoseLeft_, targetPoseRight_},
                             {rotaError});

//    casadi::Opti opti;
    this->qVar = this->opti.variable(this->dofTotal, 1);
    this->qLast = this->opti.parameter(this->dofTotal, 1);
    this->targetPoseLeft = this->opti.parameter(4, 4);
    this->targetPoseRight = this->opti.parameter(4, 4);

    this->translationalCost =
            translationalError({qVar, targetPoseLeft, targetPoseRight})[0];
    this->rotationalCost =
            rotationalError({qVar, targetPoseLeft, targetPoseRight})[0];
    this->smoothCost = casadi::MX::sumsqr(qVar - qLast);

    std::vector initPoseVec = std::vector<double>(this->initPose.data(),
                                                  this->initPose.data() + this->initPose.size());

    this->regularizationCost = casadi::MX::sumsqr(qVar);

    this->totalCost =
        config_.wTranslation * translationalCost +
        config_.wRotation * rotationalCost +
        config_.wSmooth * smoothCost +
        config_.wRegularization * regularizationCost;

    this->opti.subject_to(
                this->opti.bounded(
                    this->totalLowerBound,
                    qVar,
                    this->totalUpperBound
                    ));

    this->opti.minimize(totalCost);

    casadi::Dict opts;
    opts["ipopt.tol"] = this->relativeTol;
    opts["ipopt.max_iter"] = static_cast<int>(this->maxIteration);
//    opts["ipopt.linear_solver"] = "mumps";
    opts["ipopt.print_level"] = 0;   // <= 设置为0，禁用迭代输出
    opts["print_time"] = 0;          // <= 禁用求解时间输出
    opts["calc_lam_p"] = 0;          // 可选，关闭对偶变量计算（减少输出）

    this->opti.solver("ipopt", opts);
}
