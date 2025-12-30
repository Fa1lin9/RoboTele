#include <GenericDualArmSolver/GenericDualArmSolver.hpp>

GenericDualArmSolver::GenericDualArmSolver(const ArmSolver::BasicConfig &config)
{
    this->CheckConfig(config);

    this->LoadConfig(config);

    this->InitRobot();

    this->InitOptim();
}

GenericDualArmSolver::~GenericDualArmSolver()
{
}

boost::optional<Eigen::VectorXd> GenericDualArmSolver::Solve(
    const std::vector<Eigen::Matrix4d>& targetPose,
    const Eigen::VectorXd& qLast_,
    bool verbose)
{
    // Check the target pose
    for(size_t i=0;i<targetPose.size();i++){
        if(!MatrixUtils::IsPoseMatrix(targetPose[i])){
            std::cout<<"[GenericDualArmSolver::Solve] The targetPose is not a pose matrix"<<std::endl;
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

    // --------------------------------
    // Get the Solution of the Solver
    // --------------------------------
    casadi::DM qSolution = sol.value(this->qVar);

    // 直接用 Eigen::Map 转为 Eigen::VectorXd
    Eigen::VectorXd qEigen = Eigen::Map<Eigen::VectorXd>(qSolution.ptr(), qSolution.size1());

    // -----------------------------
    //  Debug Information
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

    return boost::optional<Eigen::VectorXd>(qEigen);
}

std::vector<pinocchio::SE3> GenericDualArmSolver::Forward(const Eigen::VectorXd& q)
{
    return std::vector<pinocchio::SE3>{};
}

size_t GenericDualArmSolver::GetTotalDof()
{
    return this->totalDof;
}

std::vector<std::string> GenericDualArmSolver::GetJointNames()
{
    return this->jointNames;
}

void GenericDualArmSolver::Info()
{
    // 空实现
}

// ===== Private Functions =====

void GenericDualArmSolver::InitRobot()
{
    LOG_FUNCTION;
    // Build the model of the robot
    pinocchio::urdf::buildModel(
                this->modelPath,
                this->robotModel
                );

    // Get Index
    this->baseFrameIndex = this->robotModel.getFrameId(this->baseFrameName[0]);
    this->leftArmEndJointIndex = this->robotModel.getJointId(this->targetFrameName[0]);
    this->rightArmEndJointIndex = this->robotModel.getJointId(this->targetFrameName[1]);

    if (this->baseFrameIndex == pinocchio::FrameIndex(-1)) {
        throw std::runtime_error("Frame not found: " + this->baseFrameName[0]);
    }
    if (this->leftArmEndJointIndex <= 0 || this->leftArmEndJointIndex >= this->robotModel.njoints ||
        this->rightArmEndJointIndex <= 0 || this->rightArmEndJointIndex >= this->robotModel.njoints) {
        throw std::runtime_error("Joint not found: " + this->targetFrameName[0]);
    }

    // Load JointNames
    this->jointNames.clear();
    for (pinocchio::JointIndex i = 1; i < robotModel.njoints; ++i) {
        const auto& joint = this->robotModel.joints[i];

        if (joint.nq() == 0) {
            continue;
        }

        this->jointNames.push_back(this->robotModel.names[i]);

        std::cout << "Joint " << i
                  << ": " << robotModel.names[i]
                  << ", nq: " << joint.nq()
                  << ", parent: " << robotModel.parents[i]
                  << std::endl;
    }

    // Modified the model of the robot to add the frames of the end effector
    // For Left Arm
    this->robotModel.addFrame(
                pinocchio::Frame(
                    "LeftArmEndEffector",
                    this->leftArmEndJointIndex,
                    pinocchio::SE3(
                        this->targetOffset[0].block<3,3>(0,0),
                        this->targetOffset[0].block<3,1>(0,3)
                        ),
                    pinocchio::FrameType::OP_FRAME
                    )
                );

    // For Right Arm
    this->robotModel.addFrame(
                pinocchio::Frame(
                    "RightArmEndEffector",
                    this->rightArmEndJointIndex,
                    pinocchio::SE3(
                        this->targetOffset[1].block<3,3>(0,0),
                        this->targetOffset[1].block<3,1>(0,3)
                        ),
                    pinocchio::FrameType::OP_FRAME
                    )
                );

    this->leftArmEndEffectorFrameIndex = this->robotModel.getFrameId("LeftArmEndEffector");
    this->rightArmEndEffectorFrameIndex = this->robotModel.getFrameId("RightArmEndEffector");

    // Get Dof
    this->totalDof = this->robotModel.nq;
    this->armDof = this->leftArmJointNames.size();
    std::cout<<"The totalDof of this robot is "<<this->totalDof<<" ! "<<std::endl;
    std::cout<<"The armDof of this robot is "<<this->armDof<<" ! "<<std::endl;

    // Get Arm JointIndex
    for(size_t i=0;i<this->armDof;i++){
        try {
            // Left Arm
            size_t leftArmIdx = this->robotModel.getJointId(this->leftArmJointNames[i]);
            this->leftArmJointIndex.push_back(leftArmIdx);
        } catch (const std::exception& e) {
            std::cerr << "[GenericDualArmSolver::InitRobot] Left arm joint '"
                      << this->leftArmJointNames[i]
                      << "' not found! Exception: " << e.what() << std::endl;
            throw;
        }

        try {
            // Right Arm
            size_t rightArmIdx = this->robotModel.getJointId(this->rightArmJointNames[i]);
            this->rightArmJointIndex.push_back(rightArmIdx);
        } catch (const std::exception& e) {
            std::cerr << "[GenericDualArmSolver::InitRobot] Right arm joint '"
                      << this->rightArmJointNames[i]
                      << "' not found! Exception: " << e.what() << std::endl;
            throw;
        }
    }
    std::cout << "Left Arm JointIndex:\n";
    for (size_t i = 0; i < this->leftArmJointIndex.size(); ++i) {
        std::cout << "Joint[" << i << "] = " << this->leftArmJointIndex[i] << "\n";
    }

    std::cout << "\nRight Arm JointIndex:\n";
    for (size_t i = 0; i < this->rightArmJointIndex.size(); ++i) {
        std::cout << "Joint[" << i << "] = " << this->rightArmJointIndex[i] << "\n";
    }

    // Bound Loading
    this->totalLowerBound.resize(this->totalDof);
    this->totalUpperBound.resize(this->totalDof);
    for(size_t i = 0; i < armDof; i++) {
        // Left Arm
        size_t leftQIdx = this->robotModel.joints[this->leftArmJointIndex[i]].idx_q();
//        std::cout<<"leftQIdx: "<<leftQIdx<<std::endl;
        double leftMin  = this->robotModel.lowerPositionLimit(leftQIdx);
        double leftMax  = this->robotModel.upperPositionLimit(leftQIdx);
        this->leftArmLowerBound.push_back(leftMin);
        this->leftArmUpperBound.push_back(leftMax);

        this->totalLowerBound[leftQIdx] = leftMin;
        this->totalUpperBound[leftQIdx] = leftMax;

        // Right Arm
        size_t rightQIdx = this->robotModel.joints[this->rightArmJointIndex[i]].idx_q();
//        std::cout<<"rightQIdx: "<<rightQIdx<<std::endl;
        double rightMin = this->robotModel.lowerPositionLimit(rightQIdx);
        double rightMax = this->robotModel.upperPositionLimit(rightQIdx);
        this->rightArmLowerBound.push_back(rightMin);
        this->rightArmUpperBound.push_back(rightMax);

        this->totalLowerBound[rightQIdx] = rightMin;
        this->totalUpperBound[rightQIdx] = rightMax;
    }
    std::cout << "\nTotal Lower Bounds:\n";
    for (size_t i = 0; i < this->totalLowerBound.size(); ++i) {
        std::cout << "Total Lower Bounds[" << i << "] = " << this->totalLowerBound[i] << "\n";
    }

    std::cout << "\nTotal Upper Bounds:\n";
    for (size_t i = 0; i < this->totalUpperBound.size(); ++i) {
        std::cout << "Total Upper Bounds[" << i << "] = " << this->totalUpperBound[i] << "\n";
    }

    assert(armActiveDof.size() == 2);
    assert(armActiveDof[0] <= armDof);
    assert(armActiveDof[1] <= armDof);
    // According to the config , set limitation to the joint
    // Left Arm
    std::cout<<"The current DOF of the left arm is "<<this->armActiveDof[0]<<std::endl;
    for (size_t i = this->armActiveDof[0]; i < this->armDof; i++) {
        size_t temp = this->robotModel.joints[this->leftArmJointIndex[i]].idx_q();
        this->totalLowerBound[temp] = 0.0;
        this->totalUpperBound[temp] = 0.0;
        std::cout << "Set limitation to left arm joint " << temp << std::endl;
    }
    // Right Arm
    std::cout<<"The current DOF of the right arm is "<<this->armActiveDof[1]<<std::endl;
    for (size_t i = this->armActiveDof[1]; i < this->armDof; i++) {
        size_t temp = this->robotModel.joints[this->rightArmJointIndex[i]].idx_q();
        this->totalLowerBound[temp] = 0.0;
        this->totalUpperBound[temp] = 0.0;
        std::cout << "Set limitation to right arm joint " << temp << std::endl;
    }

    // Initial Pose
    // TODO
    // I am unsure that if we need to provide the initial pose
    this->initPose = Eigen::VectorXd::Zero(this->totalDof);

    std::cout<<" The size of the totalBoundsLower is "<<totalLowerBound.size()<<std::endl;
    std::cout<<" The size of the totalBoundsUpper is "<<totalUpperBound.size()<<std::endl;
}

void GenericDualArmSolver::InitOptim()
{
    LOG_FUNCTION;

    // Update robotModelSX
    this->robotModelSX = this->robotModel.cast<casadi::SX>();

    this->baseOffsetSX = this->baseOffset[0].cast<casadi::SX>();

//    pinocchio::DataTpl<casadi::SX> dataSX(this->robotModelSX);
    this->dataPtrSX = std::make_shared<pinocchio::DataTpl<casadi::SX>>(this->robotModelSX);

    // Creating symbolic variables
    casadi::SX targetPoseLeft_ = casadi::SX::sym("targetPoseLeft_",4,4);
    casadi::SX targetPoseRight_ = casadi::SX::sym("targetPoseRight_",4,4);
    casadi::SX qVar_ = casadi::SX::sym("qVar_",this->totalDof);
    pinocchio::DataTpl<casadi::SX>::ConfigVectorType q =
            pinocchio::DataTpl<casadi::SX>::ConfigVectorType::Zero(this->totalDof);
    for(int i =0;i<q.size();i++){
        q(i) = qVar_(i);
    }

    pinocchio::forwardKinematics(robotModelSX, *this->dataPtrSX, q);
    pinocchio::updateFramePlacements(robotModelSX, *this->dataPtrSX);

    // extract matrix
    Eigen::Matrix<casadi::SX,4,4> basePose =
            this->baseOffsetSX * (*this->dataPtrSX).oMf[this->baseFrameIndex].toHomogeneousMatrix();
    Eigen::Matrix<casadi::SX,4,4> leftArmEndPose =
            (*this->dataPtrSX).oMf[this->leftArmEndEffectorFrameIndex].toHomogeneousMatrix();
    Eigen::Matrix<casadi::SX,4,4> rightArmEndPose =
            (*this->dataPtrSX).oMf[this->rightArmEndEffectorFrameIndex].toHomogeneousMatrix();

    Eigen::Matrix<casadi::SX,4,4> leftArmPose = basePose.inverse() * leftArmEndPose;
    Eigen::Matrix<casadi::SX,4,4> rightArmPose = basePose.inverse() * rightArmEndPose;

    casadi::SX basePoseSX = ArmSolver::Eigen2SX(basePose);
    casadi::SX leftArmEndPoseSX = ArmSolver::Eigen2SX(leftArmEndPose);
    casadi::SX rightArmEndPoseSX = ArmSolver::Eigen2SX(rightArmEndPose);

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
            = leftArmPose.block<3,3>(0,0) * ArmSolver::SX2Eigen(targetPoseLeft_).block<3,3>(0,0).transpose();
    Eigen::Matrix<casadi::SX,3,3> rightArmError
            = rightArmPose.block<3,3>(0,0) * ArmSolver::SX2Eigen(targetPoseRight_).block<3,3>(0,0).transpose();

    Eigen::Matrix<casadi::SX,3,1> leftRotaError  = pinocchio::log3(leftArmError).cast<casadi::SX>();
    Eigen::Matrix<casadi::SX,3,1> rightRotaError = pinocchio::log3(rightArmError).cast<casadi::SX>();

    casadi::SX rotaError =
            casadi::SX::sumsqr(casadi::SX::vertcat({
                ArmSolver::Eigen2SX(leftRotaError),
                ArmSolver::Eigen2SX(rightRotaError)
            }));

    this->rotationalError =
            casadi::Function("rotationalError",
                             {qVar_, targetPoseLeft_, targetPoseRight_},
                             {rotaError});

//    casadi::Opti opti;
    this->qVar = this->opti.variable(this->totalDof, 1);
    this->qLast = this->opti.parameter(this->totalDof, 1);
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
        this->wTranslation * translationalCost +
        this->wRotation * rotationalCost +
        this->wSmooth * smoothCost +
        this->wRegularization * regularizationCost;

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

void GenericDualArmSolver::CheckConfig(const ArmSolver::BasicConfig &config)
{
    LOG_FUNCTION;
    if(config.baseFrameName.size() != 1){
        std::string error =
                "[GenericDualArmSolver::CheckConfig] The size of the baseFrameName should be 1! ";
        throw std::length_error(error);
    }

    if(config.targetFrameName.size() != 2){
        std::string error =
                "[GenericDualArmSolver::CheckConfig] The size of the baseFrameName should be 2! And the order is: left arm, right arm! ";
        throw std::length_error(error);
    }

    if(config.baseOffset.size() != 1){
        std::string error =
                "[GenericDualArmSolver::CheckConfig] The size of the baseOffset should be 1! ";
        throw std::length_error(error);
    }

    if(config.targetOffset.size() != 2){
        std::string error =
                "[GenericDualArmSolver::CheckConfig] The size of the targetOffset should be 2! ";
        throw std::length_error(error);
    }

    if(this->leftArmJointNames.size() != this->rightArmJointNames.size()){
        std::string error =
                "[GenericDualArmSolver::CheckConfig] The dof of the left arm is not equal to the dof of the right arm";
        throw std::invalid_argument(error);
    }

//    if(config.initPose.size() != (config.leftArmJointNames.size() + config.rightArmJointNames.size())){
//        std::string error =
//                "[GenericDualArmSolver::CheckConfig] The size of the initPose is wrong! ";
//        throw std::length_error(error);
//    }
}

void GenericDualArmSolver::LoadConfig(const ArmSolver::BasicConfig &config)
{
    this->modelPath = config.modelPath;
    this->robotType = config.robotType;

    this->baseFrameName = config.baseFrameName;
    this->targetFrameName = config.targetFrameName;

    this->baseFrameName = config.baseFrameName;
    this->targetFrameName = config.targetFrameName;

    this->baseOffset = config.baseOffset;
    this->targetOffset = config.targetOffset;

    this->leftArmJointNames = config.leftArmJointNames;
    this->rightArmJointNames = config.rightArmJointNames;

    this->initPose = config.initPose;

    this->maxIteration = config.maxIteration;
    this->relativeTol = config.relativeTol;

    this->armActiveDof = config.armActiveDof;

    this->wRegularization = config.wRegularization;
    this->wTranslation = config.wTranslation;
    this->wRotation = config.wRotation;
    this->wSmooth = config.wSmooth;

}
