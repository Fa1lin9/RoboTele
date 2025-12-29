#include <Ti5DualArmSolver/Ti5DualArmSolver.hpp>

Ti5DualArmSolver::Ti5DualArmSolver(const ArmSolver::BasicConfig &config_)
    :maxIteration(config_.maxIteration),
     relativeTol(config_.relativeTol)
{
    LOG_FUNCTION;
    // the initialization of the model must be first
    pinocchio::urdf::buildModel(
                this->modelPath,
                this->robotModel);

    this->robotModelSX = this->robotModel.cast<casadi::SX>();

    // As For CrpRobot
    // The size of the baseFrameName should be 1
    // The size of the targetFrameName should be 2, and as the order: left arm , right arm
    if(config_.baseFrameName.size() != 1){
        std::string error = " As for Ti5RobotIKSolver,the size of the baseFrameName should be 1! ";
        throw std::length_error(error);
    }else{
        this->baseIndex = this->robotModel.getFrameId(config_.baseFrameName[0]);
    }

    if(config_.targetFrameName.size() != 2){
        std::string error = " As for Ti5RobotIKSolver,the size of the baseFrameName should be 2! And the order is: left arm, right arm! ";
        throw std::length_error(error);
    }else{
        this->leftArmEndIndex = this->robotModel.getFrameId(config_.targetFrameName[0]);
        this->rightArmEndIndex = this->robotModel.getFrameId(config_.targetFrameName[1]);
    }

    if(config_.baseOffset.size() != 1){
        std::string error = " As for Ti5RobotIKSolver,the size of the baseOffset should be 1! ";
        throw std::length_error(error);
    }else{
        this->baseOffset = config_.baseOffset[0];
        this->baseOffsetSX = this->baseOffset.cast<casadi::SX>();
    }

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

Ti5DualArmSolver::~Ti5DualArmSolver()
{

}

boost::optional<Eigen::VectorXd> Ti5DualArmSolver::Solve(
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

    Ti5DualArmSolver::SolverType solverType = SolverType::Casadi;

    switch (solverType) {
        case SolverType::Casadi :{
        // set value
        // qInit
        std::vector<double> qInitVec(qLast_.data(), qLast_.data() + qLast_.size());
        this->opti.set_value(this->qLast, casadi::DM(qInitVec));


        // targetPose
    //    std::vector<double> targetPoseLeftVec(targetPose[0].data(), targetPose[0].data() + targetPose[0].size());
    //    casadi::DM targetPoseLeftDM = casadi::DM(targetPoseLeftVec);
    //    targetPoseLeftDM = casadi::DM::reshape(targetPoseLeftDM, targetPose[0].rows(), targetPose[0].cols());
    //    this->opti.set_value(this->targetPoseLeft, targetPoseLeftDM);

    //    std::vector<double> targetPoseRightVec(targetPose[1].data(), targetPose[1].data() + targetPose[1].size());
    //    casadi::DM targetPoseRightDM = casadi::DM(targetPoseRightVec);
    //    targetPoseRightDM = casadi::DM::reshape(targetPoseRightDM, targetPose[1].rows(), targetPose[1].cols());
    //    this->opti.set_value(this->targetPoseRight, targetPoseRightDM);
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
        case SolverType::Nlopt :{
            // time consumed a lot
            this->InitAD(targetPose,qLast_);

            nlopt::opt opt;

            double (*ObjectWrapper)(const std::vector<double>& x,std::vector<double>& grad,void *data);

            // update casadi variable
        //    this->qInit = qInit.cast<casadi::SX>();
        //    for(size_t i=0;i<this->targetPose.size();i++){
        //        this->targetPose[i] = targetPose[i].cast<casadi::SX>();
        //    }
            bool useGrad = 1;
            if(useGrad){
                opt = nlopt::opt(nlopt::GD_STOGO , qLast_.size());
                ObjectWrapper = [](const std::vector<double>& x,std::vector<double>& grad,void *data)->double{
                    Eigen::Map<const Eigen::VectorXd> q(x.data(),x.size());
                    Ti5RobotData *robotData = static_cast<Ti5RobotData*>(data);
                    ArmSolver::Ti5RobotConfig config = {
                        .q = q,
                        .qInit= robotData->qInit,
                        .targetPose = robotData->targetPose,
                    };

                    casadi::DM qVar = casadi::DM(x);
                    grad.resize(robotData->solver->totalDof);
                    std::vector<casadi::DM> output = robotData->solver->mainFunc({qVar});

                    double objectiveFunc = double(output[0]);
                    casadi::DM gradFunc = output[1];
                    std::transform(
                                gradFunc->begin(), gradFunc->end(), grad.begin(),
                                [](const auto& v){ return double(v); }
                    );

                    return objectiveFunc;
                };
            }else{
                opt = nlopt::opt(nlopt::GN_DIRECT_L , qLast_.size());
                ObjectWrapper = [](const std::vector<double>& x,std::vector<double>& grad,void *data)->double{
                    Eigen::Map<const Eigen::VectorXd> q(x.data(),x.size());
                    Ti5RobotData *robotData = static_cast<Ti5RobotData*>(data);
                    ArmSolver::Ti5RobotConfig config = {
                        .q = q,
                        .qInit= robotData->qInit,
                        .targetPose = robotData->targetPose,
                    };

                    return robotData->solver->ObjectiveFunc(config);
                };
            }

            // set bounds
            opt.set_lower_bounds(this->totalLowerBound);
            opt.set_upper_bounds(this->totalUpperBound);

            opt.set_maxeval(this->maxIteration);
            opt.set_xtol_rel(this->relativeTol);

            Ti5RobotData robotData = {
                .solver = this,
                .qInit = qLast_,
                .targetPose = targetPose,
            };

            opt.set_min_objective(ObjectWrapper, &robotData);

            double funcValue;

            std::vector<double> q(this->totalDof);
//            q = this->qNeutral;


            auto start = std::chrono::high_resolution_clock::now();

            nlopt::result result = opt.optimize(q, funcValue);
            auto end = std::chrono::high_resolution_clock::now();
            auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
            std::cout << " optimazation 耗时: " << duration.count() << " ms" << std::endl;
            std::cout << " Function Value = " << funcValue << std::endl;

            if(result<0){
                std::string error = " Optimize failed! ";
                throw std::logic_error(error);

            }

            Eigen::Map<Eigen::VectorXd> qEigen(q.data(),q.size());

            if(verbose){
                // check result
                std::cout<<"------------ Solver Result ------------"<<std::endl;
                std::cout << " Joint Value =\n" << qEigen << std::endl;
                std::cout << " Function Value = " << funcValue << std::endl;
                std::cout<<" Left Arm Translation: \n"<<Forward(qEigen)[0].translation()<<std::endl;
                std::cout<<" Left Arm Rotation: \n"<<Forward(qEigen)[0].rotation()<<std::endl;
                std::cout<<" Right Arm Translation: \n"<<Forward(qEigen)[1].translation()<<std::endl;
                std::cout<<" Rigit Arm Rotation: \n"<<Forward(qEigen)[1].rotation()<<std::endl;
                std::cout<<"------------ Solver Result ------------"<<std::endl;
            }

            return boost::optional<Eigen::VectorXd>(qEigen);
        }
        default:{
            return boost::none;
        }
    }
}

void Ti5DualArmSolver::Info(){
    LOG_FUNCTION;

    std::cout << " ----------------------------------------------------- "<< std::endl;

    // 注意 robotModel.joints包含了全部关节，即包含了初始关节
    // 而整个机器人的自由度是指其可动的关节
    // 所以打印的时候我们可以看到总关节是22，但是DOF确实21
    // 因此，在我们打印相关关节的上下界的时候要注意索引的匹配

//    for(pinocchio::JointIndex i=1; i<robotModel.njoints; ++i){
//        std::cout << "Joint " << i << ": " << robotModel.names[i]
//                  << ", type: " << robotModel.joints[i].shortname()
//                  << ", parent: " << robotModel.parents[i] << std::endl;


//        std::cout << " lower limit: " << robotModel.lowerPositionLimit[i-1]
//                  << ", upper limit: " << robotModel.upperPositionLimit[i-1]<<std::endl;
//        std::cout<<std::endl;

//    }

    std::cout << " ----------------------------------------------------- "<< std::endl;

//    std::cout << "Size of joints upper limitaion: " << robotModel.upperPositionLimit.rows() << std::endl;
    std::cout << "Number of joints: " << robotModel.njoints << std::endl;
    std::cout << "Number of DOFs: " << robotModel.nv << std::endl;
    std::cout << "Number of frames: " << robotModel.nframes << std::endl;
//    for(size_t i =0;i<robotModel.nframes;i++){
//        std::cout<<" Frame "<<i<<" : "<<robotModel.frames[i]<<std::endl;
//    }
    // universe 1
    // BASE_S 1
    // JOINT and its JOINT_S 21 * 2 = 42
    // Total 44
    std::cout << "Number of q: " << robotModel.nq << std::endl;

    // Demo
    Eigen::VectorXd q;
//    q.setZero(21);
    q.setOnes(21);
    q[0] = 0;
    q.tail(20) *= 0.1;

    auto start = std::chrono::high_resolution_clock::now();

    pinocchio::Data data=pinocchio::Data(robotModel);
    pinocchio::forwardKinematics(robotModel,data,q);
    pinocchio::updateFramePlacements(robotModel,data);

    // For Left Arm
    pinocchio::JointIndex leftArmStart = robotModel.getJointId("L_SHOULDER_P");
    pinocchio::JointIndex leftArmEnd  = robotModel.getJointId("L_WRIST_R");

    pinocchio::SE3 leftShoulderPitchPose = data.oMi[leftArmStart];
    pinocchio::SE3 leftWaistRollPose  = data.oMi[leftArmEnd];

    pinocchio::SE3 leftArmPose = leftShoulderPitchPose.inverse() * leftWaistRollPose;

    // For Right Arm
    pinocchio::JointIndex rightArmStart = robotModel.getJointId("R_SHOULDER_P");
    pinocchio::JointIndex rightArmEnd= robotModel.getJointId("R_WRIST_R");

    pinocchio::SE3 rightShoulderPitchPose = data.oMi[rightArmStart];
    pinocchio::SE3 rightWaistRollPose  = data.oMi[rightArmEnd];

    pinocchio::SE3 rightArmPose = rightShoulderPitchPose.inverse() * rightWaistRollPose;

//    // 打印平移向量
//    std::cout << "leftArmPose position: " << leftArmPose.translation().transpose() << std::endl;
//    std::cout << "rightArmPose position: "  << rightArmPose.translation().transpose() << std::endl;

//    // 打印旋转矩阵
//    std::cout << "leftArmPose rotation:\n" << leftArmPose.rotation() << std::endl;
//    std::cout << "rightArmPose rotation:\n"  << rightArmPose.rotation() << std::endl;

    auto end = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
//    std::cout << "耗时: " << duration.count() << " ms" << std::endl;
//    Eigen::VectorXd testQ = Eigen::VectorXd::Zero(dofTotal);

//    testQ.segment(4, dofArm) << -2.09, -0.5236, 0.698, -1, 2.09, 0, -0.733;
//    testQ.segment(4, dofArm) << -0.2, -1.22, -2.09, -1.37, 2.79, 1.24, -0.0;

//    std::cout << " translations of left arm: \n"<<Forward(testQ)[0].translation()<<std::endl;
//    std::cout << " rotation of left arm: \n"<<Forward(testQ)[0].rotation()<<std::endl;
}

std::vector<pinocchio::SE3> Ti5DualArmSolver::Forward(const Eigen::VectorXd& q){
//    LOG_FUNCTION;
    if(q.size() != this->robotModel.nq){
        std::string error = " The size of the q should be this->robotModel.nq! ";
        throw std::length_error(error);
    }

    // updata data to better get position
    pinocchio::Data data=pinocchio::Data(robotModel);
    pinocchio::forwardKinematics(robotModel,data,q);
    pinocchio::updateFramePlacements(robotModel,data);

    // base pose
    pinocchio::SE3 BasePoseOffset = pinocchio::SE3(
                this->baseOffset.block<3,3>(0,0),
                this->baseOffset.block<3,1>(0,3));
    pinocchio::SE3 basePose = data.oMf[this->baseIndex];

//    std::cout<<"BasePose Translation: "<<basePose.translation()<<std::endl;
//    std::cout<<"BasePose Rotation: "<<basePose.rotation()<<std::endl;

    basePose = BasePoseOffset * basePose;

    // left arm
    pinocchio::SE3 leftArmEndPose = data.oMf[this->leftArmEndIndex];
    pinocchio::SE3 leftArmPose = basePose.inverse() * leftArmEndPose;


    // right arm
    pinocchio::SE3 rightArmEndPose = data.oMf[this->rightArmEndIndex];
    pinocchio::SE3 rightArmPose = basePose.inverse() * rightArmEndPose;

    return std::vector<pinocchio::SE3>{leftArmPose, rightArmPose};
}

size_t Ti5DualArmSolver::GetTotalDof(){
    return this->totalDof;
}

std::vector<std::string> Ti5DualArmSolver::GetJointNames(){
    return this->jointNames;
}

double Ti5DualArmSolver::ObjectiveFunc(const ArmSolver::Ti5RobotConfig& config_){
//    LOG_FUNCTION;
    std::vector<pinocchio::SE3> currentPose = this->Forward(config_.q);

    // translation error
//    std::cout<<" Translation Error "<<std::endl;
    Eigen::Vector3d currentLeftTrans  = currentPose[0].translation();
    Eigen::Vector3d currentRightTrans = currentPose[1].translation();
    Eigen::Vector3d targetLeftTrans   = config_.targetPose[0].block<3,1>(0,3);
    Eigen::Vector3d targetRightTrans  = config_.targetPose[1].block<3,1>(0,3);

    Eigen::VectorXd currentTrans(6), targetTrans(6);
    currentTrans << currentLeftTrans, currentRightTrans;
    targetTrans  << targetLeftTrans, targetRightTrans;

    Eigen::VectorXd transErrorVec = currentTrans - targetTrans;
    double transError = transErrorVec.squaredNorm();

    // rotation error
//    std::cout<<" rotation Error "<<std::endl;
    Eigen::Matrix3d leftArmError
            = config_.targetPose[0].block<3,3>(0,0) * currentPose[0].rotation().inverse();
    Eigen::Matrix3d rightArmError
            = config_.targetPose[1].block<3,3>(0,0) * currentPose[1].rotation().inverse();

    Eigen::VectorXd rotaErrorVec(6);
    rotaErrorVec << pinocchio::log3(leftArmError), pinocchio::log3(rightArmError);
    double rotaError = rotaErrorVec.squaredNorm();

    // smoothing error
//    std::cout<<" Smoothing Error "<<std::endl;
    Eigen::VectorXd smoothErrorVec = config_.q - config_.qInit;
    MatrixUtils::WrapAngleToPi(smoothErrorVec);
    double smoothError = smoothErrorVec.squaredNorm();

    // regularization
//    std::cout<<" Regularization "<<std::endl;
    Eigen::VectorXd reguVec;
    reguVec =
            config_.q - Eigen::VectorXd::Map(this->qNeutral.data(), this->qNeutral.size());
    double reguError = reguVec.squaredNorm();

    // weight
    double wTrans = 50.0;
    double wRota = 0.5;
    double wSmooth = 0.1;
    double wRegu = 0.02;

    double error = wTrans * transError + wRota * rotaError + wSmooth * smoothError + wRegu * reguError;
    return error;
}

void Ti5DualArmSolver::InitRobot(const ArmSolver::BasicConfig &config_){
//    LOG_FUNCTION;
    // Init Model
//    pinocchio::urdf::buildModel(
//                this->modelPath,
//                this->robotModel);

//    this->robotModelSX = this->robotModel.cast<casadi::SX>();

    // Load JointNames
    for(pinocchio::JointIndex i=1; i<robotModel.njoints; ++i){
        std::cout << "Joint " << i << ": " << robotModel.names[i]
                  << ", type: " << robotModel.joints[i].shortname()
                  << ", parent: " << robotModel.parents[i] << std::endl;
        this->jointNames.push_back(robotModel.names[i]);
    }

    double min,max;
    for(size_t i = 0;i<this->armDof;i++){
        // left arm
        min = this->robotModel.lowerPositionLimit(this->leftArmID[0] + i - 1);
        max = this->robotModel.upperPositionLimit(this->leftArmID[0] + i - 1);
        this->leftArmLowerBound.push_back(min);
        this->leftArmUpperBound.push_back(max);
        this->qLeftArmNeutral.push_back((min + max) / 2);

        // right arm
        min = this->robotModel.lowerPositionLimit(this->rightArmID[0] + i - 1);
        max = this->robotModel.upperPositionLimit(this->rightArmID[0] + i - 1);
        this->rightArmLowerBound.push_back(min);
        this->rightArmUpperBound.push_back(max);
        this->qRightArmNeutral.push_back((min + max) / 2);
    }

    // order: base(1) - waist(3) - left arm(dofArm) - neck(3) - right arm(dofArm)
    qNeutral.clear();
    qNeutral.reserve(this->totalDof);

    // base
    qNeutral.push_back(0);
    totalLowerBound.push_back(0);
    totalUpperBound.push_back(0);

    // waist
    qNeutral.insert(qNeutral.end(), 3, 0);
    totalLowerBound.insert(totalLowerBound.end(), 3, 0);
    totalUpperBound.insert(totalUpperBound.end(), 3, 0);

    // left arm
    qNeutral.insert(qNeutral.end(), qLeftArmNeutral.begin(), qLeftArmNeutral.end());
    totalLowerBound.insert(totalLowerBound.end(), leftArmLowerBound.begin(), leftArmLowerBound.end());
    totalUpperBound.insert(totalUpperBound.end(), leftArmUpperBound.begin(), leftArmUpperBound.end());

    // neck
    qNeutral.insert(qNeutral.end(), 3, 0);
    totalLowerBound.insert(totalLowerBound.end(), 3, 0);
    totalUpperBound.insert(totalUpperBound.end(), 3, 0);

    // right arm
    qNeutral.insert(qNeutral.end(), qRightArmNeutral.begin(), qRightArmNeutral.end());
    totalLowerBound.insert(totalLowerBound.end(), rightArmLowerBound.begin(), rightArmLowerBound.end());
    totalUpperBound.insert(totalUpperBound.end(), rightArmUpperBound.begin(), rightArmUpperBound.end());

    // according to the config , set limitation to the joint
    // Left Arm
    std::cout<<"The current DOF of the left arm is "<<config_.armActiveDof[0]<<std::endl;
    for(size_t i=0;i<this->armDof - config_.armActiveDof[0];i++){
        size_t temp = this->leftArmID.back() - 1 - i;
        this->totalLowerBound[temp] = 0;
        this->totalUpperBound[temp] = 0;
        std::cout<<"Set limitation to left arm joint"<<temp<<std::endl;
    }

    std::cout<<"The current DOF of the right arm is "<<config_.armActiveDof[1]<<std::endl;
    for(size_t i=0;i<this->armDof - config_.armActiveDof[1];i++){
        size_t temp = this->rightArmID.back() - 1 - i;
        this->totalLowerBound[temp] = 0;
        this->totalUpperBound[temp] = 0;
        std::cout<<"Set limitation to right arm joint"<<temp<<std::endl;
    }

    // Initial Pose
    this->initPose = Eigen::VectorXd::Zero(21);

    // Initial Pose For Real Robot
//    this->initPose.segment(4,7) << -0.72, -1.0, 0.57, -1.0, 0.83, 0, 0;
//    this->initPose.segment(14,7) << 0.72, 1.0, -0.57, 1.0, -0.83, 0, 0;

    this->initPose.segment(4,7) << -0.72, -1.0, -1.0, -1.0, 0.83, 0, 0;
    this->initPose.segment(14,7) << 0.72, 1.0, 1.0, 1.0, -0.83, 0, 0;

//    std::cout<<" The size of the totalBoundsLower is "<<totalBoundsLower.size()<<std::endl;
//    std::cout<<" The size of the totalBoundsUpper is "<<totalBoundsUpper.size()<<std::endl;
//    std::cout<<" The size of the qNeutral is "<<qNeutral.size()<<std::endl;

}

void Ti5DualArmSolver::InitOptim(const ArmSolver::BasicConfig &config_){
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
            this->baseOffsetSX * (*this->dataPtrSX).oMf[this->baseIndex].toHomogeneousMatrix();
    Eigen::Matrix<casadi::SX,4,4> leftArmEndPose = (*this->dataPtrSX).oMf[this->leftArmEndIndex].toHomogeneousMatrix();
    Eigen::Matrix<casadi::SX,4,4> rightArmEndPose = (*this->dataPtrSX).oMf[this->rightArmEndIndex].toHomogeneousMatrix();

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
//    this->regularizationCost = casadi::MX::sumsqr(qVar - casadi::MX(initPoseVec));
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

void Ti5DualArmSolver::InitAD(const std::vector<Eigen::Matrix4d>& targetPose_,
                                    const Eigen::VectorXd& qInit_){
    // Variable
    casadi::SX qVar = casadi::SX::sym("qVar",this->totalDof,1);

    pinocchio::DataTpl<casadi::SX>::ConfigVectorType q =
            pinocchio::DataTpl<casadi::SX>::ConfigVectorType::Zero(this->totalDof);

    std::vector<Eigen::Matrix<casadi::SX,4,4>> targetPose(targetPose_.size());
    for(size_t i=0;i<targetPose_.size();i++){
        targetPose[i] = targetPose_[i].cast<casadi::SX>();
    }

    Eigen::Matrix<casadi::SX,Eigen::Dynamic,1> qInit = qInit_.cast<casadi::SX>();

    for(int i =0;i<q.size();i++){
        q(i) = qVar(i);
    }

    casadi::SX costFunc = this->ObjectiveFuncSX(q,qInit,targetPose);
    casadi::SX gradFunc = gradient(costFunc,qVar);
    this->mainFunc = casadi::Function("mainFunc", {qVar}, {costFunc, gradFunc});
}

casadi::SX Ti5DualArmSolver::ObjectiveFuncSX(
            const pinocchio::ModelTpl<casadi::SX>::ConfigVectorType& q,
            const Eigen::Matrix<casadi::SX,Eigen::Dynamic,1>& qInit,
            const std::vector<Eigen::Matrix<casadi::SX,4,4>>& targetPose){
//    if(q.size() != this->robotModel.nq){
//        std::string error = " The size of the q should be this->robotModel.nq! ";
//        throw std::length_error(error);
//    }
//    if(targetPose.size() != 2){
//        std::string error = " The size of the targetPose should be 2! ";
//        throw std::length_error(error);
//    }

//    // updata data to better get position
//    pinocchio::DataTpl<casadi::SX> dataSX(robotModelSX);
//    forwardKinematics(robotModelSX, dataSX, q);
//    updateFramePlacements(robotModelSX, dataSX);

//    // extract matrix
//    Eigen::Matrix<casadi::SX,4,4> basePose =
//            this->baseOffsetSX * dataSX.oMf[this->baseIndex].toHomogeneousMatrix();
//    Eigen::Matrix<casadi::SX,4,4> leftArmEndPose = dataSX.oMf[this->leftArmEndIndex].toHomogeneousMatrix();
//    Eigen::Matrix<casadi::SX,4,4> rightArmEndPose = dataSX.oMf[this->rightArmEndIndex].toHomogeneousMatrix();

//    Eigen::Matrix<casadi::SX,4,4> leftArmPose = basePose.inverse() * leftArmEndPose;
//    Eigen::Matrix<casadi::SX,4,4> rightArmPose = basePose.inverse() * rightArmEndPose;

//    casadi::SX basePoseSX =
//            casadi::SX::mtimes(ArmSolver::Eigen2SX(this->baseOffsetSX),
//                               ArmSolver::Eigen2SX(basePose));
//    casadi::SX leftArmEndPoseSX = ArmSolver::Eigen2SX(leftArmEndPose);
//    casadi::SX rightArmEndPoseSX = ArmSolver::Eigen2SX(rightArmEndPose);

//    casadi::SX basePoseInvSX = casadi::SX::inv(basePoseSX);
//    casadi::SX leftArmPoseSX = casadi::SX::mtimes(basePoseInvSX, leftArmEndPoseSX);
//    casadi::SX rightArmPoseSX = casadi::SX::mtimes(basePoseInvSX, rightArmEndPoseSX);

//    // translation error
////    std::cout<<" Translation Error "<<std::endl;
//    casadi::SX currentLeftTrans  = leftArmPoseSX(casadi::Slice(0,3), 3);
//    casadi::SX currentRightTrans = rightArmPoseSX(casadi::Slice(0,3), 3);
//    casadi::SX targetLeftTrans   = ArmSolver::Eigen2SX(targetPose[0])(casadi::Slice(0,3), 3);
//    casadi::SX targetRightTrans  = ArmSolver::Eigen2SX(targetPose[1])(casadi::Slice(0,3), 3);

//    casadi::SX currentTrans = casadi::SX::vertcat({currentLeftTrans, currentRightTrans});
//    casadi::SX targetTrans  = casadi::SX::vertcat({targetLeftTrans,  targetRightTrans});
//    casadi::SX transError   = casadi::SX::sumsqr(currentTrans - targetTrans);

//    // rotation error
////    std::cout<<" rotation Error "<<std::endl;
//    Eigen::Matrix<casadi::SX,3,3> leftArmError
//            = leftArmPose.block<3,3>(0,0) * targetPose[0].block<3,3>(0,0).transpose();
//    Eigen::Matrix<casadi::SX,3,3> rightArmError
//            = rightArmPose.block<3,3>(0,0) * targetPose[1].block<3,3>(0,0).transpose();

//    Eigen::Matrix<casadi::SX,3,1> leftRotError  = pinocchio::log3(leftArmError).cast<casadi::SX>();
//    Eigen::Matrix<casadi::SX,3,1> rightRotError = pinocchio::log3(rightArmError).cast<casadi::SX>();

//    Eigen::Matrix<casadi::SX,6,1> rotaErrorVec;
//    rotaErrorVec.block<3,1>(0,0) = leftRotError;
//    rotaErrorVec.block<3,1>(3,0) = rightRotError;

//    casadi::SX rotaError = rotaErrorVec.squaredNorm();

//    // smoothing error
////    std::cout<<" Smoothing Error "<<std::endl;
//    pinocchio::DataTpl<casadi::SX>::ConfigVectorType smoothErrorVec =
//            q - qInit.cast<casadi::SX>();
//    casadi::SX smoothError = smoothErrorVec.squaredNorm();

//    // regularization
////    std::cout<<" Regularization "<<std::endl;
//    pinocchio::DataTpl<casadi::SX>::ConfigVectorType reguVec =
//            q - Eigen::VectorXd::Map(this->qNeutral.data(), this->qNeutral.size()).cast<casadi::SX>();
//    casadi::SX reguError = reguVec.squaredNorm();

//    // weight
//    double wTrans = 50.0;
//    double wRota = 0.5;
//    double wSmooth = 0.1;
//    double wRegu = 0.02;
////    double wSmooth = 0;
////    double wRegu = 0;

//    casadi::SX error =  casadi::SX(wTrans)  * transError +
//                        casadi::SX(wRota)   * rotaError +
//                        casadi::SX(wSmooth) * smoothError +
//                        casadi::SX(wRegu)   * reguError;
//    return error;

    if(q.size() != this->robotModel.nq){
        std::string error = " The size of the q should be this->robotModel.nq! ";
        throw std::length_error(error);
    }
    if(targetPose.size() != 2){
        std::string error = " The size of the targetPose should be 2! ";
        throw std::length_error(error);
    }

    // updata data to better get position
    pinocchio::DataTpl<casadi::SX> dataSX(robotModelSX);
    forwardKinematics(robotModelSX, dataSX, q);
    updateFramePlacements(robotModelSX, dataSX);

    // extract matrix
    Eigen::Matrix<casadi::SX,4,4> basePose =
            this->baseOffsetSX * dataSX.oMf[this->baseIndex].toHomogeneousMatrix();
    Eigen::Matrix<casadi::SX,4,4> leftArmEndPose = dataSX.oMf[this->leftArmEndIndex].toHomogeneousMatrix();
    Eigen::Matrix<casadi::SX,4,4> rightArmEndPose = dataSX.oMf[this->rightArmEndIndex].toHomogeneousMatrix();

    Eigen::Matrix<casadi::SX,4,4> leftArmPose = basePose.inverse() * leftArmEndPose;
    Eigen::Matrix<casadi::SX,4,4> rightArmPose = basePose.inverse() * rightArmEndPose;

//    Eigen::Matrix<casadi::SX,4,4> leftArmPose = this->BaseOffsetAD * leftArmEndPose;
//    Eigen::Matrix<casadi::SX,4,4> rightArmPose = this->BaseOffsetAD * rightArmEndPose;

    // translation error
//    std::cout<<" Translation Error "<<std::endl;
    Eigen::Matrix<casadi::SX,3,1> currentLeftTrans  = leftArmPose.block<3,1>(0,3);
    Eigen::Matrix<casadi::SX,3,1> currentRightTrans = rightArmPose.block<3,1>(0,3);
    Eigen::Matrix<casadi::SX,3,1> targetLeftTrans   = targetPose[0].block<3,1>(0,3);
    Eigen::Matrix<casadi::SX,3,1> targetRightTrans  = targetPose[1].block<3,1>(0,3);

    casadi::SX transError =
            casadi::SX::sumsqr(casadi::SX::vertcat({
               (ArmSolver::Eigen2SX(currentLeftTrans) - ArmSolver::Eigen2SX(targetLeftTrans)),
               (ArmSolver::Eigen2SX(currentRightTrans) - ArmSolver::Eigen2SX(targetRightTrans)),
           }));

    // rotation error
//    std::cout<<" rotation Error "<<std::endl;
    Eigen::Matrix<casadi::SX,3,3> leftArmError
            = leftArmPose.block<3,3>(0,0) * targetPose[0].block<3,3>(0,0).transpose();
    Eigen::Matrix<casadi::SX,3,3> rightArmError
            = rightArmPose.block<3,3>(0,0) * targetPose[1].block<3,3>(0,0).transpose();

    Eigen::Matrix<casadi::SX,3,1> leftRotaError  = pinocchio::log3(leftArmError).cast<casadi::SX>();
    Eigen::Matrix<casadi::SX,3,1> rightRotaError = pinocchio::log3(rightArmError).cast<casadi::SX>();

    casadi::SX rotaError =
            casadi::SX::sumsqr(casadi::SX::vertcat({
                ArmSolver::Eigen2SX(leftRotaError),
                ArmSolver::Eigen2SX(rightRotaError)
            }));

    // smoothing error
//    std::cout<<" Smoothing Error "<<std::endl;
    pinocchio::DataTpl<casadi::SX>::ConfigVectorType smoothErrorVec =
            q - qInit.cast<casadi::SX>();
    casadi::SX smoothError = smoothErrorVec.squaredNorm();

    // regularization
//    std::cout<<" Regularization "<<std::endl;
    pinocchio::DataTpl<casadi::SX>::ConfigVectorType reguVec =
            q - Eigen::VectorXd::Map(this->qNeutral.data(), this->qNeutral.size()).cast<casadi::SX>();
//    casadi::SX reguError = reguVec.squaredNorm();
    casadi::SX reguError = q.squaredNorm();

    // weight
    double wTrans = 50.0;
    double wRota = 0.5;
    double wSmooth = 0.1;
    double wRegu = 0.02;
//    double wSmooth = 0;
//    double wRegu = 0;

    casadi::SX error =  casadi::SX(wTrans)  * transError +
                        casadi::SX(wRota)   * rotaError +
                        casadi::SX(wSmooth) * smoothError +
                        casadi::SX(wRegu)   * reguError;
    return error;
}
