#include <ROHandController/ROHandController.hpp>

ROHandController::ROHandController(const HandController::BasicConfig& config_)
{
    if(config_.modbusConfig.device.empty()){
        throw std::invalid_argument("[ROHandController] Plz provide the device! ");
    }

    if(config_.modbusConfig.parity.size() == 1 &&
           (config_.modbusConfig.parity[0] == 'N' ||
            config_.modbusConfig.parity[0] == 'E'  ||
            config_.modbusConfig.parity[0] == 'O') ){
        this->device = config_.modbusConfig.device;
        this->baudrate = config_.modbusConfig.baudrate;
        this->parity = config_.modbusConfig.parity[0];
        this->stopBits = config_.modbusConfig.stopBits;
        this->dataBits = config_.modbusConfig.dataBits;
        this->slaveID = config_.modbusConfig.slaveID;
    }else{
        throw std::invalid_argument("[ROHandController] The parity should be N or E or O! ");
    }

    this->ctx = modbus_new_rtu(this->device.c_str(),
                                   this->baudrate,
                                   this->parity,
                                   this->dataBits,
                                   this->stopBits);
    if (this->ctx == nullptr) {
        throw std::logic_error("[ROHandController] Failed to create modbus RTU context! ");
    }

    // --- 设置 Slave ID 和超时 ---
    modbus_set_slave(this->ctx, this->slaveID);
    modbus_set_response_timeout(this->ctx, 1, 0);   // 1秒响应超时
    modbus_set_byte_timeout(this->ctx, 0, 100000);  // 100ms字节间超时

    std::cout << "[ROHandController] Modbus connected to device " <<
                 device << ", Slave ID: " <<
                 this->slaveID << std::endl;

    if (modbus_connect(this->ctx) == -1) {
        modbus_free(this->ctx);
        std::string error = "[ROHandController] Connection failed: " + static_cast<std::string>(modbus_strerror(errno));
        throw std::logic_error(error);
    }

    this->Init();
}

ROHandController::~ROHandController(){
    if(this->ctx){
    //    std::cout << "[ROHandController] Delete ROHandController!  " << std::endl;
        modbus_close(this->ctx);
        modbus_free(this->ctx);
        this->ctx = nullptr;
        std::cout << "[ROHandController] Modbus closed! " << std::endl;
    }else{
        std::cout << "[ROHandController] Current ctx is nullptr " << std::endl;
    }
}

std::vector<double> ROHandController::GetJointsAngle(){
    std::vector<double> ret;
    // The size of the angle must be defined
    std::vector<uint16_t> angle(this->numJoints);
    // modbus_read_registers(ctx, 起始地址, 寄存器数量, 存储结果的数组)
    int readNum = modbus_read_registers(
        this->ctx,
        ROH_FINGER_ANGLE0,
        this->numJoints,
        angle.data()
    );

    if (readNum == -1) {
        // 读取失败
        std::cerr << "[ROHandController::GetJointsAngle] Modbus read failed : " << modbus_strerror(errno) << "\n";
    } else if (readNum != this->numJoints) {
        // 实际读取数量与请求数量不符
        std::cerr << "[ROHandController::GetJointsAngle] Modbus read incomplete. Expected " << this->numJoints
                  << ", but read " << readNum << "\n";
    } else {
        // 读取成功
//        std::cout << "[ROHandController::GetJointsAngle] Successfully read " << readNum << " finger angle registers:\n";

        for (int i = 0; i < readNum; ++i) {
            uint16_t value = angle[i];

            // 原始值是 100 倍的角度
            double angleDeg = static_cast<double>(value) / 100.0;
            ret.push_back(angleDeg);

            std::cout << "  [Joint( "
                      << this->jointsName[i] << " ) "
                      << i << " at "
                      << (ROH_FINGER_ANGLE0 + i) << "]: "
                      << value << " (Raw) => " <<
                         angleDeg << " degrees\n";
        }
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(1));
    return ret;
}

Eigen::VectorXd ROHandController::GetJointsAngleEigen(){
    auto angle = this->GetJointsAngle();
    Eigen::VectorXd ret = Eigen::VectorXd::Map(angle.data(), angle.size());
    return ret;
}

bool ROHandController::SetJointsAngle(const Eigen::VectorXd& targetValue)
{
    // --- 1. 输入大小检查 ---
    if (targetValue.size() != this->numJoints) {
        std::cerr << "[ROHandController::SetJointsAngle] Wrong target size! Expect "
                  << this->numJoints << ", got " << targetValue.size() << std::endl;
        return false;
    }

    // --- 2. 存放要写入寄存器的原始 uint16 值 ---
    std::vector<uint16_t> rawValues;
    rawValues.resize(this->numJoints);

    // --- 3. Clamp + 转换成寄存器格式 ---
    for (int i = 0; i < this->numJoints; ++i) {
        double lower = this->jointsBoundsLower[i];
        double upper = this->jointsBoundsUpper[i];
        double angle = targetValue[i];

        // 裁剪
        double clamped = std::min(std::max(angle, lower), upper);

        // 记录裁剪日志
        if (angle != clamped) {
            std::cout << "[ROHandController::SetJointsAngle] Joint "
                      << i << "( "
                      << this->jointsName[i] << " ) angle "
                      << angle << " out of bounds, clamped to "
                      << clamped << std::endl;
        }

        // 转成寄存器的整数格式（角度 × 100）
        uint16_t regVal = static_cast<uint16_t>(clamped * 100.0);
        rawValues[i] = regVal;
    }

    // --- 4. 写入 Modbus ---
    int writeNum = modbus_write_registers(
        this->ctx,
        ROH_FINGER_ANGLE_TARGET0,
        this->numJoints,
        rawValues.data()
    );

    std::this_thread::sleep_for(std::chrono::milliseconds(3));

    if (writeNum == -1) {
        std::cerr << "[ROHandController::SetJointsAngle] Modbus write failed: "
                  << modbus_strerror(errno) << std::endl;
        return false;
    }

    if (writeNum != this->numJoints) {
        std::cerr << "[ROHandController::SetJointsAngle] Incomplete write. Expected "
                  << this->numJoints << ", wrote " << writeNum << std::endl;
        return false;
    }

    // --- 5. 写入成功 ---
//    std::cout << "[ROHandController::SetJointsAngle] Successfully set "
//              << writeNum << " joint angle targets.\n";
    return true;
}

bool ROHandController::BackToInitPose(){
    std::cout << "[ROHandController::BackToInitPose] Back to inintial pose " << std::endl;
    return this->SetJointsAngle(this->initPose);
}

void ROHandController::Init(){
    // https://github.com/oymotion/roh_gen2_firmware/blob/main/protocol/OHandModBusRTUProtocol_CN.md

    // ID       Joint   Lower(deg)  Upper(deg)
    // thumb    0       2.26        36.76
    // index    1       100.22      178.37
    // middle   2       97.81       176.06
    // ring     3       101.38      176.54
    // little   4       98.84       174.86
    // thumbRot 5       0           90

//    this->jointsBoundsLower = {2.26,
//                               100.22,
//                               97.81,
//                               101.38,
//                               98.84,
//                               0    };
//    this->jointsBoundsUpper = {36.76,
//                               178.37,
//                               176.06,
//                               176.54,
//                               174.86,
//                               90   };

    // modified
    this->jointsBoundsLower = {2,
                               102,
                               98,
                               102,
                               100,
                               2    };
    this->jointsBoundsUpper = {36,
                               178,
                               176,
                               176,
                               174,
                               88   };

    this->jointsName = {
        "thumb",
        "index",
        "middle",
        "ring",
        "little",
        "thumbRot",
    };

    this->initPose.resize(this->numJoints);
    this->initPose << 36, 178, 176, 176, 174, 2;
}
