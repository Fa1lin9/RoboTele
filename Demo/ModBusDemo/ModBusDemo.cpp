#include <iostream>
#include <modbus/modbus.h>
#include <modbus/modbus-rtu.h>
#include <vector>
#include <errno.h>
#include <thread>

#include <HandController/HandController.hpp>
// 引入上文定义的寄存器地址
// 在实际项目中，应包含 "roh_registers.h"
// 这里为了示例完整性，直接定义关键常量
const uint16_t ROH_FINGER_ANGLE0 = 1165;
const uint16_t ROH_FINGER_ANGLE_TARGET0 = 1155;
const int NUM_FINGER_JOINTS = 6; // 读取 ROH_FINGER_ANGLE0 到 ROH_FINGER_ANGLE9

int main()
{
//    const char* device = "/dev/ttyCH341USB0"; // 串口
////    const char* device = "/dev/ttyCH341USB1"; // 串口
//    int baud = 115200;
//    char parity = 'N';                       // N,E,O
//    int data_bits = 8;
//    int stop_bits = 1;
//    const int slave_id = 3;
////    const int slave_id = 2;

//    // --- 1. 创建 RTU 上下文 ---
//    modbus_t* ctx = modbus_new_rtu(device, baud, parity, data_bits, stop_bits);
//    if (ctx == nullptr) {
//        std::cerr << "Failed to create modbus RTU context: " << modbus_strerror(errno) << "\n";
//        return -1;
//    }

//    // --- 2. 设置 Slave ID 和超时 ---
//    modbus_set_slave(ctx, slave_id);
//    modbus_set_response_timeout(ctx, 1, 0);   // 1秒响应超时
//    modbus_set_byte_timeout(ctx, 0, 100000);  // 100ms字节间超时

//    // --- 3. 连接 ---
//    if (modbus_connect(ctx) == -1) {
//        std::cerr << "Connection failed: " << modbus_strerror(errno) << "\n";
//        modbus_free(ctx);
//        return -1;
//    }

//    std::cout << "Modbus connected to device " << device << ", Slave ID: " << slave_id << "\n";

//    // --- 4. 读取当前关节角值 ---
//    std::vector<uint16_t> angle_registers(NUM_FINGER_JOINTS);

//    std::cout << "\nAttempting to read " << NUM_FINGER_JOINTS << " registers starting at address "
//              << ROH_FINGER_ANGLE0 << "...\n";

//    // modbus_read_holding_registers(ctx, 起始地址, 寄存器数量, 存储结果的数组)
//    int num_read = modbus_read_registers(
//        ctx,
//        ROH_FINGER_ANGLE0,
//        NUM_FINGER_JOINTS,
//        angle_registers.data()
//    );

//    if (num_read == -1) {
//        // 读取失败
//        std::cerr << " Modbus read failed (Function 0x03): " << modbus_strerror(errno) << "\n";
//    } else if (num_read != NUM_FINGER_JOINTS) {
//        // 实际读取数量与请求数量不符
//        std::cerr << "⚠️ Modbus read incomplete. Expected " << NUM_FINGER_JOINTS
//                  << ", but read " << num_read << "\n";
//    } else {
//        // 读取成功
//        std::cout << "✅ Successfully read " << num_read << " finger angle registers:\n";

//        for (int i = 0; i < num_read; ++i) {
//            uint16_t raw_value = angle_registers[i];

//            // 假设：根据您之前 Python 脚本的线索，原始值可能是角度的100倍或180倍等，这里仅做示例转换。
//            // 假设原始值是 100 倍的角度（例如：值 1800 表示 18.00 度）
//            double angle_in_degrees = static_cast<double>(raw_value) / 100.0;

//            std::cout << "  [ANGLE " << i << " @ " << (ROH_FINGER_ANGLE0 + i) << "]: "
//                      << raw_value << " (Raw) => " << angle_in_degrees << " degrees\n";
//        }
//    }

//    for(size_t i=0;i<angle_registers.size();i++){
//        std::cout<<"Angle["<<i<<"]: "<<angle_registers[i]<<std::endl;
//    }

//    sleep(2);
//    // set value
//    angle_registers[0] = 3600;
//    angle_registers[1] = 17800;
//    angle_registers[2] = 17600;
//    angle_registers[3] = 17600;
//    angle_registers[4] = 17400;
//    angle_registers[5] = 200;

//    int num_write = modbus_write_registers(
//        ctx,
//        ROH_FINGER_ANGLE_TARGET0,
//        NUM_FINGER_JOINTS,
//        angle_registers.data()
//    );
//    std::this_thread::sleep_for(std::chrono::milliseconds(5));
//    if (num_write == -1) {
//        std::cerr << " Modbus write failed : " << modbus_strerror(errno) << "\n";
//    }

//    // --- 5. 关闭连接 ---
//    modbus_close(ctx);
//    modbus_free(ctx);

//    std::cout << "\nModbus closed.\n";
//    return 0;

    HandController::ModBusConfig modbusConfig = {
        .device = "/dev/ttyCH341USB0",
        .baudrate = 115200,
        .parity = "N",
        .dataBits = 8,
        .stopBits = 1,
        .slaveID = 3,
    };

//    HandController::ModBusConfig modbusConfig = {
//        .device = "/dev/ttyCH341USB1",
//        .baudrate = 115200,
//        .parity = "N",
//        .dataBits = 8,
//        .stopBits = 1,
//        .slaveID = 2,
//    };

    HandController::BasicConfig basicConfig = {
        .type = HandController::Type::ROHand,
        .modbusConfig = modbusConfig,
    };

    auto handCtrl = HandController::GetPtr(basicConfig);

    Eigen::VectorXd angle = handCtrl->GetJointsAngleEigen();
//    std::cout << "Current Angle: " << angle << std::endl;

    sleep(1);
    std::cout << "Start to move! " << std::endl;
    angle(0) = 20;
    angle(1) = 150;
    angle(2) = 150;
    handCtrl->SetJointsAngle(angle);
    std::cout << "Move over! " << std::endl;

    sleep(1);
    handCtrl->BackToInitPose();

}
