#include <CanDriver/CanDriver.h>

#include <cassert>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <ctime>
#include <iostream>
#include <optional>
#include <chrono>
#include <tuple>

#include <unistd.h>

#include <controlcan.h>

namespace CanDriver {
    namespace {

    constexpr int kDeviceType = 4;
    constexpr int kDeviceInd_1 = 0;
    constexpr int kDeviceInd_2 = 1;
    constexpr int kCANInd = 0;
    constexpr double kGearRatio = 101.0;

    constexpr double kRadtoDeg = 180.0 / M_PI;
    //constexpr double kPositonToCommand = kRadtoDeg / 360.0 * kGearRatio * 65536;
     constexpr double kPositonToCommand = kRadtoDeg / 360.0 * 262144;

    constexpr double kCommandToPosition = 1.0 / kPositonToCommand;

    constexpr int kDeviceNum = 2;
    constexpr int kCommandSetPosition = 0x1E;
    constexpr int kCommandGetPosition = 0x08;
    constexpr int kCommandSetPositionCSP = 0x44;
    constexpr int kCommandSetVelMaxVelPositive = 0x24;
    constexpr int kCommandSetVelMaxVelNegative = 0x25;
    constexpr int kCommandSetVelMaxAccPositive = 0x22;
    constexpr int kCommandSetVelMaxAccNegative = 0x23;
    constexpr int kCommandGetMaxVelPositive = 0x18;
    constexpr int kCommandGetMaxVelNegative = 0x19;
    constexpr int kCommandGetMaxAccPositive = 0x16;
    constexpr int kCommandGetMaxAccNegative = 0x17;

    bool PrepareCanDevice(int nDeviceType, int nDeviceInd, int nCANInd, VCI_INIT_CONFIG vci) {
      if (VCI_OpenDevice(nDeviceType, nDeviceInd, 0) != 1) {
        std::cerr << "open device " << nDeviceInd << " failed\n";
        return false;
      } else {
        std::cout << "open device " << nDeviceInd << " success\n";
      }

      if (VCI_InitCAN(nDeviceType, nDeviceInd, nCANInd, &vci) != 1) {
        std::cerr << "init device " << nDeviceInd << " failed\n";
        return false;
      } else {
        std::cout << "init device " << nDeviceInd << " success\n";
      }

      if (VCI_StartCAN(VCI_USBCAN2, nDeviceInd, nCANInd) != 1) {
        std::cerr << "start device " << nDeviceInd << " failed\n";
        VCI_CloseDevice(VCI_USBCAN2, nDeviceInd);
        return false;
      } else {
        std::cout << "start device " << nDeviceInd << " success\n";
      }
      return true;
    }

    // Convert an array of 4 bytes in little-endian format to a 32-bit number.
    int32_t FromHexArray(const uint8_t hex_array[4]) {
      int32_t value = 0;
      value |= (int32_t)(hex_array[0]) << 0;   // 低字节
      value |= (int32_t)(hex_array[1]) << 8;   // 次低字节
      value |= (int32_t)(hex_array[2]) << 16;  // 次高字节
      value |= (int32_t)(hex_array[3]) << 24;  // 高字节
      return value;
    }

    // Convert a 32-bit number to an array of 4 bytes in little-endian format.
    void ToHexArray(int32_t number, uint8_t* res) {
      res[0] = (number & 0xFF);        // 低字节
      res[1] = (number >> 8) & 0xFF;   // 次低字节
      res[2] = (number >> 16) & 0xFF;  // 次高字节
      res[3] = (number >> 24) & 0xFF;  // 高字节
    }

    void SendCanCommand(int can_device, uint8_t can_id, uint8_t command, int32_t parameter) {
      VCI_CAN_OBJ send[1];
      send[0].SendType = 0;
      send[0].RemoteFlag = 0;
      send[0].ExternFlag = 0;
      send[0].DataLen = 5;
      send[0].ID = can_id;
      send[0].Data[0] = command;
      uint8_t res[4];
      ToHexArray(parameter, res);
      for (int j = 0; j < 4; j++) send[0].Data[j + 1] = res[j];

      int retry = 2;
      int reclen = 0;
      while (reclen = VCI_Transmit(VCI_USBCAN2, can_device, kCANInd, send, 1) <= 0 && retry) { retry--; }
      if (retry == 0) {
        std::cerr << "ops! ID " << send[0].ID << " failed  to send after try 2 times\n";
      } else {
        // std::cout << "ID:  " << send[0].I\nD;
        // for (int c = 0; c < send[0].DataLen; c++) { printf("   %02X", send[0].Data[c]); }
        // std::cou\nt;
      }
    }

    std::optional<std::tuple<int32_t, double>> RecvCanCommand(
        int can_device, uint8_t can_id, uint8_t command, int32_t* result) {
      VCI_CAN_OBJ send[1];
      send[0].SendType = 0;
      send[0].RemoteFlag = 0;
      send[0].ExternFlag = 0;
      send[0].DataLen = 1;
      send[0].ID = can_id;
      send[0].Data[0] = command;

      int retry = 2;
      int reclen = 0;
      while (reclen = VCI_Transmit(VCI_USBCAN2, can_device, kCANInd, send, 1) <= 0 && retry) { retry--; }
      if (retry == 0) {
        std::cerr << "ops! ID " << send[0].ID << " failed  to send after try 2 times\n";
      } else {
        usleep(300);
        VCI_CAN_OBJ rec[10];  // buffer for received data, maximum 10 messages
        int reclen = 0;
        int retry = 5;
        while ((reclen = VCI_Receive(VCI_USBCAN2, can_device, kCANInd, rec, 10, 100) <= 0 && retry)) { retry--; }
        if (retry == 0) {
          std::cout << "ops! ID " << send[0].ID << " failed to recv after try 2 times\n";
        } else {
          VCI_CAN_OBJ latest_rec = rec[0];
          uint8_t hex_array[4] = {latest_rec.Data[1], latest_rec.Data[2], latest_rec.Data[3], latest_rec.Data[4]};
          // std::cout << send[0].ID << " recv id:" << latest_rec.I\nD;
          int32_t decimal = FromHexArray(hex_array);
          // 2025.9.17 --by Fa1lin9
          {
            *result = decimal;
          }
          return std::make_tuple<int32_t, double>(static_cast<int32_t>(latest_rec.ID), decimal);

        }
      }
      return std::nullopt;
    }

    std::optional<std::tuple<int32_t, double>> SendRecvCanCommand(
        int can_device, uint8_t can_id, uint8_t command, int32_t parameter, int32_t* result) {
      VCI_CAN_OBJ send[1];
      send[0].SendType = 0;
      send[0].RemoteFlag = 0;
      send[0].ExternFlag = 0;
      send[0].DataLen = 5;
      send[0].ID = can_id;
      send[0].Data[0] = command;
      uint8_t res[4];
      ToHexArray(parameter, res);
      for (int j = 0; j < 4; j++) send[0].Data[j + 1] = res[j];

      int retry = 2;
      int reclen = 0;
      while (reclen = VCI_Transmit(VCI_USBCAN2, can_device, kCANInd, send, 1) <= 0 && retry) { retry--; }
      if (retry == 0) {
        std::cerr << "ops! ID " << send[0].ID << " failed  to send after try 2 times\n";
      } else {
        usleep(80);
        VCI_CAN_OBJ rec[10];  // buffer for received data, maximum 10 messages
        int reclen = 0;
        int retry = 2;
        while ((reclen = VCI_Receive(VCI_USBCAN2, can_device, kCANInd, rec, 10, 100) <= 0 && retry)) { retry--; }
        if (retry == 0) {
          std::cout << "ops! ID " << send[0].ID << " failed to recv after try 20 times\n";
        } else {
          VCI_CAN_OBJ latest_rec = rec[0];
          uint8_t hex_array[4] = {latest_rec.Data[4], latest_rec.Data[5], latest_rec.Data[6], latest_rec.Data[7]};
          int32_t decimal = FromHexArray(hex_array);
          return std::make_tuple<int32_t, double>(static_cast<int32_t>(latest_rec.ID), decimal);
          // *result = decimal;
        }
      }
      return std::nullopt;
    }

    std::vector<std::tuple<int32_t, double>> SendRecvMultiCanCommand(
        int can_device, uint8_t can_id, uint8_t command, int32_t parameter, int32_t* result, bool recv) {
      std::vector<std::tuple<int32_t, double>> return_value;
      VCI_CAN_OBJ send[1];
      send[0].SendType = 0;
      send[0].RemoteFlag = 0;
      send[0].ExternFlag = 0;
      send[0].DataLen = 5;
      send[0].ID = can_id;
      send[0].Data[0] = command;
      uint8_t res[4];
      ToHexArray(parameter, res);
      for (int j = 0; j < 4; j++) send[0].Data[j + 1] = res[j];
      int retry = 2;
      int reclen = 0;
      usleep(100);
      VCI_Transmit(VCI_USBCAN2, can_device, kCANInd, send, 1);
      // while (reclen =  <= 0 && retry) { retry--; }
      if (retry == 0) {
        std::cerr << "ops! ID " << send[0].ID << " failed  to send after try 2 times\n";
      } else {
        usleep(100);
        VCI_CAN_OBJ rec[10];  // buffer for received data, maximum 10 messages
        memset(rec,0,sizeof(rec));
        int reclen = 0;
        int retry = 2;
        while ((reclen = VCI_Receive(VCI_USBCAN2, can_device, kCANInd, rec, 10, 100) <= 0 && retry)) { retry--; }
        // }
        if (retry == 0) {
          std::cout << "ops! ID " << send[0].ID << " failed to recv after try 5 times\n";
        } else {
          for (int i = 0; i < 10; i++) {
            VCI_CAN_OBJ latest_rec = rec[i];
            if(latest_rec.ID == 0) {
              break;
            }
            uint8_t hex_array[4] = {latest_rec.Data[4], latest_rec.Data[5], latest_rec.Data[6], latest_rec.Data[7]};
            int32_t decimal = FromHexArray(hex_array);
            return_value.emplace_back(std::make_tuple<int32_t, double>(static_cast<int32_t>(latest_rec.ID), decimal));
          }
          // *result = decimal;
        }
      }
      return return_value;
    }

    }  // namespace

    bool InitCan() {
      VCI_BOARD_INFO pInfo[50];
      VCI_INIT_CONFIG vci;

      vci.AccCode = 0x80000008;
      vci.AccMask = 0xFFFFFFFF;
      vci.Filter = 1;
      vci.Timing0 = 0x00;
      vci.Timing1 = 0x14;
      vci.Mode = 0;

      int num = VCI_FindUsbDevice2(pInfo);
      std::cout << "I find " << num << " device\n";

      // 2 devices
      if (!PrepareCanDevice(kDeviceType, kDeviceInd_1, kCANInd, vci)) { return false; }
      // if (!PrepareCanDevice(kDeviceType, kDeviceInd_2, kCANInd, vci)) { return false; }

      return true;
    }

    void CloseCan() {
      VCI_CloseDevice(VCI_USBCAN2, kDeviceInd_1);
      VCI_CloseDevice(VCI_USBCAN2, kDeviceInd_2);
    }

    void ConfigMaxVelocity(int can_device, uint8_t can_id, double max_velocity) {
      int8_t cmd;
      int32_t param_v;
      // Set max positive velocity
      cmd = kCommandSetVelMaxVelPositive;
      param_v = max_velocity;
      SendCanCommand(can_device, can_id, cmd, param_v);
      usleep(1000);
      // Set max negative velocity
      cmd = kCommandSetVelMaxVelNegative;
      param_v = -max_velocity;
      SendCanCommand(can_device, can_id, cmd, param_v);
      usleep(1000);
    }

    void ConfigMaxAcceleration(int can_device, uint8_t can_id, double max_acceleration) {
      uint8_t cmd;
      int32_t param_a;
      // Set max positive velocity
      cmd = kCommandSetVelMaxAccPositive;
      param_a = max_acceleration;
      SendCanCommand(can_device, can_id, cmd, param_a);
      usleep(1000);
      // Set max negative velocity
      cmd = kCommandSetVelMaxAccNegative;
      param_a = -max_acceleration;
      SendCanCommand(can_device, can_id, cmd, param_a);
      usleep(1000);
    }

    void RecvMaxVelocity(int can_device, uint8_t can_id, double* velocity) {
      uint8_t cmd;
      int32_t val;
      cmd = kCommandGetMaxVelPositive;
      RecvCanCommand(can_device, can_id, cmd, &val);
      *velocity = static_cast<double>(val);
      usleep(1000);
    }

    void RecvMaxAcceleration(int can_device, uint8_t can_id, double* accel) {
      uint8_t cmd;
      int32_t val;
      cmd = kCommandGetMaxAccPositive;
      RecvCanCommand(can_device, can_id, cmd, &val);
      *accel = static_cast<double>(val);
      usleep(1000);
    }

    void SendPosition(int can_device, uint8_t can_id, double position) {
      uint8_t cmd;
      int32_t pos;
      cmd = kCommandSetPosition;
      pos = static_cast<int32_t>(position * kPositonToCommand);
      SendCanCommand(can_device, can_id, cmd, pos);
      usleep(100);
    }

    std::optional<std::tuple<int32_t, double>> RecvPosition(int can_device, uint8_t can_id, double* position) {
      uint8_t cmd;
      int32_t pos = static_cast<int32_t>(*position * kPositonToCommand);
      cmd = kCommandGetPosition;
      auto result = RecvCanCommand(can_device, can_id, cmd, &pos);
      if (!result.has_value()) { return std::nullopt; }
      // *position
      int32_t id = std::get<0>(result.value());
      double status = std::get<1>(result.value());
      double real_status = static_cast<double>(status) * kCommandToPosition;
      // 2025.9.17 by Fa1lin9
      *position = real_status;
      usleep(100);
      return std::make_tuple<int32_t, double>(static_cast<int32_t>(id), static_cast<double>(real_status));
    }

    std::optional<std::tuple<int32_t, double>> SendRecvPosition(int can_device, uint8_t can_id, double* position) {
      uint8_t cmd = 0;
      int32_t pos = 0;
      int32_t recv = static_cast<int32_t>(*position * kPositonToCommand);
      cmd = kCommandSetPositionCSP;
      pos = static_cast<int32_t>(*position * kPositonToCommand);
      auto result = SendRecvCanCommand(can_device, can_id, cmd, pos, &recv);
      if (!result.has_value()) { return std::nullopt; }
      // *position = static_cast<double>(recv) * kCommandToPosition;
      int32_t id = std::get<0>(result.value());
      double status = std::get<1>(result.value());
      double real_status = static_cast<double>(status) * kCommandToPosition;
      usleep(100);
      return std::make_tuple<int32_t, double>(static_cast<int32_t>(id), static_cast<double>(real_status));
    }

    std::vector<std::tuple<int32_t, double>> SendRecvMultiPosition(int can_device, uint8_t can_id, double* position, bool is_recv) {
      std::vector<std::tuple<int32_t, double>> return_result;
      uint8_t cmd = 0;
      int32_t pos = 0;
      int32_t recv = static_cast<int32_t>(*position * kPositonToCommand);
      cmd = kCommandSetPositionCSP;
      pos = static_cast<int32_t>(*position * kPositonToCommand);
      auto result = SendRecvMultiCanCommand(can_device, can_id, cmd, pos, &recv, is_recv);
      if (result.size() == 0) { return return_result; }
      // *position = static_cast<double>(recv) * kCommandToPosition;
      for(int i=0;i<result.size();i++){
        auto currend_frame = result[i];
        int32_t id = std::get<0>(currend_frame);
        double status = std::get<1>(currend_frame);
        double real_status = static_cast<double>(status) * kCommandToPosition;
        return_result.emplace_back(std::make_tuple<int32_t, double>(static_cast<int32_t>(id), static_cast<double>(real_status)));
      }
      usleep(100);
      return return_result;
    }

}
