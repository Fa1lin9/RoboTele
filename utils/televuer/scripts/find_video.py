import pyrealsense2 as rs

ctx = rs.context()
devices = ctx.query_devices()

print("发现的 RealSense 设备数量:", len(devices))

for dev in devices:
    print("Name:", dev.get_info(rs.camera_info.name))
    print("Serial:", dev.get_info(rs.camera_info.serial_number))
    print("Firmware:", dev.get_info(rs.camera_info.firmware_version))
    print("----")
