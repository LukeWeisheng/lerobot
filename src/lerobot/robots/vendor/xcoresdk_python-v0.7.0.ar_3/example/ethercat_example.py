# -*- coding: utf-8 -*-
"""
@file: ethercat_example.py
@brief: ethercat相关示例 
@copyright: Copyright (C) 2024 ROKAE (Beijing) Technology Co., LTD. All Rights Reserved.
Information in this file is the intellectual property of Rokae Technology Co., Ltd,
And may contains trade secrets that must be stored and viewed confidentially.
"""
import setup_path
import time
import platform
# 根据操作系统导入相应的模块
if platform.system() == 'Windows':
    from Release.windows import xCoreSDK_python
    from Release.windows.xCoreSDK_python import servo
elif platform.system() == 'Linux':
    from Release.linux import xCoreSDK_python
    from Release.linux.xCoreSDK_python import servo
else:
    raise ImportError("Unsupported operating system")
from log import print_log, print_separator

BaseEthercat = servo.BaseEthercat


# MODBUS CRC16 (CRC-16-IBM) 多项式：0x8005
# 初始值：0xFFFF，结果异或：0x0000，输入反转：true，输出反转：true
def crc16_modbus(data: bytes) -> int:
    crc = 0xFFFF  # 初始值

    for byte in data:
        crc ^= byte  # XOR 字节数据

        for _ in range(8):
            if crc & 0x0001:  # 检查最低位
                crc = (crc >> 1) ^ 0xA001  # 多项式 0xA001（即 0x8005 反转）
            else:
                crc >>= 1

    return crc


def send_modbus_rtu_data(ethercat: BaseEthercat, channel: int, id: int,
                         start_addr: int, data_size: int, data: list):
    """
    发送 Modbus RTU 数据
    :param ethercat: Ethercat 对象
    :param channel: 485通道，1表示通道1，其它或者2表示通道2
    :param id: 从站 ID
    :param start_addr: 写入寄存器的起始地址
    :param data_size: 连续写入多少个数据，单位是byte，
    :param data: 写入的原数据
    """
    if len(data) != data_size:
        raise ValueError(f"data_size ({data_size}) 不等于实际数据长度 ({len(data)})")

    # 构造发送数据的长度
    valid_size = 2 + 9 + (data_size + 1) // 2 * 2
    data_send = [0] * valid_size  # 动态构造字节数组

    # 填充各个字段
    data_send[0] = valid_size - 2
    data_send[1] = 0x80
    data_send[2] = id & 0xFF
    data_send[3] = 0x10  # Modbus 功能码：写多个保持寄存器
    data_send[4] = (start_addr >> 8) & 0xFF  # 高字节
    data_send[5] = start_addr & 0xFF  # 低字节
    data_send[6] = 0  # 保留或实际用途？
    data_send[7] = ((data_size + 1) // 2) & 0xFF  # 寄存器数量？
    data_send[8] = ((data_size + 1) // 2 * 2) & 0xFF  # 字节数

    # 填入数据
    for i in range(data_size):
        data_send[9 + i] = data[i]

    # 如果数据是奇数个，补一个 0 字节
    if data_size % 2 == 1:
        data_send[9 + data_size] = 0
        crc_data_len = 9 + data_size + 1
        crc = crc16_modbus(bytes(data_send[:crc_data_len]))
        # 大端序写入 CRC（高字节在前，低字节在后）⚠️ 注意与标准 Modbus 不同
        data_send[9 + data_size + 1] = (crc >> 8) & 0xFF  # 高字节
        data_send[9 + data_size + 2] = crc & 0xFF  # 低字节
    else:
        crc_data_len = 9 + data_size
        crc = crc16_modbus(bytes(data_send[:crc_data_len]))
        # 大端序写入 CRC
        data_send[9 + data_size] = (crc >> 8) & 0xFF  # 高字节
        data_send[9 + data_size + 1] = crc & 0xFF  # 低字节

    # 计算发送偏移量
    bias = 66 if channel == 1 else 132

    # 发送 PDO
    ec = {}
    ethercat.WritePDO(10001, bias, valid_size, data_send, ec)

    # 等待响应（模拟循环 20 次）
    counter = 20
    success = False
    rev_data = [0] * valid_size
    while counter > 0:
        ethercat.ReadPDO(10001, bias, valid_size, rev_data, ec)
        if (rev_data[1] & 0x40) == 0x40:
            print("Rev success")
            response_ack = [10, 0x40]
            ethercat.WritePDO(10001, bias, 2, response_ack, ec)
            success = True
            break
        counter -= 1

    if not success:
        print("未收到成功响应")


# 获取所有从站信息
def get_all_slave_info(ethercat: BaseEthercat, ec: dict):
    slaves_info: list[servo.SlaveInfo] = ethercat.GetSlavesInfo(ec)
    print_log("GetSlavesInfo", ec)
    for info in slaves_info:
        print(
            f"name: {info.slaveName},alstatus: {info.alStatus},productcode: {info.productCode}"
        )


# 读取PDO
def read_pdo(ethercat: BaseEthercat, ec: dict):
    slave_addr: int = 2001
    offset: int = 0
    size: int = 8
    data = []
    ret = ethercat.ReadPDO(slave_addr, offset, size, data, ec)
    print_log("ReadPDO", ec, f"ret={str(ret)}")
    for d in data:
        print(f"data: {d}")


# 写入PDO
def write_pdo(ethercat: BaseEthercat, ec: dict):
    # 发送pdo的对应关系
    # 描述	index:subindex -> offset length
    # can	0x2000:01      ->    0      2
    # 		0x2000:02      ->    2      64
    # 485-1	0x2002:01      ->    66     2
    # 		0x2002:02      ->    68     64
    # 485-2	0x2004:01      ->    132    2
    # 		0x2004:02      ->    134    64
    #
    # 接收pdo的对应关系
    # 描述	index:subindex -> offset length
    # can	0x2001:01      ->    0      2
    # 		0x2001:02      ->    2      64
    # 485-1	0x2003:01      ->    66     2
    # 		0x2003:02      ->    68     64
    # 485-2	0x2005:01      ->    132    2
    # 		0x2005:02      ->    134    64
    # 按键  0x2007:00      ->    198    2

    # 通过485-1发送数据
    # 比如发送长度为10Byte的数据
    mode = 1
    bias = 0
    if mode == 1:
        bias = 0
    elif mode == 2:
        bias = 66
    elif mode == 3:
        bias = 132

    # 构造一个 12 字节的发送数据
    data_send = [8, 0x80, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10]

    # 发送 PDO 数据
    ethercat.WritePDO(10001, bias, 12, data_send, ec)

    # ======================
    # 第一次轮询：等待发送成功标志 (data_rev[1] & 0x80 == 0x80)
    # ======================
    while True:
        data_rev = []  # 接收缓冲区
        ethercat.ReadPDO(10001, bias, 12, data_rev, ec)

        # 打印接收到的数据（16进制）
        print("Received data (hex):", " ".join(f"{b:02X}" for b in data_rev))

        if (data_rev[1] & 0x80) == 0x80:
            print("✅ Send success")
            data_send_ack = [8, 0x00, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10]
            ethercat.WritePDO(10001, bias, 12, data_send_ack, ec)
            break

        time.sleep(1)  # 等待 1 秒

    # ======================
    # 第二次轮询：等待接收成功标志 (data_rev[1] & 0x40 == 0x40)
    # ======================
    while True:
        data_rev = []
        ethercat.ReadPDO(10001, bias, 12, data_rev, ec)

        print("Received data (hex):", " ".join(f"{b:02X}" for b in data_rev))

        if (data_rev[1] & 0x40) == 0x40:
            print("✅ Rev success")
            received_payload = data_rev[2:]
            print("Payload (hex):",
                  " ".join(f"{b:02X}" for b in received_payload))
            data_send_feedback = bytearray(
                [10, 0x40, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10])
            ethercat.WritePDO(10001, bias, 12, data_send_feedback, ec)
            break

        time.sleep(1)

    # 调用 Modbus RTU 发送函数，发送 10 字节的全 128 数据
    data_L10 = [128] * 10
    send_modbus_rtu_data(ethercat, 1, 27, 0, 10, data_L10)


if __name__ == '__main__':
    try:
        # 连接机器人
        # 不同的机器人对应不同的类型
        ip = "192.168.0.160"
        robot = xCoreSDK_python.xMateRobot(ip)
        ethercat: BaseEthercat = robot.ethercat()
        ec = {}
        sdk_version = robot.sdkVersion()
        print(f"SDK Version: {sdk_version}")
        get_all_slave_info(ethercat, ec)
        read_pdo(ethercat, ec)
        write_pdo(ethercat, ec)
    except Exception as e:
        print(f"An error occurred: {e}")
