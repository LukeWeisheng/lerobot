"""
DexHand021S 灵巧手控制接口
  - 485 (Modbus RTU) 通信层
  - CAN 通信层（新增）
  - 频率测试：can_benchmark(robot, hz=30 or 60)
"""
import setup_path
import struct
import platform
import time
import statistics

if platform.system() == 'Windows':
    from Release.windows import xCoreSDK_python
elif platform.system() == 'Linux':
    from Release.linux import xCoreSDK_python
else:
    raise ImportError("Unsupported operating system")

import numpy as np


# ════════════════════════════════════════════════════════════════
#  常量定义
# ════════════════════════════════════════════════════════════════

SLAVE_ID = 0x01   # DexHand021S 默认从机地址（485 & CAN 共用）

# 各电机角度范围（单位：度）
MOTOR_ANGLE_RANGE = {
    1: (0.0,  75.0),   # 手指1
    2: (0.0,  75.0),   # 手指2
    3: (0.0,  75.0),   # 手指3
    4: (0.0, 108.0),   # 旋转关节
}

# ── CAN ID 定义（说明书 5.2 节）────────────────────────────────
SYS_ID           = 0x01

CAN_ID_GLOBAL_TX = SYS_ID + 0x000   # 0x001  全局设置 / 清错（上位机 → 灵巧手）
CAN_ID_CTRL_TX   = SYS_ID + 0x100   # 0x101  运动控制（上位机 → 灵巧手）

CAN_ID_GLOBAL_RX = SYS_ID + 0x080   # 0x081  全局设置回包
CAN_ID_CTRL_RX   = SYS_ID + 0x180   # 0x181  运动控制回包
CAN_ID_AUTO_RX1  = SYS_ID + 0x280   # 0x281  自动反馈（电机数据）
CAN_ID_AUTO_RX2  = SYS_ID + 0x380   # 0x381  自动反馈（触觉数据）
CAN_ID_ERROR_RX  = SYS_ID + 0x600   # 0x601  错误反馈

# CAN ID → 可读标签
_CAN_TAG = {
    CAN_ID_GLOBAL_RX: "全局回包",
    CAN_ID_CTRL_RX:   "控制回包",
    CAN_ID_AUTO_RX1:  "电机数据",
    CAN_ID_AUTO_RX2:  "触觉数据",
    CAN_ID_ERROR_RX:  "错误反馈",
}


# ════════════════════════════════════════════════════════════════
#  485 内部工具函数（保持不变）
# ════════════════════════════════════════════════════════════════

def _modbus_crc16(data: list) -> tuple:
    """计算 Modbus CRC16，返回 (crc_low, crc_high)。"""
    crc = 0xFFFF
    for byte in data:
        crc ^= byte
        for _ in range(8):
            crc = (crc >> 1) ^ 0xA001 if (crc & 0x0001) else crc >> 1
    return (crc & 0xFF), ((crc >> 8) & 0xFF)


def _find_frame_start(raw: list, slave_id: int, func_code: int) -> int:
    """动态定位 Modbus 响应帧起始位置，规避 SDK 头部填充字节不固定问题。"""
    for i in range(len(raw) - 1):
        if raw[i] == slave_id and raw[i + 1] == func_code:
            return i
    return -1


def _parse_error_flag(err: int) -> str:
    """将错误标志解析为可读字符串。"""
    if err == 0:
        return "正常"
    errors = []
    if err & 0x0001: errors.append("电流异常")
    if err & 0x0002: errors.append("电压异常")
    if err & 0x0004: errors.append("温度异常")
    if err & 0x0008: errors.append("堵转/过载")
    if err & 0x0010: errors.append("舵机硬件错误")
    return " | ".join(errors)


# ════════════════════════════════════════════════════════════════
#  CAN 内部工具函数（新增）
# ════════════════════════════════════════════════════════════════

def _make_can_frame(can_id: int, data: list) -> "xCoreSDK_python.CANFrame":
    """构造一个 CANFrame 对象。"""
    sf = xCoreSDK_python.CANFrame()
    sf.frame_id           = can_id
    sf.frame_format       = 0          # 标准帧
    sf.frame_type         = 0          # 数据帧
    sf.frame_valid_length = len(data)
    sf.data               = data
    return sf


def _can_send(robot, can_id: int, data: list) -> dict:
    """
    发送单个 CAN 帧。
    返回 ec 字典，ec['ec'] == 0 表示成功。
    """
    ec = {}
    robot.CANSendData("uint8", [_make_can_frame(can_id, data)], ec)
    return ec


def _can_recv(robot, timeout_ms: int = 500) -> tuple:
    """
    接收单个 CAN 帧。
    返回 (ec, CANFrame)。
    """
    ec = {}
    rf = xCoreSDK_python.CANFrame()
    robot.CANReceiveData(timeout_ms, "uint8", rf, ec)
    return ec, rf


def _parse_can_motor_angles(data: list) -> dict:
    """
    解析 CAN_ID_AUTO_RX1（0x281）自动反馈帧中的电机角度数据。

    说明书 5.6.4 节：
      Byte0~1 : Motor1 角度，Int16 小端，单位 0.1°
      Byte2~3 : Motor2 角度
      Byte4~5 : Motor3 角度
      Byte6~7 : Motor4（旋转关节）角度
    """
    angles = {}
    for i in range(4):
        if len(data) < (i + 1) * 2:
            break
        lo = data[i * 2]
        hi = data[i * 2 + 1]
        val = (hi << 8) | lo
        if val > 32767:
            val -= 65536
        angles[i + 1] = round(val / 10.0, 1)   # 单位 0.1° → 度
    return angles


def _parse_can_tactile(data: list) -> dict:
    """
    解析 CAN_ID_AUTO_RX2（0x381）自动反馈帧中的触觉数据。

    说明书 5.7 节（触觉）：
      Byte0~3 : 法向力  Float32 小端
      Byte4~7 : 切向力  Float32 小端
    """
    result = {"normal_force": None, "tangential_force": None}
    if len(data) >= 4:
        result["normal_force"]      = round(struct.unpack_from('<f', bytes(data[0:4]))[0], 4)
    if len(data) >= 8:
        result["tangential_force"]  = round(struct.unpack_from('<f', bytes(data[4:8]))[0], 4)
    return result


# ════════════════════════════════════════════════════════════════
#  CAN 核心接口（新增）
# ════════════════════════════════════════════════════════════════

def can_clear_error(robot) -> bool:
    """
    通过 CAN 发送清错指令（说明书 5.7.3 节）。
    CAN_ID=0x001, Byte0=0x03, Byte1=0xA4

    返回 bool：True 表示收到正常回包。
    """
    ec = _can_send(robot, CAN_ID_GLOBAL_TX, [0x03, 0xA4])
    recv_ec, rf = _can_recv(robot, timeout_ms=500)
    ok = (recv_ec.get('ec', -1) == 0 and rf.frame_id == CAN_ID_GLOBAL_RX)
    print(f"  [can_clear_error] ec={recv_ec.get('ec')}, "
          f"回包id={hex(rf.frame_id)}, data={list(rf.data)[:4]}, "
          f"结果={'✅ 成功' if ok else '❌ 失败/超时'}")
    time.sleep(0.003)   # 说明书建议清错后间隔 ≥3ms
    return ok


def can_set_finger_angle(robot, motor_id: int, angle_deg: float) -> dict:
    """
    通过 CAN 向指定电机发送位置控制指令（说明书 5.6.1 节）。

    CAN_ID=0x101, [Byte0=0x04, Byte1=motor_id, Byte2=pos_L, Byte3=pos_H]
    目标位置 = angle_deg * 10，小端序。

    参数：
        robot     : xCoreSDK_python.ArRobot 实例
        motor_id  : 1~4
        angle_deg : 目标角度（度）

    返回 dict：
        {
            "success"   : bool,
            "motor_id"  : int,
            "angle_deg" : float,
            "error_flag": int,    # 回包中的错误字节，0=正常
            "error_msg" : str,
        }
    """
    # ── 参数校验 ──────────────────────────────────────────────────
    if motor_id not in MOTOR_ANGLE_RANGE:
        return {
            "success": False, "motor_id": motor_id,
            "angle_deg": angle_deg, "error_flag": -1,
            "error_msg": f"motor_id={motor_id} 非法，有效值为 1~4"
        }
    min_ang, max_ang = MOTOR_ANGLE_RANGE[motor_id]
    if not (min_ang <= angle_deg <= max_ang):
        return {
            "success": False, "motor_id": motor_id,
            "angle_deg": angle_deg, "error_flag": -1,
            "error_msg": f"角度 {angle_deg}° 超出范围 [{min_ang}, {max_ang}]"
        }

    # ── 构造并发送帧 ──────────────────────────────────────────────
    pos = int(round(angle_deg * 10))
    data = [0x04, motor_id, pos & 0xFF, (pos >> 8) & 0xFF]
    _can_send(robot, CAN_ID_CTRL_TX, data)

    # ── 接收回包（说明书 5.6.4 节）────────────────────────────────
    # 回包格式：CAN_ID=0x181
    #   Byte0 : motor_id
    #   Byte1 : error_flag
    #   Byte2~3 : 当前位置（可选解析）
    recv_ec, rf = _can_recv(robot, timeout_ms=300)
    raw = list(rf.data)

    if recv_ec.get('ec', -1) != 0 or rf.frame_id != CAN_ID_CTRL_RX:
        return {
            "success": False, "motor_id": motor_id,
            "angle_deg": angle_deg, "error_flag": -1,
            "error_msg": f"未收到有效回包，ec={recv_ec}, id={hex(rf.frame_id)}"
        }

    err_flag = raw[1] if len(raw) >= 2 else -1
    return {
        "success"   : err_flag == 0x00,
        "motor_id"  : motor_id,
        "angle_deg" : angle_deg,
        "error_flag": err_flag,
        "error_msg" : _parse_error_flag(err_flag),
    }


def can_get_finger_angle(robot, motor_id=None) -> dict:
    """
    通过 CAN 读取电机角度。

    实现方式：开启一次自动反馈，接收一帧 CAN_ID_AUTO_RX1（0x281）后立即关闭。
    自动反馈帧格式（说明书 5.6.4 节）：
      Byte0~1 Motor1, Byte2~3 Motor2, Byte4~5 Motor3, Byte6~7 Motor4
      值 = 角度 * 10，Int16 小端序

    参数：
        motor_id : 1~4 返回单个电机；None 或 0 返回全部

    返回 dict（全部）：
        {"success": bool, "angles": {1: float, ...}, "error_msg": str}
    返回 dict（单个）：
        {"success": bool, "motor_id": int, "angle_deg": float, "error_msg": str}
    """
    read_all = (motor_id is None or motor_id == 0)
    if not read_all and motor_id not in MOTOR_ANGLE_RANGE:
        return {
            "success": False, "motor_id": motor_id,
            "angle_deg": None,
            "error_msg": f"motor_id={motor_id} 非法，有效值为 1~4（或 None/0 读全部）"
        }

    # 开启自动反馈（20ms 间隔），说明书 5.5.1 节
    # [0x03, 0x23, 0x00, 0x00, 0x01(自动), 0x14(20ms), 0x00]
    _can_send(robot, CAN_ID_GLOBAL_TX, [0x03, 0x23, 0x00, 0x00, 0x01, 0x14, 0x00])

    # 等待并接收一帧电机数据反馈
    angles = {}
    deadline = time.time() + 0.5
    while time.time() < deadline:
        recv_ec, rf = _can_recv(robot, timeout_ms=100)
        if recv_ec.get('ec', -1) == 0 and rf.frame_id == CAN_ID_AUTO_RX1:
            angles = _parse_can_motor_angles(list(rf.data))
            break

    # 关闭自动反馈：Byte4=0x00 表示关闭
    _can_send(robot, CAN_ID_GLOBAL_TX, [0x03, 0x23, 0x00, 0x00, 0x00, 0x14, 0x00])

    if not angles:
        err_msg = "未收到电机角度反馈帧（CAN_ID=0x281）"
        if read_all:
            return {"success": False, "angles": {}, "error_msg": err_msg}
        return {"success": False, "motor_id": motor_id, "angle_deg": None, "error_msg": err_msg}

    if read_all:
        return {"success": True, "angles": angles, "error_msg": "正常"}
    else:
        return {
            "success"  : True,
            "motor_id" : motor_id,
            "angle_deg": angles.get(motor_id),
            "error_msg": "正常",
        }


def can_get_tactile(robot) -> dict:
    """
    通过 CAN 读取触觉传感器数据（一次采样）。

    开启自动反馈后等待 CAN_ID_AUTO_RX2（0x381）帧，解析法向力和切向力。

    返回 dict：
        {
            "success"           : bool,
            "normal_force"      : float,   # 法向力（N）
            "tangential_force"  : float,   # 切向力（N）
            "error_msg"         : str,
        }
    """
    _can_send(robot, CAN_ID_GLOBAL_TX, [0x03, 0x23, 0x00, 0x00, 0x01, 0x14, 0x00])

    result = {}
    deadline = time.time() + 0.5
    while time.time() < deadline:
        recv_ec, rf = _can_recv(robot, timeout_ms=100)
        if recv_ec.get('ec', -1) == 0 and rf.frame_id == CAN_ID_AUTO_RX2:
            result = _parse_can_tactile(list(rf.data))
            break

    _can_send(robot, CAN_ID_GLOBAL_TX, [0x03, 0x23, 0x00, 0x00, 0x00, 0x14, 0x00])

    if not result:
        return {
            "success": False,
            "normal_force": None, "tangential_force": None,
            "error_msg": "未收到触觉反馈帧（CAN_ID=0x381）"
        }
    return {"success": True, **result, "error_msg": "正常"}


# ════════════════════════════════════════════════════════════════
#  CAN 力控接口（新增）
# ════════════════════════════════════════════════════════════════

def can_set_finger_torque(robot, motor_id: int,
                          hall_pos: int = 1000,
                          torque: int = 200) -> dict:
    """
    【方案A：力矩控制模式】通过 CAN 向指定电机发送力矩控制指令（说明书 5.6.3 节）。

    CAN_ID=0x101, [Byte0=0x06, Byte1=motor_id, Byte2-3=hall_pos(LE), Byte4-5=torque(LE)]

    参数：
        robot     : ArRobot 实例
        motor_id  : 1~4
        hall_pos  : 目标霍尔位置；手指[1,3] 范围 [0,1000] (= hall角度*10)
                    旋转关节(motor_id=4) 范围 [0,1600]
        torque    : 力矩值，范围 [50, 800]（PWM 当量，无明确物理单位）

    返回 dict（结构同 can_set_finger_angle）
    """
    if motor_id not in (1, 2, 3, 4):
        return {"success": False, "motor_id": motor_id,
                "error_flag": -1,
                "error_msg": f"motor_id={motor_id} 非法，有效值 1~4"}

    pos_max = 1600 if motor_id == 4 else 1000
    if not (0 <= hall_pos <= pos_max):
        return {"success": False, "motor_id": motor_id,
                "error_flag": -1,
                "error_msg": f"hall_pos={hall_pos} 超出范围 [0,{pos_max}]"}

    if not (50 <= torque <= 800):
        return {"success": False, "motor_id": motor_id,
                "error_flag": -1,
                "error_msg": f"torque={torque} 超出范围 [50,800]"}

    data = [0x06, motor_id,
            hall_pos & 0xFF, (hall_pos >> 8) & 0xFF,
            torque   & 0xFF, (torque   >> 8) & 0xFF]
    _can_send(robot, CAN_ID_CTRL_TX, data)

    recv_ec, rf = _can_recv(robot, timeout_ms=300)
    raw = list(rf.data)

    if recv_ec.get('ec', -1) != 0 or rf.frame_id != CAN_ID_CTRL_RX:
        return {"success": False, "motor_id": motor_id, "error_flag": -1,
                "error_msg": f"未收到有效回包 ec={recv_ec}, id={hex(rf.frame_id)}"}

    err_flag = raw[1] if len(raw) >= 2 else -1
    return {"success": err_flag == 0x00, "motor_id": motor_id,
            "hall_pos": hall_pos, "torque": torque,
            "error_flag": err_flag, "error_msg": _parse_error_flag(err_flag)}


def can_grasp_torque(robot, hall_pos: int = 1000, torque: int = 200,
                     rotate_joint_angle: float = 25.0,
                     rotate_settle_s: float = 0.5,
                     auto_clear_error: bool = True,
                     inter_cmd_delay: float = 0.05) -> bool:
    """
    【方案A】通过 CAN 力矩模式抓取。

    执行顺序（重要）：
      1) 旋转关节先就位 → 让另两指对齐拇指（决定抓取姿态）
      2) 等待 rotate_settle_s 让旋转到位
      3) 三指进入力矩模式合拢到 hall_pos

    参数：
        hall_pos           : 目标霍尔位置（默认 1000，即完全合拢）
        torque             : 力矩值，[50,800]，默认 200
        rotate_joint_angle : 旋转关节角度（默认 25°）
        rotate_settle_s    : 旋转关节就位等待秒数（默认 0.5s）
    """
    print(f"\n── [CAN] 力矩抓取  hall_pos={hall_pos}, torque={torque}, rotate={rotate_joint_angle}° ──")
    if auto_clear_error:
        can_clear_error(robot)

    all_success = True

    # 步骤1：旋转关节先就位
    result = can_set_finger_angle(robot, 4, rotate_joint_angle)
    status  = "✅" if result["success"] else "❌"
    print(f"  [步骤1] {status} Motor_4 (旋转关节) -> {rotate_joint_angle}° | {result['error_msg']}")
    if not result["success"]:
        all_success = False

    # 步骤2：等待旋转到位
    print(f"  [步骤2] 等待 {rotate_settle_s}s 让旋转关节就位...")
    time.sleep(rotate_settle_s)

    # 步骤3：三指力矩模式合拢
    print(f"  [步骤3] 三指进入力矩模式（torque={torque}）合拢")
    for motor_id in (1, 2, 3):
        result = can_set_finger_torque(robot, motor_id, hall_pos, torque)
        status = "✅" if result["success"] else "❌"
        print(f"    {status} Motor_{motor_id} (手指{motor_id}) 力矩={torque} | {result['error_msg']}")
        if not result["success"]:
            all_success = False
        time.sleep(inter_cmd_delay)

    return all_success


def can_set_max_current(robot, motor_id: int, current_ma: int = 250) -> bool:
    """
    【方案B：最大电流限制】通过 CAN 全局设置 0x65 设置某舵机最大输出电流。
    （说明书 6.5.2 节 / 6.7.2 节）

    CAN_ID=0x001, [0x25, 0x65, motor_id, 0x02, cur_L, cur_H, 0x00, 0x00]
    （这里使用 CAN 上的"全局设置"等价帧；说明书未直接给出 CAN 帧示例，
      参考 485 的 0x25/0x65 帧字段一一对应。）

    参数：
        motor_id   : 1~4
        current_ma : 最大电流(mA)，范围 [200, 500]，默认 250

    返回 bool：True=收到正确回包。
    """
    if motor_id not in (1, 2, 3, 4):
        print(f"  ❌ motor_id={motor_id} 非法")
        return False
    if not (200 <= current_ma <= 500):
        print(f"  ❌ current_ma={current_ma} 超出范围 [200,500]")
        return False

    data = [0x25, 0x65, motor_id, 0x02,
            current_ma & 0xFF, (current_ma >> 8) & 0xFF,
            0x00, 0x00]
    _can_send(robot, CAN_ID_GLOBAL_TX, data)
    recv_ec, rf = _can_recv(robot, timeout_ms=300)
    ok = (recv_ec.get('ec', -1) == 0 and rf.frame_id == CAN_ID_GLOBAL_RX)
    print(f"  [can_set_max_current motor={motor_id} cur={current_ma}mA] "
          f"回包id={hex(rf.frame_id)}, data={list(rf.data)[:4]}, "
          f"结果={'✅ 成功' if ok else '❌ 失败/超时'}")
    return ok


def can_grasp_force_feedback(
    robot,
    target_force_n: float = 2.0,
    finger_ids=(1, 2, 3),
    max_angle_deg: float = 75.0,
    angle_step_deg: float = 2.0,
    poll_interval_s: float = 0.05,
    timeout_s: float = 8.0,
    rotate_joint_angle: float = 25.0,
) -> dict:
    """
    【方案C：触觉闭环力控抓取】通过 CAN，
    边小步合拢手指边读触觉，达到目标法向力即停止。

    流程：
      1. 清错 → 旋转关节就位
      2. 三指角度 0° 起步，每周期增加 angle_step_deg
      3. 读触觉数据，若 normal_force ≥ target_force_n，标记该指"已锁定"
         不再增加角度（保持当前角度），其余手指继续推进
      4. 全部锁定 / 全部到达 max_angle_deg / 超时 → 退出

    参数：
        target_force_n     : 目标法向力（N），默认 2.0
        finger_ids         : 参与抓取的手指 id（默认 (1,2,3)）
        max_angle_deg      : 单指最大合拢角度（默认 75°）
        angle_step_deg     : 每周期角度增量（默认 2°）
        poll_interval_s    : 控制周期（默认 0.05s = 20Hz）
        timeout_s          : 整体超时（默认 8s）
        rotate_joint_angle : 旋转关节角度（默认 25°）

    返回 dict：
        {
            "success"         : bool,            # 是否至少有一指接触到力阈
            "locked_fingers"  : [int],           # 已锁定的手指列表
            "final_angles"    : {1: float, ...}, # 最终角度
            "final_force_n"   : float,           # 最终读到的法向力
            "elapsed_s"       : float,
        }

    注意：CAN 自动反馈帧中触觉只包含一组（说明书 5.6.4 节是按手指 id 反馈），
    我们这里读取到的法向力代表"当前活跃手指"的接触力，
    所以策略上是"任一手指达到阈值就停下所有"，更稳健。
    """
    print(f"\n══════════ [CAN] 触觉闭环力控抓取  目标={target_force_n}N ══════════")
    can_clear_error(robot)
    time.sleep(0.05)

    # 旋转关节先就位
    can_set_finger_angle(robot, 4, rotate_joint_angle)
    time.sleep(0.1)

    cur_angles    = {fid: 0.0 for fid in finger_ids}
    locked        = set()
    last_force    = 0.0
    start_t       = time.perf_counter()

    while True:
        elapsed = time.perf_counter() - start_t
        if elapsed > timeout_s:
            print(f"  ⏱  超时 {timeout_s}s，结束")
            break

        # 1) 推进未锁定的手指
        all_at_max = True
        for fid in finger_ids:
            if fid in locked:
                continue
            new_ang = min(cur_angles[fid] + angle_step_deg, max_angle_deg)
            if new_ang < max_angle_deg:
                all_at_max = False
            cur_angles[fid] = new_ang
            res = can_set_finger_angle(robot, fid, new_ang)
            if not res["success"]:
                print(f"  ⚠️  Motor_{fid} 推进失败: {res['error_msg']}，立即锁定")
                locked.add(fid)

        # 2) 读触觉
        tac = can_get_tactile(robot)
        if tac["success"] and tac["normal_force"] is not None:
            last_force = tac["normal_force"]
            angle_snap = {k: round(v, 1) for k, v in cur_angles.items()}
            print(f"  t={elapsed:5.2f}s  angles={angle_snap}  "
                  f"normal_force={last_force:6.2f}N  locked={sorted(locked)}")
            if last_force >= target_force_n:
                # 任一手指法向力达到阈值 → 全部锁定
                for fid in finger_ids:
                    locked.add(fid)
                print(f"  🎯 法向力 {last_force:.2f}N ≥ 目标 "
                      f"{target_force_n}N，已锁定所有手指")

        # 3) 退出条件
        if len(locked) >= len(finger_ids):
            break
        if all_at_max:
            print(f"  📐 所有手指已到达最大角度 {max_angle_deg}°，结束")
            break

        time.sleep(poll_interval_s)

    elapsed_total = time.perf_counter() - start_t
    success = (last_force >= target_force_n) or (len(locked) >= len(finger_ids))

    result = {
        "success"        : success,
        "locked_fingers" : sorted(locked),
        "final_angles"   : {k: round(v, 2) for k, v in cur_angles.items()},
        "final_force_n"  : round(last_force, 3),
        "elapsed_s"      : round(elapsed_total, 3),
    }
    verdict = "✅ 力控抓取成功" if success else "⚠️ 未达到目标力（可能未触物）"
    print(f"\n══════════ {verdict}  耗时={elapsed_total:.2f}s  最终力={last_force:.2f}N ══════════")
    print(f"  最终角度: {result['final_angles']}")
    return result


def can_grasp(robot) -> bool:
    """
    通过 CAN 执行抓取动作：手指1/2/3 → 75°，旋转关节 → 25°。

    返回 bool：全部成功为 True。
    """
    commands = [(1, 75.0), (2, 75.0), (3, 75.0), (4, 25.0)]
    print("\n── [CAN] 执行抓取动作 ────────────────────────")
    all_success = True
    for motor_id, angle in commands:
        label  = "旋转关节" if motor_id == 4 else f"手指{motor_id}"
        result = can_set_finger_angle(robot, motor_id, angle)
        status = "✅" if result["success"] else "❌"
        print(f"  {status} Motor_{motor_id} ({label}) -> {angle}° | {result['error_msg']}")
        if not result["success"]:
            all_success = False
        time.sleep(0.05)
    return all_success


def can_release(robot) -> bool:
    """
    通过 CAN 执行释放动作：手指1/2/3 → 0°，旋转关节 → 25°。

    返回 bool：全部成功为 True。
    """
    print("\n── [CAN] 执行释放动作 ────────────────────────")
    all_success = True
    for motor_id in [1, 2, 3, 4]:
        label = "旋转关节" if motor_id == 4 else f"手指{motor_id}"
        angle = 25.0 if motor_id == 4 else 0.0
        result = can_set_finger_angle(robot, motor_id, angle)
        status = "✅" if result["success"] else "❌"
        print(f"  {status} Motor_{motor_id} ({label}) -> {angle}° | {result['error_msg']}")
        if not result["success"]:
            all_success = False
        time.sleep(0.05)
    return all_success


def can_print_all_angles(robot) -> dict:
    """
    通过 CAN 读取并打印全部4个电机的当前角度。

    返回 {1: float, 2: float, 3: float, 4: float} 或 {}。
    """
    print("\n── [CAN] 读取全部手指角度 ────────────────────")
    result = can_get_finger_angle(robot, motor_id=None)
    if not result["success"]:
        print(f"  ❌ 读取失败: {result['error_msg']}")
        return {}
    for mid, deg in result["angles"].items():
        label = "旋转关节" if mid == 4 else f"手指{mid}"
        print(f"  Motor_{mid} ({label}): {deg:.1f}°")
    return result["angles"]


# ════════════════════════════════════════════════════════════════
#  频率测试（新增）
# ════════════════════════════════════════════════════════════════

def can_benchmark(robot, target_hz: int = 30, duration_s: float = 3.0,
                  mode: str = "send_recv") -> dict:
    """
    测试 CAN 通信能否稳定达到目标频率（30Hz 或 60Hz）。

    原理：
      - 以 1/target_hz 为周期，循环发送控制帧并等待回包
      - 统计实际完成帧数、平均延迟、最大延迟、丢帧率

    参数：
        robot      : xCoreSDK_python.ArRobot 实例
        target_hz  : 目标频率，建议 30 或 60
        duration_s : 测试持续时间（秒），默认 3 秒
        mode       : "send_recv" = 发送+接收往返测试（更严格）
                     "recv_only" = 仅监听自动反馈帧（测总线吞吐）

    返回 dict：
        {
            "target_hz"     : int,
            "actual_hz"     : float,   # 实际完成频率
            "total_sent"    : int,
            "total_received": int,
            "loss_rate_pct" : float,   # 丢帧率 %
            "latency_avg_ms": float,   # 平均往返延迟 ms（send_recv 模式）
            "latency_max_ms": float,
            "latency_min_ms": float,
            "latency_std_ms": float,
            "verdict"       : str,     # "✅ 达标" / "❌ 未达标"
        }
    """
    period_s    = 1.0 / target_hz
    # 超时设置为周期的 80%，避免等待过长拖慢节奏
    timeout_ms  = max(10, int(period_s * 800))

    print(f"\n{'='*60}")
    print(f"  CAN 频率测试  目标={target_hz}Hz  模式={mode}  时长={duration_s}s")
    print(f"  周期={period_s*1000:.2f}ms  接收超时={timeout_ms}ms")
    print(f"{'='*60}")

    total_sent     = 0
    total_received = 0
    latencies      = []

    # ── send_recv 模式：发控制帧 → 等回包 ────────────────────────
    if mode == "send_recv":
        # 用手指1保持当前位置（发 0° 指令，不实际运动）
        # 也可换成 can_get_finger_angle 的请求帧
        pos    = 0
        data   = [0x04, 0x01, pos & 0xFF, (pos >> 8) & 0xFF]

        start_wall = time.perf_counter()
        next_tick  = start_wall

        while time.perf_counter() - start_wall < duration_s:
            t0 = time.perf_counter()

            # 发送
            ec_s = _can_send(robot, CAN_ID_CTRL_TX, data)
            total_sent += 1

            # 接收
            recv_ec, rf = _can_recv(robot, timeout_ms=timeout_ms)
            t1 = time.perf_counter()

            if recv_ec.get('ec', -1) == 0 and rf.frame_valid_length > 0:
                total_received += 1
                latencies.append((t1 - t0) * 1000)   # ms

            # 精确定时：等到下一个周期点
            next_tick += period_s
            sleep_t = next_tick - time.perf_counter()
            if sleep_t > 0:
                time.sleep(sleep_t)

    # ── recv_only 模式：开启自动反馈，纯监听 ─────────────────────
    elif mode == "recv_only":
        # 设置自动反馈间隔为目标周期（单位 ms，Byte5）
        interval_byte = max(1, min(255, int(period_s * 1000)))
        _can_send(robot, CAN_ID_GLOBAL_TX,
                  [0x03, 0x23, 0x00, 0x00, 0x01, interval_byte, 0x00])
        time.sleep(0.05)   # 等待设置生效

        start_wall = time.perf_counter()
        prev_t     = start_wall

        while time.perf_counter() - start_wall < duration_s:
            recv_ec, rf = _can_recv(robot, timeout_ms=timeout_ms * 2)
            t_now = time.perf_counter()
            total_sent += 1   # 期望帧数（按时间估算）

            if recv_ec.get('ec', -1) == 0 and rf.frame_valid_length > 0:
                total_received += 1
                interval_ms = (t_now - prev_t) * 1000
                latencies.append(interval_ms)
                prev_t = t_now

        # 关闭自动反馈
        _can_send(robot, CAN_ID_GLOBAL_TX,
                  [0x03, 0x23, 0x00, 0x00, 0x00, interval_byte, 0x00])

    else:
        raise ValueError(f"mode 参数非法: {mode}，应为 'send_recv' 或 'recv_only'")

    # ── 统计 ──────────────────────────────────────────────────────
    elapsed      = time.perf_counter() - (time.perf_counter() - duration_s)  # ≈ duration_s
    actual_hz    = total_received / duration_s
    loss_rate    = (1 - total_received / max(total_sent, 1)) * 100

    lat_avg = statistics.mean(latencies)      if latencies else 0.0
    lat_max = max(latencies)                  if latencies else 0.0
    lat_min = min(latencies)                  if latencies else 0.0
    lat_std = statistics.stdev(latencies)     if len(latencies) > 1 else 0.0

    # 达标判定：实际频率 ≥ 目标的 95%，丢帧率 < 5%
    verdict = "✅ 达标" if (actual_hz >= target_hz * 0.95 and loss_rate < 5.0) else "❌ 未达标"

    result = {
        "target_hz"     : target_hz,
        "actual_hz"     : round(actual_hz, 2),
        "total_sent"    : total_sent,
        "total_received": total_received,
        "loss_rate_pct" : round(loss_rate, 2),
        "latency_avg_ms": round(lat_avg, 3),
        "latency_max_ms": round(lat_max, 3),
        "latency_min_ms": round(lat_min, 3),
        "latency_std_ms": round(lat_std, 3),
        "verdict"       : verdict,
    }

    # ── 打印报告 ──────────────────────────────────────────────────
    print(f"\n  {'─'*50}")
    print(f"  目标频率   : {target_hz} Hz")
    print(f"  实际频率   : {actual_hz:.2f} Hz  {verdict}")
    print(f"  发送帧数   : {total_sent}")
    print(f"  接收帧数   : {total_received}")
    print(f"  丢帧率     : {loss_rate:.2f}%")
    print(f"  延迟 avg   : {lat_avg:.3f} ms")
    print(f"  延迟 max   : {lat_max:.3f} ms")
    print(f"  延迟 min   : {lat_min:.3f} ms")
    print(f"  延迟 std   : {lat_std:.3f} ms")
    print(f"  {'─'*50}\n")

    return result
def rs485_benchmark(robot, target_hz: int = 30, duration_s: float = 3.0,
                    mode: str = "send_recv") -> dict:
    """
    测试 485 (Modbus RTU) 通信能否稳定达到目标频率（30Hz 或 60Hz）。

    两种模式：
      "send_recv" : 循环发送 0x31 位置控制帧 → 等待回包，测往返延迟
      "read_only" : 循环发送 0x04 读输入寄存器帧 → 等待角度数据，测读取吞吐

    参数：
        robot      : xCoreSDK_python.ArRobot 实例
        target_hz  : 目标频率，建议 30 或 60
        duration_s : 测试持续时间（秒），默认 3 秒
        mode       : "send_recv" 或 "read_only"

    返回 dict：
        {
            "target_hz"     : int,
            "actual_hz"     : float,
            "total_sent"    : int,
            "total_received": int,
            "loss_rate_pct" : float,
            "latency_avg_ms": float,
            "latency_max_ms": float,
            "latency_min_ms": float,
            "latency_std_ms": float,
            "verdict"       : str,    # "✅ 达标" / "❌ 未达标"
        }
    """
    period_s   = 1.0 / target_hz
    # 超时 = 周期的 80%，最低 5ms（485 单帧收发通常 2~5ms）
    timeout_ms = max(5, int(period_s * 800))

    print(f"\n{'='*60}")
    print(f"  485 频率测试  目标={target_hz}Hz  模式={mode}  时长={duration_s}s")
    print(f"  周期={period_s*1000:.2f}ms  接收超时={timeout_ms}ms")
    print(f"{'='*60}")

    total_sent     = 0
    total_received = 0
    latencies      = []

    # ── send_recv 模式：发 0x31 控制帧（保持 0°）→ 等回包 ─────────
    if mode == "send_recv":
        # 构造一次，循环复用（目标角度固定为 0°，不实际运动）
        target_pos = 0
        payload = [SLAVE_ID, 0x31, 0x04, 0x01,
                   target_pos & 0xFF, (target_pos >> 8) & 0xFF,
                   0x00, 0x00]
        crc_l, crc_h = _modbus_crc16(payload)
        frame = np.array(payload + [crc_l, crc_h], dtype=np.uint8)
        expected_len = 6   # 0x31 回包固定 6 字节

        start_wall = time.perf_counter()
        next_tick  = start_wall

        while time.perf_counter() - start_wall < duration_s:
            t0 = time.perf_counter()

            recv = xCoreSDK_python.PyTypeVectorInt()
            ec   = {}
            robot.XPRS485SendData(len(frame), expected_len, frame, recv, ec)
            total_sent += 1

            t1  = time.perf_counter()
            raw = list(recv.content())

            # 判断是否收到有效回包：能定位到帧头即视为成功
            start_idx = _find_frame_start(raw, SLAVE_ID, 0x31)
            if start_idx >= 0 and len(raw) >= start_idx + 4:
                total_received += 1
                latencies.append((t1 - t0) * 1000)

            # 精确定时
            next_tick += period_s
            sleep_t = next_tick - time.perf_counter()
            if sleep_t > 0:
                time.sleep(sleep_t)

    # ── read_only 模式：循环发 0x04 读角度帧 → 等回包 ────────────
    elif mode == "read_only":
        reg_addr, reg_count = 0x0000, 4
        payload = [SLAVE_ID, 0x04,
                   (reg_addr >> 8) & 0xFF, reg_addr & 0xFF,
                   (reg_count >> 8) & 0xFF, reg_count & 0xFF]
        crc_l, crc_h = _modbus_crc16(payload)
        frame = np.array(payload + [crc_l, crc_h], dtype=np.uint8)
        expected_len = 13   # 0x04 读4寄存器回包 = 3头 + 8数据 + 2CRC

        start_wall = time.perf_counter()
        next_tick  = start_wall

        while time.perf_counter() - start_wall < duration_s:
            t0 = time.perf_counter()

            recv = xCoreSDK_python.PyTypeVectorInt()
            ec   = {}
            robot.XPRS485SendData(len(frame), expected_len, frame, recv, ec)
            total_sent += 1

            t1  = time.perf_counter()
            raw = list(recv.content())

            start_idx = _find_frame_start(raw, SLAVE_ID, 0x04)
            if start_idx >= 0 and len(raw) >= start_idx + 3 + 8:
                total_received += 1
                latencies.append((t1 - t0) * 1000)

            next_tick += period_s
            sleep_t = next_tick - time.perf_counter()
            if sleep_t > 0:
                time.sleep(sleep_t)

    else:
        raise ValueError(f"mode 参数非法: {mode}，应为 'send_recv' 或 'read_only'")

    # ── 统计 ──────────────────────────────────────────────────────
    actual_hz = total_received / duration_s
    loss_rate = (1 - total_received / max(total_sent, 1)) * 100

    lat_avg = statistics.mean(latencies)          if latencies else 0.0
    lat_max = max(latencies)                      if latencies else 0.0
    lat_min = min(latencies)                      if latencies else 0.0
    lat_std = statistics.stdev(latencies)         if len(latencies) > 1 else 0.0

    verdict = "✅ 达标" if (actual_hz >= target_hz * 0.95 and loss_rate < 5.0) else "❌ 未达标"

    result = {
        "target_hz"     : target_hz,
        "actual_hz"     : round(actual_hz, 2),
        "total_sent"    : total_sent,
        "total_received": total_received,
        "loss_rate_pct" : round(loss_rate, 2),
        "latency_avg_ms": round(lat_avg, 3),
        "latency_max_ms": round(lat_max, 3),
        "latency_min_ms": round(lat_min, 3),
        "latency_std_ms": round(lat_std, 3),
        "verdict"       : verdict,
    }

    print(f"\n  {'─'*50}")
    print(f"  目标频率   : {target_hz} Hz")
    print(f"  实际频率   : {actual_hz:.2f} Hz  {verdict}")
    print(f"  发送帧数   : {total_sent}")
    print(f"  接收帧数   : {total_received}")
    print(f"  丢帧率     : {loss_rate:.2f}%")
    print(f"  延迟 avg   : {lat_avg:.3f} ms")
    print(f"  延迟 max   : {lat_max:.3f} ms")
    print(f"  延迟 min   : {lat_min:.3f} ms")
    print(f"  延迟 std   : {lat_std:.3f} ms")
    print(f"  {'─'*50}\n")

    return result


# ════════════════════════════════════════════════════════════════
#  485 核心接口
# ════════════════════════════════════════════════════════════════

def rs485_clear_error(robot) -> bool:
    """
    通过 485 (Modbus RTU) 发送清错指令（说明书 6.5.5 节）。

    全局设置帧 0x25 + 子命令 0xA4：
      [0x01, 0x25, 0xA4, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, CRC_L, CRC_H]
    回包：
      [0x01, 0x25, 0xA4, 0x01, CRC_L, CRC_H]   # Byte3=0x01 表示成功

    返回 bool：True=清错成功。
    """
    payload = [SLAVE_ID, 0x25, 0xA4, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]
    crc_l, crc_h = _modbus_crc16(payload)
    frame = np.array(payload + [crc_l, crc_h], dtype=np.uint8)

    ec = {}
    recv = xCoreSDK_python.PyTypeVectorInt()
    robot.XPRS485SendData(len(frame), 6, frame, recv, ec)
    raw = list(recv.content())

    # 在回包里定位 [0x01, 0x25] 帧头
    start = -1
    for i in range(len(raw) - 2):
        if raw[i] == SLAVE_ID and raw[i + 1] == 0x25 and raw[i + 2] == 0xA4:
            start = i
            break

    ok = (start >= 0 and len(raw) >= start + 4 and raw[start + 3] == 0x01)
    print(f"  [rs485_clear_error] 原始回包={[hex(b) for b in raw]}, "
          f"结果={'✅ 成功' if ok else '❌ 失败/超时'}")
    time.sleep(0.005)   # 说明书建议清错后间隔 ≥3ms，留 5ms 余量
    return ok


def set_finger_angle(robot, motor_id: int, angle_deg: float) -> dict:
    """通过 485 向指定电机发送位置控制指令（0x31 帧）。"""
    if motor_id not in MOTOR_ANGLE_RANGE:
        return {"success": False, "motor_id": motor_id, "angle_deg": angle_deg,
                "error_flag": -1, "error_msg": f"motor_id={motor_id} 非法，有效值为 1~4"}
    min_ang, max_ang = MOTOR_ANGLE_RANGE[motor_id]
    if not (min_ang <= angle_deg <= max_ang):
        return {"success": False, "motor_id": motor_id, "angle_deg": angle_deg,
                "error_flag": -1, "error_msg": f"角度 {angle_deg}° 超出范围 [{min_ang}, {max_ang}]"}

    target_pos = int(round(angle_deg * 10))
    payload = [SLAVE_ID, 0x31, 0x04, motor_id,
               target_pos & 0xFF, (target_pos >> 8) & 0xFF, 0x00, 0x00]
    crc_l, crc_h = _modbus_crc16(payload)
    frame = np.array(payload + [crc_l, crc_h], dtype=np.uint8)

    ec = {}
    recv = xCoreSDK_python.PyTypeVectorInt()
    robot.XPRS485SendData(len(frame), 6, frame, recv, ec)
    raw = list(recv.content())

    start = _find_frame_start(raw, SLAVE_ID, 0x31)
    if start < 0 or len(raw) < start + 4:
        return {"success": False, "motor_id": motor_id, "angle_deg": angle_deg,
                "error_flag": -1,
                "error_msg": f"未收到有效响应帧，原始数据: {[hex(b) for b in raw]}"}

    err_flag = raw[start + 3]
    return {"success": err_flag == 0x00, "motor_id": motor_id, "angle_deg": angle_deg,
            "error_flag": err_flag, "error_msg": _parse_error_flag(err_flag)}


def get_finger_angle(robot, motor_id=None) -> dict:
    """通过 485 读取电机角度（0x04 读输入寄存器）。"""
    read_all = (motor_id is None or motor_id == 0)
    if not read_all and motor_id not in MOTOR_ANGLE_RANGE:
        return {"success": False, "motor_id": motor_id, "angle_deg": None,
                "raw_value": None,
                "error_msg": f"motor_id={motor_id} 非法，有效值为 1~4（或 None/0 读全部）"}

    reg_addr, reg_count = 0x0000, 4
    payload = [SLAVE_ID, 0x04,
               (reg_addr >> 8) & 0xFF, reg_addr & 0xFF,
               (reg_count >> 8) & 0xFF, reg_count & 0xFF]
    crc_l, crc_h = _modbus_crc16(payload)
    frame = np.array(payload + [crc_l, crc_h], dtype=np.uint8)

    ec = {}
    recv = xCoreSDK_python.PyTypeVectorInt()
    robot.XPRS485SendData(len(frame), 13, frame, recv, ec)
    raw = list(recv.content())

    start = _find_frame_start(raw, SLAVE_ID, 0x04)
    if start < 0 or len(raw) < start + 3 + 8:
        err_msg = f"未收到有效响应帧，原始数据: {[hex(b) for b in raw]}"
        if read_all:
            return {"success": False, "angles": {}, "error_msg": err_msg}
        return {"success": False, "motor_id": motor_id, "angle_deg": None,
                "raw_value": None, "error_msg": err_msg}

    data_start = start + 3
    all_angles = {}
    for i in range(4):
        hi  = raw[data_start + i * 2]
        lo  = raw[data_start + i * 2 + 1]
        val = (hi << 8) | lo
        if val > 32767:
            val -= 65536
        all_angles[i + 1] = round(val / 100.0, 2)

    if read_all:
        return {"success": True, "angles": all_angles, "error_msg": "正常"}
    raw_val = int(round(all_angles[motor_id] * 100))
    return {"success": True, "motor_id": motor_id,
            "angle_deg": all_angles[motor_id], "raw_value": raw_val, "error_msg": "正常"}


def grasp(robot, auto_clear_error: bool = True, inter_cmd_delay: float = 0.05) -> bool:
    """通过 485 执行抓取动作：手指1/2/3 → 75°，旋转关节 → 25°。

    参数：
        auto_clear_error: 抓取前自动发清错指令（推荐 True）
        inter_cmd_delay : 每条指令之间间隔秒数，避免连发触发堵转保护
    """
    commands = [(1, 75.0), (2, 75.0), (3, 75.0), (4, 25.0)]
    print("\n── 执行抓取动作 ──────────────────────────────")
    if auto_clear_error:
        rs485_clear_error(robot)
    all_success = True
    for motor_id, angle in commands:
        label  = "旋转关节" if motor_id == 4 else f"手指{motor_id}"
        result = set_finger_angle(robot, motor_id, angle)
        status = "✅" if result["success"] else "❌"
        print(f"  {status} Motor_{motor_id} ({label}) -> {angle}° | {result['error_msg']}")
        if not result["success"]:
            all_success = False
        if inter_cmd_delay > 0:
            time.sleep(inter_cmd_delay)
    return all_success


def release(robot, auto_clear_error: bool = True, inter_cmd_delay: float = 0.05) -> bool:
    """通过 485 执行释放动作：手指1/2/3 → 0°，旋转关节 → 25°。

    参数：
        auto_clear_error: 释放前自动发清错指令（推荐 True）
        inter_cmd_delay : 每条指令之间间隔秒数
    """
    print("\n── 执行释放动作 ──────────────────────────────")
    if auto_clear_error:
        rs485_clear_error(robot)
    all_success = True
    for motor_id in [1, 2, 3, 4]:
        label = "旋转关节" if motor_id == 4 else f"手指{motor_id}"
        angle = 25.0 if motor_id == 4 else 0.0
        result = set_finger_angle(robot, motor_id, angle)
        status = "✅" if result["success"] else "❌"
        print(f"  {status} Motor_{motor_id} ({label}) -> {angle}° | {result['error_msg']}")
        if not result["success"]:
            all_success = False
        if inter_cmd_delay > 0:
            time.sleep(inter_cmd_delay)
    return all_success


def recover(robot) -> bool:
    """
    【485】安全恢复函数：清错 → 释放（手指→0°，旋转关节→25°）→ 再次清错。

    用于灵巧手处于堵转/过载锁定，或角度卡在极限位置的情况下,
    把手安全地恢复到初始张开姿态。

    返回 bool：True=最终所有手指都成功回到目标位置。
    """
    print("\n══════════ [485] 安全恢复流程 ══════════")

    # Step 1: 清错
    print("[1/4] 清错（rs485_clear_error）")
    rs485_clear_error(robot)
    time.sleep(0.2)

    # Step 2: 强制释放（不再二次清错，避免来回打架）
    print("\n[2/4] 释放手指到张开姿态")
    success = release(robot, auto_clear_error=False, inter_cmd_delay=0.1)

    # Step 3: 等运动到位
    print("\n[3/4] 等待 2s 让手指运动到位...")
    time.sleep(2.0)

    # Step 4: 读当前角度，再清一次错（保险）
    print("\n[4/4] 读取最终角度并最终清错")
    angles = print_all_angles(robot)
    rs485_clear_error(robot)

    print(f"\n══════════ 恢复完成: {'✅ 成功' if success else '⚠️ 部分手指未到位（可能仍卡住，请人工检查）'} ══════════")
    return success


def can_recover(robot) -> bool:
    """【CAN】安全恢复函数：清错 → 释放 → 再次清错。"""
    print("\n══════════ [CAN] 安全恢复流程 ══════════")
    print("[1/4] 清错（can_clear_error）")
    can_clear_error(robot)
    time.sleep(0.2)

    print("\n[2/4] 释放手指到张开姿态")
    success = can_release(robot)

    print("\n[3/4] 等待 2s 让手指运动到位...")
    time.sleep(2.0)

    print("\n[4/4] 读取最终角度并最终清错")
    angles = can_print_all_angles(robot)
    can_clear_error(robot)

    print(f"\n══════════ 恢复完成: {'✅ 成功' if success else '⚠️ 部分手指未到位（可能仍卡住，请人工检查）'} ══════════")
    return success


# ════════════════════════════════════════════════════════════════
#  485 力控接口（新增）
# ════════════════════════════════════════════════════════════════

def rs485_set_finger_torque(robot, motor_id: int,
                            hall_pos: int = 1000,
                            torque: int = 200) -> dict:
    """
    【方案A：力矩控制模式 - 485】通过 Modbus 0x31 帧 + Mode=0x06 控制电机力矩。
    （说明书 6.6.3 节）

    帧格式：
      [SLAVE_ID, 0x31, 0x06(Mode), motor_id,
       hall_pos_L, hall_pos_H, torque_L, torque_H, CRC_L, CRC_H]

    参数：
        motor_id : 1~4
        hall_pos : 手指[1,3]∈[0,1000]，旋转关节[0,1600]（hall角度*10）
        torque   : [50, 800]（PWM 当量）

    返回 dict（结构同 set_finger_angle）
    """
    if motor_id not in (1, 2, 3, 4):
        return {"success": False, "motor_id": motor_id,
                "error_flag": -1,
                "error_msg": f"motor_id={motor_id} 非法，有效值 1~4"}
    pos_max = 1600 if motor_id == 4 else 1000
    if not (0 <= hall_pos <= pos_max):
        return {"success": False, "motor_id": motor_id,
                "error_flag": -1,
                "error_msg": f"hall_pos={hall_pos} 超出范围 [0,{pos_max}]"}
    if not (50 <= torque <= 800):
        return {"success": False, "motor_id": motor_id,
                "error_flag": -1,
                "error_msg": f"torque={torque} 超出范围 [50,800]"}

    payload = [SLAVE_ID, 0x31, 0x06, motor_id,
               hall_pos & 0xFF, (hall_pos >> 8) & 0xFF,
               torque   & 0xFF, (torque   >> 8) & 0xFF]
    crc_l, crc_h = _modbus_crc16(payload)
    frame = np.array(payload + [crc_l, crc_h], dtype=np.uint8)

    ec = {}
    recv = xCoreSDK_python.PyTypeVectorInt()
    robot.XPRS485SendData(len(frame), 6, frame, recv, ec)
    raw = list(recv.content())

    start = _find_frame_start(raw, SLAVE_ID, 0x31)
    if start < 0 or len(raw) < start + 4:
        return {"success": False, "motor_id": motor_id, "error_flag": -1,
                "error_msg": f"未收到有效响应帧: {[hex(b) for b in raw]}"}
    err_flag = raw[start + 3]
    return {"success": err_flag == 0x00, "motor_id": motor_id,
            "hall_pos": hall_pos, "torque": torque,
            "error_flag": err_flag, "error_msg": _parse_error_flag(err_flag)}


def rs485_grasp_torque(robot, hall_pos: int = 1000, torque: int = 200,
                       rotate_joint_angle: float = 25.0,
                       rotate_settle_s: float = 0.5,
                       auto_clear_error: bool = True,
                       inter_cmd_delay: float = 0.05) -> bool:
    """
    【方案A - 485】力矩模式抓取。

    执行顺序（重要）：
      1) 旋转关节先就位 → 让另两指对齐拇指（决定抓取姿态）
      2) 等待 rotate_settle_s 让旋转到位
      3) 三指进入力矩模式合拢到 hall_pos

    参数：
        hall_pos           : 目标霍尔位置（默认 1000，即完全合拢）
        torque             : 力矩值，[50,800]，默认 200
        rotate_joint_angle : 旋转关节角度（默认 25°）
        rotate_settle_s    : 旋转关节就位等待秒数（默认 0.5s）
    """
    print(f"\n── [485] 力矩抓取  hall_pos={hall_pos}, torque={torque}, rotate={rotate_joint_angle}° ──")
    if auto_clear_error:
        rs485_clear_error(robot)

    all_success = True

    # 步骤1：旋转关节先就位
    result = set_finger_angle(robot, 4, rotate_joint_angle)
    status = "✅" if result["success"] else "❌"
    print(f"  [步骤1] {status} Motor_4 (旋转关节) -> {rotate_joint_angle}° | {result['error_msg']}")
    if not result["success"]:
        all_success = False

    # 步骤2：等待旋转到位
    print(f"  [步骤2] 等待 {rotate_settle_s}s 让旋转关节就位...")
    time.sleep(rotate_settle_s)

    # 步骤3：三指力矩模式合拢
    print(f"  [步骤3] 三指进入力矩模式（torque={torque}）合拢")
    for motor_id in (1, 2, 3):
        result = rs485_set_finger_torque(robot, motor_id, hall_pos, torque)
        status = "✅" if result["success"] else "❌"
        print(f"    {status} Motor_{motor_id} (手指{motor_id}) 力矩={torque} | {result['error_msg']}")
        if not result["success"]:
            all_success = False
        time.sleep(inter_cmd_delay)

    return all_success


def rs485_set_max_current(robot, motor_id: int, current_ma: int = 250) -> bool:
    """
    【方案B：最大电流限制 - 485】全局设置 0x25 + 子命令 0x65。
    （说明书 6.5.2 节）

    帧格式：
      [0x01, 0x25, 0x65, motor_id, 0x02, cur_L, cur_H, 0x00, 0x00, CRC_L, CRC_H]
    回包：
      [0x01, 0x25, 0x65, 0x01, CRC_L, CRC_H]   # Byte3=0x01 表示成功

    参数：
        motor_id   : 1~4
        current_ma : 最大电流(mA)，[200, 500]，默认 250

    返回 bool。
    """
    if motor_id not in (1, 2, 3, 4):
        print(f"  ❌ motor_id={motor_id} 非法")
        return False
    if not (200 <= current_ma <= 500):
        print(f"  ❌ current_ma={current_ma} 超出范围 [200,500]")
        return False

    payload = [SLAVE_ID, 0x25, 0x65, motor_id, 0x02,
               current_ma & 0xFF, (current_ma >> 8) & 0xFF,
               0x00, 0x00]
    crc_l, crc_h = _modbus_crc16(payload)
    frame = np.array(payload + [crc_l, crc_h], dtype=np.uint8)

    ec = {}
    recv = xCoreSDK_python.PyTypeVectorInt()
    robot.XPRS485SendData(len(frame), 6, frame, recv, ec)
    raw = list(recv.content())

    # 在回包里定位 [0x01, 0x25, 0x65] 帧头
    start = -1
    for i in range(len(raw) - 2):
        if raw[i] == SLAVE_ID and raw[i + 1] == 0x25 and raw[i + 2] == 0x65:
            start = i
            break
    ok = (start >= 0 and len(raw) >= start + 4 and raw[start + 3] == 0x01)
    print(f"  [rs485_set_max_current motor={motor_id} cur={current_ma}mA] "
          f"原始回包={[hex(b) for b in raw]}, "
          f"结果={'✅ 成功' if ok else '❌ 失败/超时'}")
    return ok


def rs485_get_tactile(robot, sensor_id: int = 1) -> dict:
    """
    【触觉读取 - 485】读取压力传感器数据。
    （说明书 6.7.3.2 节）

    传感器基地址：sensor_id=1 → 0x10, 2 → 0x20, 3 → 0x30
    偏移地址：
      0x00~0x01 法向力 Float32
      0x04~0x05 切向力 Float32

    用 Modbus 0x04 读输入寄存器，一次读 6 个寄存器（12 字节）。

    参数：
        sensor_id : 1, 2, 3

    返回 dict：
        {
            "success"          : bool,
            "sensor_id"        : int,
            "normal_force"     : float,
            "tangential_force" : float,
            "error_msg"        : str,
        }
    """
    if sensor_id not in (1, 2, 3):
        return {"success": False, "sensor_id": sensor_id,
                "normal_force": None, "tangential_force": None,
                "error_msg": f"sensor_id={sensor_id} 非法，有效值 1~3"}

    base_addr = sensor_id * 0x10   # 1->0x10, 2->0x20, 3->0x30
    reg_count = 6                  # 0x00~0x05 → 6个寄存器

    payload = [SLAVE_ID, 0x04,
               (base_addr >> 8) & 0xFF, base_addr & 0xFF,
               (reg_count >> 8) & 0xFF, reg_count & 0xFF]
    crc_l, crc_h = _modbus_crc16(payload)
    frame = np.array(payload + [crc_l, crc_h], dtype=np.uint8)

    ec = {}
    recv = xCoreSDK_python.PyTypeVectorInt()
    expected_len = 5 + reg_count * 2   # 1+1+1 头 + 数据 + 2 CRC
    robot.XPRS485SendData(len(frame), expected_len, frame, recv, ec)
    raw = list(recv.content())

    start = _find_frame_start(raw, SLAVE_ID, 0x04)
    if start < 0 or len(raw) < start + 3 + reg_count * 2:
        return {"success": False, "sensor_id": sensor_id,
                "normal_force": None, "tangential_force": None,
                "error_msg": f"触觉读取失败: {[hex(b) for b in raw]}"}

    data_start = start + 3
    # Modbus 寄存器是大端 16-bit，组合成 Float 时按"高寄存器在前"或"低寄存器在前"两种
    # 都试一遍，取看上去合理（0~20N）的一个
    # 规则：寄存器0(L)+寄存器1(H) → Float32 小端 = bytes [hi0,lo0,hi1,lo1] 重排
    def _u16(idx):
        return (raw[data_start + idx * 2] << 8) | raw[data_start + idx * 2 + 1]

    # 法向力：寄存器 0,1
    nf_lo = _u16(0)
    nf_hi = _u16(1)
    # 切向力：寄存器 4,5
    tf_lo = _u16(4)
    tf_hi = _u16(5)

    # 按说明书"_L"在前 → 低字寄存器先：bytes = [nf_lo_LE, nf_hi_LE]
    nf_bytes = struct.pack('<HH', nf_lo, nf_hi)
    tf_bytes = struct.pack('<HH', tf_lo, tf_hi)
    normal_force     = struct.unpack('<f', nf_bytes)[0]
    tangential_force = struct.unpack('<f', tf_bytes)[0]

    return {"success": True, "sensor_id": sensor_id,
            "normal_force": round(normal_force, 4),
            "tangential_force": round(tangential_force, 4),
            "error_msg": "正常"}


def rs485_grasp_force_feedback(
    robot,
    target_force_n: float = 2.0,
    finger_ids=(1, 2, 3),
    sensor_ids=(1, 2, 3),
    max_angle_deg: float = 75.0,
    angle_step_deg: float = 2.0,
    poll_interval_s: float = 0.05,
    timeout_s: float = 8.0,
    rotate_joint_angle: float = 25.0,
) -> dict:
    """
    【方案C：触觉闭环力控抓取 - 485】
    边小步合拢手指边读触觉，达到目标法向力即停止。

    流程：
      1. 清错 → 旋转关节就位
      2. 三指角度 0° 起步，每周期增加 angle_step_deg
      3. 分别读取三个触觉传感器，若某指 normal_force ≥ target_force_n
         → 该指立即锁定，不再增加角度；其余手指继续推进
      4. 全部锁定 / 全部到达 max_angle_deg / 超时 → 退出

    注意：相比 CAN 版本，485 可以分别读取三个传感器，因此可以做到"按指锁定"。

    参数：
        target_force_n     : 目标法向力（N），默认 2.0
        finger_ids         : 参与抓取的手指 id（默认 (1,2,3)）
        sensor_ids         : 与 finger_ids 一一对应的触觉传感器 id（默认 (1,2,3)）
        max_angle_deg      : 单指最大合拢角度（默认 75°）
        angle_step_deg     : 每周期角度增量（默认 2°）
        poll_interval_s    : 控制周期（默认 0.05s = 20Hz）
        timeout_s          : 整体超时（默认 8s）
        rotate_joint_angle : 旋转关节角度（默认 25°）

    返回 dict：
        {
            "success"          : bool,           # 是否至少有一指接触到力阈
            "locked_fingers"   : [int],          # 已锁定的手指列表
            "final_angles"     : {1: float, ...},
            "final_forces_n"   : {1: float, ...},# 每指最终法向力
            "elapsed_s"        : float,
        }
    """
    print(f"\n══════════ [485] 触觉闭环力控抓取  目标={target_force_n}N ══════════")
    rs485_clear_error(robot)
    time.sleep(0.05)

    # 旋转关节先就位
    set_finger_angle(robot, 4, rotate_joint_angle)
    time.sleep(0.1)

    # finger_id → sensor_id 的映射
    fid2sid = dict(zip(finger_ids, sensor_ids))

    cur_angles  = {fid: 0.0 for fid in finger_ids}
    last_forces = {fid: 0.0 for fid in finger_ids}
    locked      = set()
    start_t     = time.perf_counter()

    while True:
        elapsed = time.perf_counter() - start_t
        if elapsed > timeout_s:
            print(f"  ⏱  超时 {timeout_s}s，结束")
            break

        # 1) 推进未锁定的手指
        all_at_max = True
        for fid in finger_ids:
            if fid in locked:
                continue
            new_ang = min(cur_angles[fid] + angle_step_deg, max_angle_deg)
            if new_ang < max_angle_deg:
                all_at_max = False
            cur_angles[fid] = new_ang
            res = set_finger_angle(robot, fid, new_ang)
            if not res["success"]:
                print(f"  ⚠️  Motor_{fid} 推进失败: {res['error_msg']}，立即锁定")
                locked.add(fid)

        # 2) 读各指对应的触觉
        for fid in finger_ids:
            if fid in locked:
                continue
            sid = fid2sid[fid]
            tac = rs485_get_tactile(robot, sensor_id=sid)
            if tac["success"] and tac["normal_force"] is not None:
                last_forces[fid] = tac["normal_force"]
                if tac["normal_force"] >= target_force_n:
                    locked.add(fid)
                    print(f"  🎯 Motor_{fid} 力 {tac['normal_force']:.2f}N "
                          f"≥ {target_force_n}N，锁定")

        # 3) 打印一次状态
        angle_snap = {k: round(v, 1) for k, v in cur_angles.items()}
        force_snap = {k: round(v, 2) for k, v in last_forces.items()}
        print(f"  t={elapsed:5.2f}s  angles={angle_snap}  "
              f"forces={force_snap}N  locked={sorted(locked)}")

        # 4) 退出条件
        if len(locked) >= len(finger_ids):
            break
        if all_at_max:
            print(f"  📐 所有手指已到达最大角度 {max_angle_deg}°，结束")
            break

        time.sleep(poll_interval_s)

    elapsed_total = time.perf_counter() - start_t
    success = (len(locked) >= 1)   # 至少一指接触

    result = {
        "success"        : success,
        "locked_fingers" : sorted(locked),
        "final_angles"   : {k: round(v, 2) for k, v in cur_angles.items()},
        "final_forces_n" : {k: round(v, 3) for k, v in last_forces.items()},
        "elapsed_s"      : round(elapsed_total, 3),
    }
    verdict = "✅ 力控抓取成功" if success else "⚠️ 未达到目标力（可能未触物）"
    max_f = max(last_forces.values()) if last_forces else 0.0
    print(f"\n══════════ {verdict}  耗时={elapsed_total:.2f}s  最大法向力={max_f:.2f}N ══════════")
    print(f"  最终角度: {result['final_angles']}")
    print(f"  最终力值: {result['final_forces_n']}")
    return result


def print_all_angles(robot) -> dict:
    """通过 485 读取并打印全部4个电机的当前角度。"""
    print("\n── 读取全部手指角度 ──────────────────────────")
    result = get_finger_angle(robot, motor_id=None)
    if not result["success"]:
        print(f"  ❌ 读取失败: {result['error_msg']}")
        return {}
    for mid, deg in result["angles"].items():
        label = "旋转关节" if mid == 4 else f"手指{mid}"
        print(f"  Motor_{mid} ({label}): {deg:.2f}°")
    return result["angles"]


# ════════════════════════════════════════════════════════════════
#  入口示例
# ════════════════════════════════════════════════════════════════

if __name__ == "__main__":
    ip    = "10.6.2.18"
    robot = xCoreSDK_python.ArRobot(ip)
    ec    = {}

    # ── 选择通信方式 ──────────────────────────────────────────────
    USE_CAN = False   # True=CAN, False=485

    # ── 选择执行动作 ──────────────────────────────────────────────
    # "recover"        : 安全恢复（清错 + 释放，把手张开到 0°）
    # "demo"           : 普通位置模式 抓取/释放循环演示
    # "release"        : 仅释放
    # "grasp"          : 仅抓取
    # "force_torque"   : 【方案A】力矩模式抓取（默认 torque=200）
    # "force_current"  : 【方案B】先设置最大电流上限，再普通位置抓取
    # "force_feedback" : 【方案C】触觉闭环力控抓取（默认目标 2.0 N）
    ACTION = "recover"

    # 力控参数
    FORCE_TORQUE       = 200    # 方案A 力矩值 [50,800]
    FORCE_HALL_POS     = 1000   # 方案A 目标霍尔位置（合拢）
    FORCE_MAX_CURRENT  = 250    # 方案B 最大电流 mA [200,500]
    FORCE_TARGET_N     = 2.0    # 方案C 目标法向力 N

    if not USE_CAN:
        # 485 路径需要先打开 24V 供电
        robot.setxPanelRS485(xCoreSDK_python.xPanelOptVout.supply24v, True, ec)

    print(f"\n>>> 通信方式: {'CAN' if USE_CAN else '485'}    动作: {ACTION}")

    if ACTION == "recover":
        # 安全恢复：清错 → 释放 → 再清错 → 读角度
        if USE_CAN:
            can_clear_error(robot)
            can_recover(robot)
        else:
            recover(robot)

    elif ACTION == "release":
        if USE_CAN:
            can_clear_error(robot)
            success = can_release(robot)
            time.sleep(2.0)
            can_print_all_angles(robot)
        else:
            success = release(robot)
            time.sleep(2.0)
            print_all_angles(robot)
        print(f"\n释放结果: {'✅ 成功' if success else '❌ 部分失败'}")

    elif ACTION == "grasp":
        if USE_CAN:
            can_clear_error(robot)
            success = can_grasp(robot)
            time.sleep(2.0)
            can_print_all_angles(robot)
        else:
            success = grasp(robot)
            time.sleep(2.0)
            print_all_angles(robot)
        print(f"\n抓取结果: {'✅ 成功' if success else '❌ 部分失败'}")

    elif ACTION == "demo":
        # 完整演示：先恢复 → 抓取 → 等 → 释放
        if USE_CAN:
            can_recover(robot)
            time.sleep(1.0)
            success = can_grasp(robot)
            print(f"\n抓取结果: {'✅ 成功' if success else '❌ 部分失败'}")
            time.sleep(3.0)
            can_print_all_angles(robot)
            success = can_release(robot)
            print(f"\n释放结果: {'✅ 成功' if success else '❌ 部分失败'}")
            time.sleep(3.0)
            can_print_all_angles(robot)
        else:
            recover(robot)
            time.sleep(1.0)
            success = grasp(robot)
            print(f"\n抓取结果: {'✅ 成功' if success else '❌ 部分失败'}")
            time.sleep(3.0)
            print_all_angles(robot)
            success = release(robot)
            print(f"\n释放结果: {'✅ 成功' if success else '❌ 部分失败'}")
            time.sleep(3.0)
            print_all_angles(robot)

    elif ACTION == "force_torque":
        # 方案A：力矩模式抓取
        if USE_CAN:
            success = can_grasp_torque(robot, hall_pos=FORCE_HALL_POS,
                                       torque=FORCE_TORQUE)
            time.sleep(2.0)
            can_print_all_angles(robot)
        else:
            success = rs485_grasp_torque(robot, hall_pos=FORCE_HALL_POS,
                                         torque=FORCE_TORQUE)
            time.sleep(2.0)
            print_all_angles(robot)
        print(f"\n力矩抓取结果: {'✅ 成功' if success else '❌ 部分失败'}")

    elif ACTION == "force_current":
        # 方案B：先给三指设置最大电流上限，再做普通位置抓取
        print(f"\n--- 步骤1: 设置三指最大电流为 {FORCE_MAX_CURRENT}mA ---")
        if USE_CAN:
            can_clear_error(robot)
            for mid in (1, 2, 3):
                can_set_max_current(robot, mid, FORCE_MAX_CURRENT)
                time.sleep(0.05)
            print(f"\n--- 步骤2: 普通位置抓取 ---")
            success = can_grasp(robot)
            time.sleep(2.0)
            can_print_all_angles(robot)
        else:
            rs485_clear_error(robot)
            for mid in (1, 2, 3):
                rs485_set_max_current(robot, mid, FORCE_MAX_CURRENT)
                time.sleep(0.05)
            print(f"\n--- 步骤2: 普通位置抓取 ---")
            success = grasp(robot)
            time.sleep(2.0)
            print_all_angles(robot)
        print(f"\n电流限制抓取结果: {'✅ 成功' if success else '❌ 部分失败'}")

    elif ACTION == "force_feedback":
        # 方案C：触觉闭环力控抓取
        if USE_CAN:
            result = can_grasp_force_feedback(robot,
                                              target_force_n=FORCE_TARGET_N)
            time.sleep(1.0)
            can_print_all_angles(robot)
        else:
            result = rs485_grasp_force_feedback(robot,
                                                target_force_n=FORCE_TARGET_N)
            time.sleep(1.0)
            print_all_angles(robot)
        print(f"\n触觉闭环抓取最终结果: {result}")

    else:
        print(f"❌ 未知 ACTION='{ACTION}'，可选: "
              f"recover / release / grasp / demo / "
              f"force_torque / force_current / force_feedback")
