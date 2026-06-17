"""
DexHand021S — 基于 485/Modbus RTU 的完整控制层
通信方式: XPRS485SendData (raw frame) + XPRWModbusRTUReg
从机地址: 0x01
波特率:   已由控制器固件配置，无需手动设置
"""
import setup_path
import struct
import platform
import time

if platform.system() == 'Windows':
    from Release.windows import xCoreSDK_python
elif platform.system() == 'Linux':
    from Release.linux import xCoreSDK_python
else:
    raise ImportError("Unsupported OS")

# ════════════════════════════════════════════════════════════════
#  常量
# ════════════════════════════════════════════════════════════════

SLAVE_ADDR = 0x01

# ── 寄存器地址（需根据灵巧手说明书确认，以下为探测起点）──────────
# 先用扫描结果推断，后续对照说明书修正
REG_FW_VERSION   = 0x0001   # 固件版本
REG_DEVICE_ID    = 0x0002   # 设备ID

# 控制寄存器（写）
REG_CTRL_MODE    = 0x0100   # 控制模式
REG_MOTOR1_POS   = 0x0101   # 电机1目标位置 (×10 = 度)
REG_MOTOR2_POS   = 0x0102
REG_MOTOR3_POS   = 0x0103
REG_MOTOR4_POS   = 0x0104
REG_CTRL_EXEC    = 0x0110   # 执行控制指令

# 状态寄存器（读）
REG_STATUS_BASE  = 0x0200   # 状态寄存器起始
REG_MOTOR1_ANGLE = 0x0200   # 电机1实际角度
REG_MOTOR2_ANGLE = 0x0201
REG_MOTOR3_ANGLE = 0x0202
REG_MOTOR4_ANGLE = 0x0203
REG_MOTOR1_CURR  = 0x0210   # 电机1电流
REG_MOTOR2_CURR  = 0x0211
REG_MOTOR3_CURR  = 0x0212
REG_MOTOR4_CURR  = 0x0213
REG_ERROR_FLAG   = 0x0220   # 错误标志

MOTOR_ANGLE_RANGE = {1: (0.0, 75.0), 2: (0.0, 75.0),
                     3: (0.0, 75.0), 4: (0.0, 108.0)}

# ════════════════════════════════════════════════════════════════
#  底层 485 收发
# ════════════════════════════════════════════════════════════════

def _crc16(data: list) -> tuple:
    crc = 0xFFFF
    for b in data:
        crc ^= b
        for _ in range(8):
            crc = (crc >> 1) ^ 0xA001 if crc & 1 else crc >> 1
    return crc & 0xFF, (crc >> 8) & 0xFF


def _raw_send(robot, send_data: list, recv_len: int) -> tuple:
    """
    发送原始 Modbus RTU 帧，返回 (ec_dict, resp_bytes)。
    resp_bytes 是去掉前导零填充后的真实响应。
    """
    rev = xCoreSDK_python.PyTypeVectorInt()
    ec  = {}
    robot.XPRS485SendData(len(send_data), recv_len, send_data, rev, ec)
    raw = rev.get()

    # SDK 会在响应前填充若干 0x00，真实帧从最后 recv_len 字节开始
    if ec.get('ec') == 0 and len(raw) >= recv_len:
        resp = raw[-recv_len:]
    else:
        resp = []
    return ec, resp


def _fc03_read(robot, slave: int, reg_addr: int, num: int) -> tuple:
    """FC03 读保持寄存器，返回 (ok: bool, values: list[int])。"""
    frame = [slave, 0x03,
             (reg_addr >> 8) & 0xFF, reg_addr & 0xFF,
             (num >> 8) & 0xFF,      num & 0xFF]
    cl, ch = _crc16(frame)
    frame += [cl, ch]

    recv_len = 5 + num * 2   # 从机地址(1)+功能码(1)+字节数(1)+数据(num*2)+CRC(2)
    ec, resp = _raw_send(robot, frame, recv_len)

    if ec.get('ec') != 0 or len(resp) < recv_len:
        return False, []

    # 验证 CRC
    cl2, ch2 = _crc16(resp[:-2])
    if cl2 != resp[-2] or ch2 != resp[-1]:
        print(f"  [WARN] CRC 校验失败: {[hex(b) for b in resp]}")
        return False, []

    # 解析数据
    byte_count = resp[2]
    values = []
    for i in range(num):
        hi = resp[3 + i * 2]
        lo = resp[4 + i * 2]
        values.append((hi << 8) | lo)
    return True, values


def _fc06_write(robot, slave: int, reg_addr: int, value: int) -> bool:
    """FC06 写单个保持寄存器。"""
    frame = [slave, 0x06,
             (reg_addr >> 8) & 0xFF, reg_addr & 0xFF,
             (value   >> 8) & 0xFF,  value   & 0xFF]
    cl, ch = _crc16(frame)
    frame += [cl, ch]

    ec, resp = _raw_send(robot, frame, 8)
    return ec.get('ec') == 0 and len(resp) >= 6


def _fc16_write(robot, slave: int, reg_addr: int, values: list) -> bool:
    """FC16 写多个保持寄存器。"""
    n     = len(values)
    frame = [slave, 0x10,
             (reg_addr >> 8) & 0xFF, reg_addr & 0xFF,
             0x00, n,
             n * 2]
    for v in values:
        frame += [(v >> 8) & 0xFF, v & 0xFF]
    cl, ch = _crc16(frame)
    frame += [cl, ch]

    ec, resp = _raw_send(robot, frame, 8)
    return ec.get('ec') == 0 and len(resp) >= 6


# ════════════════════════════════════════════════════════════════
#  寄存器扫描（用于确认实际寄存器映射）
# ════════════════════════════════════════════════════════════════

def scan_registers(robot, start=0x0000, end=0x0050, chunk=10):
    """扫描寄存器范围，打印非零值。"""
    print(f"\n── 扫描寄存器 {hex(start)}~{hex(end)} ────────────────")
    addr = start
    while addr <= end:
        n = min(chunk, end - addr + 1)
        ok, vals = _fc03_read(robot, SLAVE_ADDR, addr, n)
        if ok:
            for i, v in enumerate(vals):
                reg = addr + i
                mark = " ◀ 非零" if v != 0 else ""
                print(f"  [{hex(reg):>6}] = {v:>6}  (0x{v:04X}){mark}")
        else:
            print(f"  [{hex(addr):>6}] 读取失败")
        addr += n
        time.sleep(0.02)


# ════════════════════════════════════════════════════════════════
#  控制接口
# ════════════════════════════════════════════════════════════════

def hand_set_angle(robot, motor_id: int, angle_deg: float) -> dict:
    """设置单个电机角度（寄存器地址待确认后修正）。"""
    if motor_id not in MOTOR_ANGLE_RANGE:
        return {"success": False, "error": f"motor_id={motor_id} 非法"}
    lo, hi = MOTOR_ANGLE_RANGE[motor_id]
    if not (lo <= angle_deg <= hi):
        return {"success": False,
                "error": f"角度 {angle_deg}° 超出范围 [{lo}, {hi}]"}

    reg  = REG_MOTOR1_POS + (motor_id - 1)
    val  = int(round(angle_deg * 10))   # 0.1° 精度
    ok   = _fc06_write(robot, SLAVE_ADDR, reg, val)
    return {"success": ok, "motor_id": motor_id,
            "angle_deg": angle_deg, "reg": hex(reg), "val": val}


def hand_get_angles(robot) -> dict:
    """读取全部4电机实际角度。"""
    ok, vals = _fc03_read(robot, SLAVE_ADDR, REG_MOTOR1_ANGLE, 4)
    if not ok:
        return {"success": False, "angles": {}}
    angles = {mid: vals[mid-1] / 10.0 for mid in range(1, 5)}
    return {"success": True, "angles": angles}


def hand_get_currents(robot) -> dict:
    """读取全部4电机电流 mA。"""
    ok, vals = _fc03_read(robot, SLAVE_ADDR, REG_MOTOR1_CURR, 4)
    if not ok:
        return {"success": False, "currents": {}}
    return {"success": True,
            "currents": {mid: vals[mid-1] for mid in range(1, 5)}}


def hand_get_error(robot) -> dict:
    """读取错误标志寄存器。"""
    ok, vals = _fc03_read(robot, SLAVE_ADDR, REG_ERROR_FLAG, 1)
    if not ok:
        return {"success": False, "error_flag": -1}
    flag = vals[0]
    errors = []
    if flag & 0x0001: errors.append("电流异常")
    if flag & 0x0002: errors.append("电压异常")
    if flag & 0x0004: errors.append("温度异常")
    if flag & 0x0008: errors.append("堵转/过载")
    return {"success": True, "error_flag": flag,
            "errors": errors if errors else ["正常"]}


def hand_grasp(robot) -> bool:
    """抓取动作。"""
    print("\n── 执行抓取 ──────────────────────────────────")
    all_ok = True
    for mid, ang in [(1, 75.0), (2, 75.0), (3, 75.0), (4, 25.0)]:
        r = hand_set_angle(robot, mid, ang)
        s = "✅" if r["success"] else "❌"
        print(f"  {s} Motor_{mid} → {ang}°")
        if not r["success"]: all_ok = False
        time.sleep(0.05)
    return all_ok


def hand_release(robot) -> bool:
    """释放动作。"""
    print("\n── 执行释放 ──────────────────────────────────")
    all_ok = True
    for mid in range(1, 5):
        ang = 25.0 if mid == 4 else 0.0
        r   = hand_set_angle(robot, mid, ang)
        s   = "✅" if r["success"] else "❌"
        print(f"  {s} Motor_{mid} → {ang}°")
        if not r["success"]: all_ok = False
        time.sleep(0.05)
    return all_ok


# ════════════════════════════════════════════════════════════════
#  入口：先扫描确认寄存器映射，再测试控制
# ════════════════════════════════════════════════════════════════

if __name__ == "__main__":
    ip    = "192.168.2.160"
    robot = xCoreSDK_python.ArRobot(ip)
    SLAVE_ADDR = 0x01

    def crc16(data):
        crc = 0xFFFF
        for b in data:
            crc ^= b
            for _ in range(8):
                crc = (crc >> 1) ^ 0xA001 if crc & 1 else crc >> 1
        return crc & 0xFF, (crc >> 8) & 0xFF

    def read1(reg) -> int | None:
        """读单个寄存器，返回值或 None。"""
        frame = [SLAVE_ADDR, 0x03,
                 (reg >> 8) & 0xFF, reg & 0xFF, 0x00, 0x01]
        cl, ch = crc16(frame)
        frame += [cl, ch]
        rev = xCoreSDK_python.PyTypeVectorInt()
        ec  = {}
        robot.XPRS485SendData(len(frame), 7, frame, rev, ec)
        raw = rev.get()
        if ec.get('ec') == 0 and len(raw) >= 7:
            r = raw[-7:]
            return (r[3] << 8) | r[4]
        return None

    def write1(reg, val) -> bool:
        """写单个寄存器，返回是否成功。"""
        frame = [SLAVE_ADDR, 0x06,
                 (reg >> 8) & 0xFF, reg & 0xFF,
                 (val >> 8) & 0xFF, val & 0xFF]
        cl, ch = crc16(frame)
        frame += [cl, ch]
        rev = xCoreSDK_python.PyTypeVectorInt()
        ec  = {}
        robot.XPRS485SendData(len(frame), 7, frame, rev, ec)
        return ec.get('ec') == 0

    # ══════════════════════════════════════════════════════════
    # Step1: 验证写入是否真的让手指运动
    #        写 0x0003~0x0005（上次写100后读回100，最可疑）
    # ══════════════════════════════════════════════════════════
    print("=== Step1: 写角度值，观察手指是否运动 ===")
    print("  当前值:")
    for reg in range(0x00, 0x16):
        v = read1(reg)
        if v is not None and v not in (0, 0xFFFF):
            print(f"    [{hex(reg)}] = {v}")

    print("\n  写 0x03=750 (75.0°), 0x04=750, 0x05=750, 0x06=1080 ...")
    print("  ⚠️  请观察手指是否运动！")
    write1(0x03, 750)
    time.sleep(0.5)
    write1(0x04, 750)
    time.sleep(0.5)
    write1(0x05, 750)
    time.sleep(0.5)
    write1(0x06, 250)
    time.sleep(2.0)

    print("\n  写后读回:")
    for reg in [0x03, 0x04, 0x05, 0x06]:
        v = read1(reg)
        print(f"    [{hex(reg)}] = {v}")

    print("\n  复位 → 0 ...")
    for reg in [0x03, 0x04, 0x05]:
        write1(reg, 0)
        time.sleep(0.3)
    write1(0x06, 0)
    time.sleep(2.0)

    # ══════════════════════════════════════════════════════════
    # Step2: 写 0x0010~0x0014（另一组候选）
    # ══════════════════════════════════════════════════════════
    print("\n=== Step2: 写 0x10~0x14，观察手指运动 ===")
    print("  当前值:")
    for reg in [0x10, 0x11, 0x12, 0x13, 0x14]:
        v = read1(reg)
        print(f"    [{hex(reg)}] = {v}")

    print("\n  写 0x10=750, 0x11=750, 0x12=750, 0x13=750 ...")
    for reg in [0x10, 0x11, 0x12, 0x13]:
        write1(reg, 750)
        time.sleep(0.5)

    print("  写后读回:")
    for reg in [0x10, 0x11, 0x12, 0x13, 0x14]:
        v = read1(reg)
        print(f"    [{hex(reg)}] = {v}")
    time.sleep(2.0)

    print("  复位 → 0 ...")
    for reg in [0x10, 0x11, 0x12, 0x13]:
        write1(reg, 0)
        time.sleep(0.3)
    time.sleep(2.0)

    # ══════════════════════════════════════════════════════════
    # Step3: 写 0x0015（状态/使能寄存器？）后再写角度
    # ══════════════════════════════════════════════════════════
    print("\n=== Step3: 先写使能(0x15=1)，再写角度 ===")
    write1(0x15, 1)
    time.sleep(0.1)
    print("  使能后写 0x03=500 (50°) ...")
    write1(0x03, 500)
    time.sleep(1.0)
    v = read1(0x03)
    print(f"  读回 0x03 = {v}")
    time.sleep(2.0)
    write1(0x03, 0)
    time.sleep(1.0)

    # ══════════════════════════════════════════════════════════
    # Step4: 扫描更大范围（逐个，recv_len=7）
    #        0x0020 ~ 0x00FF
    # ══════════════════════════════════════════════════════════
    print("\n=== Step4: 扫描 0x0020~0x00FF（逐个）===")
    found = {}
    for reg in range(0x0020, 0x0100):
        v = read1(reg)
        if v is not None:
            found[reg] = v
            if v not in (0, 0xFFFF):
                print(f"  [{hex(reg):>6}] = {v:>6}  (0x{v:04X}) ◀")
            # else: 静默
        time.sleep(0.008)

    print(f"\n  可读寄存器总数: {len(found)}")
    print(f"  非零非FFFF: {[(hex(k), v) for k,v in found.items() if v not in (0,0xFFFF)]}")

    print("\n完成")


