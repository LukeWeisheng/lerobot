# -*- coding: utf-8 -*-
"""
@file: ar_example.py
@brief: AR机型示例，点位基于AR5-5_0.7L-W4C1C1,侧装，A=90°
@copyright: Copyright (C) 2025 ROKAE (Beijing) Technology Co., LTD. All Rights Reserved.
Information in this file is the intellectual property of Rokae Technology Co., Ltd,
And may contains trade secrets that must be stored and viewed confidentially.
"""
import setup_path
import platform
import time
import math

# 根据操作系统导入相应的模块
if platform.system() == "Windows":
    from Release.windows import xCoreSDK_python
    from Release.windows.xCoreSDK_python import utility
    from Release.windows.xCoreSDK_python import RtControllerMode
elif platform.system() == "Linux":
    from Release.linux import xCoreSDK_python
    from Release.linux.xCoreSDK_python import utility
    from Release.linux.xCoreSDK_python import RtControllerMode
else:
    raise ImportError("Unsupported operating system")
from log import print_log, print_separator
from move_example import wait_robot

M_PI = math.pi

def rad2deg(rad):
    return rad * 180 / M_PI

def m2mm(m):
    return m * 1000

def print_cart_pose(cart_pose: xCoreSDK_python.CartesianPosition):
    [
        print(f"elbow,{rad2deg(cart_pose.elbow)}"),  # 臂角
        print(f"hasElbow,{cart_pose.hasElbow}"),  # 是否有臂角
        print(f"confData,{','.join(map(str,cart_pose.confData))}"),  # conf数据
        print(f"external,{','.join(map(str,cart_pose.external))}"),  # 外部轴数据
        print(f"trans,{','.join(map(str,map(m2mm,cart_pose.trans)))}"),  # xyz
        print(f"rpy,{','.join(map(str,map(rad2deg,cart_pose.rpy)))}"),  # abc
    ]

# 获取关节位置，前7位是机械臂关节位置，后面是附加轴
def get_jointpos(robot: xCoreSDK_python.ArRobot, ec: dict):
    joint_pos = robot.jointPos(ec)
    print_log("jointPos", ec)
    print(f"joint_pos: {list(map(rad2deg,joint_pos))}")


# 获取笛卡尔位置
def get_cartpos(robot: xCoreSDK_python.ArRobot, ec: dict):
    pos = robot.posture(
        xCoreSDK_python.CoordinateType.endInRef, ec
    )  # 获取工具基于参考坐标系的笛卡尔位置
    print_log("posture", ec)
    print(f"pos trans: {list(map(m2mm,pos[:3]))}")
    print(f"pos rpy: {list(map(rad2deg,pos[3:]))}")

    # 获取笛卡尔位置，数据更多
    cart_posture = robot.cartPosture(xCoreSDK_python.CoordinateType.endInRef, ec)
    print_log("cartPosture", ec)
    print_cart_pose(cart_posture)


# 正解，关节角度->笛卡尔坐标
def calcFk(robot: xCoreSDK_python.ArRobot, ec: dict):
    start_angle = robot.jointPos(ec)[:7]
    robot_model: xCoreSDK_python.model.Model_1_7 = robot.model()
    cart_pose = robot_model.calcFk(start_angle, ec)  # 默认工具工件
    print_log("calcFk", ec)
    print_cart_pose(cart_pose)

    # 获取具体工具工件下的正解笛卡尔坐标
    toolset: xCoreSDK_python.Toolset = xCoreSDK_python.Toolset()
    toolset.end.trans = [0.01, 0.01, 0.01]
    toolset.end.rpy = [0.01, 0.01, 0.01]
    toolset.ref.trans = [0, 0, 0]
    toolset.ref.rpy = [0, 0, 0]
    cart_pose = robot_model.calcFk(start_angle, toolset, ec)
    print_log("calcFk", ec)
    print_cart_pose(cart_pose)


# 计算逆解，笛卡尔坐标 -> 关节角度
def calcIk(robot: xCoreSDK_python.ArRobot, ec: dict):
    # cart_pos = robot.cartPosture(xCoreSDK_python.CoordinateType.endInRef, ec)
    cart_pos = xCoreSDK_python.CartesianPosition(
        [0.53466, -0.451432, 1.69627e-16, 1.5708, 2.06757e-16, 1.5708]
    )
    cart_pos.elbow = 16.6993 * M_PI / 180;
    robot_model: xCoreSDK_python.model.Model_1_7 = robot.model()

    # 默认工具工件下的逆解
    joint_pos = robot_model.calcIk(cart_pos, ec)
    print_log("calcIk", ec, ",".join(map(str, map(rad2deg, joint_pos))))

    # 获取具体工具工件下的逆解
    toolset: xCoreSDK_python.Toolset = xCoreSDK_python.Toolset()
    toolset.end.trans = [0.01, 0.01, 0.01]
    toolset.end.rpy = [0.01, 0.01, 0.01]
    toolset.ref.trans = [0, 0, 0]
    toolset.ref.rpy = [0, 0, 0]
    cart_pos = xCoreSDK_python.CartesianPosition(
        [
            0.54466, -0.441432, 0.01, 1.5807, -0.0099995, 1.5808
        ]
    )
    joint_pos = robot_model.calcIk(cart_pos, toolset, ec)
    print_log("calcIk", ec, ",".join(map(str, map(rad2deg, joint_pos))))


def movej(robot: xCoreSDK_python.ArRobot, ec: dict):
    cart_pos1 = xCoreSDK_python.CartesianPosition(
        [0.53466, -0.451432, 1.69627e-16, 1.5708, 2.06757e-16, 1.5708]
    )
    cart_pos1.elbow = 16.6993 * M_PI / 180;
    cart_pos2 = xCoreSDK_python.CartesianPosition(
        [0.53466, -0.351432, 1.69627e-16, 1.5708, 2.06757e-16, 1.5708]
    )
    cart_pos2.elbow = 16.6993 * M_PI / 180;
    movejcmd = xCoreSDK_python.MoveJCommand(cart_pos1, 100, 10)
    movejcmd2 = xCoreSDK_python.MoveJCommand(cart_pos2, 100, 10)
    cmdID = xCoreSDK_python.PyString()
    robot.moveAppend(
        [movejcmd, movejcmd2], cmdID, ec
    )  # [movejcmd]指令列表，可以添加多条指令，须为同类型指令
    print("Command ID:", cmdID.content())
    print_log("moveAppend", ec)
    robot.moveStart(ec)
    print_log("moveStart", ec)
    wait_robot(robot, ec)


def movel(robot: xCoreSDK_python.ArRobot, ec: dict):
    cart_pos1 = xCoreSDK_python.CartesianPosition(
        [0.53466, -0.451432, 1.69627e-16, 1.5708, 2.06757e-16, 1.5708]
    )
    cart_pos1.elbow = 16.6993 * M_PI / 180;
    cart_pos2 = xCoreSDK_python.CartesianPosition(
        [0.53466, -0.351432, 1.69627e-16, 1.5708, 2.06757e-16, 1.5708]
    )
    cart_pos2.elbow = 16.6993 * M_PI / 180;
    movejcmd = xCoreSDK_python.MoveLCommand(cart_pos1, 100, 10)
    movejcmd2 = xCoreSDK_python.MoveLCommand(cart_pos2, 100, 10)
    cmdID = xCoreSDK_python.PyString()
    robot.moveAppend(
        [movejcmd, movejcmd2], cmdID, ec
    )  # [movejcmd]指令列表，可以添加多条指令，须为同类型指令
    print("Command ID:", cmdID.content())
    print_log("moveAppend", ec)
    robot.moveStart(ec)
    print_log("moveStart", ec)
    wait_robot(robot, ec)


def moveabsj(robot: xCoreSDK_python.ArRobot, ec: dict):
    joint_pos1 = [
        0.0,0.667174589250972,0.0,0.4152523686488143,
        0.0,0.48836936889511035,0.0,
    ]
    joint_pos2 = [
        0.0,0.5235987755982988,0.0,1.0471975511965976,
        0.0,0,0.0,
    ]
    absjcmd1 = xCoreSDK_python.MoveAbsJCommand(joint_pos1, 100, 10)
    absjcmd2 = xCoreSDK_python.MoveAbsJCommand(joint_pos2, 100, 10)
    cmdID = xCoreSDK_python.PyString()
    robot.moveAppend(
        [absjcmd1, absjcmd2], cmdID, ec
    )  # [absjcmd]指令列表，可以添加多条指令，须为同类型指令
    print("Command ID:", cmdID.content())
    print_log("moveAppend", ec)
    robot.moveStart(ec)
    print_log("moveStart", ec)
    wait_robot(robot, ec)


# 实时模式-笛卡尔方式控制
def rt_cart_controller(robot: xCoreSDK_python.ArRobot, ec: dict):
    # 先用非实时movej到第一个点位
    start_pos = [0.53466, -0.351432, 0, M_PI / 2, 0, M_PI / 2]
    robot.setMotionControlMode(xCoreSDK_python.MotionControlMode.NrtCommandMode, ec)
    robot.setOperateMode(xCoreSDK_python.OperateMode.automatic, ec)
    robot.setPowerState(True, ec)
    start_tcp_point = xCoreSDK_python.CartesianPosition(start_pos)
    move_j_cmd = xCoreSDK_python.MoveJCommand(start_tcp_point, 100, 10)
    cmdID = xCoreSDK_python.PyString()
    robot.moveAppend([move_j_cmd], cmdID, ec)
    robot.moveStart(ec)
    wait_robot(robot, ec)
    # 实时模式-笛卡尔方式控制
    try:
        robot.setRtNetworkTolerance(100, ec)
        robot.setMotionControlMode(xCoreSDK_python.MotionControlMode.RtCommandMode, ec)
        rtCon: xCoreSDK_python.motioncontrolRT.PyRTmotioncontrol7 = robot.getRtMotionController()
        rtCon.setFilterLimit(True, 10)
        rtCon.setFilterFrequency(10, 10, 10, ec);
        def create_callback():  # 回调函数,务必创建两层，否则只能返回callback函数的地址
            start_pos_rt = [0.53466, 0, 0.351432, -M_PI, M_PI / 2, M_PI]
            cmd: xCoreSDK_python.CartesianPosition = xCoreSDK_python.CartesianPosition()
            cmd.pos = utility.postureToTransArray(start_pos_rt)
            cmd.hasElbow = True
            time = 0.0
            def callback():
                nonlocal time, cmd  # 允许修改外部变量
                time += 0.001
                list_pos = list(cmd.pos)
                list_pos[11] -= 0.0001  # z方向偏移
                cmd.pos = list_pos
                if time > 2:
                    cmd.setFinished()
                return cmd  # 返回joints和external两个参数值给回调函数callback
            return callback  # 返回闭包函数,保证回调函数传入的参数类型是函数而不是其它变量参数或结构体
        # 创建回调
        callback = create_callback()  # 返回值类型为函数
        # 设置回调函数
        rtCon.setControlLoopCar(callback)
        rtCon.startMove(RtControllerMode.cartesianPosition)
        rtCon.startLoop(False)  # False 表示非阻塞，仅单步运行.非阻塞模式下，函数只执行单次控制周期.这种方式能够插入调试代码（如 time.sleep 和打印语句）,确保控制周期被多次执行
        time.sleep(5)  # 模拟控制周期多次执行
        print("控制结束")
        robot.setMotionControlMode(xCoreSDK_python.MotionControlMode.NrtCommandMode, ec)  # 关闭实时模式并下电
        robot.setOperateMode(xCoreSDK_python.OperateMode.manual, ec)
        robot.setPowerState(False, ec)
    except Exception as e:
        print(f"An error occurred: {e}")
        robot.setMotionControlMode(xCoreSDK_python.MotionControlMode.Idle, ec)
        robot.setPowerState(False, ec)


# 实时模式-轴角度方式控制
def rt_joint_controller(robot: xCoreSDK_python.ArRobot, ec: dict):
    # 先用非实时moveabsj到第一个点位
    jntPos = [0.0, 0.5235987755982988, 0.0, 1.0471975511965976, 0.0, 0.0, 0.0]
    robot.setMotionControlMode(xCoreSDK_python.MotionControlMode.NrtCommandMode, ec)
    robot.setOperateMode(xCoreSDK_python.OperateMode.automatic, ec)
    robot.setPowerState(True, ec)
    moveabsj_cmd = xCoreSDK_python.MoveAbsJCommand(jntPos, 100, 10)
    cmdID = xCoreSDK_python.PyString()
    robot.moveAppend([moveabsj_cmd], cmdID, ec)
    robot.moveStart(ec)
    wait_robot(robot, ec)

    # 实时模式-轴角度方式控制
    try:
        robot.setRtNetworkTolerance(100, ec)
        robot.setMotionControlMode(xCoreSDK_python.MotionControlMode.RtCommandMode, ec)
        rtCon: xCoreSDK_python.motioncontrolRT.PyRTmotioncontrol7 = robot.getRtMotionController()
        rtCon.setFilterLimit(True, 10)
        def create_callback():  # 回调函数,务必创建两层，否则只能返回callback函数的地址
            # 需要修改的状态变量
            time = 0.0
            def callback():
                nonlocal time  # 允许修改外部变量
                time += 0.001
                delta_angle = M_PI / 20.0 * (1 - math.cos(M_PI / 2.5 * time)) / 5
                cmd = xCoreSDK_python.JointPosition()
                cmd.joints = [
                    jntPos[0] + delta_angle,
                    jntPos[1] + delta_angle,
                    jntPos[2] - delta_angle,
                    jntPos[3] + delta_angle,
                    jntPos[4] - delta_angle,
                    jntPos[5] + delta_angle,
                    jntPos[6] - delta_angle,
                ]
                if time > 2:
                    cmd.setFinished()
                return cmd

            return callback

        # 创建回调
        callback = create_callback()  # 返回值类型为函数
        # 设置回调函数
        rtCon.setControlLoopJoi(callback)
        rtCon.startMove(RtControllerMode.jointPosition)
        rtCon.startLoop(False)  # False 表示非阻塞，仅单步运行.非阻塞模式下，函数只执行单次控制周期.这种方式能够插入调试代码（如 time.sleep 和打印语句）,确保控制周期被多次执行
        time.sleep(5)  # 模拟控制周期多次执行
        print("控制结束")
        robot.setMotionControlMode(xCoreSDK_python.MotionControlMode.NrtCommandMode, ec)  # 关闭实时模式并下电
        robot.setPowerState(False, ec)
    except Exception as e:
        print(f"An error occurred: {e}")
        robot.setMotionControlMode(xCoreSDK_python.MotionControlMode.Idle, ec)
        robot.setPowerState(False, ec)


if __name__ == "__main__":
    try:
        # 连接机器人
        ip = "10.6.2.203"
        lcoal_ip = "10.6.2.250"
        # 实例化AR机型并连接
        robot = xCoreSDK_python.ArRobot(ip, lcoal_ip)
        ec = {}
        # get_jointpos(robot, ec)
        # get_cartpos(robot, ec)
        # calcFk(robot, ec)
        # calcIk(robot, ec)
        # movej(robot, ec)
        # movel(robot, ec)
        # moveabsj(robot, ec)
        # rt_cart_controller(robot, ec)
        # rt_joint_controller(robot, ec)
    except Exception as e:
        print(f"An error occurred: {e}")
