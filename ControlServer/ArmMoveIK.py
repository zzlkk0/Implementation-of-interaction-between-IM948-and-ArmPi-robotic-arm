#!/usr/bin/env python3
# encoding:utf-8
import sys
sys.path.append('/home/ubuntu/Sensor/')
import time
import numpy as np
from math import sqrt
#from matplotlib import pyplot as plt  # 不需要绘图
from InverseKinematics import *
from Transform import getAngle
#from mpl_toolkits.mplot3d import Axes3D
from HiwonderSDK.Board import setBusServoPulse, getBusServoPulse
import termios, tty

# 定义 getKey 函数，用于获取单个键盘输入（类似 getch）
def getKey():
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        ch = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return ch

# 初始化逆运动学对象
ik = IK('arm')
# 设置连杆长度，根据实际情况调整
l1 = ik.l1 + 0.75
l4 = ik.l4 - 0.15
ik.setLinkLength(L1=l1, L4=l4)

class ArmIK:
    # 定义各舵机的脉宽与角度范围
    servo3Range = (0, 1000.0, 0, 240.0)
    servo4Range = (0, 1000.0, 0, 240.0)
    servo5Range = (0, 1000.0, 0, 240.0)
    servo6Range = (0, 1000.0, 0, 240.0)

    def __init__(self):
        self.setServoRange()

    def setServoRange(self, servo3_Range=servo3Range, servo4_Range=servo4Range, 
                      servo5_Range=servo5Range, servo6_Range=servo6Range):
        self.servo3Range = servo3_Range
        self.servo4Range = servo4_Range
        self.servo5Range = servo5_Range
        self.servo6Range = servo6_Range
        self.servo3Param = (self.servo3Range[1] - self.servo3Range[0]) / (self.servo3Range[3] - self.servo3Range[2])
        self.servo4Param = (self.servo4Range[1] - self.servo4Range[0]) / (self.servo4Range[3] - self.servo4Range[2])
        self.servo5Param = (self.servo5Range[1] - self.servo5Range[0]) / (self.servo5Range[3] - self.servo5Range[2])
        self.servo6Param = (self.servo6Range[1] - self.servo6Range[0]) / (self.servo6Range[3] - self.servo6Range[2])

    def transformAngelAdaptArm(self, theta3, theta4, theta5, theta6):
        # 将逆运动学得到的角度转换为舵机对应的脉宽值
        servo3 = int(round(theta3 * self.servo3Param + (self.servo3Range[1] + self.servo3Range[0]) / 2))
        if servo3 > self.servo3Range[1] or servo3 < self.servo3Range[0] + 60:
            print('servo3(%s)超出范围(%s, %s)' % (servo3, self.servo3Range[0] + 60, self.servo3Range[1]))
            return False

        servo4 = int(round(theta4 * self.servo4Param + (self.servo4Range[1] + self.servo4Range[0]) / 2))
        if servo4 > self.servo4Range[1] or servo4 < self.servo4Range[0]:
            print('servo4(%s)超出范围(%s, %s)' % (servo4, self.servo4Range[0], self.servo4Range[1]))
            return False

        servo5 = int(round((self.servo5Range[1] + self.servo5Range[0]) / 2 - (90.0 - theta5) * self.servo5Param))
        if servo5 > ((self.servo5Range[1] + self.servo5Range[0]) / 2 + 90 * self.servo5Param) or servo5 < ((self.servo5Range[1] + self.servo5Range[0]) / 2 - 90 * self.servo5Param):
            print('servo5(%s)超出范围(%s, %s)' % (servo5, self.servo5Range[0], self.servo5Range[1]))
            return False

        if theta6 < -(self.servo6Range[3] - self.servo6Range[2]) / 2:
            servo6 = int(round(((self.servo6Range[3] - self.servo6Range[2]) / 2 + (90 + (180 + theta6))) * self.servo6Param))
        else:
            servo6 = int(round(((self.servo6Range[3] - self.servo6Range[2]) / 2 - (90 - theta6)) * self.servo6Param))
        if servo6 > self.servo6Range[1] or servo6 < self.servo6Range[0]:
            print('servo6(%s)超出范围(%s, %s)' % (servo6, self.servo6Range[0], self.servo6Range[1]))
            return False

        return {"servo3": servo3, "servo4": servo4, "servo5": servo5, "servo6": servo6}

    def servosMove(self, servos, movetime=None):
        # 实际驱动3,4,5,6号舵机转动
        time.sleep(0.02)
        if movetime is None:
            max_d = 0
            for i in range(0, 4):
                d = abs(getBusServoPulse(i + 3) - servos[i])
                if d > max_d:
                    max_d = d
            movetime = int(max_d * 4)
        setBusServoPulse(3, servos[0], movetime)
        setBusServoPulse(4, servos[1], movetime)
        setBusServoPulse(5, servos[2], movetime)
        setBusServoPulse(6, servos[3], movetime)
        return movetime

    def setPitchRange(self, coordinate_data, alpha1, alpha2, da=1):
        """
        给定坐标（单位 cm）和俯仰角范围 alpha1 到 alpha2，
        遍历寻找合适的逆运动学解，返回 (舵机数据, 对应俯仰角)，若无解返回 False
        """
        x, y, z = coordinate_data
        if alpha1 >= alpha2:
            da = -da
        for alpha in np.arange(alpha1, alpha2, da):
            result = ik.getRotationAngle((x, y, z), alpha)
            if result:
                theta3 = result['theta3']
                theta4 = result['theta4']
                theta5 = result['theta5']
                theta6 = result['theta6']
                servos = self.transformAngelAdaptArm(theta3, theta4, theta5, theta6)
                if servos != False:
                    return servos, alpha
        return False

    def setPitchRangeMoving(self, coordinate_data, alpha, alpha1, alpha2, movetime=None):
        """
        根据输入坐标（cm）和目标俯仰角 alpha，以及允许的俯仰角范围 [alpha1, alpha2]，
        自动寻找最接近目标的逆运动学解，并实际执行舵机运动。
        返回 (舵机数据, 实际使用的俯仰角, 运动时间)，若无解返回 False。
        """
        x, y, z = coordinate_data
        result1 = self.setPitchRange((x, y, z), alpha, alpha1)
        result2 = self.setPitchRange((x, y, z), alpha, alpha2)
        if result1 != False:
            data = result1
            if result2 != False:
                if abs(result2[1] - alpha) < abs(result1[1] - alpha):
                    data = result2
        else:
            if result2 != False:
                data = result2
            else:
                return False
        servos, used_alpha = data[0], data[1]
        movetime = self.servosMove((servos["servo3"], servos["servo4"], servos["servo5"], servos["servo6"]), movetime)
        return servos, used_alpha, movetime

    def calculateIK(self, coordinate_data, alpha, alpha1, alpha2):
        """
        根据输入坐标（cm）和目标俯仰角 alpha，以及允许的俯仰角范围 [alpha1, alpha2]，
        仅计算逆运动学解，不执行实际运动，返回 (舵机数据, 实际使用的俯仰角)
        """
        result1 = self.setPitchRange(coordinate_data, alpha, alpha1)
        result2 = self.setPitchRange(coordinate_data, alpha, alpha2)
        if result1 != False:
            data = result1
            if result2 != False:
                if abs(result2[1] - alpha) < abs(result1[1] - alpha):
                    data = result2
        else:
            if result2 != False:
                data = result2
            else:
                return False
        servos, used_alpha = data[0], data[1]
        return servos, used_alpha

if __name__ == "__main__":
    AK = ArmIK()

    # 设置初始目标坐标（单位 cm）和目标俯仰角（度）
    coordinate = [0.2, 18, 18.5]
    target_alpha = 0      # 目标俯仰角
    alpha_min = -90       # 允许的最小俯仰角
    alpha_max = 0         # 允许的最大俯仰角

    # 每次微调步长 0.1 cm
    delta = 1

    print("使用键盘控制坐标微调并执行实际运动：")
    print("  上箭头：X 轴增加")
    print("  下箭头：X 轴减少")
    print("  右箭头：Y 轴增加")
    print("  左箭头：Y 轴减少")
    print("  n 键：Z 轴增加")
    print("  m 键：Z 轴减少")
    print("  q 键：退出")
    
    while True:
        print("\n当前坐标: X={:.2f} cm, Y={:.2f} cm, Z={:.2f} cm".format(coordinate[0], coordinate[1], coordinate[2]))
        print("请按键操作:", end=' ', flush=True)
        key = getKey()
        if key == '\x1b':  # 检测箭头键开始
            getKey()  # 忽略 '['
            key_arrow = getKey()
            if key_arrow == 'A':       # 上箭头：X 增加
                coordinate[0] += delta
            elif key_arrow == 'B':     # 下箭头：X 减少
                coordinate[0] -= delta
            elif key_arrow == 'C':     # 右箭头：Y 增加
                coordinate[1] += delta
            elif key_arrow == 'D':     # 左箭头：Y 减少
                coordinate[1] -= delta
        elif key.lower() == 'n':         # n 键：Z 增加
            coordinate[2] += delta
        elif key.lower() == 'm':         # m 键：Z 减少
            coordinate[2] -= delta
        elif key.lower() == 'q':         # q 键：退出
            print("\n退出控制")
            break
        else:
            print("\n无效按键")
            continue

        # 根据新的坐标计算 IK 并执行实际运动
        result = AK.setPitchRangeMoving(tuple(coordinate), target_alpha, alpha_min, alpha_max)
        if result:
            servos, used_alpha, movetime = result
            print("执行运动，舵机数据:", servos, "使用的俯仰角:", used_alpha, "运动时长:", movetime, "ms")
        else:
            print("该坐标无有效逆运动学解")

