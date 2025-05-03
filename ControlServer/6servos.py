#!/usr/bin/env python3
# encoding:utf-8
import sys
sys.path.append('/home/ubuntu/Sensor/')
import time
import numpy as np
from math import sqrt
#import matplotlib.pyplot as plt
from InverseKinematics import *
from Transform import getAngle
#from mpl_toolkits.mplot3d import Axes3D
from HiwonderSDK.Board import setBusServoPulse, getBusServoPulse

def servosMove(servos, movetime=None):
    """
    控制舵机1至6转动到指定的脉宽值。

    参数:
      servos: 长度为6的列表或数组，依次对应舵机1、2、3、4、5、6的脉宽值。
      movetime: （可选）运动时间（毫秒），如果不指定，则根据当前脉宽与目标脉宽差值自动计算。
    返回:
      最终运动时间 movetime。
    """
    # 等待20毫秒
    time.sleep(0.02)
    
    # 如果未指定movetime，则遍历所有舵机计算当前与目标脉宽的最大差值，根据该差值自动计算移动时间
    if movetime is None:
        max_d = 0
        # 舵机编号从1到6，输入列表下标依次为0~5
        for i in range(6):
            # 根据舵机编号选取对应的通道号
            servo_channel = i + 1
            current_pulse = getBusServoPulse(servo_channel)
            d = abs(current_pulse - servos[i])
            if d > max_d:
                max_d = d
        movetime = int(max_d * 4)
    
    # 分别设置舵机1至舵机6的脉宽值
    setBusServoPulse(1, servos[0], movetime)
    setBusServoPulse(2, servos[1], movetime)
    setBusServoPulse(3, servos[2], movetime)
    setBusServoPulse(4, servos[3], movetime)
    setBusServoPulse(5, servos[4], movetime)
    setBusServoPulse(6, servos[5], movetime)
    return movetime

if __name__ == "__main__":
    # 用户输入6个舵机的脉宽值，使用逗号分隔，例如: 200,500,600,700,800,900
    user_input = input("请输入舵机1到舵机6的脉宽值 (例如：200,500,600,700,800,900): ")
    try:
        servo_values = [int(x.strip()) for x in user_input.split(",")]
        if len(servo_values) != 6:
            raise ValueError("输入的脉宽值数量不等于6")
    except Exception as e:
        print("输入格式错误:", e)
        sys.exit(1)
    
    # 可选：提示用户输入运动时间（单位毫秒），如果留空，则自动计算
    time_input = input("请输入运动时间（毫秒，留空自动计算）: ")
    movetime = None
    if time_input.strip():
        try:
            movetime = int(time_input.strip())
        except Exception:
            print("运动时间输入错误，将使用自动计算的运动时间。")
    
    # 调用 servosMove 函数并输出实际使用的运动时间
    result_time = servosMove(servo_values, movetime)
    print("舵机运动完成，使用运动时间为:", result_time, "毫秒")

