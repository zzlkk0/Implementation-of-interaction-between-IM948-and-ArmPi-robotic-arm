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
#!/usr/bin/env python3
# encoding: utf-8
import sys
import time
from HiwonderSDK.Board import setBusServoPulse, getBusServoPulse

# 定义步长
STEP = 10

def servosMove(servos, movetime=None):
    """
    控制舵机1至6转动到指定的脉宽值。

    参数:
      servos: 长度为6的列表或数组，依次对应舵机1、2、3、4、5、6的脉宽值。
      movetime: （可选）运动时间（毫秒），如果不指定，则根据当前脉宽与目标脉宽差值自动计算。
    返回:
      最终运动时间 movetime。
    """
    time.sleep(0.02)
    if movetime is None:
        max_d = 0
        # 遍历所有舵机，计算当前脉宽与目标脉宽的最大差值
        for i in range(6):
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

# 跨平台的 getch 函数，获取单个键盘输入
def getch():
    try:
        import termios, tty
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(fd)
            ch = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return ch
    except ImportError:
        # Windows 平台
        import msvcrt
        return msvcrt.getch().decode()

def keyboard_control():
    """
    通过键盘控制舵机：
      - 默认模式下，小写字母 n, m, k, l, o, p 分别使舵机1~6的脉宽增加 STEP，
        大写字母则分别使对应舵机脉宽减少 STEP。
      - 如果先按 b 键，再按 n, m, k, l, o, p（无论大小写）则会以递减方式改变对应舵机脉宽（减少 STEP）。
      - 按 q 退出键盘控制模式。
    """
    # 获取当前舵机脉宽值作为初始目标值
    servos = []
    for i in range(6):
        pulse = getBusServoPulse(i + 1)
        servos.append(pulse)
        
    print("进入键盘控制模式：")
    print("  n / m / k / l / o / p 分别控制舵机1~6：默认小写增加，大写减少")
    print("  如果先按 b 键，再按 n/m/k/l/o/p 则均为减少操作（递减模式）")
    print("  按 q 退出")
    print("初始脉宽值：", servos)
    
    reverse_mode = False  # 递减模式标识
    while True:
        print("\n等待按键...", end="", flush=True)
        key = getch()
        # 退出键
        if key.lower() == 'q':
            print("\n退出键盘控制模式。")
            break
        
        # 判断是否进入递减模式
        if key.lower() == 'b':
            reverse_mode = True
            print("\n进入递减模式：后续 n/m/k/l/o/p 均为递减操作")
            continue
        
        # 检查按键是否属于控制舵机的命令 (n,m,k,l,o,p)
        if key.lower() in ['n', 'm', 'k', 'l', 'o', 'p']:
            # 确定舵机索引
            if key.lower() == 'n':
                idx = 0
            elif key.lower() == 'm':
                idx = 1
            elif key.lower() == 'k':
                idx = 2
            elif key.lower() == 'l':
                idx = 3
            elif key.lower() == 'o':
                idx = 4
            elif key.lower() == 'p':
                idx = 5

            # 如果处于递减模式，则无条件递减
            if reverse_mode:
                servos[idx] -= STEP
                # 使用完递减模式后，重置该标识
                reverse_mode = False
            else:
                # 非递减模式下，根据按键大小写判断增减
                if key.islower():
                    servos[idx] += STEP
                else:
                    servos[idx] -= STEP

            print("\n更新后脉宽：", servos)
            used_time = servosMove(servos)
            print("更新舵机脉宽成功，运动时间为 {} 毫秒".format(used_time))
        else:
            print(" 无效键: {}".format(key))
            # 非控制按键，若处于递减模式，则清除该标识
            reverse_mode = False

if __name__ == "__main__":
    print("请选择运行模式：")
    print("  1 - 直接输入舵机脉宽")
    print("  2 - 键盘控制舵机")
    choice = input("请输入选项 (1/2): ").strip()
    
    if choice == "1":
        # 用户输入6个舵机的脉宽值，例如：200,500,600,700,800,900
        user_input = input("请输入舵机1到舵机6的脉宽值 (例如：200,500,600,700,800,900): ")
        try:
            servo_values = [int(x.strip()) for x in user_input.split(",")]
            if len(servo_values) != 6:
                raise ValueError("输入的脉宽值数量不等于6")
        except Exception as e:
            print("输入格式错误:", e)
            sys.exit(1)
        
        # 可选：提示用户输入运动时间（毫秒），如果留空，则自动计算
        time_input = input("请输入运动时间（毫秒，留空自动计算）: ")
        movetime = None
        if time_input.strip():
            try:
                movetime = int(time_input.strip())
            except Exception:
                print("运动时间输入错误，将使用自动计算的运动时间。")
        
        result_time = servosMove(servo_values, movetime)
        print("舵机运动完成，使用运动时间为:", result_time, "毫秒")
    
    elif choice == "2":
        keyboard_control()
    
    else:
        print("无效选项！")


