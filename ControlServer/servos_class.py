#!/usr/bin/env python3
# encoding:utf-8
import sys
sys.path.append('/home/ubuntu/Sensor/')
import time
from HiwonderSDK.Board import setBusServoPulse, getBusServoPulse

class ServoController:
    """
    控制多个舵机的运动。
    """
    def __init__(self, servo_count=6):
        """
        参数:
          servo_count: 舵机数量，默认6个。
        """
        self.servo_count = servo_count

    def move(self, servos, movetime=10):
        """
        控制舵机转动到指定脉宽值。

        参数:
          servos: 长度等于servo_count的序列，每个元素为目标脉宽值。
          movetime: 运动时间（毫秒），如果为None，则自动计算。

        返回:
          实际使用的运动时间（毫秒）。
        """
        if len(servos) != self.servo_count:
            raise ValueError(f"Expected {self.servo_count} servo values, got {len(servos)}")
        # 等待20毫秒以保证命令发送稳定
        time.sleep(0.01)
        # 自动计算运动时间
        if movetime is None:
            max_d = 0
            for i in range(self.servo_count):
                channel = i + 1
                current_pulse = getBusServoPulse(channel)
                d = abs(current_pulse - servos[i])
                if d > max_d:
                    max_d = d
            movetime = int(max_d * 4)
        # 发送脉宽到各舵机
        for i in range(self.servo_count):
            channel = i + 1
            setBusServoPulse(channel, servos[i], movetime)
        return movetime

if __name__ == "__main__":
    controller = ServoController()
    user_input = input("请输入舵机1到舵机6的脉宽值 (例如：200,500,600,700,800,900): ")
    try:
        servo_values = [int(x.strip()) for x in user_input.split(",")]
        if len(servo_values) != controller.servo_count:
            raise ValueError("输入的脉宽值数量不等于6")
    except Exception as e:
        print("输入格式错误:", e)
        sys.exit(1)
    time_input = input("请输入运动时间（毫秒，留空自动计算）: ")
    if time_input.strip():
        try:
            movetime = int(time_input.strip())
        except ValueError:
            print("运动时间输入错误，将使用自动计算的运动时间。")
            movetime = None
    else:
        movetime = None
    result_time = controller.move(servo_values, movetime)
    print(f"舵机运动完成，使用运动时间为: {result_time} 毫秒")

