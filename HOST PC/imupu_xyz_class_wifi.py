import sys
import time
import threading
import numpy as np
import asyncio
import msvcrt
# 假定 IMUTCPReceiver 类已经定义或导入
from esp32_data_extract import IMUTCPReceiver


class IMUPosition:
    def __init__(self, host='0.0.0.0', port=9999, reset_all=True):
        """
        使用 TCP 接收 IMU 数据，替换串口数据接收
        host: TCP 服务器监听地址
        port: TCP 服务器监听端口
        reset_all: 是否将所有坐标在启动时归零（True归零）
        """
        # 初始化 TCP 数据接收器
        self.receiver = IMUTCPReceiver(host=host, port=port)
        self._collection_running = False

        # 存储从 TCP 接收器收集的偏移量数据，每个元素格式为 [timestamp, offsetX, offsetY, offsetZ]
        self.offset_data = []

        # 累计位置及其他状态变量
        self.cum_position = np.array([0.0, 0.0, 0.0])
        self.in_dynamic = False
        self.last_dynamic = np.array([0.0, 0.0, 0.0])
        self.dominant_axis = None  # 'x'、'y'、'z' 或 'all'
        self.cum_traj = []  # 存储 (timestamp, 累计位置) 的轨迹
        self.processed_index = 0

        if reset_all:
            self.reset_all()

    def update_position(self):
        """定期从 TCP 接收器获取最新位移数据，并更新累计位置"""
        # 从 TCP 接收器获取偏移数据，返回 (offsetX, offsetY, offsetZ)
        offset = self.receiver.get_offset()
        # 打印调试信息
        # print("获取到 offset:", offset)

        if offset is None or any(v is None for v in offset):
            return

        current_time = time.time()
        # 将当前读取的数据加入记录列表
        self.offset_data.append([current_time, offset[0], offset[1], offset[2]])

        # 处理新采集的数据，更新累计位置（处理逻辑保持与原来一致）
        new_points = self.offset_data[self.processed_index:]
        for pt in new_points:
            timestamp, x, y, z = pt
            reading = np.array([x, y, z])
            # 对各轴进行一定的补偿
            reading_adjusted = reading.copy()
            # Z 轴补偿
            if reading[2] > 0:
                reading_adjusted[2] = reading[2] * 0.95
            elif reading[2] < 0:
                reading_adjusted[2] = reading[2] * 1.1
            # X 轴补偿
            if reading[0] > 0:
                reading_adjusted[0] = reading[0] * 1.1
            elif reading[0] < 0:
                reading_adjusted[0] = reading[0] * 1.0
            # Y 轴补偿
            if reading[1] > 0:
                reading_adjusted[1] = reading[1] * 1.1
            elif reading[1] < 0:
                reading_adjusted[1] = reading[1] * 1.0

            # 判断是否处于静止状态
            if np.allclose(reading, [0.0, 0.0, 0.0]):
                if self.in_dynamic:
                    # 动态阶段结束时，将 last_dynamic 限制在 ±0.3 范围内后累加到累计位置中
                    increment = np.clip(self.last_dynamic, -0.3, 0.3)
                    # 如果主导轴为 'z' 则只累加 Z 轴
                    if self.dominant_axis == 'z':
                        increment[0] = 0.0
                        increment[1] = 0.0
                    self.cum_position = self.cum_position + increment
                    self.last_dynamic = np.array([0.0, 0.0, 0.0])
                    self.dominant_axis = None
                    self.in_dynamic = False
                display_val = self.cum_position.copy()
            else:
                if not self.in_dynamic:
                    self.in_dynamic = True
                    self.last_dynamic = reading_adjusted.copy()
                    # 判断主导轴：当某一轴的变化绝对值最大且超过阈值时将其设为主导轴
                    threshold = 0.05
                    abs_x = abs(reading_adjusted[0])
                    abs_y = abs(reading_adjusted[1])
                    abs_z = abs(reading_adjusted[2])
                    if abs_x > abs_y and abs_x > abs_z and abs_x > threshold:
                        self.dominant_axis = 'x'
                    elif abs_y > abs_x and abs_y > abs_z and abs_y > threshold:
                        self.dominant_axis = 'y'
                    elif abs_z > abs_x and abs_z > abs_y and abs_z > threshold:
                        self.dominant_axis = 'z'
                    else:
                        self.dominant_axis = 'all'
                else:
                    if self.dominant_axis in ['x', 'y', 'z']:
                        dom_index = {'x': 0, 'y': 1, 'z': 2}[self.dominant_axis]
                        # 仅保留主导轴，其它轴数据置 0
                        for j in range(3):
                            if j != dom_index:
                                reading_adjusted[j] = 0.0
                        # 根据数据符号更新 last_dynamic
                        if self.last_dynamic[dom_index] >= 0:
                            if reading_adjusted[dom_index] > 0:
                                self.last_dynamic[dom_index] = max(self.last_dynamic[dom_index],
                                                                   reading_adjusted[dom_index])
                        else:
                            if reading_adjusted[dom_index] < 0:
                                self.last_dynamic[dom_index] = min(self.last_dynamic[dom_index],
                                                                   reading_adjusted[dom_index])
                    else:
                        for j in range(3):
                            if self.last_dynamic[j] >= 0:
                                if reading_adjusted[j] > 0:
                                    self.last_dynamic[j] = max(self.last_dynamic[j], reading_adjusted[j])
                            else:
                                if reading_adjusted[j] < 0:
                                    self.last_dynamic[j] = min(self.last_dynamic[j], reading_adjusted[j])
                # 限制 last_dynamic 不超过 ±0.3
                self.last_dynamic = np.clip(self.last_dynamic, -0.3, 0.3)
                display_val = self.cum_position + self.last_dynamic

            self.cum_traj.append((timestamp, display_val.copy()))
        self.processed_index += len(new_points)
        # 清理 10 秒前的数据
        self.cum_traj = [(t, pos) for t, pos in self.cum_traj if t >= current_time - 10]
        # 打印当前累计位置
        # print("当前累计位置: {:.3f}, {:.3f}, {:.3f}".format(
        # self.cum_position[0], self.cum_position[1], self.cum_position[2]))

    def processing_loop(self):
        """每隔一段时间处理新采集的数据"""
        while self._collection_running:
            self.update_position()
            time.sleep(0.1)

    def _run_receiver(self):
        """在单独线程中运行 TCP 接收器的异步服务"""
        # 注意：不要直接使用 asyncio.run()，因为它在 start_tcp() 执行完毕后会关闭事件循环，
        # 导致 TCP 服务器任务无法持续运行。
        loop = asyncio.new_event_loop()
        asyncio.set_event_loop(loop)
        loop.run_until_complete(self.receiver.start_tcp())
        loop.run_forever()

    def start(self):
        """启动 TCP 数据接收及数据处理线程"""
        if not self._collection_running:
            self._collection_running = True
            # 启动 TCP 接收器线程
            self.receiver_thread = threading.Thread(target=self._run_receiver, daemon=True)
            self.receiver_thread.start()
            # 启动数据处理线程
            self.process_thread = threading.Thread(target=self.processing_loop, daemon=True)
            self.process_thread.start()

    def stop(self):
        """停止数据采集与处理"""
        self._collection_running = False
        # 停止 TCP 服务：这里需要合理地关闭事件循环
        try:
            asyncio.run(self.receiver.stop_tcp())
        except Exception as e:
            print("停止 TCP 服务时出错:", e)
        if hasattr(self, 'process_thread'):
            self.process_thread.join()
        if hasattr(self, 'receiver_thread'):
            self.receiver_thread.join()

    def reset_x(self):
        self.cum_position[0] = 0.0
        self.cum_traj.append((time.time(), self.cum_position.copy()))
        print("X归零, 当前累计位置:", self.cum_position)

    def reset_y(self):
        self.cum_position[1] = 0.0
        self.cum_traj.append((time.time(), self.cum_position.copy()))
        print("Y归零, 当前累计位置:", self.cum_position)

    def reset_z(self):
        self.cum_position[2] = 0.0
        self.cum_traj.append((time.time(), self.cum_position.copy()))
        print("Z归零, 当前累计位置:", self.cum_position)

    def reset_all(self):
        self.cum_position = np.array([0.0, 0.0, 0.0])
        self.in_dynamic = False
        self.last_dynamic = np.array([0.0, 0.0, 0.0])
        self.dominant_axis = None
        self.cum_traj.append((time.time(), self.cum_position.copy()))
        print("全部归零, 当前累计位置:", self.cum_position)

    def get_current_position(self):
        """返回当前累计位置"""
        return self.cum_position.copy()

    def get_current_adc(self):
        """
        获取当前 ESP32 ADC 电压与初始 offset 的差值：
        1) 首次调用时，将当前电压设为 offset，返回 0.0。
        2) 后续调用返回 (current_voltage - offset)。
        如果当前还没有收到 ADC 数据，将返回 None。
        """
        volt = self.receiver.get_adc_esp32()
        if volt is None:
            return None

        # 第一次：定义 offset
        if not hasattr(self, '_adc_offset_defined') or not self._adc_offset_defined:
            self._adc_offset = volt
            self._adc_offset_defined = True
            # 返回 0 表示已将当前值视为基准
            return 0

        # 已有 offset：返回差值
        return -(volt - self._adc_offset)

    def clear_adc_offset(self):
        """
        清除当前 ADC offset，并在下一次 get_current_adc 调用时
        把那时的电压作为新的 offset（即重新归零）。
        """
        # 重置标志，下次 get_current_adc 会重新设定 offset
        self._adc_offset_defined = False
        print("ESP32 ADC offset cleared; next reading will be new zero point.")

    def get_current_angle(self):
        """
        获取当前原始角度，前两秒内只收集历史；两秒结束后：
        1) 判断哪些轴在启动两秒里始终满足 |angle|>160，记录到 self._correction_axes。
        2) 对这些轴做“负则+180，正则-180”矫正；其他轴不变。
        3) 将那一刻的矫正值设为 offset，并返回 (0,0,0)。
        4) 后续调用直接用“矫正后值 - offset”输出。
        如果数据无效，返回 (None, None, None)。
        """
        import time
        current = self.receiver.get_angle()
        if current is None or any(v is None for v in current):
            return None, None, None

        now = time.time()

        # 初始化启动时刻和历史
        if not hasattr(self, '_startup_time'):
            self._startup_time = now
            self._angle_history = []  # 存 (time, angle) 直到两秒结束
            self._correction_axes = set()  # 哪些轴需要矫正
            self._offset_defined = False

        # 启动两秒内，只收集历史，不输出偏移
        if now - self._startup_time <= 2:
            self._angle_history.append(current)
            # 等待两秒结束，再在下一次调用时定义 offset
            return (0.0, 0.0, 0.0) if self._offset_defined else (None, None, None)

        # 两秒之后，第一次定义 offset
        if not self._offset_defined:
            # 判断哪些轴在这段历史里始终满足 |angle|>160
            for axis in range(3):
                if all(abs(ang[axis]) > 140 for ang in self._angle_history):
                    self._correction_axes.add(axis)
            # 对当前值做矫正
            corrected = list(current)
            for ax in self._correction_axes:
                if corrected[ax] < 0:
                    corrected[ax] += 180
                else:
                    corrected[ax] -= 180
            # 设定 offset
            self._angle_offset = tuple(corrected)
            self._offset_defined = True
            print("► 初始矫正角度设定为 offset:", self._angle_offset)
            return (0.0, 0.0, 0.0)

        # 两秒后且 offset 已定义：输入–矫正–输出
        corrected = list(current)
        for ax in self._correction_axes:
            if corrected[ax] < 0:
                corrected[ax] += 180
            else:
                corrected[ax] -= 180

        return tuple(c - o for c, o in zip(corrected, self._angle_offset))

    def clear_angle_offset(self):
        """
        清除当前的角度偏移，并将当前角度设为新的基准（即视为 0 角度）。
        同时重新开始2秒启动阶段，用于重新判定需要跨界矫正的轴。
        """
        import time
        # 获取当前原始角度（未经矫正）
        angle = self.receiver.get_angle()
        # 检查数据有效性
        if angle is None or any(a is None for a in angle):
            print("无法清零角度：当前角度数据无效。")
            return

        # 重置启动阶段相关状态
        self._startup_time = time.time()
        self._angle_history = []  # 前两秒内收集的原始角度列表
        self._correction_axes = set()  # 哪些轴需要跨界矫正
        self._offset_defined = False  # 下次 get_current_angle 会重新定义 offset

        # 提示：下一次调用 get_current_angle 时，会用该角度来判定 offset
        print(f"角度清零：已重置启动两秒阶段，原始角度 {angle} 将作为新基准采样。")


# ---------------- 示例用法 ----------------
# if __name__ == "__main__":
#     imu = IMUPosition(host='0.0.0.0', port=9999, reset_all=True)
#     try:
#         imu.start()
#         # 主循环中实时获取并打印累计位置
#         while True:
#             pos = imu.get_current_position()
#             # print("实时位置:", pos)
#             time.sleep(1)
#     except KeyboardInterrupt:
#         imu.stop()
#         print("采集停止。")

#
# if __name__ == "__main__":
#     imu = IMUPosition(host='0.0.0.0', port=9999, reset_all=True)
#     try:
#         imu.start()
#         print("正在接收 IMU 数据。按下回车键可清零角度，Ctrl+C 退出。")
#         while True:
#             angle = imu.get_current_angle()
#             # 判断获取的数据是否有效：角度不为 None
#             if angle is not None and all(v is not None for v in angle):
#                 print("当前角度变化量 (°): X={:.2f}, Y={:.2f}, Z={:.2f}".format(*angle))
#             else:
#                 print("等待角度数据中...")
#             # 检查是否按下了“回车键”
#             if msvcrt.kbhit():
#                 key = msvcrt.getwch()
#                 if key == '\r':  # 回车键
#                     imu.clear_angle_offset()
#             time.sleep(0.5)
#     except KeyboardInterrupt:
#         imu.stop()
#         print("\n采集停止。")

if __name__ == "__main__":
    imu = IMUPosition(host='0.0.0.0', port=9999, reset_all=True)
    try:
        imu.start()
        print("正在接收 IMU 数据。按下回车键可清零角度，Ctrl+C 退出。")
        while True:
            angle = imu.get_current_adc()
            print('adc=',angle)
            # 检查是否按下了“回车键”
            if msvcrt.kbhit():
                key = msvcrt.getwch()
                if key == '\r':  # 回车键
                    imu.clear_adc_offset()
            time.sleep(0.5)
    except KeyboardInterrupt:
        imu.stop()
        print("\n采集停止。")
