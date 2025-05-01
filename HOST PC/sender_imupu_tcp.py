import asyncio
import os
import sys
import threading
import tkinter as tk

# 将 src 文件夹添加到搜索路径
src_path = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', 'src'))
if src_path not in sys.path:
    sys.path.insert(0, src_path)

from client import TransmissionClient
from imupu_xyz_class_wifi import IMUPosition  # 使用基于 TCP 接收的 IMUPosition


def start_tk(imu):
    """
    创建一个简单的 Tkinter 窗口，包含：
      - 发送模式切换（位移/角度）
      - 开始/停止发送按钮
      - 同时归零两个数据的“归零”按钮
    """
    root = tk.Tk()
    root.title("IMU 控制面板")
    root.geometry("300x220")

    # 发送模式切换：用本地变量，回调更新 imu.send_mode
    mode_var = tk.IntVar(value=imu.send_mode)
    mode_frame = tk.Frame(root)
    tk.Label(mode_frame, text="发送模式:", font=("Arial", 12)).pack(side="left")
    tk.Radiobutton(
        mode_frame, text="位移", variable=mode_var, value=0,
        font=("Arial", 12),
        command=lambda: setattr(imu, 'send_mode', mode_var.get())
    ).pack(side="left", padx=5)
    tk.Radiobutton(
        mode_frame, text="角度", variable=mode_var, value=1,
        font=("Arial", 12),
        command=lambda: setattr(imu, 'send_mode', mode_var.get())
    ).pack(side="left")
    mode_frame.pack(pady=10)

    # 发送控制：开始/停止
    control_frame = tk.Frame(root)
    tk.Button(
        control_frame, text="开始发送", font=("Arial", 12),
        command=lambda: setattr(imu, 'sending', True)
    ).pack(side="left", padx=10)
    tk.Button(
        control_frame, text="停止发送", font=("Arial", 12),
        command=lambda: setattr(imu, 'sending', False)
    ).pack(side="left", padx=10)
    control_frame.pack(pady=5)

    # 归零按钮：同时归零位移和角度
    def reset_all():
        imu.reset_all()
        if hasattr(imu, 'clear_angle_offset'):
            imu.clear_angle_offset()
            imu.clear_adc_offset()
    tk.Button(
        root, text="归零", font=("Arial", 16),
        command=reset_all
    ).pack(expand=True, fill="both", padx=20, pady=10)

    root.mainloop()


async def sender():
    # 初始化客户端
    client = TransmissionClient(config_path="config.yaml")
    await client.connect()

    @client.on_message("ack")
    async def handle_ack(data):
        print(f"收到 ack: {data}")

    # 启动 IMU
    imu = IMUPosition(host='0.0.0.0', port=9999, reset_all=True)
    imu.start()
    # 预设发送模式和控制标志
    imu.send_mode = 0   # 0=位移, 1=角度
    imu.sending   = False

    # 启动 UI 窗口线程
    tk_thread = threading.Thread(target=start_tk, args=(imu,), daemon=True)
    tk_thread.start()

    try:
        while True:
            # 如果未处于发送状态，跳过
            if not imu.sending:
                await asyncio.sleep(0.1)
                continue

            # 根据 mode 获取数据
            if imu.send_mode == 0:
                data = imu.get_current_position()
            else:
                data = imu.get_current_angle()

            # 检查数据有效性
            if data is None or any(v is None for v in data):
                await asyncio.sleep(0.1)
                continue

            # 构造并发送消息：前缀 + 三轴数据
            prefix  = imu.send_mode
            rounded = [round(v, 3) for v in data]
            volt=imu.get_current_adc()
            message = [prefix] + rounded+ [volt]
            print(f"发送: {message}")
            # await client.send_message("REMOTE_CONTROL", {"values": message})
            await client.send_message("REMOTE_CONTROL", message)
            await asyncio.sleep(0.1)

    except asyncio.CancelledError:
        print("发送任务已取消。")
    finally:
        imu.stop()


if __name__ == "__main__":
    asyncio.run(sender())
