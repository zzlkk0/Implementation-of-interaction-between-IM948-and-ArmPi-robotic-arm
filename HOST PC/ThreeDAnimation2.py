#!/usr/bin/env python3
# -*- coding: utf-8 -*-
from http.server import BaseHTTPRequestHandler, HTTPServer
import sys
import threading
import asyncio
import io
import time
import numpy as np
import matplotlib
matplotlib.use("QtAgg")
import matplotlib.pyplot as plt
matplotlib.rcParams['font.sans-serif'] = ['SimHei']
matplotlib.rcParams['axes.unicode_minus'] = False
import threading
import time
import io
from http.server import BaseHTTPRequestHandler, HTTPServer
from socketserver import ThreadingMixIn
from PIL import Image

# 全局最新帧与锁
_latest_frame = None
_frame_lock = threading.Lock()

class ThreadingHTTPServer(ThreadingMixIn, HTTPServer):
    """允许多线程处理请求，每个连接一个线程"""
    daemon_threads = True

class MJPEGHandler(BaseHTTPRequestHandler):
    # 捕获客户端断开
    def handle(self):
        try:
            super().handle()
        except ConnectionResetError:
            return

    # 不输出日志
    def log_message(self, fmt, *args):
        pass

    def do_GET(self):
        if self.path != '/':
            self.send_error(404)
            return
        # 发送 MJPEG 头
        self.send_response(200)
        self.send_header('Age','0')
        self.send_header('Cache-Control','no-cache, private')
        self.send_header('Pragma','no-cache')
        self.send_header('Content-Type','multipart/x-mixed-replace; boundary=FRAME')
        self.end_headers()

        global _latest_frame, _frame_lock
        try:
            while True:
                with _frame_lock:
                    frame = _latest_frame
                if frame:
                    self.wfile.write(b"--FRAME\r\n")
                    self.send_header('Content-Type','image/jpeg')
                    self.send_header('Content-Length',str(len(frame)))
                    self.end_headers()
                    self.wfile.write(frame)
                    self.wfile.write(b"\r\n")
                time.sleep(0.016)
        except Exception:
            # 客户端断开等
            pass
from PyQt6.QtWidgets import (
    QApplication, QMainWindow,
    QPushButton, QVBoxLayout,
    QWidget, QHBoxLayout, QLabel
)
from PyQt6.QtCore import QTimer
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

from http.server import BaseHTTPRequestHandler, HTTPServer
from PIL import Image

from esp32_data_extract import IMUTCPReceiver
from filter import IMUFilter, rotate_vector

# 全局最新帧与锁
_latest_frame = None
_frame_lock = threading.Lock()

def quaternion_to_euler(qw, qx, qy, qz):
    sinr = 2*(qw*qx + qy*qz)
    cosr = 1-2*(qx*qx + qy*qy)
    roll = np.arctan2(sinr, cosr)
    sinp = 2*(qw*qy - qz*qx)
    pitch = np.sign(sinp)*np.pi/2 if abs(sinp)>=1 else np.arcsin(sinp)
    siny = 2*(qw*qz + qx*qy)
    cosy = 1-2*(qy*qy + qz*qz)
    yaw = np.arctan2(siny, cosy)
    return roll, pitch, yaw


def start_mjpeg_server(port=7777):
    """启动多线程 MJPEG HTTP Server，支持多客户端并发连接"""
    server = ThreadingHTTPServer(('0.0.0.0', port), MJPEGHandler)
    threading.Thread(target=server.serve_forever, daemon=True).start()
    print(f"[INFO] MJPEG HTTP server started on port {port}")
class IMUGUI(QMainWindow):
    def __init__(self, receiver: IMUTCPReceiver):
        super().__init__()
        self.setWindowTitle("IMU 可视化 + HTTP MJPEG 推流 (7777)")
        self.resize(1600, 700)

        # IMU & filter
        self.receiver = receiver
        self.filter = IMUFilter()
        self.data_raw = []
        self.data_filt = []
        self.pos_hist = []

        # 启动 MJPEG 服务
        start_mjpeg_server(7777)

        # Figures
        self.fig2d, self.axs2d = plt.subplots(3,2,figsize=(6,4))
        self.canvas2d = FigureCanvas(self.fig2d)

        self.fig3d = plt.figure(figsize=(4,4))
        self.ax3d  = self.fig3d.add_subplot(111,projection='3d')
        self.canvas3d = FigureCanvas(self.fig3d)

        self.fig_cube = plt.figure(figsize=(4,4))
        self.ax_cube = self.fig_cube.add_subplot(111,projection='3d')
        self.canvas_cube = FigureCanvas(self.fig_cube)
        self._init_cube()

        # Labels & buttons
        self.acc_label   = QLabel("加速度: N/A")
        self.euler_label = QLabel("欧拉角: roll=0°, pitch=0°, yaw=0°")
        self.start_btn = QPushButton("开始")
        self.start_btn.clicked.connect(self._start_timer)
        self.stop_btn  = QPushButton("停止")
        self.stop_btn.clicked.connect(self._stop_timer)

        # Plot timer
        self.timer = QTimer(self); self.timer.setInterval(16)
        self.timer.timeout.connect(self.update_plot)

        # Layout
        left = QVBoxLayout()
        left.addWidget(self.acc_label)
        left.addWidget(self.euler_label)
        left.addWidget(self.canvas2d)
        left.addWidget(self.start_btn)
        left.addWidget(self.stop_btn)

        middle = QVBoxLayout()
        middle.addWidget(self.canvas3d)

        right = QVBoxLayout()
        right.addWidget(self.canvas_cube)

        main = QHBoxLayout()
        main.addLayout(left,2)
        main.addLayout(middle,2)
        main.addLayout(right,2)

        container = QWidget()
        container.setLayout(main)
        self.setCentralWidget(container)

        # 自动开始
        self._start_timer()

    def _start_timer(self):
        self.timer.start()

    def _stop_timer(self):
        self.timer.stop()

    def _init_cube(self):
        r = [-0.5,0.5]
        verts = np.array([[x,y,z] for x in r for y in r for z in r])
        faces = [[0,1,3,2],[4,5,7,6],[0,1,5,4],
                 [2,3,7,6],[0,2,6,4],[1,3,7,5]]
        self._cube_verts = verts
        self._cube_faces = faces
        self._cube_colors = ['red','green','blue','yellow','cyan','magenta']
        self.ax_cube.set_box_aspect((1,1,1))

    def _draw_cube(self, R):
        self.ax_cube.cla()
        verts = self._cube_verts @ R.T
        faces = [[verts[i] for i in face] for face in self._cube_faces]
        poly = Poly3DCollection(faces, facecolors=self._cube_colors,
                                edgecolors='k', alpha=0.7)
        self.ax_cube.add_collection3d(poly)
        self.ax_cube.set_xlim(-1,1)
        self.ax_cube.set_ylim(-1,1)
        self.ax_cube.set_zlim(-1,1)
        self.canvas_cube.draw()

    def update_plot(self):
        qw,qx,qy,qz = self.receiver.get_quaternion()
        gx,gy,gz  = self.receiver.get_gyro()
        ax,ay,az  = self.receiver.get_acceleration()
        if None in (qw,qx,qy,qz,gx,gy,gz,ax,ay,az):
            return

        z = np.array([qw,qx,qy,qz,gx,gy,gz,ax,ay,az])
        x_f, pos = self.filter.update(z)

        # history
        self.data_raw.append(z); self.data_filt.append(x_f); self.pos_hist.append(pos.copy())
        if len(self.data_raw)>200:
            self.data_raw.pop(0); self.data_filt.pop(0); self.pos_hist.pop(0)

        raw = np.array(self.data_raw); filt = np.array(self.data_filt)

        # 2D
        titles=["wx","wy","wz","ax","ay","az"]
        for i,axp in enumerate(self.axs2d.flatten()):
            axp.cla()
            axp.plot(raw[:,i],alpha=0.5)
            axp.plot(filt[:,i])
            axp.set_title(titles[i])
        self.fig2d.tight_layout()
        self.canvas2d.draw()

        # acc label
        a_w = rotate_vector(x_f[0:4], x_f[7:10])
        self.acc_label.setText(
            f"加速度: ax={a_w[0]:.3f}, ay={a_w[1]:.3f}, az={a_w[2]:.3f}"
        )

        # 3D traj
        pos_arr = np.array(self.pos_hist)
        self.ax3d.cla()
        self.ax3d.plot(pos_arr[:,0],pos_arr[:,1],pos_arr[:,2])
        self.ax3d.set_xlabel("X(m)"); self.ax3d.set_ylabel("Y(m)"); self.ax3d.set_zlabel("Z(m)")
        self.canvas3d.draw()

        # Euler & cube
        roll,pitch,yaw = quaternion_to_euler(qw,qx,qy,qz)
        self.euler_label.setText(
            f"欧拉角: roll={np.degrees(roll):.1f}°, pitch={np.degrees(pitch):.1f}°, yaw={np.degrees(yaw):.1f}°"
        )
        R = np.array([
            [1-2*(qy*qy+qz*qz),   2*(qx*qy - qz*qw),   2*(qx*qz + qy*qw)],
            [2*(qx*qy + qz*qw),   1-2*(qx*qx+qz*qz),   2*(qy*qz - qx*qw)],
            [2*(qx*qz - qy*qw),   2*(qy*qz + qx*qw),   1-2*(qx*qx+qy*qy)]
        ])
        self._draw_cube(R)

        # 组合并生成 JPEG
        f2 = np.asarray(self.canvas2d.renderer.buffer_rgba())[...,:3]
        f3 = np.asarray(self.canvas3d.renderer.buffer_rgba())[...,:3]
        fc = np.asarray(self.canvas_cube.renderer.buffer_rgba())[...,:3]
        H = 400
        im2 = Image.fromarray(f2).resize((500,H))
        imc = Image.fromarray(f3).resize((400,H))
        im3= Image.fromarray(fc).resize((400,H))
        wide = np.hstack([np.array(im2), np.array(im3), np.array(imc)])
        buf = io.BytesIO()
        Image.fromarray(wide).save(buf, format='JPEG')
        frame = buf.getvalue()

        # 更新全局
        global _latest_frame, _frame_lock
        with _frame_lock:
            _latest_frame = frame

if __name__ == "__main__":
    # 启动 TCP 接收器
    receiver = IMUTCPReceiver(host='0.0.0.0', port=9999)
    loop = asyncio.new_event_loop()
    def tcp_thread():
        asyncio.set_event_loop(loop)
        loop.run_until_complete(receiver.start_tcp())
        loop.run_forever()
    threading.Thread(target=tcp_thread, daemon=True).start()

    # 启动 Qt GUI & HTTP MJPEG
    app = QApplication(sys.argv)
    gui = IMUGUI(receiver)
    gui.show()
    sys.exit(app.exec())
