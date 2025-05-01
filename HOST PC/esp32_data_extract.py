#!/usr/bin/env python3
# encoding:utf-8

import asyncio
import socket
import json
import os

class IMUTCPReceiver:

    # —— 类级共享状态 —— #
    _shared_server = None
    _shared_server_task = None
    _shared_imu_data = {}
    _ref_count = 0
    _start_lock = None

    # 缓存文件
    _cache_file = os.path.abspath("imu_data_cache.json")
    _poll_interval = 0.1  # 文件轮询间隔（秒）

    def __init__(self, host='0.0.0.0', port=9999):
        self.host = host
        self.port = port
        # 实例引用同一份共享数据或文件模式数据
        self._imu_data = IMUTCPReceiver._shared_imu_data
        self._use_file = False
        self._file_task = None
        IMUTCPReceiver._ref_count += 1

    @staticmethod
    def to_int16(val: int) -> int:
        v = val & 0xFFFF
        return v - 0x10000 if (v & 0x8000) else v

    @staticmethod
    def to_int32_from_3bytes(b0, b1, b2) -> int:
        raw = (b2 << 16) | (b1 << 8) | b0
        if raw & 0x800000:
            raw |= ~0xFFFFFF
        return raw

    async def start_tcp(self):
        """启动 TCP 服务器；首个实例实际绑定，失败则转为文件模式。"""
        if IMUTCPReceiver._start_lock is None:
            IMUTCPReceiver._start_lock = asyncio.Lock()
        async with IMUTCPReceiver._start_lock:
            if IMUTCPReceiver._shared_server is None:
                # 尝试绑定端口
                try:
                    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
                    sock.bind((self.host, self.port))
                    sock.close()
                except OSError:
                    # 端口被占用：启动文件模式
                    print(f"[WARN] Port {self.port} in use → file-cache mode")
                    self._use_file = True
                    self._start_file_poll()
                    return

                # 启动 TCP 服务
                server = await asyncio.start_server(self.handle_client, self.host, self.port)
                IMUTCPReceiver._shared_server = server
                IMUTCPReceiver._shared_server_task = asyncio.create_task(server.serve_forever())
                print(f"[INFO] IMU TCP Serving on {self.host}:{self.port}")
            else:
                # 已有服务，直接复用
                print(f"[INFO] Using existing IMU TCP server on {self.host}:{self.port}")

    def _start_file_poll(self):
        """在文件模式下，启动后台任务定期读取缓存文件。"""
        if self._file_task is None:
            self._file_task = asyncio.create_task(self._file_poll_loop())
            print(f"[INFO] Started file-polling on {IMUTCPReceiver._cache_file}")

    async def _file_poll_loop(self):
        """周期性从缓存文件读取最新 IMU 数据。"""
        while self._use_file:
            try:
                with open(IMUTCPReceiver._cache_file, 'r') as f:
                    data = json.load(f)
                    self._imu_data = data
            except Exception:
                pass
            await asyncio.sleep(IMUTCPReceiver._poll_interval)

    async def stop_tcp(self):
        """停止 TCP/文件服务；最后一个实例才真正关闭。"""
        IMUTCPReceiver._ref_count = max(0, IMUTCPReceiver._ref_count - 1)
        if IMUTCPReceiver._ref_count > 0:
            # 仍有实例，不关闭服务
            self._use_file = False
            if self._file_task:
                self._file_task.cancel()
                self._file_task = None
            return

        # 关闭 TCP 服务器
        if IMUTCPReceiver._shared_server_task:
            IMUTCPReceiver._shared_server_task.cancel()
            try: await IMUTCPReceiver._shared_server_task
            except: pass
        if IMUTCPReceiver._shared_server:
            IMUTCPReceiver._shared_server.close()
            await IMUTCPReceiver._shared_server.wait_closed()
        IMUTCPReceiver._shared_server = None
        IMUTCPReceiver._shared_server_task = None
        print("[INFO] TCP server stopped.")

        # 停止文件模式
        self._use_file = False
        if self._file_task:
            self._file_task.cancel()
            self._file_task = None

    async def handle_client(self, reader: asyncio.StreamReader, writer: asyncio.StreamWriter):
        """TCP 客户端处理：逐字节读，通过 _parse_imu 解析并写缓存文件。"""
        addr = writer.get_extra_info('peername')
        print(f"[INFO] New TCP client from {addr}")
        buf = bytearray()
        try:
            while True:
                byte = await reader.readexactly(1)
                buf.append(byte[0])
                # 尝试解析完整包
                while len(buf) >= 5:
                    try:
                        start = buf.index(0x49)
                    except ValueError:
                        buf.clear()
                        break
                    if start > 0:
                        del buf[:start]
                    if len(buf) < 3:
                        break
                    n = buf[2]
                    if n == 0 or n > 75:
                        del buf[0]
                        continue
                    pkt_len = n + 5
                    if len(buf) < pkt_len:
                        break
                    pkt = buf[:pkt_len]
                    # 校验结束码
                    if pkt[-1] != 0x4D:
                        del buf[0]
                        continue
                    # 校验和
                    if sum(pkt[1:3+n]) & 0xFF != pkt[3+n]:
                        del buf[0]
                        continue

                    payload = pkt[3:3+n]
                    self._parse_imu(payload)
                    del buf[:pkt_len]
        except asyncio.IncompleteReadError:
            pass
        except Exception as e:
            print("Error in connection handler:", e)
        finally:
            print(f"[INFO] TCP client {addr} disconnected")
            writer.close()
            await writer.wait_closed()

    def _parse_imu(self, buf: bytearray):
        """
        解析 IMU 数据体，然后提取尾部两字节的 ESP32 ADC 原始值并计算电压。
        解析结果写入共享字典并缓存到 JSON 文件。
        """
        imu = {}

        # 标度因子
        scaleAccel       = 0.00478515625
        scaleQuat        = 0.000030517578125
        scaleAngle       = 0.0054931640625
        scaleAngleSpeed  = 0.06103515625
        scaleMag         = 0.15106201171875
        scaleTemperature = 0.01
        scaleAirPressure = 0.0002384185791
        scaleHeight      = 0.0010728836

        if not buf or buf[0] != 0x11:
            return

        ctl = (buf[2] << 8) | buf[1]
        L = 7

        # 加速度通道1
        if ctl & 0x0001:
            raw = (buf[L+1]<<8)|buf[L]; L+=2
            imu['aX'] = self.to_int16(raw)*scaleAccel
            raw = (buf[L+1]<<8)|buf[L]; L+=2
            imu['aY'] = self.to_int16(raw)*scaleAccel
            raw = (buf[L+1]<<8)|buf[L]; L+=2
            imu['aZ'] = self.to_int16(raw)*scaleAccel

        # 加速度通道2
        if ctl & 0x0002:
            raw = (buf[L+1]<<8)|buf[L]; L+=2
            imu['AX'] = self.to_int16(raw)*scaleAccel
            raw = (buf[L+1]<<8)|buf[L]; L+=2
            imu['AY'] = self.to_int16(raw)*scaleAccel
            raw = (buf[L+1]<<8)|buf[L]; L+=2
            imu['AZ'] = self.to_int16(raw)*scaleAccel

        # 角速度
        if ctl & 0x0004:
            raw = (buf[L+1]<<8)|buf[L]; L+=2
            imu['GX'] = self.to_int16(raw)*scaleAngleSpeed
            raw = (buf[L+1]<<8)|buf[L]; L+=2
            imu['GY'] = self.to_int16(raw)*scaleAngleSpeed
            raw = (buf[L+1]<<8)|buf[L]; L+=2
            imu['GZ'] = self.to_int16(raw)*scaleAngleSpeed

        # 磁场
        if ctl & 0x0008:
            raw = (buf[L+1]<<8)|buf[L]; L+=2
            imu['CX'] = self.to_int16(raw)*scaleMag
            raw = (buf[L+1]<<8)|buf[L]; L+=2
            imu['CY'] = self.to_int16(raw)*scaleMag
            raw = (buf[L+1]<<8)|buf[L]; L+=2
            imu['CZ'] = self.to_int16(raw)*scaleMag

        # 温度/气压/高度
        if ctl & 0x0010:
            raw = (buf[L+1]<<8)|buf[L]; L+=2
            imu['temperature'] = raw*scaleTemperature
            raw = self.to_int32_from_3bytes(buf[L],buf[L+1],buf[L+2]); L+=3
            imu['airPressure'] = raw*scaleAirPressure
            raw = self.to_int32_from_3bytes(buf[L],buf[L+1],buf[L+2]); L+=3
            imu['height']      = raw*scaleHeight

        # 四元数
        if ctl & 0x0020:
            raw = (buf[L+1]<<8)|buf[L]; L+=2
            imu['qw'] = self.to_int16(raw)*scaleQuat
            raw = (buf[L+1]<<8)|buf[L]; L+=2
            imu['qx'] = self.to_int16(raw)*scaleQuat
            raw = (buf[L+1]<<8)|buf[L]; L+=2
            imu['qy'] = self.to_int16(raw)*scaleQuat
            raw = (buf[L+1]<<8)|buf[L]; L+=2
            imu['qz'] = self.to_int16(raw)*scaleQuat

        # 角度
        if ctl & 0x0040:
            raw = (buf[L+1]<<8)|buf[L]; L+=2
            imu['angleX'] = self.to_int16(raw)*scaleAngle
            raw = (buf[L+1]<<8)|buf[L]; L+=2
            imu['angleY'] = self.to_int16(raw)*scaleAngle
            raw = (buf[L+1]<<8)|buf[L]; L+=2
            imu['angleZ'] = self.to_int16(raw)*scaleAngle

        # 偏移
        if ctl & 0x0080:
            raw = (buf[L+1]<<8)|buf[L]; L+=2
            imu['offsetX'] = self.to_int16(raw)/1000.0
            raw = (buf[L+1]<<8)|buf[L]; L+=2
            imu['offsetY'] = self.to_int16(raw)/1000.0
            raw = (buf[L+1]<<8)|buf[L]; L+=2
            imu['offsetZ'] = self.to_int16(raw)/1000.0

        # 步数与活动
        if ctl & 0x0100:
            steps = (buf[L+3]<<24)|(buf[L+2]<<16)|(buf[L+1]<<8)|buf[L]; L+=4
            flags = buf[L]; L+=1
            imu['steps']   = steps
            imu['walking'] = 100 if (flags&0x01) else 0
            imu['running'] = 100 if (flags&0x02) else 0
            imu['biking']  = 100 if (flags&0x04) else 0
            imu['driving'] = 100 if (flags&0x08) else 0

        # 线性加速度
        if ctl & 0x0200:
            raw = (buf[L+1]<<8)|buf[L]; L+=2
            imu['asX'] = self.to_int16(raw)*scaleAccel
            raw = (buf[L+1]<<8)|buf[L]; L+=2
            imu['asY'] = self.to_int16(raw)*scaleAccel
            raw = (buf[L+1]<<8)|buf[L]; L+=2
            imu['asZ'] = self.to_int16(raw)*scaleAccel

        # 原始 IMU ADC
        if ctl & 0x0400:
            raw = (buf[L+1]<<8)|buf[L]; L+=2
            imu['adc_imu'] = raw

        # GPIO1
        if ctl & 0x0800:
            imu['GPIO1'] = buf[L]; L+=1

        # ESP32 ADC 两字节
        if len(buf) >= 2:
            hi, lo = buf[-2], buf[-1]
            raw = (hi<<8)|lo
            volt = raw*3.3/4095.0
            imu['adc_esp32_raw']     = raw
            imu['adc_esp32_voltage'] = volt

        # 更新共享与缓存
        IMUTCPReceiver._shared_imu_data = imu
        self._imu_data = imu
        try:
            with open(IMUTCPReceiver._cache_file, 'w') as f:
                json.dump(imu, f)
        except Exception as e:
            print("Failed to write cache:", e)

    # ---------- Getter 方法 ---------- #
    def get_acceleration(self):
        return (self._imu_data.get('aX'),
                self._imu_data.get('aY'),
                self._imu_data.get('aZ'))

    def get_acceleration2(self):
        return (self._imu_data.get('AX'),
                self._imu_data.get('AY'),
                self._imu_data.get('AZ'))

    def get_gyro(self):
        return (self._imu_data.get('GX'),
                self._imu_data.get('GY'),
                self._imu_data.get('GZ'))

    def get_magnet(self):
        return (self._imu_data.get('CX'),
                self._imu_data.get('CY'),
                self._imu_data.get('CZ'))

    def get_temperature_pressure_height(self):
        return (self._imu_data.get('temperature'),
                self._imu_data.get('airPressure'),
                self._imu_data.get('height'))

    def get_quaternion(self):
        return (self._imu_data.get('qw'),
                self._imu_data.get('qx'),
                self._imu_data.get('qy'),
                self._imu_data.get('qz'))

    def get_angle(self):
        return (self._imu_data.get('angleX'),
                self._imu_data.get('angleY'),
                self._imu_data.get('angleZ'))

    def get_offset(self):
        return (self._imu_data.get('offsetX'),
                self._imu_data.get('offsetY'),
                self._imu_data.get('offsetZ'))

    def get_steps_and_activity(self):
        return (self._imu_data.get('steps'),
                self._imu_data.get('walking'),
                self._imu_data.get('running'),
                self._imu_data.get('biking'),
                self._imu_data.get('driving'))

    def get_linear_acceleration(self):
        return (self._imu_data.get('asX'),
                self._imu_data.get('asY'),
                self._imu_data.get('asZ'))

    def get_adc(self):
        return self._imu_data.get('adc_imu')

    def get_GPIO1(self):
        return self._imu_data.get('GPIO1')

    def get_adc_esp32(self):
        return (self._imu_data.get('adc_esp32_raw')
                )


# ---------- 示例并发启动两个实例 ----------
if __name__ == '__main__':
    async def demo():
        r1 = IMUTCPReceiver()
        r2 = IMUTCPReceiver()
        await asyncio.gather(r1.start_tcp(), r2.start_tcp())
        try:
            while True:
                print("r1 angle:", r1.get_angle(),
                      "r2 ESP32 ADC:", r2.get_adc_esp32())
                await asyncio.sleep(1.0)
        except KeyboardInterrupt:
            await r1.stop_tcp()
            await r2.stop_tcp()

    asyncio.run(demo())
