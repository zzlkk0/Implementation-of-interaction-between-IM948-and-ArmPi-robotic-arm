import asyncio
import json
import zmq
import zmq.asyncio
from Board import getBusServoPulse
from loguru import logger

class ZmqRemoteController:
    """
    基于 ZMQ 的远程控制器。
    功能：
      - 打开/关闭接收，可设置 PC Pub 地址；
      - 异步监听并处理 REMOTE_VALUE 消息；
      - 提供 get_data() 和 get_last_remote_value() 给上层调用；
      - 同时周期性记录舵机脉冲信息。
    """
    def __init__(self):
        self.active = False
        self.last_remote_value = None
        self._sub = None
        self._zmq_ctx = None
        self._listen_task = None
        self._servo_task = None

    def turn_on(self, zmq_sub_url: str):
        """
        打开接收器并连接到指定的 ZMQ Pub 地址。
        """
        if self.active:
            logger.warning("Controller already active.")
            return
        self._zmq_ctx = zmq.asyncio.Context()
        self._sub = self._zmq_ctx.socket(zmq.SUB)
        self._sub.connect(zmq_sub_url)
        self._sub.setsockopt_string(zmq.SUBSCRIBE, '')
        self.active = True
        self._listen_task = asyncio.create_task(self._zmq_listener())
        self._servo_task  = asyncio.create_task(self._send_servo_pulses())
        logger.info(f"Activated ZMQ listener on {zmq_sub_url}")

    def turn_off(self):
        """
        关闭接收器，停止接收和舵机任务。
        """
        if not self.active:
            logger.warning("Controller not active.")
            return
        self.active = False
        self._listen_task.cancel()
        self._servo_task.cancel()
        self._sub.close()
        self._zmq_ctx.term()
        logger.info("Deactivated controller and closed ZMQ connection.")

    async def _zmq_listener(self):
        """
        异步监听 ZMQ SUB socket，接收 REMOTE_VALUE 并存储原始数据。
        """
        while self.active:
            try:
                msg = await self._sub.recv()
            except asyncio.CancelledError:
                break
            try:
                data = json.loads(msg.decode('utf-8'))
            except (json.JSONDecodeError, UnicodeDecodeError):
                data = eval(msg.decode('utf-8'))
            # data 格式: [prefix, x, y, z]
            self.last_remote_value = data
            logger.info(f"Raw REMOTE_VALUE received: {data}")

    async def _send_servo_pulses(self):
        """
        周期性记录舵机当前脉冲到日志。
        """
        try:
            while self.active:
                pulses = {sid: getBusServoPulse(sid) for sid in range(1, 7)}
                logger.info(f"Servo pulses: {pulses}")
                await asyncio.sleep(3)
        except asyncio.CancelledError:
            pass

    def get_data(self):
        """
        直接返回最近接收到的原始 REMOTE_VALUE 列表，比如 [0, x, y, z]。
        """
        return self.last_remote_value

    def get_last_remote_value(self):
        """
        兼容旧执行器接口：
        - 仅当 prefix==0 时，把后三位封装成 {'imu': [x, y, z]}，否则返回 None。
        """
        d = self.last_remote_value
        if (isinstance(d, list) and len(d) == 4 and d[0] == 0):
            return {'imu': d[1:4]}
        return None

# 以下仅为演示，不会在 import 时执行
if __name__ == '__main__':
    import asyncio
    async def demo():
        ctrl = ZmqRemoteController()
        ctrl.turn_on('tcp://192.168.1.63:5556')
        try:
            while True:
                print("get_data:", ctrl.get_data(), "get_last:", ctrl.get_last_remote_value())
                await asyncio.sleep(1)
        except KeyboardInterrupt:
            ctrl.turn_off()
    asyncio.run(demo())

