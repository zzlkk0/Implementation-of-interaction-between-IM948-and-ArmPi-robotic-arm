#!/usr/bin/env python3
# run_zmq_servo.py

import asyncio
from loguru import logger
from Listen_test_class import ZmqRemoteController  # ZMQ 控制类
from servos_class import ServoController             # 舵机控制类

import time
from HiwonderSDK.Board import setBusServoPulse, getBusServoPulse

async def main():
    # 实例化并启动 ZMQ 控制
    zmq_ctrl = ZmqRemoteController()
    zmq_ctrl.turn_on('tcp://192.168.1.63:5556')

    # 实例化舵机控制器，初始化6个舵机位置
    servo_ctrl = ServoController(servo_count=6)
    positions = [100,473,268,634,502,500]

    try:
        while True:
            # 获取最新远程值 [prefix, x, y, z,volt]
            data = zmq_ctrl.get_data()
            if data:
                prefix, x, y, z,volt = data
                logger.info(f"Received REMOTE_VALUE: {data}")
                # 角度模式：prefix==1 时控制舵机2、3、6
                if prefix == 1:
                    positions[0] = int(100 + volt / 7)  # 舵机1
                    positions[1] = int(500 + x * 3)  # 舵机2
                    positions[2] = int(268 + y * 1)  # 舵机3
                    positions[3] = int(634 - y * 1)  # 舵机4
                    positions[5] = int(500 + z * 3)  # 舵机6
                    movetime = servo_ctrl.move(positions)
                    logger.info(f"Servo moved: positions={positions}, time={movetime}ms")
            await asyncio.sleep(0.1)
    except KeyboardInterrupt:
        logger.info("Stopping controller...")
    finally:
        zmq_ctrl.turn_off()
        logger.info("Controller stopped.")

if __name__ == '__main__':
    asyncio.run(main())

