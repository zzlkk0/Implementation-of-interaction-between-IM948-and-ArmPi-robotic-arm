#!/usr/bin/env python3
# run_imu_actions.py

import asyncio
from loguru import logger

from Listen_test_class import ZmqRemoteController  # ZMQ 控制类
from servos_class import ServoController    
from imu_action_executor import IMUActionExecutor       # 动作执行类

async def main():
    # 1. 启动 ZMQ 控制器
    zmq_ctrl = ZmqRemoteController()
    zmq_ctrl.turn_on('tcp://192.168.1.63:5556')  # 替换为你的 ZMQ PUB 地址
    #print(zmq_ctrl)
    # 2. 初始化执行器
    executor = IMUActionExecutor(zmq_ctrl,0.1)

    # 3. 执行动作控制：内部会循环读取 ctrl.get_last_remote_value()
    try:
        await executor.run()
    except KeyboardInterrupt:
        logger.info("Received interrupt, shutting down...")
    finally:
        zmq_ctrl.turn_off()
        logger.info("Controller stopped.")

if __name__ == '__main__':
    asyncio.run(main())

