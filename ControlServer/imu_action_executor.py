import asyncio
from ArmMoveIK import ArmIK  # 你已有的逆运动学控制类
from HiwonderSDK.Board import setBusServoPulse, getBusServoPulse
from HiwonderSDK.Board import setBusServoPulse, getBusServoPulse

class IMUActionExecutor:
    def __init__(self, controller, interval=0.1):
        self.controller = controller  # RemoteController 实例
        self.arm = ArmIK()
        self.interval = interval  # 控制执行间隔
        self.last_coordinate = None  # 保存上一次接收到的坐标

    async def run(self):
        print("[执行器] 动作控制器启动...")
        while True:
            await asyncio.sleep(self.interval)
            data = self.controller.get_last_remote_value()

            if not data or not isinstance(data, dict):
                continue

            imu = data.get("REMOTE_VALUE")
            if not imu or not isinstance(imu, list) or len(imu) != 5:
                continue

            prefix,x, y, z,volt = imu
            # 根据需要对数据进行缩放和偏
            if prefix==0:
                    x = 80 * x
                    y = 30 * y + 18
                    z = 30 * z + 18
                    coordinate = (x, y, z)
                            #positions[0] = int(100 + volt / 7)  # 舵机1
                    # 如果两次接收到的位置相同，则不执行任何动作
                    if self.last_coordinate == coordinate:
                        continue

                    self.last_coordinate = coordinate

                    target_alpha = 0
                    alpha_min = -90
                    alpha_max = 0

                    result = self.arm.setPitchRangeMoving(coordinate, target_alpha, alpha_min, alpha_max)
                    if result:
                        servos, used_alpha, movetime = result
                        print(f"[动作执行] 舵机: {servos}, α: {used_alpha}, 用时: {movetime}ms")
                    else:
                        print(f"[警告] 无法执行当前坐标: {coordinate}")

