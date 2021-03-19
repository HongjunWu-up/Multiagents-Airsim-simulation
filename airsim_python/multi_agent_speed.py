"""
 飞正方形（速度控制）
 """
import airsim
import time
#本程序存在问题，先后进行，并不是同时操作的。
client = airsim.MultirotorClient()  # connect to the AirSim simulator
client.confirmConnection()
client.enableApiControl(True, "Drone1")
client.enableApiControl(True, "Drone2")
client.armDisarm(True, "Drone1")
client.armDisarm(True, "Drone2")
uav1 = client.takeoffAsync(vehicle_name="Drone1")  # 第一阶段：起飞
uav2 = client.takeoffAsync(vehicle_name="Drone2")  # 第一阶段：起飞
uav1.join()
uav2.join()
uav1 = client.moveToZAsync(-5, 1, vehicle_name="Drone1",)  # 第二阶段：上升到5米高度
uav2 = client.moveToZAsync(-5, 1, vehicle_name="Drone2",)  # 第二阶段：上升到5米高度
uav1.join()
uav2.join()

# 飞正方形 速度为1m/s
uav1 = client.moveByVelocityZAsync(1, 0, -5, 8, vehicle_name="Drone1")  # 第三阶段：以1m/s速度向前飞8秒钟
uav2 = client.moveByVelocityZAsync(1, 0, -5, 8, vehicle_name="Drone2")  # 第三阶段：以1m/s速度向前飞8秒钟
uav1.join()
uav2.join()
uav1 = client.moveByVelocityZAsync(0, 1, -5, 8, vehicle_name="Drone1")  # 第三阶段：以1m/s速度向右飞8秒钟
uav2 = client.moveByVelocityZAsync(0, 1, -5, 8, vehicle_name="Drone2")  # 第三阶段：以1m/s速度向右飞8秒钟
uav1.join()
uav2.join()
uav1 = client.moveByVelocityZAsync(-1, 0, -5, 8, vehicle_name="Drone1")  # 第三阶段：以1m/s速度向后飞8秒钟
uav2 = client.moveByVelocityZAsync(-1, 0, -5, 8, vehicle_name="Drone2")  # 第三阶段：以1m/s速度向后飞8秒钟
uav1.join()
uav2.join()
uav1 = client.moveByVelocityZAsync(0, -1, -5, 8, vehicle_name="Drone1")  # 第三阶段：以1m/s速度向左飞8秒钟
uav2 = client.moveByVelocityZAsync(0, -1, -5, 8, vehicle_name="Drone2")  # 第三阶段：以1m/s速度向左飞8秒钟
uav1.join()
uav2.join()

# 悬停 2 秒钟
uav1 = client.hoverAsync(vehicle_name="Drone1")  # 第四阶段：悬停6秒钟
uav2 = client.hoverAsync(vehicle_name="Drone2")  # 第四阶段：悬停6秒钟
uav1.join()
uav2.join()
time.sleep(6)
uav1 = client.landAsync(vehicle_name="Drone1")  # 第五阶段：降落
uav2 = client.landAsync(vehicle_name="Drone2")  # 第五阶段：降落
uav1.join()
uav2.join()

client.armDisarm(False, "Drone1")  # 上锁
client.armDisarm(False, "Drone2")  # 上锁
client.enableApiControl(False, "Drone1")  # 释放控制权
client.enableApiControl(False, "Drone2")  # 释放控制权
