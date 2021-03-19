
"""
程序功能：一阶积分器，有一个leader，最终X坐标达到一致（一维空间）
author： wuhongjun
date  ： 2021.3.11

 """
import airsim
import time
import numpy as np
X = np.array([[3.0], [6.0], [9.0], [12.0]])  # 转置比较麻烦，先写成行向量，后面再修改；
U = np.array([[0.0], [0.0], [0.0], [0.0]])
print("U:", U)
print(X)
A = np.array([[0.0, 0.0, 0.0, 0.0], # 初始化 adjacency matrix A
              [1.0, 0.0, 0.0, 1.0],
              [1.0, 1.0, 0.0, 0.0],
              [0.0, 1.0, 0.0, 0.0]])
D = np.array([[0.0, 0.0, 0.0, 0.0], [0.0, 2.0, 0.0, 0.0], [0.0, 0.0, 2.0, 0.0], [0.0, 0.0, 0.0, 1.0]])  # 初始化度矩阵D
L = D - A
U = np.dot(-L, X)
print("U:", U)
"""理论值计算"""
DL, V = np.linalg.eig(L)
print(V, DL)
V = 1/V[0][3] * V
print(V, DL)
W = np.linalg.inv(V)
print(W)
X_final = 0
for i in range(0, 4):
    X_final = X_final+W[3][i] * X[i][0]
print("X_final: ", X_final)


client = airsim.MultirotorClient()  # connect to the AirSim simulator
client.confirmConnection()
client.enableApiControl(True, "Drone1")
client.enableApiControl(True, "Drone2")
client.enableApiControl(True, "Drone3")
client.enableApiControl(True, "Drone4")
client.armDisarm(True, "Drone1")
client.armDisarm(True, "Drone2")
client.armDisarm(True, "Drone3")
client.armDisarm(True, "Drone4")
# Drone1 = client.takeoffAsync(vehicle_name="Drone1")  # 第一阶段：起飞
# Drone2 = client.takeoffAsync(vehicle_name="Drone2")  # 第一阶段：起飞
# Drone3 = client.takeoffAsync(vehicle_name="Drone3")  # 第一阶段：起飞
# Drone4 = client.takeoffAsync(vehicle_name="Drone4")  # 第一阶段：起飞
# Drone1.join()
# Drone2.join()
# Drone3.join()
# Drone4.join()
# Drone1 = client.goHomeAsync(vehicle_name="Drone1")  # 回到起始位置
# Drone2 = client.goHomeAsync(vehicle_name="Drone2")
# Drone3 = client.goHomeAsync(vehicle_name="Drone3")  # 回到起始位置
# Drone4 = client.goHomeAsync(vehicle_name="Drone4")
# Drone1.join()
# Drone2.join()
# Drone3.join()
# Drone4.join()
Drone1 = client.takeoffAsync(vehicle_name="Drone1")  # 第一阶段：起飞
Drone2 = client.takeoffAsync(vehicle_name="Drone2")  # 第一阶段：起飞
Drone3 = client.takeoffAsync(vehicle_name="Drone3")  # 第一阶段：起飞
Drone4 = client.takeoffAsync(vehicle_name="Drone4")  # 第一阶段：起飞
Drone1.join()
Drone2.join()
Drone3.join()
Drone4.join()

Drone1 = client.moveToZAsync(-5, 1, vehicle_name="Drone1",)  # 第二阶段：上升到5米高度
Drone2 = client.moveToZAsync(-5, 1, vehicle_name="Drone2",)  # 第二阶段：上升到5米高度
Drone3 = client.moveToZAsync(-5, 1, vehicle_name="Drone3",)  # 第二阶段：上升到5米高度
Drone4 = client.moveToZAsync(-5, 1, vehicle_name="Drone4",)  # 第二阶段：上升到5米高度
Drone1.join()
Drone2.join()
Drone3.join()
Drone4.join()

# 飞正方形 速度为1m/s
while (1):
    state1 = client.getMultirotorState(vehicle_name="Drone1")
    state2 = client.getMultirotorState(vehicle_name="Drone2")
    state3 = client.getMultirotorState(vehicle_name="Drone3")
    state4 = client.getMultirotorState(vehicle_name="Drone4")
    X[0][0] = state1.kinematics_estimated.position.x_val+3  # 机体坐标系转换为全局坐标系。
    X[1][0] = state2.kinematics_estimated.position.x_val+6
    X[2][0] = state3.kinematics_estimated.position.x_val+9
    X[3][0] = state4.kinematics_estimated.position.x_val+12
    print("X:", X)
    U = np.dot(-L, X)
    for i in range(0, 4):
        if U[i][0] >= 3:
            U[i][0] = 3
        elif U[i][0] <= -3:
            U[i][0] = -3
    print("U:", U)
    print("information(drone1)_XYZ:", state1.kinematics_estimated.position.x_val+3, state1.kinematics_estimated.position.y_val+3, state1.kinematics_estimated.position.z_val)
    print("information(drone2)_XYZ:", state2.kinematics_estimated.position.x_val+6, state2.kinematics_estimated.position.y_val+6, state2.kinematics_estimated.position.z_val)
    print("information(drone3)_XYZ:", state3.kinematics_estimated.position.x_val+9, state3.kinematics_estimated.position.y_val+9, state3.kinematics_estimated.position.z_val)
    print("information(drone4)_XYZ:", state4.kinematics_estimated.position.x_val+12, state4.kinematics_estimated.position.y_val+12, state4.kinematics_estimated.position.z_val)
    # 注意此API接口是固定高度的，所以仿真结果更明显为直线
    Drone1 = client.moveByVelocityZAsync(U[0][0], 0, -5, 0.1, vehicle_name="Drone1")
    Drone2 = client.moveByVelocityZAsync(U[1][0], 0, -5, 0.1, vehicle_name="Drone2")
    Drone3 = client.moveByVelocityZAsync(U[2][0], 0, -5, 0.1, vehicle_name="Drone3")
    Drone4 = client.moveByVelocityZAsync(U[3][0], 0, -5, 0.1, vehicle_name="Drone4")

    #  注意此API接口是不固定高度的，随后最后的仿真结果看起来是有些偏差，其实有高度差造成的，但是从y轴负方向向正方向看去是一条直线，所有agent的横坐标收敛为一个值。
    # Drone1 = client.moveByVelocityAsync(U[0][0], 0, 0, 0.1, vehicle_name="Drone1")
    # Drone2 = client.moveByVelocityAsync(U[1][0], 0, 0, 0.1, vehicle_name="Drone2")
    # Drone3 = client.moveByVelocityAsync(U[2][0], 0, 0, 0.1, vehicle_name="Drone3")
    # Drone4 = client.moveByVelocityAsync(U[3][0], 0, 0, 0.1, vehicle_name="Drone4")

    Drone1.join()
    Drone2.join()
    Drone3.join()
    Drone4.join()

# 悬停 2 秒钟
Drone1 = client.hoverAsync(vehicle_name="Drone1")  # 第四阶段：悬停6秒钟
Drone2 = client.hoverAsync(vehicle_name="Drone2")  # 第四阶段：悬停6秒钟
Drone3 = client.hoverAsync(vehicle_name="Drone3")  # 第四阶段：悬停6秒钟
Drone4 = client.hoverAsync(vehicle_name="Drone4")  # 第四阶段：悬停6秒钟
Drone1.join()
Drone2.join()
Drone3.join()
Drone4.join()
time.sleep(6)
Drone1 = client.landAsync(vehicle_name="Drone1")  # 第五阶段：降落
Drone2 = client.landAsync(vehicle_name="Drone2")  # 第五阶段：降落
Drone3 = client.landAsync(vehicle_name="Drone3")  # 第五阶段：降落
Drone4 = client.landAsync(vehicle_name="Drone4")  # 第五阶段：降落
Drone1.join()
Drone2.join()
Drone3.join()
Drone4.join()

client.armDisarm(False, "Drone1")  # 上锁
client.armDisarm(False, "Drone2")  # 上锁
client.armDisarm(False, "Drone3")  # 上锁
client.armDisarm(False, "Drone4")  # 上锁
client.enableApiControl(False, "Drone1")  # 释放控制权
client.enableApiControl(False, "Drone2")  # 释放控制权
client.enableApiControl(False, "Drone3")  # 释放控制权
client.enableApiControl(False, "Drone4")  # 释放控制权

