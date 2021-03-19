
"""
程序功能：一阶积分器，有一个leader，有相对偏差，最终在三维空间实现固定队型。
但是由于没有考虑队形变化时无人机的碰撞问题，故在变换过程中有非常大的几率出现碰撞，后续设计中考虑该问题。在前期时可保证初始位置和队形差距不大。


author： wuhongjun
date  ： 2021.3.11
 """
import airsim
import time
import numpy as np
X = np.array([[3.0], [6.0], [9.0], [12.0]])
Y = np.array([[3.0], [6.0], [9.0], [12.0]])
Z = np.array([[-5.0], [-4.5], [-4.0], [-6.0]])

Delta_X = np.array([[0.0], [-3.0], [-3.0], [0.0]])
Delta_Y = np.array([[0.0], [0.0], [3.0], [3.0]])
Delta_Z = np.array([[0.0], [0.0], [0.0], [0.0]])

U_X = np.array([[0.0], [0.0], [0.0], [0.0]])
U_Y = np.array([[0.0], [0.0], [0.0], [0.0]])
U_Z = np.array([[0.0], [0.0], [0.0], [0.0]])

A = np.array([[0.0, 0.0, 0.0, 0.0], [1.0, 0.0, 0.0, 1.0], [1.0, 1.0, 0.0, 0.0], [0.0, 1.0, 0.0, 0.0]])   #  初始化 adjacency matrix A
D = np.array([[0.0, 0.0, 0.0, 0.0], [0.0, 2.0, 0.0, 0.0], [0.0, 0.0, 2.0, 0.0], [0.0, 0.0, 0.0, 1.0]])  # 初始化度矩阵D
L = D - A
"""理论值计算"""
DL, V = np.linalg.eig(L)
print(V, DL)
V = 1/V[0][3] * V
print(V, DL)
W = np.linalg.inv(V)
print(W)
X_final_hat = 0
Y_final_hat = 0
Z_final_hat = 0
X_final = np.zeros((4, 1))
Y_final = np.zeros((4, 1))
Z_final = np.zeros((4, 1))

for i in range(0, 4):
    X_final_hat = X_final_hat+W[3][i] * (X[i][0]-Delta_X[i][0])
    Y_final_hat = Y_final_hat+W[3][i] * (Y[i][0]-Delta_Y[i][0])
    Z_final_hat = Z_final_hat+W[3][i] * (Z[i][0]-Delta_Z[i][0])
print("XYZ_final_hat: ", X_final_hat,  Y_final_hat,  Z_final_hat)
for i in range(0, 4):
    X_final[i][0] = X_final_hat+Delta_X[i][0]
    Y_final[i][0] = Y_final_hat+Delta_Y[i][0]
    Z_final[i][0] = Z_final_hat+Delta_Z[i][0]
print("X_final: ", X_final)
print("Y_final: ", Y_final)
print("Z_final: ", Z_final)
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

Drone1 = client.takeoffAsync(vehicle_name="Drone1")  # 第一阶段：起飞
Drone2 = client.takeoffAsync(vehicle_name="Drone2")  # 第一阶段：起飞
Drone3 = client.takeoffAsync(vehicle_name="Drone3")  # 第一阶段：起飞
Drone4 = client.takeoffAsync(vehicle_name="Drone4")  # 第一阶段：起飞
Drone1.join()
Drone2.join()
Drone3.join()
Drone4.join()

Drone1 = client.moveToZAsync(Z[0][0], 1, vehicle_name="Drone1",)  # 第二阶段：上升到5米高度
Drone2 = client.moveToZAsync(Z[1][0], 1, vehicle_name="Drone2",)
Drone3 = client.moveToZAsync(Z[2][0], 1, vehicle_name="Drone3",)
Drone4 = client.moveToZAsync(Z[3][0], 1, vehicle_name="Drone4",)
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
    X[0][0] = state1.kinematics_estimated.position.x_val+3  # X机体坐标系转换为全局坐标系,加上差值。
    X[1][0] = state2.kinematics_estimated.position.x_val+6
    X[2][0] = state3.kinematics_estimated.position.x_val+9
    X[3][0] = state4.kinematics_estimated.position.x_val+12
    print("X:", X)
    Y[0][0] = state1.kinematics_estimated.position.y_val+3 # X机体坐标系转换为全局坐标系，高度两者一致。
    Y[1][0] = state2.kinematics_estimated.position.y_val+6
    Y[2][0] = state3.kinematics_estimated.position.y_val+9
    Y[3][0] = state4.kinematics_estimated.position.y_val+12
    print("Z:", Z)
    Z[0][0] = state1.kinematics_estimated.position.z_val # X机体坐标系转换为全局坐标系，高度两者一致。
    Z[1][0] = state2.kinematics_estimated.position.z_val
    Z[2][0] = state3.kinematics_estimated.position.z_val
    Z[3][0] = state4.kinematics_estimated.position.z_val
    print("Z:", Z)
    U_X = np.dot(-L, X-Delta_X)
    U_Y = np.dot(-L, Y-Delta_Y)
    U_Z = np.dot(-L, Z-Delta_Z)
    for i in range(0, 4):  # x轴速度限制
        if U_X[i][0] >= 3:
            U_X[i][0] = 3
        elif U_X[i][0] <= -3:
            U_X[i][0] = -3
        elif U_Y[i][0] >= 3:
            U_Y[i][0] = 3
        elif U_Y[i][0] <= -3:
            U_Y[i][0] = -3
        elif U_Z[i][0] >= 3:
            U_Z[i][0] = 3
        elif U_Z[i][0] <= -3:
            U_Z[i][0] = -3
    print("U_X:", U_X)
    print("U_Z:", U_Y)
    print("U_Z:", U_Z)
    print("information(drone1)_XYZ:", state1.kinematics_estimated.position.x_val+3, state1.kinematics_estimated.position.y_val+3, state1.kinematics_estimated.position.z_val)
    print("information(drone2)_XYZ:", state2.kinematics_estimated.position.x_val+6, state2.kinematics_estimated.position.y_val+6, state2.kinematics_estimated.position.z_val)
    print("information(drone3)_XYZ:", state3.kinematics_estimated.position.x_val+9, state3.kinematics_estimated.position.y_val+9, state3.kinematics_estimated.position.z_val)
    print("information(drone4)_XYZ:", state4.kinematics_estimated.position.x_val+12, state4.kinematics_estimated.position.y_val+12, state4.kinematics_estimated.position.z_val)

    Drone1 = client.moveByVelocityAsync(U_X[0][0], U_Y[0][0], U_Z[0][0], 0.1, vehicle_name="Drone1")
    Drone2 = client.moveByVelocityAsync(U_X[1][0], U_Y[1][0], U_Z[1][0], 0.1, vehicle_name="Drone2")
    Drone3 = client.moveByVelocityAsync(U_X[2][0], U_Y[2][0], U_Z[2][0], 0.1, vehicle_name="Drone3")
    Drone4 = client.moveByVelocityAsync(U_X[3][0], U_Y[3][0], U_Z[3][0], 0.1, vehicle_name="Drone4")

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

