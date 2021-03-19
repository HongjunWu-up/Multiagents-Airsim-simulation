
"""
function：一阶积分器算法，track a reference model.有相对偏差，实现编队。MTF: model to father 假设只有父节点知道模型信息！！！
问题：要求知道模型的位置信息和速度信息，假如在airsim仿真里面还可以实现，假设忽略其他代码执行的时间，每次循环对时间进行积分，对路径进行叠加。
程序后续优化，尽量把重复同类型的代码用for循环书写，相同类型的数据保存在列表里面，思路，可以用一个字符串的变量数组代表drone？具体看怎么实现
author： Wu Hongjun
date  ： 2021.3.16
 """
import airsim
import time
import numpy as np

# 参数初始化
T_control = 0.01  # 初始化无人机api接口的执行周期

X = np.array([[0.0], [3.0], [6.0], [9.0], [12.0], [15.0], [18.0], [21.0]])  # 初始化坐标信息
Y = np.array([[0.0], [0.0], [0.0], [0.0], [0.0], [0.0], [0.0], [0.0]])
Z = np.array([[-5.0], [-4.5], [-4.0], [-6.0], [-5.0], [-4.5], [-4.0], [-6.0]])

X_refer = np.array([[0.0], [0.0], [0.0], [0.0], [0.0], [0.0], [0.0], [0.0]])  # 初始化模型初始坐标
Y_refer = np.array([[0.0], [0.0], [0.0], [0.0], [0.0], [0.0], [0.0], [0.0]])
Z_refer = np.array([[-5.0], [-5.0], [-5.0], [-5.0], [-5.0], [-5.0], [-5.0], [-5.0]])

Vx_refer1 = np.ones((8, 1))   # 生成数组默认为浮点型，运动方向朝向x正方向，记住后面程序有对速度进行限幅，速度不能超过3 m/s
Vy_refer1 = np.zeros((8, 1))
Vz_refer1 = np.zeros((8, 1))

Vx_refer2 = -2*np.ones((8, 1))   # 运动方向朝向x负方向，记住后面程序有对速度进行限幅，速度不能超过3 m/s
Vy_refer2 = np.zeros((8, 1))
Vz_refer2 = np.zeros((8, 1))

Delta_X = np.array([[0.0], [-3.0], [-6.0], [-9.0], [-12.0], [-9.0], [-6.0], [-3.0]])  # 初始化偏差信息
Delta_Y = np.array([[0.0], [-2.5], [-5.0], [-2.5], [0.0], [2.5], [5.0], [2.5]])
Delta_Z = np.array([[0.0], [0.0], [0.0], [0.0], [0.0], [0.0], [0.0], [0.0]])

U_X = np.array([[0.0], [0.0], [0.0], [0.0], [0.0], [0.0], [0.0], [0.0]])  # 初始化控制参数
U_Y = np.array([[0.0], [0.0], [0.0], [0.0], [0.0], [0.0], [0.0], [0.0]])
U_Z = np.array([[0.0], [0.0], [0.0], [0.0], [0.0], [0.0], [0.0], [0.0]])

A = np.array([[0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],  # 初始化 adjacency matrix A
              [1.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0],
              [1.0, 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0],
              [0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0],
              [0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0],
              [0.0, 0.0, 1.0, 0.0, 1.0, 0.0, 0.0, 0.0],
              [0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0],
              [0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0]])
D = np.array([[0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],  # 初始化度矩阵D
              [0.0, 2.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
              [0.0, 0.0, 3.0, 0.0, 0.0, 0.0, 0.0, 0.0],
              [0.0, 0.0, 0.0, 2.0, 0.0, 0.0, 0.0, 0.0],
              [0.0, 0.0, 0.0, 0.0, 2.0, 0.0, 0.0, 0.0],
              [0.0, 0.0, 0.0, 0.0, 0.0, 2.0, 0.0, 0.0],
              [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0],
              [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0]])
Q = np.array([[1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],  # 初始化增益矩阵
              [0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
              [0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0],
              [0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0],
              [0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0],
              [0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0],
              [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0],
              [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0]])

L = D - A   # 拉普拉斯矩阵
"""理论值计算，V列表示特征向量，DL表示对角阵。动态的不能实时预估，仅保留代码，没有任何意义"""
DL, V = np.linalg.eig(L)
print(V, DL)
V = 1/V[0][7] * V
print(V, DL)
W = np.linalg.inv(V)
print(W)
X_final_hat = 0
Y_final_hat = 0
Z_final_hat = 0
X_final = np.zeros((8, 1))
Y_final = np.zeros((8, 1))
Z_final = np.zeros((8, 1))
for i in range(0, 8):   # range 创建列表[0, 1 ... , 7]
    X_final_hat = X_final_hat+W[7][i] * (X[i][0]-Delta_X[i][0])
    Y_final_hat = Y_final_hat+W[7][i] * (Y[i][0]-Delta_Y[i][0])
    Z_final_hat = Z_final_hat+W[7][i] * (Z[i][0]-Delta_Z[i][0])
print("XYZ_final_hat: ", X_final_hat,  Y_final_hat,  Z_final_hat)
for i in range(0, 8):
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
client.enableApiControl(True, "Drone5")
client.enableApiControl(True, "Drone6")
client.enableApiControl(True, "Drone7")
client.enableApiControl(True, "Drone8")
client.armDisarm(True, "Drone1")
client.armDisarm(True, "Drone2")
client.armDisarm(True, "Drone3")
client.armDisarm(True, "Drone4")
client.armDisarm(True, "Drone5")
client.armDisarm(True, "Drone6")
client.armDisarm(True, "Drone7")
client.armDisarm(True, "Drone8")

Drone1 = client.takeoffAsync(vehicle_name="Drone1")  # 第一阶段：起飞
Drone2 = client.takeoffAsync(vehicle_name="Drone2")  # 第一阶段：起飞
Drone3 = client.takeoffAsync(vehicle_name="Drone3")  # 第一阶段：起飞
Drone4 = client.takeoffAsync(vehicle_name="Drone4")  # 第一阶段：起飞
Drone5 = client.takeoffAsync(vehicle_name="Drone5")  # 第一阶段：起飞
Drone6 = client.takeoffAsync(vehicle_name="Drone6")  # 第一阶段：起飞
Drone7 = client.takeoffAsync(vehicle_name="Drone7")  # 第一阶段：起飞
Drone8 = client.takeoffAsync(vehicle_name="Drone8")  # 第一阶段：起飞
Drone1.join()
Drone2.join()
Drone3.join()
Drone4.join()
Drone5.join()
Drone6.join()
Drone7.join()
Drone8.join()

Drone1 = client.moveToZAsync(Z[0][0], 1, vehicle_name="Drone1",)  # 第二阶段：上升到指定高度
Drone2 = client.moveToZAsync(Z[1][0], 1, vehicle_name="Drone2",)
Drone3 = client.moveToZAsync(Z[2][0], 1, vehicle_name="Drone3",)
Drone4 = client.moveToZAsync(Z[3][0], 1, vehicle_name="Drone4",)
Drone5 = client.moveToZAsync(Z[4][0], 1, vehicle_name="Drone5",)  # 第二阶段：上升到指定高度
Drone6 = client.moveToZAsync(Z[5][0], 1, vehicle_name="Drone6",)
Drone7 = client.moveToZAsync(Z[6][0], 1, vehicle_name="Drone7",)
Drone8 = client.moveToZAsync(Z[7][0], 1, vehicle_name="Drone8",)
Drone1.join()
Drone2.join()
Drone3.join()
Drone4.join()
Drone5.join()
Drone6.join()
Drone7.join()
Drone8.join()

# 飞正方形 速度为1m/s
while (1):
    state1 = client.getMultirotorState(vehicle_name="Drone1")  # 获取状态
    state2 = client.getMultirotorState(vehicle_name="Drone2")
    state3 = client.getMultirotorState(vehicle_name="Drone3")
    state4 = client.getMultirotorState(vehicle_name="Drone4")
    state5 = client.getMultirotorState(vehicle_name="Drone5")
    state6 = client.getMultirotorState(vehicle_name="Drone6")
    state7 = client.getMultirotorState(vehicle_name="Drone7")
    state8 = client.getMultirotorState(vehicle_name="Drone8")
    X[0][0] = state1.kinematics_estimated.position.x_val+0  # X机体坐标系转换为全局坐标系,加上差值。
    X[1][0] = state2.kinematics_estimated.position.x_val+3
    X[2][0] = state3.kinematics_estimated.position.x_val+6
    X[3][0] = state4.kinematics_estimated.position.x_val+9
    X[4][0] = state5.kinematics_estimated.position.x_val+12
    X[5][0] = state6.kinematics_estimated.position.x_val+15
    X[6][0] = state7.kinematics_estimated.position.x_val+18
    X[7][0] = state8.kinematics_estimated.position.x_val+21

    Y[0][0] = state1.kinematics_estimated.position.y_val  # Y-机体坐标系转换为全局坐标系，两者一致。
    Y[1][0] = state2.kinematics_estimated.position.y_val
    Y[2][0] = state3.kinematics_estimated.position.y_val
    Y[3][0] = state4.kinematics_estimated.position.y_val
    Y[4][0] = state5.kinematics_estimated.position.y_val
    Y[5][0] = state6.kinematics_estimated.position.y_val
    Y[6][0] = state7.kinematics_estimated.position.y_val
    Y[7][0] = state8.kinematics_estimated.position.y_val

    Z[0][0] = state1.kinematics_estimated.position.z_val  # Z-机体坐标系转换为全局坐标系，高度两者一致。
    Z[1][0] = state2.kinematics_estimated.position.z_val
    Z[2][0] = state3.kinematics_estimated.position.z_val
    Z[3][0] = state4.kinematics_estimated.position.z_val
    Z[4][0] = state5.kinematics_estimated.position.z_val
    Z[5][0] = state6.kinematics_estimated.position.z_val
    Z[6][0] = state7.kinematics_estimated.position.z_val
    Z[7][0] = state8.kinematics_estimated.position.z_val

    U_X = np.dot(-L, X-Delta_X)
    U_Y = np.dot(-L, Y-Delta_Y)
    U_Z = np.dot(-L, Z-Delta_Z)
#    print("U_X:", U_X, "U_Z:", U_Y, "U_Z:", U_Z)  # 输出限幅前的控制输入
    for i in range(0, 8):  # x轴速度限制
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
    print("model", " :  u_xyz:", Vx_refer1[0][0], Vy_refer1[0][0], Vz_refer1[0][0],
          "XYZ:", X_refer[0][0], Y_refer[0][0], Z_refer[0][0])
    for i in range(0, 8):  # 打印agent的控制输入信息、位置信息
        print("drone", i+1, ":  u_xyz:", U_X[i][0], U_Y[i][0], U_Z[i][0], "XYZ:", X[i][0], Y[i][0], Z[i][0])
    # print("drone", 3, ":  u_xyz:", U_X[2][0], U_Y[2][0], U_Z[2][0], "XYZ:", X[2][0], Y[2][0], Z[2][0])
    # drone1 作为father  node ，只有他知道model的信息
    Drone1 = client.moveByVelocityAsync(0.01, Vy_refer1[0][0], Vz_refer1[0][0], T_control, vehicle_name="Drone1")
    Drone2 = client.moveByVelocityAsync(U_X[1][0], U_Y[1][0], U_Z[1][0], T_control, vehicle_name="Drone2")
    Drone3 = client.moveByVelocityAsync(U_X[2][0], U_Y[2][0], U_Z[2][0], T_control, vehicle_name="Drone3")
    Drone4 = client.moveByVelocityAsync(U_X[3][0], U_Y[3][0], U_Z[3][0], T_control, vehicle_name="Drone4")
    Drone5 = client.moveByVelocityAsync(U_X[4][0], U_Y[4][0], U_Z[4][0], T_control, vehicle_name="Drone5")
    Drone6 = client.moveByVelocityAsync(U_X[5][0], U_Y[5][0], U_Z[5][0], T_control, vehicle_name="Drone6")
    Drone7 = client.moveByVelocityAsync(U_X[6][0], U_Y[6][0], U_Z[6][0], T_control, vehicle_name="Drone7")
    Drone8 = client.moveByVelocityAsync(U_X[7][0], U_Y[7][0], U_Z[7][0], T_control, vehicle_name="Drone8")
    Drone1.join()
    Drone2.join()
    Drone3.join()
    Drone4.join()
    Drone5.join()
    Drone6.join()
    Drone7.join()
    Drone8.join()

# 悬停 2 秒钟
Drone1 = client.hoverAsync(vehicle_name="Drone1")  # 第四阶段：悬停6秒钟
Drone2 = client.hoverAsync(vehicle_name="Drone2")  # 第四阶段：悬停6秒钟
Drone3 = client.hoverAsync(vehicle_name="Drone3")  # 第四阶段：悬停6秒钟
Drone4 = client.hoverAsync(vehicle_name="Drone4")  # 第四阶段：悬停6秒钟
Drone5 = client.hoverAsync(vehicle_name="Drone5")  # 第四阶段：悬停6秒钟
Drone6 = client.hoverAsync(vehicle_name="Drone6")  # 第四阶段：悬停6秒钟
Drone7 = client.hoverAsync(vehicle_name="Drone7")  # 第四阶段：悬停6秒钟
Drone8 = client.hoverAsync(vehicle_name="Drone8")  # 第四阶段：悬停6秒钟
Drone1.join()
Drone2.join()
Drone3.join()
Drone4.join()
Drone5.join()
Drone6.join()
Drone7.join()
Drone8.join()

time.sleep(6)  # 保持6秒
Drone1 = client.landAsync(vehicle_name="Drone1")  # 第五阶段：降落
Drone2 = client.landAsync(vehicle_name="Drone2")  # 第五阶段：降落
Drone3 = client.landAsync(vehicle_name="Drone3")  # 第五阶段：降落
Drone4 = client.landAsync(vehicle_name="Drone4")  # 第五阶段：降落
Drone5 = client.landAsync(vehicle_name="Drone5")  # 第五阶段：降落
Drone6 = client.landAsync(vehicle_name="Drone6")  # 第五阶段：降落
Drone7 = client.landAsync(vehicle_name="Drone7")  # 第五阶段：降落
Drone8 = client.landAsync(vehicle_name="Drone8")  # 第五阶段：降落
Drone1.join()
Drone2.join()
Drone3.join()
Drone4.join()
Drone5.join()
Drone6.join()
Drone7.join()
Drone8.join()

client.armDisarm(False, "Drone1")  # 上锁
client.armDisarm(False, "Drone2")  # 上锁
client.armDisarm(False, "Drone3")  # 上锁
client.armDisarm(False, "Drone4")  # 上锁
client.armDisarm(False, "Drone5")  # 上锁
client.armDisarm(False, "Drone6")  # 上锁
client.armDisarm(False, "Drone7")  # 上锁
client.armDisarm(False, "Drone8")  # 上锁

client.enableApiControl(False, "Drone1")  # 释放控制权
client.enableApiControl(False, "Drone2")  # 释放控制权
client.enableApiControl(False, "Drone3")  # 释放控制权
client.enableApiControl(False, "Drone4")  # 释放控制权
client.enableApiControl(False, "Drone5")  # 释放控制权
client.enableApiControl(False, "Drone6")  # 释放控制权
client.enableApiControl(False, "Drone7")  # 释放控制权
client.enableApiControl(False, "Drone8")  # 释放控制权