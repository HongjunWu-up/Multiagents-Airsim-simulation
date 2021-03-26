
"""
function：二阶积分器，对应任伟老师书上算法（4.2），one leader，有相对偏差，最终在三维空间实现固定队型动态移动。
但是由于没有考虑队形变化时无人机的碰撞问题，故在变换过程中有非常大的几率出现碰撞，后续设计中考虑该问题。在前期时可保证初始位置和队形差距不大。
author： wu hong jun
date  ： 2021.3.24
 """
import airsim
import time
import numpy as np
import math
# 参数初始化
# 初始化载入的无人机，数量用len(Drone)表示 注意不要写成 All_Drones = ['"Drone1"', '"Drone2"'...] 形式，虽然print是"Drone1"，但是实际是'"Drone1"'
All_Drones = ["Drone1", "Drone2", "Drone3", "Drone4", "Drone5", "Drone6", "Drone7", "Drone8"]  # 无人机在此添加
NumOfDrones = len(All_Drones)  # 得到无人机的数量
NumOfState = 3  # 后修改state的个数直接在此修改，现在只用了X Y Z三个state，后面可以用一个矩阵保存所有state的信息，该矩阵的列数就是状态数
T_control = 0.001  # 初始化无人机api接口的执行周期
# gamma = 10  # 定义3.11算法的增益gamma初值  相当于速度所占的权重是位置的10倍，所以速度可以很快收敛，但是位置是慢慢收敛
Kr = 20  # 定义5.14算法的增益gamma初值
Kv = 15
# 注意通信拓扑修改
# 实例化各个函数类对象
Drone_takeoff = [airsim.MultirotorClient().takeoffAsync() for i in range(NumOfDrones)]  # 起飞实例化多个对象
Drone_moveToZ = [airsim.MultirotorClient().moveToZAsync(0, 0) for j in range(NumOfDrones)]  # 上升到指定高度实例化多个对象
Drone_move = [airsim.MultirotorClient().moveByVelocityAsync(0, 0, 0, 0) for k in range(NumOfDrones)]  # 移动实例化多个对象
# 后面其实可以用nx3的矩阵保存数据
Z = np.array([[-5.0], [-4.5], [-4.0], [-6.0], [-5.0], [-4.5], [-4.0], [-6.0]])    # 代码优化后可以设置为初始飞行指定高度
deviation_X = np.array([[0.0], [-3.0], [-9.0], [-15.0], [-21.0], [-18.0], [-12.0], [-6.0]])  # 机体坐标系和全局坐标系之间的初始差距,
deviation = np.array([[0.0,   0.0, 0.0],
                      [-3.0,  0.0, 0.0],
                      [-9.0,  0.0, 0.0],
                      [-15.0, 0.0, 0.0],
                      [-21.0, 0.0, 0.0],
                      [-18.0, 0.0, 0.0],
                      [-12.0, 0.0, 0.0],
                      [-6.0,  0.0, 0.0]])
state = np.zeros((len(All_Drones), NumOfState))   # 每个agent的state 信息，写成矩阵形式
V_speed = np.zeros((len(All_Drones), NumOfState))   # 每个agent的speed 信息，写成矩阵形式
A_acceleration = np.zeros((len(All_Drones), NumOfState))   # 每个agent的加速度信息，写成矩阵形式
V_speed[0][0] = 2.0  # 使得agent1 X的速度为2m/s
speed_limit = 5   # 速度限制
model_state = np.zeros((1, NumOfState))  # 初始化模型初始坐标,可以初始为0，对应setting文件中设置好了的初始位置会在while（）第一次循环中得到
model_acceleration = np.zeros((1, NumOfState))  # 初始化模型初始加速度,可以初始为0，对应setting文件中设置好了的初始位置会在while（）第一次循环中得到
# 模型速度，写成1x3矩阵形式，分别代表x y z 的速度,运动方向朝向x正方向，记住后面程序有对速度进行限幅，速度不能超过3 m/s
V1_model = np.array([[2.0, 0.0, 0.0]])  # 设定值
dV1_model = np.array([[0.0, 0.0, 0.0]])  # 写成1x3矩阵形式，分别代表x y z 的速度
V2_model = np.array([[-2.0, 0.0, 0.0]])  # 写成1x3矩阵形式，分别代表x y z 的速度
V_mode_check = np.zeros((1, NumOfState))   # 检测值

# 初始化 delta 矩阵，第一列是X坐标差值，第二列是坐标差值，第三列是z坐标差值，后面有增加的state再增加
Delta = np.array([[-3.0,  0.0,  0.0],
                  [-6.0,  -2.5,  0.0],
                  [-9.0,  -5.0, 0.0],
                  [-12.0, -2.5, 0.0],
                  [-15.0, 0.0,  0.0],
                  [-12.0, 2.5,  0.0],
                  [-9.0,  5.0,  0.0],
                  [-6.0,  2.5,  0.0]])
U_control = np.zeros((len(All_Drones), NumOfState))  # 用无人机数量*状态量的矩阵来保存控制信息


# A = np.array([[0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0],  # 初始化 adjacency matrix A，只有父节点知道模型信息
#               [1.0, 0.0, 0.0, 1.0, 1.0, 0.0, 0.0, 0.0, 0.0],
#               [1.0, 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0],
#               [0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0],
#               [0.0, 0.0, 1.0, 1.0, 0.0, 0.0, 1.0, 1.0, 0.0],
#               [0.0, 0.0, 1.0, 0.0, 1.0, 1.0, 0.0, 0.0, 0.0],
#               [0.0, 0.0, 0.0, 1.0, 1.0, 0.0, 0.0, 0.0, 0.0],
#               [0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 0.0, 0.0, 0.0],
#               [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]])
# D = np.array([[1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],  # 初始化度矩阵D
#               [0.0, 3.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
#               [0.0, 0.0, 3.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
#               [0.0, 0.0, 0.0, 2.0, 0.0, 0.0, 0.0, 0.0, 0.0],
#               [0.0, 0.0, 0.0, 0.0, 4.0, 0.0, 0.0, 0.0, 0.0],
#               [0.0, 0.0, 0.0, 0.0, 0.0, 3.0, 0.0, 0.0, 0.0],
#               [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 2.0, 0.0, 0.0],
#               [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 2.0, 0.0],
#               [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]])
A = np.array([[0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0],  # 换一种通信模式，所有followers都知道模型的信息。
              [1.0, 0.0, 0.0, 1.0, 1.0, 0.0, 0.0, 0.0, 1.0],
              [1.0, 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 1.0],
              [0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 1.0],
              [0.0, 0.0, 1.0, 1.0, 0.0, 0.0, 1.0, 1.0, 1.0],
              [0.0, 0.0, 1.0, 0.0, 1.0, 1.0, 0.0, 0.0, 1.0],
              [0.0, 0.0, 0.0, 1.0, 1.0, 0.0, 0.0, 0.0, 1.0],
              [0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 0.0, 0.0, 1.0],
              [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]])
D = np.array([[1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],  # 初始化度矩阵D
              [0.0, 4.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
              [0.0, 0.0, 4.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
              [0.0, 0.0, 0.0, 3.0, 0.0, 0.0, 0.0, 0.0, 0.0],
              [0.0, 0.0, 0.0, 0.0, 5.0, 0.0, 0.0, 0.0, 0.0],
              [0.0, 0.0, 0.0, 0.0, 0.0, 4.0, 0.0, 0.0, 0.0],
              [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 3.0, 0.0, 0.0],
              [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 3.0, 0.0],
              [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]])

L = D - A   # 拉普拉斯矩阵
"""理论值计算，V列表示特征向量，DL表示对角阵。动态的不能实时预估，仅保留代码，没有任何意义"""
# DL, V = np.linalg.eig(L)
# print("V:", V, "\r\nDL", DL, "\r\n")
# V = 1/V[0][NumOfDrones-1] * V
# print("V:", V, "\r\nDL", DL, "\r\n")
# W = np.linalg.inv(V)
# print(W)

client = airsim.MultirotorClient()  # connect to the AirSim simulator

client.confirmConnection()
client.enableApiControl(True, "model")  # get control.
for i in range(NumOfDrones):
    client.enableApiControl(True, All_Drones[i])

client.armDisarm(True, "model")  # 解锁
for i in range(NumOfState):
    client.armDisarm(True, All_Drones[i])

model = client.takeoffAsync(vehicle_name="model")  # 第一阶段：起飞
for i in range(NumOfDrones):
    Drone_takeoff[i] = client.takeoffAsync(vehicle_name=All_Drones[i])
model.join()
for i in range(NumOfState):
    Drone_takeoff[i].join()

model = client.moveToZAsync(-5, 1, vehicle_name="model",)  # 第二阶段：上升到指定高度
for i in range(NumOfDrones):
    Drone_moveToZ[i] = client.moveToZAsync(Z[i][0], 1, vehicle_name=All_Drones[i])  # 第二阶段：上升到指定高度
model.join()
for i in range(0, len(All_Drones)):
    Drone_moveToZ[i].join()
start_time = time.time()

pos_reserve_model = np.zeros((1, NumOfState))   # 画图，上一个位置点
pos_reserve_followers = np.zeros((len(All_Drones), NumOfState))
point_reserve_followers = [airsim.Vector3r(0, 0, 0) for n in range(NumOfDrones)]

while 1:
    transform_state = client.getMultirotorState(vehicle_name="model").kinematics_estimated
    model_state = np.array([[transform_state.position.x_val+3, transform_state.position.y_val, transform_state.position.z_val]])
    V_mode_check = np.array([[transform_state.linear_velocity.x_val, transform_state.linear_velocity.y_val, transform_state.linear_velocity.z_val]])
    model_acceleration = np.array([[transform_state.linear_acceleration.x_val, transform_state.linear_acceleration.y_val, transform_state.linear_acceleration.z_val]])

    for i in range(NumOfDrones):  # 获取状态 deviation 是机体坐标和全局坐标的偏差
        transform_state = client.getMultirotorState(vehicle_name=All_Drones[i]).kinematics_estimated
        state[i] = np.array([[transform_state.position.x_val + deviation[i][0], transform_state.position.y_val + deviation[i][1], transform_state.position.z_val + deviation[i][2]]])
        V_speed[i] = np.array([[transform_state.linear_velocity.x_val, transform_state.linear_velocity.y_val, transform_state.linear_velocity.z_val]])
        A_acceleration[i] = np.array([[transform_state.linear_acceleration.x_val, transform_state.linear_acceleration.y_val, transform_state.linear_acceleration.z_val]])
    ## 突然想到一个概念，如果是追踪虚拟模型的话就用计算值，如果是追踪实际的leaders的话，就用实际返回值。
    # for i in range(NumOfState):  #算法3.11 做测试
    #     for j in range(NumOfDrones):
    #         add = 0  # 中间求和迭代值
    #         for k in range(NumOfDrones):
    #             add += A[j][k]*(U_control[k][i]-((state[j][i]-state[k][i])-(Delta[j][i]-Delta[k][i])))
    #         U_control[j][i] = 1/D[j][j]*(add+A[j][NumOfDrones] *
    #                                      (V1_model[0][i]-(state[j][i]-Delta[j][i]-model_state[0][i])))
    # for i in range(NumOfState):  # 两种情况，一种是直接获取，一种计算得到。这是第一种，和模型加速度获取方式一致，都是API接口获得。理论上是这种形式，因为就是通过现有的信息去得到下一步的控制值。
    #     for j in range(NumOfDrones):
    #         add = 0  # 中间求和迭代值
    #         for k in range(NumOfDrones):
    #             add += A[j][k]*(A_acceleration[k][i]-Kr*((state[j][i]-state[k][i])-(Delta[j][i]-Delta[k][i]))-Kv*(V_speed[j][i]-V_speed[k][i]))
    #         U_control[j][i] = 1/D[j][j]*(add+A[j][NumOfDrones] * (model_acceleration[0][i]-Kr*(state[j][i]-Delta[j][i]-model_state[0][i])-Kv*(V_speed[j][i]-V_mode_check[0][i])))

    # for i in range(NumOfState):  # 两种情况，一种是直接获取，一种计算得到。这是第二种，都是计算得到。
    #     for j in range(NumOfDrones):
    #         add = 0  # 中间求和迭代值
    #         for k in range(NumOfDrones):
    #             add += A[j][k]*(U_control[k][i]-Kr*((state[j][i]-state[k][i])-(Delta[j][i]-Delta[k][i]))-Kv*(V_speed[j][i]-V_speed[k][i]))
    #         U_control[j][i] = 1/D[j][j]*(add+A[j][NumOfDrones] * (dV1_model[0][i]-Kr*(state[j][i]-Delta[j][i]-model_state[0][i])-Kv*(V_speed[j][i]-V1_model[0][i])))

    for i in range(NumOfState):  # 第三种种情况，部分是直接获取，部分计算得到。这是第二种，都是计算得到。
        for j in range(NumOfDrones):
            add = 0  # 中间求和迭代值
            for k in range(NumOfDrones):
                add += A[j][k]*(U_control[k][i]-Kr*((state[j][i]-state[k][i])-(Delta[j][i]-Delta[k][i]))-Kv*(V_speed[j][i]-V_speed[k][i]))
            U_control[j][i] = 1/D[j][j]*(add+A[j][NumOfDrones] * (model_acceleration[0][i]-Kr*(state[j][i]-Delta[j][i]-model_state[0][i])-Kv*(V_speed[j][i]-V_mode_check[0][i])))

    end_time = time.time()  # 计算好U_CONTROL 后开始执行速度控制程序的时候为开始时间
    V_speed += (end_time - start_time) * U_control
    print("time:", end_time - start_time)
    start_time = time.time()  # 计算好U_CONTROL 后开始执行速度控制程序的时候为开始时间

    code_time = time.time()
    V1_model = np.array([[2.0, math.sin(code_time/5), 0.0]])
    dV1_model = np.array([[0.0, 1/5*math.cos(code_time), 0.0]])   # 注意有两种方法，一种是根据api接口得到实际的加速度，另一种是设定。先实验第一种。本行注释掉。
    for i in range(NumOfState):
        for j in range(NumOfDrones):  # x轴速度限制
            if V_speed[j][i] >= speed_limit:
                V_speed[j][i] = speed_limit
            elif V_speed[j][i] <= -speed_limit:
                V_speed[j][i] = -speed_limit
    print("model", " :  A_xyz:", model_acceleration[0], "speed_xyz", V_mode_check[0], "XYZ:", model_state[0])
    for i in range(len(All_Drones)):  # 打印agent的控制输入信息、位置信息
        print(All_Drones[i], ":  A_xyz:", U_control[i], "speed_xyz", V_speed[i],  "XYZ:", state[i])

    model = client.moveByVelocityAsync(V1_model[0][0], V1_model[0][1], V1_model[0][2], T_control,
                                       vehicle_name="model")  # 速度控制
    for i in range(NumOfDrones):
        Drone_move[i] = client.moveByVelocityAsync(V_speed[i][0], V_speed[i][1], V_speed[i][2], T_control,
                                                   vehicle_name=All_Drones[i])
        # Drone_move[i] = client.moveByVelocityAsync(U_control[i][0], U_control[i][1], U_control[i][2], T_control, # 算法3.11 测试
        #                                            vehicle_name=All_Drones[i])
    model.join()
    for i in range(NumOfDrones):
        Drone_move[i].join()
    # 下面的几行代码是画图代码
    # model
    point_reserve_model = [airsim.Vector3r(pos_reserve_model[0, 0], pos_reserve_model[0, 1], pos_reserve_model[0, 2])]
    point_model = [airsim.Vector3r(model_state[0][0], model_state[0][1], model_state[0][2])]
    client.simPlotLineList(point_reserve_model + point_model, color_rgba=[1.0, 0.0, 0.0, 1.0], is_persistent=True)
    pos_reserve_model = model_state
    # followers
    point_reserve_followers1 = [
        airsim.Vector3r(pos_reserve_followers[0, 0], pos_reserve_followers[0, 1], pos_reserve_followers[0, 2])]
    point_reserve_followers2 = [
        airsim.Vector3r(pos_reserve_followers[1, 0], pos_reserve_followers[1, 1], pos_reserve_followers[1, 2])]
    point_reserve_followers3 = [
        airsim.Vector3r(pos_reserve_followers[2, 0], pos_reserve_followers[2, 1], pos_reserve_followers[2, 2])]
    point_reserve_followers4 = [
        airsim.Vector3r(pos_reserve_followers[3, 0], pos_reserve_followers[3, 1], pos_reserve_followers[3, 2])]
    point_reserve_followers5 = [
        airsim.Vector3r(pos_reserve_followers[4, 0], pos_reserve_followers[4, 1], pos_reserve_followers[4, 2])]
    point_reserve_followers6 = [
        airsim.Vector3r(pos_reserve_followers[5, 0], pos_reserve_followers[5, 1], pos_reserve_followers[5, 2])]
    point_reserve_followers7 = [
        airsim.Vector3r(pos_reserve_followers[6, 0], pos_reserve_followers[0, 1], pos_reserve_followers[6, 2])]
    point_reserve_followers8 = [
        airsim.Vector3r(pos_reserve_followers[7, 0], pos_reserve_followers[7, 1], pos_reserve_followers[7, 2])]

    point_followers1 = [airsim.Vector3r(state[0, 0], state[0, 1], state[0, 2])]
    point_followers2 = [airsim.Vector3r(state[1, 0], state[1, 1], state[1, 2])]
    point_followers3 = [airsim.Vector3r(state[2, 0], state[2, 1], state[2, 2])]
    point_followers4 = [airsim.Vector3r(state[3, 0], state[3, 1], state[3, 2])]
    point_followers5 = [airsim.Vector3r(state[4, 0], state[4, 1], state[4, 2])]
    point_followers6 = [airsim.Vector3r(state[5, 0], state[5, 1], state[5, 2])]
    point_followers7 = [airsim.Vector3r(state[6, 0], state[0, 1], state[6, 2])]
    point_followers8 = [airsim.Vector3r(state[7, 0], state[7, 1], state[7, 2])]

    client.simPlotLineList(point_reserve_followers1 + point_followers1, color_rgba=[0.0, 0.0, 1.0, 1.0],
                           is_persistent=True)
    client.simPlotLineList(point_reserve_followers2 + point_followers2, color_rgba=[0.0, 1.0, 0.0, 1.0],
                           is_persistent=True)
    client.simPlotLineList(point_reserve_followers3 + point_followers3, color_rgba=[1.0, 1.0, 0.5, 1.0],
                           is_persistent=True)
    client.simPlotLineList(point_reserve_followers4 + point_followers4, color_rgba=[1.0, 0.5, 0.6, 1.0],
                           is_persistent=True)
    client.simPlotLineList(point_reserve_followers5 + point_followers5, color_rgba=[0.5, 0.5, 1.0, 1.0],
                           is_persistent=True)
    client.simPlotLineList(point_reserve_followers6 + point_followers6, color_rgba=[0.0, 0.5, 1.0, 1.0],
                           is_persistent=True)
    client.simPlotLineList(point_reserve_followers7 + point_followers7, color_rgba=[0.0, 0.5, 1.0, 1.0],
                           is_persistent=True)
    client.simPlotLineList(point_reserve_followers8 + point_followers8, color_rgba=[0.5, 0.0, 1.0, 1.0],
                           is_persistent=True)

    pos_reserve_followers = state
    # follower

# 悬停 2 秒钟,由于是while（1），所以不会执行到这一句
model = client.hoverAsync(vehicle_name="model")  # 第四阶段：悬停6秒钟
Drone = [airsim.MultirotorClient().hoverAsync() for i in range(len(All_Drones))]  # 实例化多个对象
for i in range(0, len(All_Drones)):
    Drone[i] = client.hoverAsync(vehicle_name=All_Drones[i])
model.join()
for i in range(0, len(All_Drones)):
    Drone[i].join()

time.sleep(6)  # 保持6秒

model = client.landAsync(vehicle_name="model")  # 第五阶段：降落
Drone = [airsim.MultirotorClient().landAsync() for i in range(len(All_Drones))]  # 实例化多个对象
for i in range(0, len(All_Drones)):
    Drone[i] = client.landAsync(vehicle_name=All_Drones[i])
model.join()
for i in range(0, len(All_Drones)):
    Drone[i].join()

client.armDisarm(False, "model")  # 上锁
for i in range(0, len(All_Drones)):
    client.armDisarm(False, All_Drones[i])  # 上锁

client.enableApiControl(False, "model")  # 释放控制权
for i in range(0, len(All_Drones)):
    client.enableApiControl(False, All_Drones[i])  # 释放控制权
