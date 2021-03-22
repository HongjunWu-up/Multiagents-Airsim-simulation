
"""
function：利用任伟老师书上算法（3.11），一阶积分器算法，track a reference model.有相对偏差，实现编队。MTP:model to partial agents 假设是部分的agents都知道模型信息！！！
问题1：要求知道模型的位置信息和速度信息，假如在 airsim 仿真里面还可以实现，假设忽略其他代码执行的时间，每次循环对时间进行积分，对路径进行叠加。
注意，T_control = 0.01  # 初始化无人机api接口的执行周期 这个参数很重要，越小整个系统越接近连续系统。周期大了的话，则系统有可能发散，
T_control = 0.1效果就不好
疑问2：这种方式实现了固定队型追踪，但是最终速度不是2m/s；有点迷。直接加入一个模型无人机，问题得到解决，此时可以直接获取模型的信息
程序后续优化，尽量把重复同类型的代码用for循环书写，相同类型的数据保存在列表里面，思路，可以用一个字符串的变量数组代表drone？具体看怎么实现
author： Wu Hongjun
date  ： 2021.3.22
 """
import airsim
import numpy as np
# 参数初始化
# 初始化载入的无人机，数量用len(Drone)表示 注意不要写成 All_Drones = ['"Drone1"', '"Drone2"'...] 形式，虽然print是"Drone1"，但是实际是'"Drone1"'
All_Drones = ["Drone1", "Drone2", "Drone3", "Drone4", "Drone5", "Drone6", "Drone7", "Drone8"]  # 无人机在此添加
NumOfDrones = len(All_Drones)  # 得到无人机的数量
NumOfState = 3  # 后修改state的个数直接在此修改，现在只用了X Y Z三个state，后面可以用一个矩阵保存所有state的信息，该矩阵的列数就是状态数
T_control = 0.001  # 初始化无人机api接口的执行周期
# 实例化各个函数类对象
Drone_takeoff = [airsim.MultirotorClient().takeoffAsync() for i in range(NumOfDrones)]  # 起飞实例化多个对象
Drone_moveToZ = [airsim.MultirotorClient().moveToZAsync(0, 0) for j in range(NumOfDrones)]  # 上升到指定高度实例化多个对象
Drone_move = [airsim.MultirotorClient().moveByVelocityAsync(0, 0, 0, 0) for k in range(NumOfDrones)]  # 移动实例化多个对象
# 后面其实可以用nx3的矩阵保存数据
X = np.array([[0.0], [-3.0], [-9.0], [-15.0], [-21.0], [-18.0], [-12.0], [-6.0]])  # 初始化坐标信息,其实可以设置为0，后面程序中会检测的。
Y = np.array([[0.0], [0.0], [0.0], [0.0], [0.0], [0.0], [0.0], [0.0]])
Z = np.array([[-5.0], [-4.5], [-4.0], [-6.0], [-5.0], [-4.5], [-4.0], [-6.0]])    # 代码优化后可以设置为初始飞行指定高度
deviation_X = np.array([[0.0], [-3.0], [-9.0], [-15.0], [-21.0], [-18.0], [-12.0], [-6.0]])  # 机体坐标系和全局坐标系之间的初始差距,
# 可以改成矩阵形式的
state = np.zeros((len(All_Drones), NumOfState))

model_X = np.array([[3.0], [3.0], [3.0], [3.0], [3.0], [3.0], [3.0], [3.0]])  # 初始化模型初始坐标,对应setting文件中设置好了的
model_Y = np.array([[0.0], [0.0], [0.0], [0.0], [0.0], [0.0], [0.0], [0.0]])
model_Z = np.array([[-5.0], [-5.0], [-5.0], [-5.0], [-5.0], [-5.0], [-5.0], [-5.0]])
model_state = np.zeros((1, NumOfState))

Vx_refer1 = 2*np.ones((8, 1))   # 生成数组默认为浮点型，运动方向朝向x正方向，记住后面程序有对速度进行限幅，速度不能超过3 m/s
Vy_refer1 = np.zeros((8, 1))
Vz_refer1 = np.zeros((8, 1))
V1_model = np.array([[2.0, 0.0, 0.0]])  # 写成1x3矩阵形式，分别代表x y z 的速度
Vx_refer2 = -2*np.ones((8, 1))   # 运动方向朝向x负方向，记住后面程序有对速度进行限幅，速度不能超过3 m/s
Vy_refer2 = np.zeros((8, 1))
Vz_refer2 = np.zeros((8, 1))
V2_model = np.array([[-2.0, 0.0, 0.0]])  # 写成1x3矩阵形式，分别代表x y z 的速度

Delta_X = np.array([[-3.0], [-6.0], [-9.0], [-12.0], [-15.0], [-12.0], [-9.0], [-6.0]])  # 初始化与模型的偏差信息
Delta_Y = np.array([[0.0], [-2.5], [-5.0], [-2.5], [0.0], [2.5], [5.0], [2.5]])
Delta_Z = np.array([[0.0], [0.0], [0.0], [0.0], [0.0], [0.0], [0.0], [0.0]])
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
gamma = 1  # 定义3.11算法的增益gamma初值
U_X = np.array([[0.0], [0.0], [0.0], [0.0], [0.0], [0.0], [0.0], [0.0]])  # 初始化控制参数
U_Y = np.array([[0.0], [0.0], [0.0], [0.0], [0.0], [0.0], [0.0], [0.0]])
U_Z = np.array([[0.0], [0.0], [0.0], [0.0], [0.0], [0.0], [0.0], [0.0]])

A = np.array([[0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0],  # 初始化 adjacency matrix A
              [1.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0],
              [1.0, 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0],
              [0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0],
              [0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0, 0.0],
              [0.0, 0.0, 1.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0],
              [0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0],
              [0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0],
              [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]])
D = np.array([[1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],  # 初始化度矩阵D
              [0.0, 2.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
              [0.0, 0.0, 3.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
              [0.0, 0.0, 0.0, 2.0, 0.0, 0.0, 0.0, 0.0, 0.0],
              [0.0, 0.0, 0.0, 0.0, 2.0, 0.0, 0.0, 0.0, 0.0],
              [0.0, 0.0, 0.0, 0.0, 0.0, 2.0, 0.0, 0.0, 0.0],
              [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0],
              [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0],
              [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]])

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

while 1:

    model_state[0][0] = client.getMultirotorState(vehicle_name="model").kinematics_estimated.position.x_val+3
    model_state[0][1] = client.getMultirotorState(vehicle_name="model").kinematics_estimated.position.y_val
    model_state[0][2] = client.getMultirotorState(vehicle_name="model").kinematics_estimated.position.z_val

    for i in range(NumOfDrones):  # 获取状态 deviation 是机体坐标和全局坐标的偏差
        state[i][0] = client.getMultirotorState(vehicle_name=All_Drones[i]).kinematics_estimated.position.x_val+deviation_X[i]
        state[i][1] = client.getMultirotorState(vehicle_name=All_Drones[i]).kinematics_estimated.position.y_val
        state[i][2] = client.getMultirotorState(vehicle_name=All_Drones[i]).kinematics_estimated.position.z_val
    for i in range(NumOfState):
        for j in range(NumOfDrones):
            add = 0  # 中间求和迭代值
            for k in range(NumOfDrones):
                add += A[j][k]*(U_control[k][i]-gamma*((state[j][i]-state[k][i])-(Delta[j][i]-Delta[k][i])))
            U_control[j][i] = 1/D[j][j]*(add+A[j][NumOfDrones] *
                                         (V1_model[0][i]-gamma*(state[j][i]-Delta[j][i]-model_state[0][i])))

    # U_X = np.dot(-(L+Q), X-Delta_X-model_X)+Vx_refer1
    # U_Y = np.dot(-(L+Q), Y-Delta_Y-model_Y)+Vy_refer1
    # U_Z = np.dot(-(L+Q), Z-Delta_Z-model_Z)+Vz_refer1
#    print("U_X:", U_X, "U_Z:", U_Y, "U_Z:", U_Z)  # 输出限幅前的控制输入
    for i in range(NumOfState):
        for j in range(NumOfDrones):  # x轴速度限制
            if U_control[j][i] >= 3:
                U_control[j][i] = 3
            elif U_control[j][i] <= -3:
                U_control[j][i] = -3
    print("model", " :  u_xyz:", V1_model[0], "XYZ:", model_state[0])
    for i in range(len(All_Drones)):  # 打印agent的控制输入信息、位置信息
        print(All_Drones[i], ":  u_xyz:", U_control[i], "XYZ:", state[i])
        # print(All_Drones[i], ":  u_xyz:", U_control[i][0], U_control[i][1], U_control[i][2],
        #       "XYZ:", state[i][0], state[i][1], state[i][2])
    model = client.moveByVelocityAsync(V1_model[0][0], V1_model[0][1], V1_model[0][2], T_control,
                                       vehicle_name="model")  # 速度控制
    for i in range(NumOfDrones):
        Drone_move[i] = client.moveByVelocityAsync(U_control[i][0], U_control[i][1], U_control[i][2], T_control,
                                                   vehicle_name=All_Drones[i])
    model.join()
    for i in range(NumOfDrones):
        Drone_move[i].join()

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
