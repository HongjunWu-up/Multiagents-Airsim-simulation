"""
test python environment
"""
import airsim

# connect to the AirSim simulator
client = airsim.MultirotorClient() #与sirsim建立联系
client.confirmConnection() #确认联系建立成功

# get control
client.enableApiControl(True) #获得API控制权，确保不会被遥控器抢占。使用 isApiControlEnabled 可以检查 API 是否具有控制权。

# unlock
client.armDisarm(True) #与现实结合，考虑到解锁上锁的操作

# Async methods returns Future. Call join() to wait for task to complete.
client.takeoffAsync().join() #起飞，。join等待完成
client.landAsync().join() #降落

# lock
client.armDisarm(False)

# release control
client.enableApiControl(False) #释放API控制权