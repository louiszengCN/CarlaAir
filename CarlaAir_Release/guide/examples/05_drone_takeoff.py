#!/usr/bin/env python3
"""CarlaAir Quick Start - Step 5: 无人机起飞与飞行"""

import airsim

client = airsim.MultirotorClient(port=41451)
client.confirmConnection()
client.enableApiControl(True)
client.armDisarm(True)
print("无人机解锁，准备起飞...")

# 起飞
client.takeoffAsync().join()
print("已起飞!")

# 飞到城市中心 (NED: x=80内陆, y=0, z=-30即30m高)
print("飞往城市中心...")
client.moveToPositionAsync(80, 0, -30, 5).join()

state = client.getMultirotorState()
pos = state.kinematics_estimated.position
print(f"当前位置 NED: ({pos.x_val:.1f}, {pos.y_val:.1f}, {pos.z_val:.1f})")
print(f"海拔高度: {abs(pos.z_val):.1f}m")

# 正方形航线
print("执行正方形航线...")
for i, (x, y, z) in enumerate([(90,0,-30), (90,10,-30), (80,10,-30), (80,0,-30)]):
    print(f"  航点 {i+1}/4: ({x}, {y}, {z})")
    client.moveToPositionAsync(x, y, z, 5).join()

# 降落
print("降落中...")
client.landAsync().join()
client.armDisarm(False)
client.enableApiControl(False)
print("降落完成!")
