#!/usr/bin/env python3
"""CarlaAir Quick Start - Step 6: 无人机多模态传感器"""
import airsim
import numpy as np
import cv2

client = airsim.MultirotorClient(port=41451)
client.confirmConnection()
client.enableApiControl(True)
client.armDisarm(True)

client.takeoffAsync().join()
client.moveToPositionAsync(80, 0, -25, 5).join()
print("无人机已到达城市中心上空")

# 采集 RGB + 深度 + 语义分割
print("采集多模态图像...")
responses = client.simGetImages([
    airsim.ImageRequest("0", airsim.ImageType.Scene, False, False),
    airsim.ImageRequest("0", airsim.ImageType.DepthPerspective, True, False),
    airsim.ImageRequest("0", airsim.ImageType.Segmentation, False, False),
])

if responses[0].height > 0:
    img = np.frombuffer(responses[0].image_data_uint8, dtype=np.uint8)
    cv2.imwrite('/tmp/drone_rgb.png', img.reshape(responses[0].height, responses[0].width, 3))
    print(f"  RGB: {responses[0].width}x{responses[0].height}")

if responses[1].height > 0:
    depth = airsim.list_to_2d_float_array(responses[1].image_data_float, responses[1].width, responses[1].height)
    depth_color = cv2.applyColorMap(np.clip(depth/100*255, 0, 255).astype(np.uint8), cv2.COLORMAP_JET)
    cv2.imwrite('/tmp/drone_depth.png', depth_color)
    print(f"  Depth: range [{depth.min():.1f}, {depth.max():.1f}]m")

if responses[2].height > 0:
    img = np.frombuffer(responses[2].image_data_uint8, dtype=np.uint8)
    cv2.imwrite('/tmp/drone_seg.png', img.reshape(responses[2].height, responses[2].width, 3))
    print(f"  Segmentation: OK")

client.landAsync().join()
client.armDisarm(False)
client.enableApiControl(False)
print(f"图像保存在 /tmp/drone_*.png")
