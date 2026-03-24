# CarlaAir 坐标系换算说明

## 问题

CarlaAir 内部同时运行 CARLA 和 AirSim 两套系统，**它们使用不同的坐标系**：

| | CARLA | AirSim |
|---|---|---|
| 坐标原点 | UE4 世界原点 | AirSim PlayerStart 位置 |
| X 轴 | 同 UE4 | 同 UE4，但有原点偏移 |
| Y 轴 | 同 UE4 | 同 UE4，但有原点偏移 |
| Z 轴 | **向上为正** (z-up) | **向下为正** (NED, z-down) |
| 单位 | 厘米 (UE4) → Python API 返回**米** | 米 |

## 换算公式

在 Town10HD 地图上实测（CarlaAir v0.1.6）：

```
airsim_x =  carla_x + 172.20
airsim_y =  carla_y - 183.86
airsim_z = -carla_z +  27.45
```

姿态角（pitch/yaw/roll）两侧一致，**不需要换算**。

## 换算参数表

| 参数 | 值 | 说明 |
|------|-----|------|
| `offset_x` | **+172.20** | AirSim 原点在 CARLA X 方向的偏移 |
| `offset_y` | **-183.86** | AirSim 原点在 CARLA Y 方向的偏移 |
| `offset_z` | **+27.45** | AirSim 原点在 CARLA Z 方向的偏移（取反后加） |
| z 轴翻转 | `airsim_z = -carla_z + offset_z` | CARLA z-up → AirSim NED z-down |

> **注意：** 偏移量取决于地图和 AirSim PlayerStart 配置。换地图后需要重新标定。

## 如何标定

在两个系统中同时读取无人机位置即可算出偏移量：

```python
import carla, airsim

# CARLA 侧
client = carla.Client('localhost', 2000)
world = client.get_world()
drone = [a for a in world.get_actors() if 'drone' in a.type_id][0]
cl = drone.get_location()

# AirSim 侧
ac = airsim.MultirotorClient()
ac.confirmConnection()
ap = ac.getMultirotorState().kinematics_estimated.position

# 偏移量
offset_x = ap.x_val - cl.x
offset_y = ap.y_val - cl.y
offset_z = ap.z_val - (-cl.z)
print(f"offset_x={offset_x:.4f}, offset_y={offset_y:.4f}, offset_z={offset_z:.4f}")
```

## 脚本中的处理

| 脚本 | 录制坐标系 | 回放方式 |
|------|-----------|---------|
| `record_drone.py` | **CARLA** (`transform` 字段) | — |
| `record_drone_airsim_only.py` | **AirSim NED** (`airsim_ned` 字段) | — |
| `record_vehicle.py` | **CARLA** (`transform` 字段) | CARLA `set_transform` |
| `record_walker.py` | **CARLA** (`transform` 字段) | CARLA `set_transform` |
| `demo_director.py` 车辆/行人 | — | CARLA `set_transform`（直接用） |
| `demo_director.py` 无人机 | — | AirSim `simSetVehiclePose`（自动校准偏移） |
| `replay_drone_airsim_only.py` | — | AirSim `simSetVehiclePose`（需 `airsim_ned`） |

### demo_director 自动校准

`DroneReplayer` 初始化时：
1. 检测 JSON 中是否有 `airsim_ned` 字段
2. 如果只有 `transform`（CARLA 坐标）→ 读取无人机在 CARLA 和 AirSim 两侧的当前位置，计算偏移量
3. 回放时每帧 `CARLA transform + 偏移 → AirSim NED → simSetVehiclePose`

如果 JSON 中有 `airsim_ned` → 直接使用，不做转换。

## 为什么不直接用 AirSim API 录制？

连接 AirSim Python 客户端会导致 CarlaAir 窗口中的无人机**丢失键盘控制并坠落**。
因此 `record_drone.py` 只从 CARLA 侧读取 `drone_actor.get_transform()`，完全不碰 AirSim。
