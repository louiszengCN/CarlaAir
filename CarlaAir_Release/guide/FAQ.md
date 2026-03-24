# CarlaAir FAQ / 常见问题

---

## 连接问题 / Connection Issues

### Q: CARLA Client 连接超时
**A**: 首次启动需要 2-5 分钟加载资源（首次可能需要约 10 分钟编译着色器）。

```bash
# 检查端口是否就绪
ss -tlnp | grep 2000
```

### Q: AirSim 无法连接（端口 41451）
**A**: AirSim 服务器在 CARLA 之后启动，需要额外等待。确认 `~/Documents/AirSim/settings.json` 存在。

### Q: 两个 API 可以同时使用吗？
**A**: 是的，这正是 CarlaAir 的核心功能。CARLA API (port 2000) 和 AirSim API (port 41451) 同时运行，操控同一个世界。

---

## 同步 / 异步模式 / Sync & Async Mode

### Q: 什么时候用同步模式，什么时候用异步模式？
**A**:
- **异步模式（默认）**: 适合交互式操作 — 键盘驾驶车辆、手动操控无人机、实时探索
- **同步模式**: 适合数据采集、演示回放、视频录制 — 需要固定时间步和帧帧一致

### Q: 如何开启同步模式？
**A**: 需要同时配置 World 和 Traffic Manager：

```python
settings = world.get_settings()
settings.synchronous_mode = True
settings.fixed_delta_seconds = 1.0 / 30
world.apply_settings(settings)

tm = client.get_trafficmanager(8000)
tm.set_synchronous_mode(True)

# 主循环中每帧调用
world.tick()
```

### Q: 同步模式下仿真器冻结了
**A**: 脚本退出时**必须恢复异步模式**，否则仿真器会一直等待 `world.tick()` 而冻结：

```python
settings = world.get_settings()
settings.synchronous_mode = False
settings.fixed_delta_seconds = None
world.apply_settings(settings)
```

建议在脚本中使用 `try/finally` 结构确保退出时始终恢复。

---

## 坐标系统 / Coordinate Systems

### Q: AirSim 和 CARLA 的坐标系统有什么区别？
**A**: 两套 API 使用不同的坐标约定：

| | CARLA | AirSim (NED) |
|---|---|---|
| X | 向前 (East) | 向前 (North) |
| Y | 向右 | 向右 (East) |
| Z | **向上为正** | **向下为正** |

关键换算：AirSim NED z = -CARLA z。
例如 CARLA 中 30 米高空对应 AirSim NED z = -30。

### Q: 无人机位置和 CARLA 车辆位置对不上
**A**: 两套 API 的世界原点不同，不能直接用一个 API 的坐标去另一个 API 中定位。详细换算方法请参考 `examples_record_demo/COORDINATE_SYSTEMS.md`。

---

## 车辆和行人 / Vehicles & Pedestrians

### Q: 车辆行驶异常/抽搐/路径不稳
**A**: 使用 Traffic Manager 的 `hybrid_physics_mode` 可以改善远距离车辆的物理模拟稳定性：

```python
tm = client.get_trafficmanager(8000)
tm.set_hybrid_physics_mode(True)
tm.set_hybrid_physics_radius(50.0)  # 50 米内使用完整物理
```

### Q: Walker 使用 `go_to_location()` 崩溃
**A**: 已知限制。Walker AI Controller 在 CarlaAir 中会触发 segfault。**只使用静态 Walker**（不调用 `go_to_location()`）。

### Q: 多少车辆才安全？
**A**:
- 纯 CARLA（无无人机）：50+ 辆安全
- AirSim 无人机激活时：建议 ≤ 8 辆 autopilot
- 使用 `try_spawn_actor()` 而非 `spawn_actor()` 避免碰撞点生成失败

---

## 无人机 / Drone

### Q: 无人机不响应指令
**A**: 确保调用了正确的初始化序列：

```python
client = airsim.MultirotorClient(port=41451)
client.confirmConnection()
client.enableApiControl(True)
client.armDisarm(True)
client.takeoffAsync().join()
```

### Q: AirSim 客户端连接后干扰了无人机控制
**A**: 如果你在交互飞行的同时另一个脚本也连接了 AirSim，可能导致控制冲突。录制无人机轨迹时请使用专用脚本 `examples_record_demo/record_drone.py`，它内部处理了控制权管理。

### Q: `simSetCameraPose` 导致崩溃
**A**: 在 Shipping 构建中 `simSetCameraPose` 会触发 C++ abort。**不要调用此函数**。使用无人机自身的 yaw + 轨道飞行来改变视角。

---

## 天气和地图 / Weather & Maps

### Q: 如何切换地图？
**A**: 启动时指定地图名称：

```bash
./CarlaAir.sh Town03
```

可用地图：Town01, Town02, Town03, Town04, Town05, Town10HD。

### Q: 天气变化会影响无人机吗？
**A**: 是的，天气是统一的。CARLA 的 `set_weather()` 会同时影响地面和空中视角（雨、雾、光照等）。

---

## 性能 / Performance

### Q: 帧率太低
**A**:
- 使用较小的地图（Town01 比 Town10HD 轻量得多）
- 减少生成的 Actor 数量
- 降低窗口分辨率
- 使用 `-quality-level=Low` 参数

### Q: GPU 显存不足
**A**: Town10HD 需要约 8GB 显存。尝试 Town01/Town02 等小地图，或使用 `_Opt` 变体地图（分层加载，减少显存占用）。
