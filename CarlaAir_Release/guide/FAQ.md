# CarlaAir 常见问题 (FAQ)

## 连接问题

### Q: CARLA Client 连接超时
**A**: 首次启动需要 2-5 分钟加载资源（首次约 10 分钟编译 DDC/着色器）。

```bash
# 检查端口是否就绪
ss -tlnp | grep 2000
```

### Q: AirSim 无法连接（端口 41451）
**A**: AirSim 服务器在 CARLA 之后启动，需要额外等待。确认 `~/Documents/AirSim/settings.json` 存在。

### Q: 两个 API 可以同时使用吗？
**A**: 是的，这正是 CarlaAir 的核心功能。CARLA API (port 2000) 和 AirSim API (port 41451) 同时运行，操控同一个世界。

## 车辆和行人

### Q: 车辆穿过地面/掉下去
**A**: 这个 Bug 已在 v0.1 中修复（移除了 `SetNewWorldOrigin()` 调用）。确保使用最新构建。

### Q: Walker 使用 `go_to_location()` 崩溃
**A**: 已知限制。Walker AI Controller 在 CarlaAir 中会触发 segfault。**只使用静态 Walker**（不调用 `go_to_location()`）。

### Q: 多少车辆才安全？
**A**:
- 纯 CARLA（无无人机）：50+ 辆安全
- AirSim 无人机激活时：建议 ≤8 辆 autopilot
- 使用 `try_spawn_actor()` 而非 `spawn_actor()` 避免碰撞失败

## 无人机

### Q: 无人机不响应指令
**A**: 确保调用了正确的初始化序列：
```python
client.enableApiControl(True)
client.armDisarm(True)
client.takeoffAsync().join()
```

### Q: 坐标系统是什么？
**A**: AirSim 使用 NED（North-East-Down）坐标系：
- NED x = CARLA x（向内陆）
- NED y = CARLA y
- NED z = -CARLA z（NED 下为正，CARLA 上为正）
- 城市中心: NED (80, 0, -30) = CARLA (80, 0, 30m高)

### Q: `simSetCameraPose` 导致崩溃
**A**: 在 Shipping 构建中 `simSetCameraPose` 会触发 C++ abort。**不要调用此函数**。使用无人机自身的 yaw + 轨道飞行来改变视角。

## 天气和地图

### Q: 如何切换地图？
**A**: 启动时指定：`./carla_air.sh Town03`。可用地图：Town01-05, Town10HD。

### Q: 天气变化会影响无人机吗？
**A**: 是的，天气是统一的。CARLA 的 `set_weather()` 会同时影响地面和空中视角（雨、雾、光照等）。

## 性能

### Q: 帧率太低
**A**:
- 降低分辨率：`./carla_air.sh Town10HD 1280 720`
- 使用 Low quality：编辑脚本中的 `quality-level`
- 减少生成的 Actor 数量
- 使用更小的地图（Town01 比 Town10HD 轻量）

### Q: GPU 显存不足
**A**: Town10HD 需要约 8GB 显存。尝试 `-quality-level=Low` 或使用 Town01/Town02。
