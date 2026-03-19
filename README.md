<p align="center">
  <img src="logo_upload.png" alt="CarlaAir Logo" width="180"/>
</p>

<p align="center">
  <img src="teaser_upload.png" alt="CarlaAir Teaser" width="100%"/>
</p>

<h3 align="center">CarlaAir: A Unified Air-Ground Co-Simulation Platform</h3>

<p align="center">
  <b>Integrating CARLA 0.9.16 and AirSim into a single Unreal Engine 4 process.</b>
  <br>
  <i>Developed by Southern University of Science and Technology (SUSTech)</i>
</p>

<p align="center">
  <a href="README.md">English</a> | <a href="README_CN.md">简体中文</a>
</p>

---

**CarlaAir** is an open-source, in-process integration of the world's leading autonomous driving simulator (CARLA) and robotics simulator (AirSim). By merging both into a single `ASimWorldGameMode`, it provides frame-level sensor synchronization, unified physics, and dual Python APIs for seamless air-ground cooperative research.

## ✨ Highlights

- 🚀 **Single-Process Integration**: No bridge, no latency. CARLA and AirSim share the same UE4 world, weather, and physics engine.
- 🎯 **Absolute Coordinate Alignment**: Exact `0.0000m` error between CARLA (Left-handed) and AirSim (NED) coordinate systems.
- 🚁 **Built-in FPS Drone Control**: Fly the drone directly in the viewport using `WASD` + Mouse, without writing any Python scripts.
- 🚦 **Realistic Urban Traffic**: Auto-spawns 30 vehicles and 50 pedestrians on startup.
- 📸 **18-Channel Synchronized Sensors**: Simultaneous data collection from RGB, LiDAR, Depth, Semantic Segmentation, IMU, and GNSS across both ground and aerial agents.
- 🐍 **Dual API Support**: Use `carla.Client` (port 2000) and `airsim.MultirotorClient` (port 41451) in the exact same script.

---

## 🎮 Quick Start

### Option A: Binary Release (Recommended)

```bash
# 1. Download and extract CarlaAir-v0.1.6
tar xzf CarlaAir-v0.1.6.tar.gz
cd CarlaAir-v0.1.6

# 2. Launch the simulator (auto-spawns traffic)
./CarlaAir.sh Town10HD

# 3. In another terminal, test the Dual API
python3 -c "import carla; c=carla.Client('localhost',2000); print(c.get_world().get_map().name)"
python3 -c "import airsim; c=airsim.MultirotorClient(port=41451); c.confirmConnection()"
```

### Option B: Build from Source

Please refer to the [Build Guide](CarlaAir_Release/source/BUILD_GUIDE.md) for detailed instructions on compiling CarlaAir with UE4.26.

---

## ⌨️ Flight Controls

When the simulator is running, click inside the window to capture the mouse and use the built-in FPS controller:

| Key | Action |
|-----|--------|
| `W` / `A` / `S` / `D` | Move Forward / Left / Backward / Right |
| `Space` / `Shift` | Ascend / Descend |
| `Mouse` | Yaw (Turn Left/Right) |
| `Scroll Wheel` | Adjust Flight Speed |
| `N` | Cycle Weather Presets (Clear, Rain, Fog, Night, etc.) |
| `P` | Toggle Collision Mode (Physics vs. Noclip/Invincible) |
| `H` | Show/Hide On-Screen Help Menu |
| `Tab` | Release / Capture Mouse |

---

## 📚 Documentation & Examples

We provide **24 ready-to-run Python examples** covering various air-ground cooperative scenarios.

- `demo_drive_and_fly.py`: Simultaneous ground vehicle + drone control
- `drone_car_chase.py`: Drone tracking a moving ground vehicle
- `data_collector.py`: Multi-sensor synchronized data collection
- `aerial_surveillance.py`: Drone surveillance with camera capture

**Full Documentation:**
- [Quick Start Guide](CarlaAir_Release/guide/Quick-Start.md)
- [Architecture Details](CarlaAir_Release/source/ARCHITECTURE.md)
- [Modifications from Upstream](CarlaAir_Release/source/MODIFICATIONS.md)

---

## 📜 License & Acknowledgments

CarlaAir is built upon the shoulders of giants. We sincerely thank the developers of:
- [CARLA Simulator](https://github.com/carla-simulator/carla) (MIT License)
- [Microsoft AirSim](https://github.com/microsoft/AirSim) (MIT License)
- [Unreal Engine](https://www.unrealengine.com/)

CarlaAir specific code is distributed under the **MIT License**. CARLA specific assets are distributed under the CC-BY License.
