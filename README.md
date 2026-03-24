<table>
  <tr>
    <td width="160" valign="middle" align="center">
      <img src="logo_upload.png" alt="CarlaAir Logo" width="140"/>
    </td>
    <td valign="middle" align="left">
      <h1>CarlaAir</h1>
      <p><b>Fly drones inside a CARLA world.</b><br/>
      A unified simulation platform that brings CARLA and AirSim together —<br/>
      ground vehicles and aerial drones, one world, one script.</p>
      <p>
        <a href="https://github.com/louiszengCN/CarlaAir/releases/tag/v0.1.7"><img src="https://img.shields.io/badge/version-v0.1.7-blue" alt="Version"/></a>
        <img src="https://img.shields.io/badge/License-MIT-yellow.svg" alt="License: MIT"/>
        <img src="https://img.shields.io/badge/python-3.8+-blue" alt="Python 3.8+"/>
        <img src="https://img.shields.io/badge/CARLA-0.9.16-green" alt="CARLA 0.9.16"/>
        <img src="https://img.shields.io/badge/AirSim-1.8.1-orange" alt="AirSim 1.8.1"/>
        <img src="https://img.shields.io/badge/platform-Ubuntu%2020.04%20%7C%2022.04-lightgrey" alt="Platform"/>
        <img src="https://img.shields.io/badge/arXiv-coming%20soon-b31b1b" alt="arXiv"/>
      </p>
      <p>
        <a href="README.md">English</a> | <a href="README_CN.md">简体中文</a>
      </p>
    </td>
  </tr>
</table>

<p align="center">
  <a href="https://github.com/louiszengCN/CarlaAir/releases/tag/v0.1.7">
    <img src="teaser_upload.gif" alt="CarlaAir Teaser — click to watch the full demo with Hans Zimmer soundtrack" width="100%"/>
  </a>
</p>

<p align="center">
  <i>Click the GIF or </i><a href="https://github.com/louiszengCN/CarlaAir/releases/tag/v0.1.7"><b>watch our 30s demo video with Hans Zimmer soundtrack</b></a> 🎵
</p>

---

**CarlaAir** is an open-source, in-process integration of the world's leading autonomous driving simulator (CARLA) and robotics simulator (AirSim). By merging both into a single `ASimWorldGameMode`, it provides frame-level sensor synchronization, unified physics, and dual Python APIs for seamless air-ground cooperative research.

## 🔥 News

- **[2026-03]** `v0.1.7` released — VSync fix, stable traffic, one-click env setup, drone recording toolkit, coordinate docs
- **[2026-03]** `v0.1.6` released — Auto traffic spawn, UE4 native Sweep collision, ground clamping
- **[2026-03]** `v0.1.5` released — 12-direction collision system, bilingual help overlay (`H`)
- **[2026-03]** `v0.1.4` released — ROS2 validation (63 topics), first official binary release

---

## ✨ Highlights

| | |
|---|---|
| 🚀 **Single-Process Integration** | No bridge, no latency. CARLA and AirSim share the same UE4 world, weather, and physics engine. |
| 🎯 **Absolute Coordinate Alignment** | Exact `0.0000m` error between CARLA (Left-handed) and AirSim (NED) coordinate systems. |
| 🚁 **Built-in FPS Drone Control** | Fly the drone in the viewport using `WASD` + Mouse — no Python scripts needed. |
| 🚦 **Realistic Urban Traffic** | Auto-spawns 30 vehicles and 50 pedestrians on startup across 13 maps. |
| 📸 **18-Channel Synchronized Sensors** | RGB · LiDAR · Depth · Semantic Seg · IMU · GNSS — all frame-aligned across air and ground. |
| 🐍 **Dual Python API** | `carla.Client` (port 2000) + `airsim.MultirotorClient` (port 41451) in one script. |
| ✅ **89/89 API Tests Passed** | Full compatibility verified across both upstream APIs. |

---

## 🎮 Quick Start

### Option A: Binary Release (Recommended)

```bash
# 1. Download and extract CarlaAir-v0.1.7
tar xzf CarlaAir-v0.1.7.tar.gz
cd CarlaAir-v0.1.7

# 2. One-click environment setup (first time only)
bash env_setup/setup_env.sh      # creates conda env, installs deps, deploys carla module
conda activate carlaAir
bash env_setup/test_env.sh        # verify: should show all PASS

# 3. Launch the simulator (auto-spawns traffic)
./CarlaAir.sh Town10HD

# 4. Run the showcase! (in another terminal)
conda activate carlaAir
python3 examples/quick_start_showcase.py
```

> **What you'll see:** A Tesla cruises through the city while a drone chases it from above. A 4-panel display shows **RGB · Depth · Semantic Segmentation · LiDAR BEV** — all synchronized. Weather cycles automatically.

### Option B: Build from Source

Please refer to the [Build Guide](CarlaAir_Release/source/BUILD_GUIDE.md) for detailed instructions on compiling CarlaAir with UE4.26.

---

## 🐍 One Script, Two Worlds

Both APIs share the **same simulated world** — no bridge, no sync headaches.

```python
import carla, airsim

carla_client = carla.Client("localhost", 2000)
air_client   = airsim.MultirotorClient(port=41451)
world = carla_client.get_world()

# One weather call affects EVERY sensor — ground AND air
world.set_weather(carla.WeatherParameters.HardRainSunset)

# Spawn a car, let it drive
vehicle = world.spawn_actor(vehicle_bp, spawn_point)
vehicle.set_autopilot(True)

# Fly the drone above — same world, same rain, same physics
air_client.takeoffAsync().join()
air_client.moveToPositionAsync(80, 30, -25, 5)
```

**6 demo scripts** — try them all:

```bash
python3 examples/quick_start_showcase.py   # 🎬 4-panel sensors + drone chase + weather cycling
python3 examples/drive_vehicle.py          # 🚗 Drive a Tesla with WASD
python3 examples/walk_pedestrian.py        # 🚶 Walk the city on foot (mouse look)
python3 examples/switch_maps.py            # 🗺️  Fly through all 13 maps automatically
python3 examples/sensor_gallery.py         # 📸 6-grid sensor showcase on one vehicle
python3 examples/air_ground_sync.py        # 🔄 Car + drone split-screen: same rain, same world
```

**Recording toolkit** — record trajectories for vehicle, drone, and pedestrian, then replay them with a director camera:

```bash
python3 examples/recording/record_vehicle.py     # 🚗 Drive & record vehicle trajectory
python3 examples/recording/record_drone.py       # 🚁 Fly & record drone trajectory (zero intrusion)
python3 examples/recording/record_walker.py      # 🚶 Walk & record pedestrian trajectory
python3 examples/recording/demo_director.py \    # 🎬 Replay all + free camera + MP4 recording
    trajectories/vehicle_*.json trajectories/drone_*.json
```

> **Docs:** [Coordinate Systems (CARLA↔AirSim)](CarlaAir_Release/guide/COORDINATE_SYSTEMS.md) · [Quick Start Guide](CarlaAir_Release/guide/Quick-Start.md) · [FAQ](CarlaAir_Release/guide/FAQ.md)

---

## ⌨️ Flight Controls

When the simulator is running, click inside the window to capture the mouse:

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

We provide **6 curated Python examples** showcasing the core air-ground cooperative capabilities:

| Example | Description |
|---------|-------------|
| `demo_drive_and_fly.py` | Simultaneous ground vehicle + drone control |
| `drone_car_chase.py` | Drone tracking a moving ground vehicle |
| `aerial_surveillance.py` | Drone surveillance with camera capture |
| `data_collector.py` | Multi-sensor synchronized data collection |
| `city_tour.py` | Air-ground dual-perspective city tour |
| `fly_drone_keyboard.py` | Interactive keyboard drone flight |

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
