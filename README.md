# CARLA-Air: Fly Drones Inside a CARLA World — A Unified Infrastructure for Air-Ground Embodied Intelligence

<p align="center">
  <a href="https://youtu.be/a0fZG2dmT1Q">
    <img src="docs/images/teaser_video.gif" alt="CARLA-Air Teaser — click to watch the full demo" width="100%"/>
  </a>
</p>

**CARLA-Air** is an open-source infrastructure that unifies high-fidelity urban driving and physics-accurate multirotor flight within a single Unreal Engine process, providing a practical simulation foundation for air-ground embodied intelligence research.

<div align="center">
  <a href="report/CarlaAir_arxiv_version.pdf"><img src="https://img.shields.io/badge/Paper-PDF-red" alt="Paper PDF"/></a>
  <a href="https://arxiv.org/abs/XXXX.XXXXX"><img src="https://img.shields.io/badge/arXiv-XXXX.XXXXX-b31b1b?logo=arxiv&logoColor=white" alt="arXiv"/></a>
  <a href="https://github.com/louiszengCN/CarlaAir/stargazers"><img src="https://img.shields.io/github/stars/louiszengCN/CarlaAir?style=social" alt="GitHub Stars"/></a>
  <a href="https://github.com/louiszengCN/CarlaAir/releases/tag/v0.1.7"><img src="https://img.shields.io/badge/version-v0.1.7-blue" alt="Version"/></a>
  <img src="https://img.shields.io/badge/License-MIT-yellow.svg" alt="License: MIT"/>
  <img src="https://img.shields.io/badge/python-3.8+-blue" alt="Python 3.8+"/>
  <img src="https://img.shields.io/badge/CARLA-0.9.16-green" alt="CARLA 0.9.16"/>
  <img src="https://img.shields.io/badge/AirSim-1.8.1-orange" alt="AirSim 1.8.1"/>
  <img src="https://img.shields.io/badge/platform-Ubuntu%2020.04%20%7C%2022.04-lightgrey" alt="Platform"/>
</div>

<br>

<p align="center">
  <a href="README.md">English</a> | <a href="README_CN.md">简体中文</a> &nbsp;&nbsp;|&nbsp;&nbsp;
  📄 <a href="report/CarlaAir_arxiv_version.pdf"><b>Paper</b></a> &nbsp;|&nbsp;
  🌐 <a href="#"><b>Project Page</b></a> <i>(coming soon)</i> &nbsp;|&nbsp;
  📖 <a href="CarlaAir_Release/guide/Quick-Start.md"><b>Docs</b></a> &nbsp;|&nbsp;
  🎬 <a href="https://youtu.be/a0fZG2dmT1Q"><b>Video</b></a> &nbsp;|&nbsp;
  💻 <a href="https://github.com/louiszengCN/CarlaAir/releases/tag/v0.1.7"><b>Binary Release</b></a>
</p>

## 📌 Table of Contents

- [🔥 News](#news)
- [✨ Highlights](#highlights)
- [🏆 Platform Comparison](#platform-comparison) — 15 simulators, side-by-side
- [🎮 Quick Start](#quick-start) — up and running in 4 steps
- [🐍 One Script, Two Worlds](#one-script-two-worlds) — dual API code examples
- [🔬 Research Directions & Workflows](#research-directions--workflows) — W1–W5 validated workflows
- [⌨️ Flight Controls](#flight-controls)
- [📚 Documentation & Tutorials](#documentation--tutorials) — 8 step-by-step tutorials
- [🗺️ Roadmap](#roadmap)
- [📝 Citation](#citation)
- [📜 License & Acknowledgments](#license--acknowledgments)
- [⭐ Star History](#star-history)

<a id="news"></a>

## 🔥 News

- **[2026-03-31]** 🚀 Coming soon -- Project page, tutorial docs, and ready-to-use binary release. Stay tuned!
- **[2026-03-30]** 📄 Technical report released -- [Read the paper](report/CarlaAir_arxiv_version.pdf)
- **[2026-03]** `v0.1.7` released -- VSync fix, stable traffic, one-click env setup, drone recording toolkit, coordinate docs
- **[2026-03]** `v0.1.6` released -- Auto traffic spawn, UE4 native Sweep collision, ground clamping
- **[2026-03]** `v0.1.5` released -- 12-direction collision system, bilingual help overlay (`H`)
- **[2026-03]** `v0.1.4` released -- ROS2 validation (63 topics), first official binary release

---

<a id="highlights"></a>

## ✨ Highlights

| | |
|---|---|
| 🏗️ **Single-Process Composition** | `CARLAAirGameMode` inherits CARLA and composes AirSim. Only 3 upstream files modified (~35 lines). No bridge, no latency. |
| 🎯 **Absolute Coordinate Alignment** | Exact `0.0000 m` error between CARLA (left-handed) and AirSim (NED) coordinate frames. |
| 📸 **Up to 18 Sensor Modalities** | RGB, Depth, Semantic Seg, Instance Seg, LiDAR, Radar, Surface Normals, IMU, GNSS, Barometry -- all frame-aligned across air and ground. |
| 🔄 **Zero-Modification Code Migration** | Existing CARLA and AirSim Python scripts and ROS 2 nodes run on CARLA-Air without any code changes. 89/89 CARLA API tests passing. |
| ⚡ **~20 FPS Joint Workloads** | Moderate joint configuration (vehicles + drone + 8 sensors) sustains 19.8 +/- 1.1 FPS. Communication overhead < 0.5 ms (vs. 1--5 ms bridge co-sim). |
| 🛡️ **3-Hour Stability Verified** | 357 spawn/destroy cycles, zero crashes, zero memory accumulation (R² = 0.11). |
| 🚁 **Built-in FPS Drone Control** | Fly the drone in viewport using WASD + Mouse -- no Python scripts needed. |
| 🚦 **Realistic Urban Traffic** | Rule-compliant traffic flow, socially-aware pedestrians, 13 urban maps. |
| 🧩 **Extensible Asset Pipeline** | Import custom robot platforms, UAV configurations, vehicles, and environment maps. |

<p align="center">
  <img src="docs/images/teaser_final.jpg" alt="CARLA-Air architecture overview" width="95%"/>
</p>

---

<a id="platform-comparison"></a>

## 🏆 Platform Comparison

A comprehensive comparison of CARLA-Air against 14 existing simulation platforms (based on Table 1 from the [technical report](report/CarlaAir_arxiv_version.pdf)).

<table>
  <thead>
    <tr>
      <th>Category</th>
      <th>Platform</th>
      <th>Urban Traffic</th>
      <th>Pedestrians</th>
      <th>UAV Flight</th>
      <th>Single Process</th>
      <th>Shared Renderer</th>
      <th>Native APIs</th>
      <th>Joint Sensors</th>
      <th>Prebuilt Binary</th>
      <th>Test Suite</th>
      <th>Custom Assets</th>
      <th>Open Source</th>
    </tr>
  </thead>
  <tbody>
    <tr>
      <td rowspan="5"><i>Autonomous Driving</i></td>
      <td>CARLA</td>
      <td>✓</td><td>✓</td><td>✗</td><td>✓</td><td>✓</td><td>✓</td><td>✗</td><td>✓</td><td>✗</td><td>✓</td><td>✓</td>
    </tr>
    <tr>
      <td>LGSVL</td>
      <td>✓</td><td>✓</td><td>✗</td><td>✓</td><td>✓</td><td>✓</td><td>✗</td><td>✓</td><td>✗</td><td>~</td><td>✓</td>
    </tr>
    <tr>
      <td>SUMO</td>
      <td>✓</td><td>✓</td><td>✗</td><td>✓</td><td>✗</td><td>✓</td><td>✗</td><td>✓</td><td>✗</td><td>✗</td><td>✓</td>
    </tr>
    <tr>
      <td>MetaDrive</td>
      <td>✓</td><td>~</td><td>✗</td><td>✓</td><td>~</td><td>✓</td><td>✗</td><td>✗</td><td>✗</td><td>✗</td><td>✓</td>
    </tr>
    <tr>
      <td>VISTA</td>
      <td>~</td><td>✗</td><td>✗</td><td>✓</td><td>~</td><td>✓</td><td>✗</td><td>✗</td><td>✗</td><td>✗</td><td>✓</td>
    </tr>
    <tr>
      <td rowspan="6"><i>Aerial / UAV</i></td>
      <td>AirSim</td>
      <td>✗</td><td>✗</td><td>✓</td><td>✓</td><td>✓</td><td>✓</td><td>✗</td><td>✓</td><td>✗</td><td>✓</td><td>✓</td>
    </tr>
    <tr>
      <td>Flightmare</td>
      <td>✗</td><td>✗</td><td>✓</td><td>✓</td><td>✓</td><td>✓</td><td>✗</td><td>✗</td><td>✗</td><td>~</td><td>✓</td>
    </tr>
    <tr>
      <td>FlightGoggles</td>
      <td>✗</td><td>✗</td><td>✓</td><td>✓</td><td>✓</td><td>✓</td><td>✗</td><td>✗</td><td>✗</td><td>~</td><td>✓</td>
    </tr>
    <tr>
      <td>Gazebo/RotorS</td>
      <td>~</td><td>~</td><td>✓</td><td>✓</td><td>~</td><td>✓</td><td>✗</td><td>✓</td><td>✗</td><td>✓</td><td>✓</td>
    </tr>
    <tr>
      <td>OmniDrones</td>
      <td>✗</td><td>✗</td><td>✓</td><td>✓</td><td>✓</td><td>✓</td><td>✗</td><td>✗</td><td>✗</td><td>✓</td><td>✓</td>
    </tr>
    <tr>
      <td>gym-pybullet-drones</td>
      <td>✗</td><td>✗</td><td>✓</td><td>✓</td><td>~</td><td>✓</td><td>✗</td><td>✗</td><td>✗</td><td>~</td><td>✓</td>
    </tr>
    <tr>
      <td rowspan="3"><i>Joint / Co-Sim</i></td>
      <td>TranSimHub</td>
      <td>✓</td><td>✓</td><td>✓</td><td>✗</td><td>✗</td><td>✗</td><td>~</td><td>—</td><td>✗</td><td>✗</td><td>✓</td>
    </tr>
    <tr>
      <td>CARLA+SUMO</td>
      <td>✓</td><td>✓</td><td>✗</td><td>✗</td><td>✗</td><td>~</td><td>✗</td><td>—</td><td>✗</td><td>✗</td><td>✓</td>
    </tr>
    <tr>
      <td>AirSim+Gazebo</td>
      <td>~</td><td>~</td><td>✓</td><td>✗</td><td>✗</td><td>~</td><td>~</td><td>—</td><td>✗</td><td>~</td><td>✓</td>
    </tr>
    <tr>
      <td rowspan="5"><i>Embodied AI & RL</i></td>
      <td>Isaac Lab</td>
      <td>✗</td><td>✗</td><td>~</td><td>✓</td><td>✓</td><td>✓</td><td>✗</td><td>✗</td><td>✓</td><td>✓</td><td>✓</td>
    </tr>
    <tr>
      <td>Isaac Gym</td>
      <td>✗</td><td>✗</td><td>~</td><td>✓</td><td>✓</td><td>✓</td><td>✗</td><td>✗</td><td>✗</td><td>✗</td><td>✓</td>
    </tr>
    <tr>
      <td>Habitat</td>
      <td>✗</td><td>~</td><td>✗</td><td>✓</td><td>✓</td><td>✓</td><td>✗</td><td>✗</td><td>✗</td><td>✗</td><td>✓</td>
    </tr>
    <tr>
      <td>SAPIEN</td>
      <td>✗</td><td>✗</td><td>✗</td><td>✓</td><td>✓</td><td>✓</td><td>✗</td><td>✗</td><td>✗</td><td>~</td><td>✓</td>
    </tr>
    <tr>
      <td>RoboSuite</td>
      <td>✗</td><td>✗</td><td>✗</td><td>✓</td><td>✓</td><td>✓</td><td>✗</td><td>✗</td><td>✗</td><td>~</td><td>✓</td>
    </tr>
    <tr>
      <td><b>Ours</b></td>
      <td><b>CARLA-Air</b></td>
      <td><b>✓</b></td><td><b>✓†</b></td><td><b>✓</b></td><td><b>✓</b></td><td><b>✓</b></td><td><b>✓</b></td><td><b>✓</b></td><td><b>✓</b></td><td><b>✓</b></td><td><b>✓</b></td><td><b>✓</b></td>
    </tr>
  </tbody>
</table>

<p><sup>✓ = supported; ~ = partial or constrained; ✗ = not supported; — = not applicable.<br/>
† Pedestrian AI is inherited from CARLA and fully functional; behavior under high actor density in joint scenarios is an active engineering target.</sup></p>

---

<a id="quick-start"></a>

## 🎮 Quick Start

### Option A: Binary Release (Recommended)

```bash
# 1. Download and extract CARLA-Air v0.1.7
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

> **What you'll see:** A Tesla cruises through the city while a drone chases it from above. A 4-panel display shows **RGB, Depth, Semantic Segmentation, and LiDAR BEV** -- all synchronized. Weather cycles automatically.

### Option B: Build from Source

Please refer to the [Build Guide](CarlaAir_Release/source/BUILD_GUIDE.md) for detailed instructions on compiling CARLA-Air with UE4.26.

---

<a id="one-script-two-worlds"></a>

## 🐍 One Script, Two Worlds

Both APIs share the **same simulated world** -- no bridge, no sync headaches.

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

**6 demo scripts** -- try them all:

```bash
python3 examples/quick_start_showcase.py   # 🎬 4-panel sensors + drone chase + weather cycling
python3 examples/drive_vehicle.py          # 🚗 Drive a Tesla with WASD
python3 examples/walk_pedestrian.py        # 🚶 Walk the city on foot (mouse look)
python3 examples/switch_maps.py            # 🗺️  Fly through all 13 maps automatically
python3 examples/sensor_gallery.py         # 📸 6-grid sensor showcase on one vehicle
python3 examples/air_ground_sync.py        # 🔄 Car + drone split-screen: same rain, same world
```

**Recording toolkit** -- record trajectories for vehicle, drone, and pedestrian, then replay them with a director camera:

```bash
python3 examples/recording/record_vehicle.py     # 🚗 Drive & record vehicle trajectory
python3 examples/recording/record_drone.py       # 🚁 Fly & record drone trajectory (zero intrusion)
python3 examples/recording/record_walker.py      # 🚶 Walk & record pedestrian trajectory
python3 examples/recording/demo_director.py \    # 🎬 Replay all + free camera + MP4 recording
    trajectories/vehicle_*.json trajectories/drone_*.json
```

> **Docs:** [Coordinate Systems (CARLA to AirSim)](CarlaAir_Release/guide/COORDINATE_SYSTEMS.md) | [Quick Start Guide](CarlaAir_Release/guide/Quick-Start.md) | [FAQ](CarlaAir_Release/guide/FAQ.md)

---

<a id="research-directions--workflows"></a>

## 🔬 Research Directions & Workflows

CARLA-Air is designed to support four major research directions in air-ground embodied intelligence:

1. **Air-Ground Cooperation** -- Heterogeneous air-ground agents collaborating in a shared urban environment.
2. **Embodied Navigation (VLN/VLA)** -- Vision-and-language-driven navigation and action in photorealistic cities.
3. **Multi-Modal Perception and Dataset Collection** -- Synchronized aerial-ground sensor data acquisition across diverse conditions.
4. **RL-Based Policy Training** -- Closed-loop reinforcement learning with joint air-ground interaction.

The platform provides five reference workflows that cover these directions:

| | Workflow | Direction | Key Result |
|---|---|---|---|
| W1 | Cooperative Precision Landing | Air-ground cooperation | < 0.5 m landing error |
| W2 | VLN/VLA Data Generation | Embodied navigation | Cross-view VLN data pipeline |
| W3 | Multi-Modal Dataset Collection | Perception and dataset | 12-stream sync, 1-tick alignment |
| W4 | Cross-View Perception | Perception and dataset | 14/14 weather presets verified |
| W5 | RL Training Environment | RL policy training | 357 reset cycles, 0 crashes |

<table>
  <tr>
    <td align="center" width="50%">
      <img src="docs/images/landing.jpg" alt="W1: Cooperative Precision Landing" width="100%"/><br/>
      <b>W1: Cooperative Precision Landing</b>
    </td>
    <td align="center" width="50%">
      <img src="docs/images/vln.jpg" alt="W2: VLN/VLA Data Generation" width="100%"/><br/>
      <b>W2: VLN/VLA Data Generation</b>
    </td>
  </tr>
  <tr>
    <td align="center">
      <img src="docs/images/datasets.jpg" alt="W3: Multi-Modal Dataset Collection" width="100%"/><br/>
      <b>W3: Multi-Modal Dataset Collection</b>
    </td>
    <td align="center">
      <img src="docs/images/w4.jpg" alt="W4: Cross-View Perception" width="100%"/><br/>
      <b>W4: Cross-View Perception</b>
    </td>
  </tr>
  <tr>
    <td align="center">
      <img src="docs/images/RL.jpg" alt="W5: RL Training Environment" width="100%"/><br/>
      <b>W5: RL Training Environment</b>
    </td>
    <td align="center">
      <img src="docs/images/asset.jpg" alt="Custom Asset Import" width="100%"/><br/>
      <b>Custom Asset Import</b>
    </td>
  </tr>
</table>

---

<a id="flight-controls"></a>

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

<a id="documentation--tutorials"></a>

## 📚 Documentation & Tutorials

We provide **6 curated Python examples** showcasing the core air-ground cooperative capabilities:

| Example | Description |
|---------|-------------|
| `quick_start_showcase.py` | 4-panel sensors + drone chase + weather cycling |
| `drive_vehicle.py` | Drive a Tesla with WASD keyboard control |
| `walk_pedestrian.py` | Walk the city on foot with mouse look |
| `switch_maps.py` | Fly through all 13 maps automatically |
| `sensor_gallery.py` | 6-grid sensor showcase on one vehicle |
| `air_ground_sync.py` | Car + drone split-screen: same rain, same world |

**Step-by-step tutorials** (8 scripts in `CarlaAir_Release/guide/examples/`, beginner-friendly):

| # | Tutorial | What You Will Learn |
|---|----------|---------------------|
| 01 | `01_hello_world.py` | Connect to both APIs, verify setup |
| 02 | `02_weather_control.py` | Change weather parameters in real-time |
| 03 | `03_spawn_traffic.py` | Generate vehicles and pedestrians |
| 04 | `04_sensor_capture.py` | Attach and read sensors |
| 05 | `05_drone_takeoff.py` | Basic drone flight commands |
| 06 | `06_drone_sensors.py` | Aerial sensor configuration |
| 07 | `07_combined_demo.py` | Air-ground joint operation |
| 08 | `08_full_showcase.py` | Full platform capabilities |

**Full Documentation:**
- [Quick Start Guide](CarlaAir_Release/guide/Quick-Start.md)
- [Coordinate Systems (CARLA to AirSim)](CarlaAir_Release/guide/COORDINATE_SYSTEMS.md)
- [Architecture Details](CarlaAir_Release/source/ARCHITECTURE.md)
- [Modifications from Upstream](CarlaAir_Release/source/MODIFICATIONS.md)
- [FAQ](CarlaAir_Release/guide/FAQ.md)

---

<a id="roadmap"></a>

## 🗺️ Roadmap

- [x] Single-process CARLA + AirSim integration (UE4.26)
- [x] FPS drone control (WASD + Mouse)
- [x] Auto traffic spawn (vehicles + pedestrians)
- [x] 18-channel synchronized sensors
- [x] Dual Python API (CARLA + AirSim)
- [x] ROS2 validation (63 topics)
- [x] One-click environment setup
- [x] Recording toolkit (vehicle, drone, pedestrian trajectories)
- [x] Technical report ([PDF](report/CarlaAir_arxiv_version.pdf))
- [ ] Project page (in progress)
- [ ] Tutorial documentation
- [ ] 3DGS rendering pipeline integration
- [ ] World Model integration
- [ ] Multi-drone support

---

<a id="citation"></a>

## 📝 Citation

If you find CARLA-Air useful in your research, please consider citing our paper:

```bibtex
@article{zeng2026carlaair,
  title   = {CARLA-Air: Fly Drones Inside a CARLA World --
             A Unified Infrastructure for Air-Ground Embodied Intelligence},
  author  = {Zeng, Tianle and Chen, Hanxuan and Wen, Yanci and Zhang, Hong},
  journal = {arXiv preprint arXiv:XXXX.XXXXX},
  year    = {2026}
}
```

---

<a id="license--acknowledgments"></a>

## 📜 License & Acknowledgments

CARLA-Air is built upon the shoulders of giants. We sincerely thank the developers of:
- [CARLA Simulator](https://github.com/carla-simulator/carla) (MIT License)
- [Microsoft AirSim](https://github.com/microsoft/AirSim) (MIT License)
- [Unreal Engine](https://www.unrealengine.com/)

CARLA-Air specific code is distributed under the **MIT License**. CARLA specific assets are distributed under the CC-BY License.

---

<a id="star-history"></a>

## ⭐ Star History

<a href="https://star-history.com/#louiszengCN/CarlaAir&Date">
 <picture>
   <source media="(prefers-color-scheme: dark)" srcset="https://api.star-history.com/svg?repos=louiszengCN/CarlaAir&type=Date&theme=dark" />
   <source media="(prefers-color-scheme: light)" srcset="https://api.star-history.com/svg?repos=louiszengCN/CarlaAir&type=Date" />
   <img alt="Star History Chart" src="https://api.star-history.com/svg?repos=louiszengCN/CarlaAir&type=Date" />
 </picture>
</a>
