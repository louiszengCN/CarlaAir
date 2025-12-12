# SimWorld: A Unified Air-Ground Simulation Platform for Autonomous Systems Research

> **Technical Reference Document for Paper Writing**
> Last Updated: 2026-03-10

---

## 1. Introduction and Motivation

### 1.1 The Problem

Research in autonomous systems increasingly requires **joint air-ground simulation** -- scenarios where unmanned aerial vehicles (UAVs) and ground vehicles/pedestrians coexist and interact in shared urban environments. Applications include:

- **Aerial surveillance of autonomous traffic**: A drone monitors and tracks ground vehicles
- **Cooperative perception**: UAVs provide bird's-eye-view to complement vehicle-level sensing
- **Social navigation**: Pedestrians, vehicles, and drones sharing urban space
- **Multi-modal dataset generation**: Synchronized ground-level and aerial sensor data
- **Search and rescue**: Drones scouting while ground robots navigate debris

However, the two dominant open-source simulators serve disjoint domains:
- **CARLA** (Intel Labs, CVC Barcelona): High-fidelity autonomous driving simulator with vehicles, pedestrians, traffic management, 10+ sensor types, and OpenDRIVE road networks -- but **no native UAV support**
- **AirSim** (Microsoft Research): Drone and car simulation with physics-based flight dynamics, aerial sensors, and multiple flight controller backends -- but **no traffic management, pedestrian AI, or urban road networks**

Running them as **separate processes** with inter-process synchronization is impractical: no shared physics, no shared rendering, no coordinate system alignment, and severe performance overhead from duplicating the UE4 rendering pipeline.

### 1.2 Our Solution

**SimWorld** unifies CARLA 0.9.16 and AirSim 1.8.1 into a **single Unreal Engine 4.26 process**, enabling both Python APIs (`carla.Client` on port 2000, `airsim.MultirotorClient` on port 41451) to operate simultaneously on the same simulated world. The system requires **zero modifications to user-facing Python APIs** -- existing CARLA and AirSim scripts work unmodified.

### 1.3 Key Contributions

1. **Unified GameMode Architecture**: A novel single-inheritance + composition pattern (`ASimWorldGameMode`) that resolves UE4's fundamental one-GameMode-per-world constraint
2. **Dual API Server Coexistence**: Two independent RPC servers (CARLA rpclib + AirSim rpclib) operating on different ports within the same process
3. **Spectator Pawn Decoupling**: A technique to satisfy CARLA's spectator requirement while preserving AirSim's pawn possession model
4. **Comprehensive Air-Ground Capabilities**: 220+ actor blueprints, 16+ sensor types, 14 weather presets, 13 maps, and physics-based drone flight -- all simultaneously accessible
5. **Off-Screen Dataset Generation**: Headless multi-modal data collection combining ground-level and aerial perspectives

---

## 2. System Architecture

### 2.1 The Core Challenge: UE4's Single GameMode Constraint

Unreal Engine 4 enforces a strict architectural constraint: **each World (level) can have exactly one GameMode**. The GameMode controls fundamental simulation lifecycle -- player spawning, game rules, match states, and initialization ordering.

- CARLA uses `ACarlaGameModeBase` as its GameMode, which initializes the Episode system, Weather, Traffic Light Manager, Actor Factories, Recorder, and the RPC server
- AirSim uses `AAirSimGameMode` as its GameMode, which initializes the SimHUD, reads settings.json, and spawns the SimMode actor

These two GameModes **cannot coexist** through standard UE4 mechanisms. Previous approaches (e.g., level streaming, plugin-only integration) fail because both systems assume GameMode-level control over the simulation lifecycle.

### 2.2 The Solution: Single-Inheritance + Composition Pattern

We exploit an architectural asymmetry between the two systems:

- CARLA's subsystems (Episode, Weather, Traffic Manager, Actor Factories) are **tightly coupled** to `ACarlaGameModeBase` through inheritance and `friend` declarations
- AirSim's simulation logic lives in `ASimModeBase`, which inherits from `AActor` (not `AGameModeBase`), making it spawnable as a regular world actor

Our unified `ASimWorldGameMode` class uses:

```
                    UE4 Single GameMode Slot
                            |
                    AGameModeBase (UE4)
                            |
                    ACarlaGameModeBase          ← CARLA subsystems via inheritance
                            |
                    ASimWorldGameMode           ← Occupies the GameMode slot
                            |
                            |--- [owns] ASimModeBase (spawned as AActor)  ← AirSim via composition
                            |              |
                            |              |--- FastPhysicsEngine (333 Hz async thread)
                            |              |--- MultirotorRpcLibServer (port 41451)
                            |              |--- AFlyingPawn (drone)
                            |              |--- ApiProvider (vehicle APIs)
                            |
                            |--- [inherits] UCarlaEpisode
                            |--- [inherits] AWeather
                            |--- [inherits] ACarlaRecorder
                            |--- [inherits] ATrafficLightManager
                            |--- [inherits] ActorDispatcher + ActorFactories
```

**Key Insight**: By having `ASimWorldGameMode` **inherit** from `ACarlaGameModeBase`, we get all CARLA subsystems through the standard UE4 lifecycle (`InitGame` → `BeginPlay` → `Tick`). AirSim's `ASimModeBase` is then **spawned as a regular Actor** in the world during `BeginPlay`, after CARLA's initialization is complete. This avoids any slot conflict because `ASimModeBase` never competes for the GameMode slot.

### 2.3 Class Hierarchy

```cpp
// The unified GameMode (in AirSim plugin module, depends on Carla module)
UCLASS()
class AIRSIM_API ASimWorldGameMode : public ACarlaGameModeBase {
    // CARLA: inherited Episode, Weather, Recorder, ActorFactories, Traffic
    // AirSim: composed SimMode, HUD widget, input bindings
private:
    UPROPERTY() ASimModeBase* SimMode_;        // AirSim simulation (spawned actor)
    UPROPERTY() USimHUDWidget* AirSimWidget_;  // Debug overlay
    TSubclassOf<ASimModeBase> SimModeClass_;   // From settings.json
};
```

### 2.4 Initialization Sequence

```
UE4 Engine Start
│
├── ASimWorldGameMode Constructor
│   ├── ACarlaGameModeBase() → creates Episode, Recorder
│   ├── DefaultPawnClass = nullptr  ← Critical for AirSim
│   ├── Load BP_Weather blueprint class
│   ├── Register 8 Actor Factories (5 C++ + 3 Blueprint)
│   │   ├── ASensorFactory, AStaticMeshFactory, ATriggerFactory
│   │   ├── AUtilActorFactory, AAIControllerFactory
│   │   ├── BP VehicleFactory, WalkerFactory, PropFactory
│   └── Initialize AirSim logger + ImageWrapper module
│
├── ACarlaGameModeBase::InitGame()
│   ├── Spawn Weather actor (from BP_Weather)
│   ├── Spawn & register all ActorFactories with Episode
│   ├── Parse OpenDRIVE road network (.xodr)
│   ├── Generate spawn points from road topology
│   └── Initialize CarlaGameInstance → FCarlaEngine → Start CARLA RPC (port 2000)
│
├── ASimWorldGameMode::BeginPlay()
│   ├── Phase 1: ACarlaGameModeBase::BeginPlay()
│   │   ├── Semantic tagging of all world actors
│   │   ├── Traffic Light Manager initialization
│   │   ├── Episode::InitializeAtBeginPlay()
│   │   ├── Apply default weather parameters
│   │   ├── Start Recorder/Replayer
│   │   └── Register environment objects
│   │
│   ├── Phase 2: Spectator Pawn Creation
│   │   ├── Spawn ASpectatorPawn (unpossessed)
│   │   ├── Register with Episode->Spectator (friend access)
│   │   └── Register with ActorDispatcher registry
│   │
│   └── Phase 3: AirSim Bootstrap
│       ├── InitializeAirSimSettings() → read settings.json
│       ├── SetUnrealEngineSettings() → disable motion blur, custom depth
│       ├── CreateSimMode() → SpawnActor<ASimModeWorldMultiRotor>()
│       │   └── ASimModeBase::BeginPlay()
│       │       ├── NED coordinate system setup
│       │       ├── Create ApiProvider + WorldSimApi
│       │       ├── Spawn BP_FlyingPawn (drone pawn)
│       │       ├── Create MultirotorPawnSimApi
│       │       │   ├── MultiRotorPhysicsBody (4 rotors + 6 drag faces)
│       │       │   ├── SimpleFlight controller
│       │       │   └── Sensors: IMU, Barometer, GPS, Magnetometer
│       │       ├── Create FastPhysicsEngine
│       │       └── Start async physics thread @ 333 Hz
│       ├── CreateAirSimWidget() → UMG debug overlay
│       ├── SetupAirSimInputBindings()
│       └── SimMode_->startApiServer() → port 41451
│
└── Both APIs Ready
    ├── CARLA Python API on port 2000
    └── AirSim Python API on port 41451
```

### 2.5 The Spectator Pawn Problem

Setting `DefaultPawnClass = nullptr` is **required** for AirSim because UE4's default behavior auto-spawns and possesses a pawn for the player controller. If any pawn is possessed, AirSim's `SimpleFlightApi` input pipeline breaks -- the drone cannot receive control commands.

However, CARLA expects `Episode->Spectator` to point to a valid pawn (used for camera control, scene observation, and as the Python API's `world.get_spectator()` return value).

**Our solution**: We spawn an `ASpectatorPawn` but **do not possess it**. It exists as an "orphan" pawn -- visible to CARLA's API but invisible to UE4's player controller system. We use C++ `friend class` declarations to directly set `Episode->Spectator` (a private member), bypassing CARLA's normal initialization flow.

```cpp
// In ASimWorldGameMode::BeginPlay(), after Super::BeginPlay()
FActorSpawnParameters SpawnParams;
SpawnParams.SpawnCollisionHandlingOverride = ESpawnActorCollisionHandlingMethod::AlwaysSpawn;
ASpectatorPawn* Spectator = GetWorld()->SpawnActor<ASpectatorPawn>(
    ASpectatorPawn::StaticClass(), &SpawnTransform, SpawnParams);
// Do NOT call PlayerController->Possess(Spectator)
Episode->Spectator = Spectator;  // Direct friend access
Episode->ActorDispatcher->GetActorRegistry().Register(
    FCarlaActor::IdType(Spectator->GetUniqueID()), *Spectator);
```

### 2.6 Build System Integration

The two plugins are compiled as part of the same UE4 project:

```
CarlaUE4.uproject
├── Plugins/Carla/    (LoadingPhase: PostConfigInit)
│   └── Carla.Build.cs  → libcarla_server.a, librpc.a
└── Plugins/AirSim/   (LoadingPhase: PreDefault)
    └── AirSim.Build.cs → PrivateDependency: "Carla"
                         → AirLib (header-only), rpclib, Eigen3, MavLinkCom
```

The **one-way dependency** (AirSim → Carla) is critical: `AirSim.Build.cs` declares `PrivateDependencyModuleNames.Add("Carla")`, allowing `SimWorldGameMode.cpp` to include CARLA headers. The reverse dependency does not exist -- CARLA's code is unaware of AirSim. Only two modifications were made to CARLA source:

1. `CarlaGameModeBase.h`: `WeatherClass` and `ActorFactories` changed from `private` to `protected`
2. `CarlaEpisode.h`: Added `friend class ASimWorldGameMode`

### 2.7 Configuration

**`DefaultEngine.ini`** (key entries):
```ini
GlobalDefaultGameMode=/Script/AirSim.SimWorldGameMode
GameInstanceClass=/Script/Carla.CarlaGameInstance
TransitionMap=/AirSim/AirSimAssets
r.CustomDepth=3           ; Required for semantic segmentation
bEnableEnhancedDeterminism=True
```

**`DefaultGame.ini`** (packaging):
```ini
; 13 CARLA maps + AirSim asset map
+MapsToCook=(FilePath="/Game/Carla/Maps/Town01")
+MapsToCook=(FilePath="/AirSim/AirSimAssets")
; 6 AirSim content directories for cooking
+DirectoriesToAlwaysCook=(Path="AirSim/Blueprints")
+DirectoriesToAlwaysCook=(Path="AirSim/Models")
+DirectoriesToAlwaysCook=(Path="AirSim/Weather")
```

**`~/Documents/AirSim/settings.json`** (runtime):
```json
{
    "SettingsVersion": 1.2,
    "SimMode": "Multirotor",
    "Vehicles": {
        "SimpleFlight": {
            "VehicleType": "SimpleFlight",
            "AutoCreate": true,
            "Cameras": {
                "0": { "CaptureSettings": [{"ImageType": 0, "Width": 1280, "Height": 960}] }
            }
        }
    }
}
```

---

## 3. Dual Communication Architecture

### 3.1 CARLA Server (Port 2000)

CARLA uses a **three-port architecture**:

| Port | Protocol | Purpose |
|------|----------|---------|
| 2000 | rpclib (MessagePack-RPC) | Synchronous commands: spawn, destroy, weather, tick |
| 2001 | TCP streaming | High-throughput sensor data (camera images, LiDAR) |
| 2002 | TCP secondary | Multi-GPU synchronization |

**RPC binding pattern**: Operations that modify UE4 state are **sync-bound** to the game thread via `boost::asio::io_context` post-and-wait. Read-only operations are **async-bound** to RPC worker threads.

**Sensor data pipeline**:
1. Sensor captures data during UE4 tick (game thread)
2. `FPixelReader::SendPixelsInRenderThread()` enqueues a render command
3. GPU pixels are read via `FRHICommandListImmediate`
4. Data is serialized through `SensorRegistry::Serialize()` (type-dispatched)
5. Sent via dedicated TCP stream (each sensor has a unique stream token)

### 3.2 AirSim Server (Port 41451)

AirSim uses a **single-port RPC architecture** via rpclib:

| Port | Protocol | Purpose |
|------|----------|---------|
| 41451 | rpclib (MessagePack-RPC) | All commands: flight control, image capture, sensor queries |

**Image capture**: Unlike CARLA's streaming model, AirSim images are requested on-demand through RPC calls (`simGetImages`). The server reads the UE4 render target synchronously and returns raw pixel data in the RPC response.

**Physics thread interaction**: Flight commands (e.g., `moveToPositionAsync`) are posted to AirSim's internal command queue. The `FastPhysicsEngine` running on a dedicated async thread at 333 Hz reads commands and updates the physics state. The RPC server reads back state on demand.

### 3.3 Coexistence

Both RPC servers run within the same process, sharing the UE4 world. They do not interfere because:
- They bind to different ports (2000 vs 41451)
- They use independent rpclib server instances
- Game-thread operations are serialized naturally by UE4's single-threaded game loop
- AirSim's async physics thread operates on its own kinematic state, reading UE4 collision geometry but not modifying CARLA actors

---

## 4. Sensor Systems

### 4.1 CARLA Ground Sensors (10 types)

CARLA provides a comprehensive ground-vehicle sensor suite, all spawnable as actors attached to any vehicle:

| Sensor | Class Hierarchy | Data Format | Key Parameters |
|--------|----------------|-------------|----------------|
| RGB Camera | `ASceneCaptureCamera` : `ASceneCaptureSensor` : `ASensor` | BGRA uint8 | Resolution, FOV, lens distortion |
| Depth Camera | `ADepthCamera` : `AShaderBasedSensor` : `ASceneCaptureSensor` | Encoded R+G×256+B×65536, normalized to meters | Same as RGB |
| Semantic Segmentation | `ASemanticSegmentationCamera` : `AShaderBasedSensor` | Label index in R channel (20+ CityScapes-compatible classes) | Same as RGB |
| Instance Segmentation | `AInstanceSegmentationCamera` : `AShaderBasedSensor` | Per-instance unique color | Same as RGB |
| LiDAR | `ARayCastLidar` : `ARayCastSemanticLidar` : `ASensor` | float32 [x, y, z, intensity] per point | Channels (16-128), range, points/sec, rotation freq |
| Radar | `ARadar` : `ASensor` | Per-detection: azimuth, altitude, depth, velocity | H-FOV, V-FOV, range, points/sec |
| IMU | `AInertialMeasurementUnit` : `ASensor` | Accelerometer (3D), gyroscope (3D), compass | Noise std dev, bias |
| GNSS | `AGnssSensor` : `ASensor` | Latitude, longitude, altitude | Noise deviation |
| Collision | `ACollisionSensor` : `ASensor` | Event-driven: other actor, impulse | -- |
| Lane Invasion | `ALaneInvasionSensor` : `ASensor` | Event-driven: crossed lane markings | -- |

**Shader-based rendering**: Depth, segmentation, normals, and optical flow cameras apply post-process materials to UE4's scene capture pipeline. Materials write to the G-buffer channels, which are then read back as pixel data. This achieves pixel-perfect alignment with RGB images at negligible additional rendering cost.

**LiDAR implementation**: Uses UE4 `LineTraceSingleByChannel` for ray-casting. Points are generated in a rotating pattern matching real LiDAR scan patterns. Intensity is computed via an attenuation model: `α × I₀ + β`, where I₀ depends on hit distance and material reflectivity. Supports atmospheric attenuation for rain/fog conditions.

### 4.2 AirSim Aerial Sensors (6 types)

| Sensor | Implementation | Data Format | Noise Model |
|--------|---------------|-------------|-------------|
| RGB Camera | `APIPCamera` + `USceneCaptureComponent2D` | uint8 BGRA | -- |
| Depth Camera | Post-process material on same component | float32 (meters) or uint8 (visualization) | -- |
| Segmentation | Post-process material on same component | uint8 label colors | -- |
| IMU | `ImuSimple` (AirLib) | Accelerometer, gyroscope, compass | Angle Random Walk, Velocity Random Walk, Gaussian-Markov bias drift |
| GPS | `GpsSimple` (AirLib) | Lat, lon, alt, velocity | EPH/EPV convergence filter, update rate limiting, latency |
| Barometer | `BarometerSimple` (AirLib) | Altitude (m), pressure (Pa) | Gaussian-Markov drift (~10m/hr), uncorrelated noise (~0.2m σ) |
| Magnetometer | `MagnetometerSimple` (AirLib) | Body-frame magnetic field (3D) | Gaussian noise + uniform bias |

**Camera system**: Each drone has 5 camera mount points (front_center, front_right, front_left, bottom_center, back_center). Each mount creates multiple `USceneCaptureComponent2D` instances for different image types. Images are captured on-demand through the RPC API. Default resolution: 1280×960.

**Sensor noise models**: AirSim implements physically-motivated noise models based on Oliver Woodman's "An introduction to inertial navigation" (Cambridge TR-696):
- **IMU**: ARW (angle random walk) for gyroscope drift, VRW (velocity random walk) for accelerometer bias, with configurable turn-on bias
- **GPS**: First-order low-pass filter for horizontal/vertical position error convergence, simulating cold-start behavior
- **Barometer**: Gaussian-Markov process for slow pressure drift, plus high-frequency measurement noise

### 4.3 Combined Sensor Capabilities

In SimWorld, **all 16 sensor types can operate simultaneously**:

| Perspective | Sensors Available | Total Streams |
|-------------|-------------------|---------------|
| Ground Vehicle | RGB, Depth, Segmentation, Instance Seg, LiDAR, Radar, IMU, GNSS, Collision, Lane Invasion | 10 |
| Aerial Drone | RGB, Depth, Segmentation, Infrared, IMU, GPS, Barometer, Magnetometer | 8 |
| **Combined** | **All of the above, synchronized to the same world state** | **18** |

This is a unique capability -- no other simulator provides synchronized ground + aerial multi-modal sensor data in a single rendering pass.

---

## 5. Physics and Flight Dynamics

### 5.1 Ground Vehicle Physics

CARLA uses UE4's native PhysX vehicle physics (`AWheeledVehicle`):
- 4-wheel (standard cars) and N-wheel (trucks, buses) configurations
- Torque curve, steering curve, mass, drag, center of mass tuning
- Ackermann steering model with PID controller
- Integration with Project Chrono for higher-fidelity tire models (optional)

### 5.2 Drone Flight Dynamics (FastPhysicsEngine)

AirSim implements a **custom rigid-body dynamics engine** running on a dedicated async thread at 333 Hz:

**Integration**: Second-order Verlet method for both linear and angular motion:
```
v(t+dt) = v(t) + a(t) × dt
x(t+dt) = x(t) + v(t+dt) × dt
```

**Force model**: Each rotor contributes a thrust force vector and torque. Total wrench is the sum over 4 rotors. Aerodynamic drag uses a 6-face box model with velocity-squared dependence:
```
F_drag = -0.5 × ρ × Cd × A × |v_rel|² × v̂_rel
```
where `v_rel` accounts for wind.

**Collision response**: Impulse-based method with Coulomb friction:
```
j = -(1 + e) × (v · n) / (1/m + ((I⁻¹(r × n)) × r) · n)
```
Ground lock prevents micro-bouncing on landing surfaces.

**Flight controllers**:
- **SimpleFlight** (default): Built-in PID controller for position, velocity, and attitude control. Runs within AirLib, no external dependencies.
- **PX4** (optional): Software-in-the-loop with PX4 Autopilot via MAVLink
- **ArduPilot** (optional): SITL integration

### 5.3 Coordinate Systems

| System | Convention | Origin | Units |
|--------|-----------|--------|-------|
| UE4 / CARLA | Left-handed, Z-up | Map origin (0, 0, 0) | Centimeters (internally), meters (API) |
| AirSim | NED (North-East-Down) | PlayerStart location | Meters |

**Coordinate mapping**: AirSim creates a `NedTransform` at BeginPlay that maps between UE4 world coordinates and the NED frame anchored at the drone's spawn point. For drone-follows-car applications, we use a **relative offset method**:
1. Record initial CARLA car position `(cx₀, cy₀)` and AirSim drone NED position `(dx₀, dy₀)`
2. Per frame: `Δ = (cx - cx₀, cy - cy₀)`, target drone NED = `(dx₀ + Δx, dy₀ + Δy, -altitude)`

---

## 6. Traffic and Environment

### 6.1 Traffic Manager

CARLA's Traffic Manager is a **client-side pipeline architecture** running on a dedicated thread:

| Stage | Function |
|-------|----------|
| ALSM | Actor Lifecycle and State Management -- syncs simulation state |
| Localization | Waypoint horizon maintenance, lane change decisions |
| Collision | Polygon-based collision prediction (boost::geometry) |
| Traffic Light | Signal compliance logic |
| Motion Plan | PID controller: separate urban/highway parameters |
| Vehicle Light | Automatic headlight/brake light management |

The TM maintains an **InMemoryMap** (cached waypoint graph) for O(1) spatial queries and a **SimulationState** tracking all managed actors' kinematics.

### 6.2 Weather System

15 continuous parameters controlling atmospheric conditions:

| Parameter | Range | Effect |
|-----------|-------|--------|
| Cloudiness | 0-100 | Cloud cover density |
| Precipitation | 0-100 | Rain intensity |
| PrecipitationDeposits | 0-100 | Water puddle accumulation |
| WindIntensity | 0-100 | Wind speed |
| SunAzimuthAngle | 0-360 | Sun horizontal angle |
| SunAltitudeAngle | -90 to 90 | Sun elevation (negative = night) |
| FogDensity | 0-100 | Volumetric fog thickness |
| FogDistance | 0+ | Fog start distance (m) |
| Wetness | 0-100 | Surface wetness (reflections) |
| DustStorm | 0-100 | Dust/sandstorm intensity |

14 named presets covering day/night, clear/cloudy/rain/storm conditions. Weather affects **both** CARLA rendering and AirSim's aerial views simultaneously.

### 6.3 Maps and Road Networks

| Map | Description | Spawn Points | Area |
|-----|-------------|--------------|------|
| Town01 | Small town, T-junctions | 252 | ~1 km² |
| Town02 | Residential area | ~100 | ~0.5 km² |
| Town03 | Urban downtown, highway ramp | 265 | ~2 km² |
| Town04 | Highway with small town | ~300 | ~5 km² |
| Town05 | Urban grid with multi-lane roads | 302 | ~3 km² |
| Town10HD | HD downtown with detailed buildings | 155 | ~1.5 km² |

Each map includes:
- **OpenDRIVE** (.xodr): Complete road network definition with lane geometry, junctions, signals
- **Navigation mesh**: Recast-generated pedestrian walkable areas
- **Traffic infrastructure**: Traffic lights (with timing), stop signs, speed limits
- **Environment objects**: Buildings, vegetation, street furniture (13000+ objects in Town10HD)

---

## 7. Actor System

### 7.1 Actor Blueprint Library

| Category | Count | Examples |
|----------|-------|---------|
| Vehicles | 41 | Tesla Model 3, BMW Gran Tourer, Audi A2, Bus, Truck, Motorcycle, Bicycle |
| Walkers | 52 | Male/female adults, children, with diverse clothing and body types |
| Sensors | 25 | All sensor types listed in Section 4 |
| Props | 99 | Barriers, containers, trash cans, vendor stalls, benches |
| Other | 3 | Controller types |
| **Total** | **220** | |

### 7.2 Actor Spawn Pipeline

```
Python: world.spawn_actor(blueprint, transform)
  → RPC: carla::rpc::Command::SpawnActor
    → FCarlaServer::FPimpl (game thread)
      → UCarlaEpisode::SpawnActorWithInfo()
        → UActorDispatcher::SpawnActor()
          → ACarlaActorFactory::SpawnActor()  (type-specific factory)
            → UWorld::SpawnActor<>()  (UE4 native spawn)
          → FActorRegistry::Register(actor_id, actor)
        → return carla::rpc::Actor (serialized to client)
```

### 7.3 Pedestrian AI Navigation

Walkers use UE4's **Recast navigation mesh** for pathfinding:
- Navigation mesh is pre-built from map geometry
- `AWalkerAIController` provides the Python API interface
- `controller.go_to_location(target)` triggers A* pathfinding on the navmesh
- `controller.set_max_speed(speed)` controls movement speed

---

## 8. Application Domains and Research Capabilities

### 8.1 Autonomous Driving

- Full ego-vehicle sensor suite (RGB, depth, segmentation, LiDAR, radar, IMU, GNSS)
- Traffic Manager with per-vehicle behavioral parameters
- Ackermann steering model with PID controller
- 41 vehicle types across 6+ maps with OpenDRIVE road networks
- Deterministic simulation mode for reproducible experiments

### 8.2 UAV Navigation and Control

- 6-DOF flight control: position, velocity, body-frame velocity, yaw rate, path following
- Physics-based dynamics at 333 Hz with wind and aerodynamic drag
- Multi-camera aerial sensing (5 mount points per drone)
- Configurable flight controllers (SimpleFlight, PX4, ArduPilot)
- Sensor noise models (IMU, GPS, barometer, magnetometer) for realistic perception

### 8.3 Air-Ground Cooperative Systems (Novel)

This is SimWorld's unique research capability:

- **Drone-car tracking**: UAV follows ground vehicles with real-time coordinate translation
- **Aerial traffic monitoring**: Bird's-eye-view vehicle detection and counting
- **Cooperative perception**: Ground-level + aerial sensor fusion datasets
- **Multi-agent coordination**: Multiple CARLA vehicles + AirSim drone in shared scenarios
- **Emergency response**: Aerial scouting + ground navigation in urban environments

### 8.4 Social Navigation

- FPS-style first-person pedestrian control with mouse-look
- 52 pedestrian types with AI-controlled navigation
- Large-scale crowd simulation (50+ vehicles + 20+ pedestrians)
- Pedestrian-vehicle interaction in traffic environments
- Variable weather and lighting conditions

### 8.5 Multi-Modal Dataset Generation

SimWorld enables generating datasets with **unprecedented modality coverage**:

| Modality | Source | Format | Resolution |
|----------|--------|--------|------------|
| Ground RGB | CARLA camera | PNG/JPG | Configurable (up to 4K) |
| Ground Depth | CARLA depth camera | float32 NPY (meters) | Same as RGB |
| Ground Segmentation | CARLA seg camera | uint8 NPY (20+ classes) | Same as RGB |
| Ground Instance Seg | CARLA instance camera | RGB (per-instance) | Same as RGB |
| Ground LiDAR | CARLA ray-cast | float32 NPY (x,y,z,intensity) | 16-128 channels |
| Ground Radar | CARLA radar | float32 (azimuth, depth, velocity) | Configurable |
| Ground IMU | CARLA IMU | JSON (accel, gyro, compass) | 100+ Hz |
| Ground GNSS | CARLA GNSS | JSON (lat, lon, alt) | 10+ Hz |
| Aerial RGB | AirSim camera | PNG/NPY | 1280×960 (configurable) |
| Aerial Depth | AirSim depth | float32 NPY (meters) | 1280×960 |
| Aerial Segmentation | AirSim seg | PNG (label colors) | 1280×960 |
| Aerial IMU | AirSim IMU | JSON (with noise) | 333 Hz |
| Aerial GPS | AirSim GPS | JSON (with convergence) | Configurable |
| Aerial Barometer | AirSim barometer | JSON (altitude, pressure) | Configurable |
| Ego Vehicle Pose | CARLA transform | JSON (x,y,z,pitch,yaw,roll) | Per frame |
| Drone Pose | AirSim kinematics | JSON (NED position, quaternion) | Per frame |

All modalities are **synchronized to the same simulation frame** and share the same world state. The off-screen rendering mode (`-RenderOffScreen`) enables headless operation on GPU servers for large-scale data production.

---

## 9. ROS Integration

SimWorld supports dual ROS2 bridges for integration with robotics stacks:

### 9.1 CARLA ROS Bridge

- Workspace: `carla-ros-bridge` (ROS2 Humble)
- Published topics: `/carla/world_info`, `/carla/status`, `/clock`, per-vehicle sensor topics
- Services: `/carla/spawn_object`, `/carla/destroy_object`, `/carla/get_blueprints`
- Namespace: `/carla/*`

### 9.2 AirSim ROS Wrapper

- Package: `airsim_ros_pkgs` (ROS2 Humble)
- Published topics: `/airsim_node/SimpleFlight/imu/imu`, `/airsim_node/SimpleFlight/front_center/Scene`, GPS, barometer, magnetometer, odometry
- Services: takeoff, land, reset
- Namespace: `/airsim_node/*`

### 9.3 Simultaneous Operation

Both bridges run concurrently under the same ROS master with **zero namespace conflicts** (`/carla/` vs `/airsim_node/`). This enables standard ROS2 tools (rviz2, rosbag2, nav2) to consume both ground and aerial data streams.

---

## 10. Performance Characteristics

### 10.1 System Requirements

| Component | Minimum | Recommended |
|-----------|---------|-------------|
| GPU | 4 GB VRAM (Vulkan) | 8+ GB VRAM (NVIDIA RTX) |
| RAM | 16 GB | 32 GB |
| CPU | 4 cores | 8+ cores |
| Disk | 20 GB (packaged) | 40 GB (source + build) |
| OS | Ubuntu 18.04+ | Ubuntu 20.04/22.04 |

### 10.2 GPU Memory Usage (Tested on RTX A4000, 16 GB)

| Scenario | VRAM Usage |
|----------|------------|
| Idle (Town10HD loaded) | ~3700 MiB |
| + 3 vehicles + 2 walkers + drone | ~3870 MiB |
| 3-hour continuous operation | ~3878 MiB (stable, no leak) |
| Peak during map loading | ~5000 MiB |

### 10.3 Stability

| Test | Duration | Cycles | Errors | Result |
|------|----------|--------|--------|--------|
| Continuous spawn/destroy + drone | 3 hours | 357 | 0 | PASS |
| Map switching (Town01→03→05) | ~15 min | 3 maps | 0 | PASS |
| 89-item comprehensive API test | ~5 min | 89 tests | 0 | PASS |

### 10.4 Known Limitations

| Limitation | Cause | Workaround |
|-----------|-------|-----------|
| Autopilot vehicles ≤ 10 with drone | AirSim physics ↔ CARLA Traffic Manager quaternion conflict | Limit traffic count or disable autopilot |
| Map switch latency: 2-5 min | Full asset reload in Shipping build | Use API with 300s timeout |
| Single drone instance | AirSim settings.json limitation | Configurable for multi-drone |

---

## 11. Build and Deployment

### 11.1 Compilation

```bash
# Module-level compilation (avoids features.h cross-compile error)
export UE4_ROOT=/path/to/UnrealEngine-4.26
${UE4_ROOT}/Engine/Build/BatchFiles/Linux/Build.sh CarlaUE4Editor Linux Development \
    -project="/path/to/CarlaUE4.uproject" -module=Carla    # ~290s
${UE4_ROOT}/Engine/Build/BatchFiles/Linux/Build.sh CarlaUE4Editor Linux Development \
    -project="/path/to/CarlaUE4.uproject" -module=AirSim   # ~100s
```

### 11.2 Packaging

```bash
./Util/BuildTools/Package.sh --config=Shipping --no-zip
# Output: Dist/CARLA_Shipping_<version>/LinuxNoEditor/ (~19 GB)
# Compile: ~800s (656 units), Cook: ~2h (13459 packages)
```

### 11.3 Distribution

Packaged build: **7.3 GB** compressed (excluding debug symbols), containing:
- Shipping binary for Linux x86_64
- 13 cooked maps + AirSim assets
- 17 example Python scripts
- CARLA PythonAPI source
- AirSim configuration template

---

## 12. Comparison with Related Work

| Feature | CARLA | AirSim | LGSVL | ISAAC Sim | **SimWorld** |
|---------|-------|--------|-------|-----------|-------------|
| Ground vehicles | ✅ 41 types | ✅ Limited | ✅ | ✅ | ✅ 41 types |
| Pedestrians | ✅ 52 types | ❌ | ✅ | ✅ | ✅ 52 types |
| Drones (physics-based) | ❌ | ✅ | ❌ | ❌ | ✅ |
| Traffic Manager | ✅ | ❌ | ✅ | ❌ | ✅ |
| OpenDRIVE road network | ✅ | ❌ | ✅ | ❌ | ✅ |
| Weather system | ✅ 15 params | ✅ Basic | ✅ | ✅ | ✅ 15 params |
| Ground sensors (10+) | ✅ | ❌ | ✅ | ✅ | ✅ |
| Aerial sensors (6+) | ❌ | ✅ | ❌ | ❌ | ✅ |
| Sensor noise models | Basic | ✅ Detailed | Basic | ✅ | ✅ Detailed |
| ROS integration | ✅ | ✅ | ✅ | ✅ | ✅ Dual bridges |
| Air-ground joint sim | ❌ | ❌ | ❌ | ❌ | ✅ |
| Single-process rendering | N/A | N/A | N/A | N/A | ✅ |
| Open source | ✅ MIT | ✅ MIT | ❌ Discontinued | ❌ Proprietary | ✅ |

**Key differentiator**: SimWorld is the only platform providing physics-based drone flight, comprehensive ground traffic management, and multi-modal synchronized air-ground sensor data in a single rendering process.

---

## 13. Potential Paper Contributions

### For NeurIPS / ICLR Submission

1. **System paper**: Novel architecture solving the single-GameMode constraint for simulator unification
2. **Benchmark paper**: New benchmark for air-ground cooperative perception
3. **Dataset paper**: Large-scale multi-modal dataset with synchronized ground + aerial annotations
4. **Application paper**: Aerial-assisted autonomous driving / social navigation with air-ground data

### Potential Evaluation Experiments

- **Object detection**: Compare detection accuracy using ground-only vs. ground+aerial data
- **Trajectory prediction**: Pedestrian/vehicle trajectory prediction with aerial context
- **Multi-modal fusion**: Sensor fusion benchmarks across 16+ synchronized modalities
- **Sim-to-real transfer**: Domain adaptation from SimWorld synthetic data to real-world datasets
- **Cooperative perception**: Communication-constrained air-ground perception fusion
- **Social navigation**: Pedestrian interaction modeling with drone presence
- **RL-based drone control**: Training drone policies for car-following, surveillance, search tasks

### Comparable Published Work at Top Venues

- **CARLA**: "CARLA: An Open Urban Driving Simulator" (CoRL 2017)
- **AirSim**: "AirSim: High-Fidelity Visual and Physical Simulation for Autonomous Vehicles" (FSR 2017)
- **Habitat**: "Habitat: A Platform for Embodied AI Research" (ICCV 2019)
- **ISAAC Sim**: "Isaac Sim: An Extensible Robotics Simulator" (ICRA 2022)
- **MetaDrive**: "MetaDrive: Composing Diverse Driving Scenarios for Generalizable RL" (TPAMI 2022)

SimWorld fills the gap between driving simulators and aerial simulators, enabling a new class of research at the intersection of autonomous driving, drone navigation, and multi-agent systems.

---

## Appendix A: Complete File Inventory

### Core Architecture Files

| File | Description |
|------|-------------|
| `Plugins/AirSim/Source/SimWorldGameMode.h/.cpp` | Unified GameMode (432 lines) |
| `Plugins/Carla/Source/Carla/Game/CarlaGameModeBase.h/.cpp` | CARLA base GameMode |
| `Plugins/Carla/Source/Carla/Game/CarlaEpisode.h/.cpp` | Episode manager |
| `Plugins/AirSim/Source/SimMode/SimModeBase.h/.cpp` | AirSim simulation mode |
| `Plugins/AirSim/Source/AirSim.Build.cs` | Build config (Carla dependency) |
| `Config/DefaultEngine.ini` | GameMode registration |
| `Config/DefaultGame.ini` | Packaging config |

### CARLA Sensor Files

| File | Sensor |
|------|--------|
| `Sensor/SceneCaptureCamera.h/.cpp` | RGB Camera |
| `Sensor/DepthCamera.h/.cpp` | Depth Camera |
| `Sensor/SemanticSegmentationCamera.h/.cpp` | Semantic Segmentation |
| `Sensor/InstanceSegmentationCamera.h/.cpp` | Instance Segmentation |
| `Sensor/RayCastLidar.h/.cpp` | LiDAR |
| `Sensor/Radar.h/.cpp` | Radar |
| `Sensor/InertialMeasurementUnit.h/.cpp` | IMU |
| `Sensor/GnssSensor.h/.cpp` | GNSS |

### AirSim Core Files

| File | Component |
|------|-----------|
| `Vehicles/Multirotor/FlyingPawn.h/.cpp` | Drone pawn |
| `Vehicles/Multirotor/MultirotorPawnSimApi.h/.cpp` | Drone API wrapper |
| `Vehicles/Multirotor/SimModeWorldMultiRotor.h/.cpp` | Multirotor sim mode |
| `AirLib/include/physics/FastPhysicsEngine.hpp` | Drone physics |
| `AirLib/include/vehicles/multirotor/MultiRotorPhysicsBody.hpp` | Rigid body model |
| `AirLib/include/vehicles/multirotor/api/MultirotorRpcLibServer.hpp` | API server |
| `AirLib/include/sensors/imu/ImuSimple.hpp` | IMU with noise |
| `AirLib/include/sensors/gps/GpsSimple.hpp` | GPS with noise |
| `AirLib/include/sensors/barometer/BarometerSimple.hpp` | Barometer with noise |

## Appendix B: Python API Quick Reference

### CARLA API (port 2000)
```python
import carla
client = carla.Client('localhost', 2000)
world = client.get_world()

# Environment
world.set_weather(carla.WeatherParameters.ClearNoon)
world.get_map().get_spawn_points()

# Actors
bp = world.get_blueprint_library().find('vehicle.tesla.model3')
vehicle = world.spawn_actor(bp, spawn_point)
vehicle.set_autopilot(True)

# Sensors
cam_bp = world.get_blueprint_library().find('sensor.camera.rgb')
camera = world.spawn_actor(cam_bp, transform, attach_to=vehicle)
camera.listen(lambda image: process(image))
```

### AirSim API (port 41451)
```python
import airsim
client = airsim.MultirotorClient(port=41451)
client.confirmConnection()
client.enableApiControl(True)
client.armDisarm(True)

# Flight
client.takeoffAsync().join()
client.moveToPositionAsync(x, y, z, speed).join()
client.moveByVelocityAsync(vx, vy, vz, duration).join()

# Sensors
responses = client.simGetImages([
    airsim.ImageRequest("0", airsim.ImageType.Scene, False, False),
    airsim.ImageRequest("0", airsim.ImageType.DepthPerspective, True),
    airsim.ImageRequest("0", airsim.ImageType.Segmentation, False, False),
])
imu = client.getImuData()
gps = client.getGpsData()
```

### Combined Usage
```python
import carla, airsim, threading

# Both APIs on the same world
carla_client = carla.Client('localhost', 2000)
air_client = airsim.MultirotorClient(port=41451)

# Spawn CARLA traffic while flying AirSim drone
world = carla_client.get_world()
vehicle = world.spawn_actor(vehicle_bp, spawn_point)
vehicle.set_autopilot(True)

air_client.takeoffAsync().join()
air_client.moveToZAsync(-30, 5).join()  # 30m altitude

# Collect synchronized ground + aerial data
ground_image = carla_camera.listen(callback)
aerial_images = air_client.simGetImages([...])
```
