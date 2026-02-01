# CarlaAir (CARLA-Air): A Unified Air-Ground Simulation Platform for Autonomous Systems Research

> **Technical Reference Document for Paper Writing**
> Last Updated: 2026-03-18
> Project Name: CarlaAir (formerly SimWorld)

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

**CarlaAir** (CARLA-Air, formerly SimWorld) unifies CARLA 0.9.16 and AirSim 1.8.1 into a **single Unreal Engine 4.26 process**, enabling both Python APIs (`carla.Client` on port 2000, `airsim.MultirotorClient` on port 41451) to operate simultaneously on the same simulated world. The system requires **zero modifications to user-facing Python APIs** -- existing CARLA and AirSim scripts work unmodified.

### 1.3 Key Contributions

1. **Unified GameMode Architecture**: A novel single-inheritance + composition pattern (`ASimWorldGameMode`) that resolves UE4's fundamental one-GameMode-per-world constraint
2. **Dual API Server Coexistence**: Two independent RPC servers (CARLA rpclib + AirSim rpclib) operating on different ports within the same process
3. **Spectator Pawn Decoupling**: A technique to satisfy CARLA's spectator requirement while preserving AirSim's pawn possession model
4. **Comprehensive Air-Ground Capabilities**: 220+ actor blueprints, 16+ sensor types, 14 weather presets, 13 maps, and physics-based drone flight -- all simultaneously accessible
5. **Off-Screen Dataset Generation**: Headless multi-modal data collection combining ground-level and aerial perspectives
6. **Built-in FPS Interactive Control**: A C++ keyboard/mouse drone control system with 3rd-person chase camera, enabling real-time interactive exploration without Python scripting
7. **Collision Physics System**: Teleport-mode collision with 12-directional LineTrace detection and cumulative push forces, avoiding false collisions in CARLA maps
8. **Coordinate Exact Alignment**: NED-to-CARLA coordinate offset with 0.0000m error, enabling precise cross-API actor localization via CARLA ActorDispatcher registration
9. **Air-Ground Demo Recording Pipeline**: Standardized 4-grid recording system across 7 maps with unified trajectory JSON format, using CARLA sensors to avoid sync-mode rendering deadlock

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

### 2.8 Built-in FPS Drone Control System

CarlaAir includes a built-in C++ FPS-style drone control system that allows real-time interactive drone piloting without any Python scripting.

**Architecture**: The game thread reads keyboard and mouse input, which is passed through an `FCriticalSection` lock to an `FDroneControlWorker` (an `FRunnable` background thread running at ~20 Hz). The worker thread calls `moveByVelocity()` to issue flight commands.

**Controls**:
- **WASD**: Horizontal movement (forward/backward/strafe)
- **Space / Shift**: Vertical movement (ascend / descend)
- **Mouse**: Yaw rotation only (no pitch control -- pitch-affects-flight was tested over 5 iterations and found to be unintuitive for users)
- **Mouse scroll wheel**: Speed adjustment (1-30 m/s)
- **Tab**: Toggle mouse capture mode

**3rd-Person Chase Camera**: The drone is viewed from a chase camera positioned 8m behind and 3.5m above the drone, providing a natural third-person perspective for interactive flight.

**Delayed Activation**: The control system does **not** call `enableApiControl()` or `armDisarm()` at startup. API control is activated only on first keyboard input. This design ensures that the Python API can control the drone freely at launch without interference from the built-in controller.

**Yaw Synchronization**: The system always sends `moveByVelocity(vx, vy, vz, yaw)` even when the drone is hovering (with zero velocity), to keep the yaw angle synchronized between the input state and the flight controller.

### 2.9 Drone CARLA Registration

To enable cross-API actor discovery, `SimWorldGameMode::Tick()` registers the AirSim drone pawn with CARLA's `ActorDispatcher` using `type_id='airsim.drone'`. This allows Python code to locate the drone via `world.get_actors()` and obtain its exact CARLA world coordinates.

The NED-to-CARLA coordinate offset is exact, with **0.0000m error**, enabling precise coordinate translation between the two APIs.

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

### 4.3 Sensor Concurrency Fix (2026-03-12)

A critical sensor concurrency crash was identified and **fixed on 2026-03-12**.

**Root cause**: Two independent issues caused crashes when running 4+ sensors simultaneously:
1. **IMU fatal assertion**: `InertialMeasurementUnit.cpp` contained a `check(GetOwner() != nullptr)` assertion that triggered a fatal `SIGSEGV` when the owning actor was destroyed while the sensor was still processing
2. **GPU readback flooding**: No throttling on GPU pixel readback operations, causing the render thread to be overwhelmed with concurrent readback requests

**Fix**:
1. **Null guard**: Added a null check in `InertialMeasurementUnit.cpp` replacing the fatal `check()` assertion with a graceful early-return when `GetOwner()` is null
2. **GPU readback throttle**: Added a `GPendingReadbacks` atomic counter in `PixelReader.cpp` that limits concurrent GPU readback operations to 16 or fewer (`<= 16` concurrent)

**Result**: The system is now stable with 6 sensors (3 cameras + LiDAR + IMU + GNSS) running for 10 minutes continuously, producing 28986/24866/59053 frames across test configurations. The previous limit was 3 cameras; the new confirmed limit is 6 sensors.

### 4.4 Combined Sensor Capabilities

In CarlaAir, **all 16 sensor types can operate simultaneously**:

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

### 5.3 Collision System (v0.1.5)

CarlaAir v0.1.5 implements a custom collision system designed to work reliably within CARLA's complex urban maps.

**Teleport Mode**: The drone uses UE4's `SetActorLocation` with **Teleport mode** (not Sweep mode) for position updates. Sweep mode was found to produce false collision detections against CARLA map geometry (e.g., invisible collision volumes, LOD boundaries), making it unsuitable for urban flight.

**Ground Clamp**: A vertical `LineTrace` is cast from 100m above the drone's position to 5m below, detecting the ground surface. If the drone is below the detected ground, it is pushed upward to prevent falling through terrain.

**Building Collision**: A 12-directional `LineTrace` system detects nearby obstacles:
- **8 horizontal rays**: N, NE, E, SE, S, SW, W, NW directions
- **4 diagonal-down rays**: 45-degree downward angles for detecting approaching rooftops and ledges
- **Detection distance**: 150 cm
- **Bounce distance**: 180 cm

When multiple rays detect hits simultaneously, the push forces are **cumulative**, creating a natural deflection effect that guides the drone away from corners and tight spaces.

**Physics Mode Toggle**: The **P key** toggles between physics collision mode (drone bounces off buildings and terrain) and invincible pass-through mode (drone flies through all geometry). This is useful for positioning the drone in tight spaces or debugging.

**AirSim collision_info Integration**: When a collision is detected by the LineTrace system, the result is manually written to `state_.collision_info`, ensuring that AirSim's Python API `simGetCollisionInfo()` correctly reports collisions detected by the custom system.

### 5.4 Coordinate Systems

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

This is CarlaAir's unique research capability:

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

CarlaAir enables generating datasets with **unprecedented modality coverage**:

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

CarlaAir supports dual ROS2 bridges for integration with robotics stacks. Both bridges have been verified functional with ROS2 Humble on Ubuntu 22.04.

### 9.1 CARLA ROS Bridge

- Workspace: `carla_ros_ws` (ROS2 Humble, pre-built)
- Launch: `ros2 launch carla_ros_bridge carla_ros_bridge.launch.py host:=localhost port:=2000`
- **Verified topics** (25 total with ego vehicle):
  - `/carla/world_info` — map name + OpenDRIVE
  - `/carla/status` — frame counter, sync mode, delta
  - `/clock` — simulation clock
  - `/carla/hero/front/image` — RGB camera (640×480, bgra8)
  - `/carla/hero/depth_front/image` — depth camera (32FC1)
  - `/carla/hero/semantic_front/image` — semantic segmentation
  - `/carla/hero/lidar` — LiDAR point cloud (637+ points)
  - `/carla/hero/imu` — IMU with orientation quaternion
  - `/carla/hero/gnss` — GNSS with lat/lon/alt
  - `/carla/hero/vehicle_status` — velocity, acceleration
- Namespace: `/carla/*`
- **Note**: Bridge activates synchronous mode; use `world.tick()` or let bridge tick

### 9.2 AirSim ROS2 Wrapper

- Package: `airsim_ros_pkgs` (ROS2 Humble, pre-built)
- Launch: `ros2 launch airsim_ros_pkgs airsim_node.launch.py host:=localhost`
- **Verified topics** (14 total):
  - `/airsim_node/SimpleFlight/odom_local_ned` — odometry (NED)
  - `/airsim_node/SimpleFlight/front_center/Scene` — RGB image (1280×960, bgr8)
  - `/airsim_node/SimpleFlight/imu/imu` — IMU with orientation + angular velocity
  - `/airsim_node/SimpleFlight/gps/gps` — GPS (lat/lon/alt)
  - `/airsim_node/SimpleFlight/altimeter/barometer` — altitude + pressure + QNH
  - `/airsim_node/SimpleFlight/magnetometer/magnetometer` — magnetic field vector
  - `/airsim_node/SimpleFlight/vel_cmd_body_frame` — velocity control input (body frame)
  - `/airsim_node/SimpleFlight/vel_cmd_world_frame` — velocity control input (world frame)
- Namespace: `/airsim_node/*`

### 9.3 Simultaneous Operation

Both bridges run concurrently with **zero namespace conflicts** (`/carla/` vs `/airsim_node/`). This enables standard ROS2 tools (rviz2, rosbag2, nav2) to consume both ground and aerial data streams simultaneously.

**Total verified topics**: 63 (43 CARLA + 14 AirSim + 6 general), all verified with rviz2 visualization.

---

## 10. Performance Characteristics

### 10.1 System Requirements

| Component | Minimum | Recommended |
|-----------|---------|-------------|
| GPU | 4 GB VRAM (Vulkan) | 8+ GB VRAM (NVIDIA RTX) |
| RAM | 16 GB | 64 GB |
| CPU | 4 cores | 8+ cores |
| Disk | 20 GB (packaged) | 40 GB (source + build) |
| OS | Ubuntu 18.04+ | Ubuntu 20.04/22.04 |

### 10.2 Performance Benchmarks (Tested on RTX A4000, 16 GB)

| Scenario | FPS | VRAM | Stability |
|----------|-----|------|-----------|
| Empty scene (Town10HD) | 99.7 | 4335 MiB | Stable |
| 1 vehicle + 2 sensors + drone | 56.1 | 4335 MiB | Stable |
| 3 vehicles + 4 sensors + drone | 56.9 | 4335 MiB | Stable |
| 8 vehicles + 8 sensors + drone | 56.2 | 4335 MiB | Stable |
| 10 autopilot vehicles (10 min) | ~56 | 4741 MiB | 10/10 alive |
| 3 cameras (5 min) | ~56 | ~5400 MiB | 9017 frames |
| 4 sensors: 3cam+IMU (10 min) | ~56 | ~5700 MiB | 28986 frames |
| 5 sensors: 3cam+LiDAR+IMU (10 min) | ~56 | ~5500 MiB | 24866 frames |
| 6 sensors: 3cam+LiDAR+IMU+GNSS (10 min) | ~56 | ~5300 MiB | 59053 frames |
| Peak during map loading | - | ~5000 MiB | - |

AirSim integration overhead: ~40 FPS reduction (from 100 to 56), which is acceptable for co-simulation.

### 10.3 Stability

| Test | Duration | Cycles | Errors | Result |
|------|----------|--------|--------|--------|
| 10 autopilot vehicles | 10 min | 600s | 0 | PASS |
| Drone + CARLA alternating calls | 10 min | 599 each | 0 | PASS |
| High-frequency spawn/destroy | 200 cycles | 155 success | 0 crashes | PASS |
| Client disconnect/reconnect | - | 4 tests | 0 | PASS |
| Invalid input handling | - | 4 tests | 0 | PASS |
| 3-round repeatability | 3 rounds | - | 0 | PASS (±0.1m) |
| 4 sensors (3cam+IMU) | 10 min | 28986 frames | 0 | PASS |
| 5 sensors (3cam+LiDAR+IMU) | 10 min | 24866 frames | 0 | PASS |
| 6 sensors (3cam+LiDAR+IMU+GNSS) | 10 min | 59053 frames | 0 | PASS |

### 10.4 Known Limitations

| Limitation | Status | Cause | Workaround |
|-----------|--------|-------|-----------|
| Concurrent CARLA sensors | **FIXED** (2026-03-12) | IMU `GetOwner()` null assertion + no GPU readback throttle | Null guard + `GPendingReadbacks` atomic counter (≤16 concurrent). Now ≤6 sensors tested stable for 10min |
| Walker AI in async mode | **FIXED** | Navigation mesh tick-driven in Shipping build | Works correctly in sync mode (`synchronous_mode = True`) |
| Vehicle falling through ground | **FIXED** | `SetNewWorldOrigin` displacing actor positions | Disabled `SetNewWorldOrigin`; ground clamp LineTrace as additional safety |
| Sweep collision false positives | **FIXED** | UE4 Sweep mode detecting invisible CARLA collision volumes | Replaced with Teleport mode + custom 12-direction LineTrace collision |
| Recorder unavailable in Shipping build | Active | `start_recorder` triggers RPC error | Use Python-side recording instead |
| `simSetCameraPose` crash in Shipping | Active | C++ abort in Shipping binary | Use drone yaw + orbital flight for camera angles |
| Autopilot vehicles ≤ 10 with drone | Active | AirSim physics ↔ CARLA Traffic Manager interaction | Limit traffic count |
| Map switch latency: 2-5 min | Active | Full asset reload in Shipping build | Use API with 300s timeout |

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

**Current version**: v0.1.5

**Binary package** (`CarlaAir-v0.1.5/`): **18 GB**, containing:
- Shipping binary for Linux x86_64
- 13 cooked maps + AirSim assets
- 17 example Python scripts + 5 recording scripts + `demo_scripts/`
- CARLA PythonAPI source
- AirSim configuration template
- `setup.py` + `requirements.txt` for Python dependency management

**Source package** (`CarlaAir-v0.1.5-source/`): **651 MB**, containing:
- Full Unreal Engine project source
- Both plugin source trees (Carla + AirSim)
- Build scripts and configuration

---

## 12. Interactive Control System

CarlaAir includes a built-in C++ interactive control system that enables real-time drone operation and environment manipulation without Python scripting.

### 12.1 Keyboard/Mouse Drone Control

The FPS-style drone control system (described in Section 2.8) provides intuitive flight control with WASD horizontal movement, Space/Shift vertical movement, and mouse yaw control. The 3rd-person chase camera (8m behind, 3.5m above) provides situational awareness during flight.

### 12.2 Interactive Key Bindings

| Key | Function | Implementation |
|-----|----------|---------------|
| **H** | Bilingual help overlay (English/Chinese) | Slate Widget with pure black background (alpha = 0.92), large bold 18pt font |
| **N** | Weather cycling | Cycles through 6 weather presets (ClearNoon, CloudyNoon, WetNoon, SoftRainNoon, HardRainNoon, ClearNight) |
| **P** | Physics/invincible mode toggle | Switches between collision physics mode and pass-through mode |
| **Tab** | Mouse capture toggle | Toggles between UI mouse mode and flight control mouse capture |
| **Mouse Scroll** | Speed adjustment | Adjusts drone movement speed from 1 to 30 m/s |
| **WASD** | Horizontal movement | Forward/backward/strafe relative to drone heading |
| **Space / Shift** | Vertical movement | Ascend / descend |

### 12.3 Help Overlay

The **H key** toggles a bilingual help overlay rendered as an UE4 Slate Widget. The overlay uses a pure black background with alpha = 0.92 for readability, and displays all key bindings in large bold 18pt text. The overlay is designed to be immediately visible regardless of the underlying scene lighting or weather conditions.

---

## 13. Demo Recording System

### 13.1 Overview

CarlaAir includes a comprehensive demo recording pipeline for generating standardized demonstration videos across multiple maps and scenarios. A total of **30 demos** have been recorded at **1920x1080** resolution, with a combined size of **2.1 GB**, spanning **7 different maps**.

### 13.2 Standard 4-Grid Layout

All demo recordings use a standardized 4-grid layout combining four simultaneous camera perspectives:

| Grid Position | Camera | Description |
|--------------|--------|-------------|
| Top-left | CAM-A | Drone FPV (first-person view from drone) |
| Top-right | CAM-B | Drone 3rd Person (chase camera behind drone) |
| Bottom-left | CAM-C | Vehicle FPV (first-person view from ground vehicle) |
| Bottom-right | CAM-D | Vehicle 3rd Person (follow camera behind vehicle) |

This layout provides a comprehensive view of air-ground interactions from all relevant perspectives in a single video frame.

### 13.3 Key Technical Discovery: Rendering Deadlock

A critical technical finding during demo development: **CARLA sync mode + AirSim `simGetImages()` causes a rendering deadlock**. In synchronous mode, CARLA's render thread waits for the simulation tick to complete, while AirSim's `simGetImages()` attempts to read the UE4 render target synchronously. Both rendering systems end up waiting for each other, causing a permanent hang.

**Solution**: All cameras in the demo recording pipeline use **CARLA sensors** (`sensor.camera.rgb` attached to actors) instead of the AirSim API (`simGetImages()`). CARLA sensors operate through the streaming pipeline (port 2001), which is compatible with synchronous mode and does not compete with AirSim's render thread.

### 13.4 Recording Scripts

| Script | Purpose |
|--------|---------|
| `record_walker.py` | Record pedestrian walking scenarios |
| `record_vehicle.py` | Record ground vehicle driving scenarios |
| `record_drone.py` | Record drone flight scenarios |
| `replay_trajectories.py` | Replay recorded trajectories for re-recording with different cameras/weather |
| `record_video.py` | Convert recorded frames into 4-grid layout videos |
| `record_all.py` | Master script to orchestrate full recording pipeline |

### 13.5 Trajectory JSON Format

Recordings use a unified JSON trajectory format with per-frame entries containing:
- Timestamp and frame number
- Actor transforms (position, rotation) for all tracked entities
- Weather state
- Camera parameters

This format enables trajectory replay across different maps and conditions, supporting reproducible demo generation.

---

## 14. Comparison with Related Work

| Feature | CARLA | AirSim | LGSVL | ISAAC Sim | **CarlaAir** |
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
| Built-in interactive control | ❌ | ❌ | ❌ | ❌ | ✅ FPS drone control |
| Air-ground demo recording | ❌ | ❌ | ❌ | ❌ | ✅ 4-grid multi-view |
| Open source | ✅ MIT | ✅ MIT | ❌ Discontinued | ❌ Proprietary | ✅ |

**Key differentiator**: CarlaAir is the only platform providing physics-based drone flight, comprehensive ground traffic management, built-in interactive FPS control, air-ground demo recording, and multi-modal synchronized air-ground sensor data in a single rendering process.

---

## 15. Potential Paper Contributions

### For NeurIPS / ICLR Submission

1. **System paper**: Novel architecture solving the single-GameMode constraint for simulator unification
2. **Benchmark paper**: New benchmark for air-ground cooperative perception
3. **Dataset paper**: Large-scale multi-modal dataset with synchronized ground + aerial annotations
4. **Application paper**: Aerial-assisted autonomous driving / social navigation with air-ground data

### Potential Evaluation Experiments

- **Object detection**: Compare detection accuracy using ground-only vs. ground+aerial data
- **Trajectory prediction**: Pedestrian/vehicle trajectory prediction with aerial context
- **Multi-modal fusion**: Sensor fusion benchmarks across 16+ synchronized modalities
- **Sim-to-real transfer**: Domain adaptation from CarlaAir synthetic data to real-world datasets
- **Cooperative perception**: Communication-constrained air-ground perception fusion
- **Social navigation**: Pedestrian interaction modeling with drone presence
- **RL-based drone control**: Training drone policies for car-following, surveillance, search tasks

### Comparable Published Work at Top Venues

- **CARLA**: "CARLA: An Open Urban Driving Simulator" (CoRL 2017)
- **AirSim**: "AirSim: High-Fidelity Visual and Physical Simulation for Autonomous Vehicles" (FSR 2017)
- **Habitat**: "Habitat: A Platform for Embodied AI Research" (ICCV 2019)
- **ISAAC Sim**: "Isaac Sim: An Extensible Robotics Simulator" (ICRA 2022)
- **MetaDrive**: "MetaDrive: Composing Diverse Driving Scenarios for Generalizable RL" (TPAMI 2022)

CarlaAir fills the gap between driving simulators and aerial simulators, enabling a new class of research at the intersection of autonomous driving, drone navigation, and multi-agent systems.

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
