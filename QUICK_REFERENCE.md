# CarlaAir Quick Reference: Typing & Linting

## ✅ What Was Improved

### 1. **Type Annotations**
- All functions in `auto_traffic.py` now have full type annotations
- Navigation agents use modern Python 3.9+ syntax (`list[...]`, `X | None`)
- `TypedDict` for configuration dictionaries
- `TYPE_CHECKING` guards to avoid runtime import overhead

### 2. **Enums**
- `ActorCategory` - vehicle, walker, controller
- `WalkerSpeedMode` - walk, run
- All magic numbers replaced with named constants

### 3. **Dataclasses**
```python
@dataclass(frozen=True, slots=True)
class WalkerEntry:
    walker_id: ActorId
    controller_id: ActorId | None = None
```

### 4. **Pydantic v2 Models**
- `SimulationConfig` - full simulation settings with validation
- `TrafficManagerConfig` - TM parameters
- `WalkerSpawnConfig` - walker spawning settings
- `VehicleSpawnConfig` - vehicle spawning settings
- All models use `Field()` with constraints (`gt=0`, `ge=0`, etc.)

### 5. **No Magic Numbers**
All constants are now named and documented:
```python
_DEFAULT_VEHICLE_COUNT: int = 10
_CONNECTION_TIMEOUT: float = 20.0
_VEHICLE_STALL_SPEED_THRESHOLD: float = 0.01
# ... etc
```

### 6. **Linting**
- **Ruff**: ✅ All checks passed (0 errors)
- **MyPy**: Configured for strict checking
- **Ty**: Configured (requires carla stubs for full checking)

## 🛠️ Usage

### Run Linting
```bash
cd PythonAPI/carla
python3 -m ruff check agents/ --config pyproject.toml
```

### Run Type Checking
```bash
# MyPy
python3 -m mypy agents/

# Ty (requires carla module)
python3 -m ty check agents/
```

### Use Pydantic Configs
```python
from auto_traffic import SimulationConfig

# Create with defaults
config = SimulationConfig()

# Create with custom values
config = SimulationConfig(
    vehicle_count=30,
    walker_count=50,
    carla_port=2000,
    tm_port=8000,
)

# Validation errors are caught early
try:
    bad_config = SimulationConfig(vehicle_count=-5)
except ValueError as e:
    print(f"Invalid config: {e}")
```

## 📚 Files Modified

1. `auto_traffic.py` - Complete rewrite with typing
2. `PythonAPI/carla/pyproject.toml` - Enhanced linting config
3. `PythonAPI/carla/agents/tools/misc.py` - Fixed imports, added public alias
4. `PythonAPI/carla/agents/navigation/basic_agent.py` - Type-safe options handling
5. `PythonAPI/carla/agents/navigation/behavior_agent.py` - Improved imports
6. `PythonAPI/carla/agents/navigation/constant_velocity_agent.py` - Type-safe options

## 📖 Documentation

See `IMPROVEMENTS.md` for detailed information about:
- All Pydantic models created
- Constants extracted and categorized
- Type-safe patterns implemented
- Linting rules configured
- Best practices enforced

## 🎯 Key Benefits

✅ **Type safety** - Catches errors at development time  
✅ **Self-documenting** - Types and validation describe intent  
✅ **No magic numbers** - All constants named and documented  
✅ **Immutable configs** - Frozen Pydantic models  
✅ **Validated inputs** - Field constraints catch bad values early  
✅ **Clean linting** - Ruff passes with 0 errors  
✅ **Modern Python** - Uses 3.9+ features
