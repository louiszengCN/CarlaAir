# CarlaAir v0.1.7 Windows 发行版

这是面向 `Windows 11 x86_64` 的 CarlaAir 运行版说明。该发行版使用原生 Windows 打包产物、PowerShell 启动器和 Win64 Python wheel，不需要安装 UE4 或重新编译工程。

## 快速开始

在发行目录依次执行：

```bat
SetupEnv.bat
TestEnv.bat
StartCarlaAir.bat Town10HD
```

默认 Conda 环境名为 `carlaAir`。停止仿真器：

```bat
StopCarlaAir.bat
```

第一次排查建议不要启动自动交通：

```bat
StartCarlaAir.bat Town10HD --no-traffic
```

常用入口：

- `CarlaAir.ps1`：主启动器
- `StartCarlaAir.bat`：启动 CarlaAir
- `StopCarlaAir.bat`：停止 CarlaAir 与自动交通
- `SetupEnv.bat`：配置 Python 3.10 环境
- `TestEnv.bat`：检查依赖与端口
- `examples\`：Python 示例
- `examples_record_demo\`：轨迹录制与导演回放

## 同步采集 AirSim 图像

CARLA 同步模式下，不要在唯一的 tick 线程中依次执行 `world.tick()` 和 `simGetImages()`。AirSim 图像 RPC 会等待 Unreal 渲染帧，而下一帧又需要脚本继续调用 `world.tick()`，两者可能互相等待。

本发行版提供 `AirSimImageFetcher` 兼容层。运行验证示例：

```bat
StartCarlaAir.bat Town10HD --no-traffic
conda activate carlaAir
python examples\sync_airsim_images.py --frames 3 --output recordings\sync_airsim
```

自定义代码可以使用：

```python
from airsim_image_fetcher import AirSimImageFetcher

with AirSimImageFetcher.for_multirotor(
    host="127.0.0.1",
    port=41451,
    rpc_timeout=10.0,
) as fetcher:
    responses = fetcher.get_images_while_ticking(
        world,
        requests,
        timeout=10.0,
    )
```

限制：

- `world` 必须已经开启同步模式。
- 只能由一个线程或进程负责 `world.tick()`。
- 捕获超时后必须创建新的 fetcher。
- AirSim 图像不包含 CARLA frame ID，因此不能承诺与 CARLA 传感器严格原子级同帧。

更多启动和排查说明见 `STARTUP_GUIDE_CN.md` 与 `guide\FAQ.md`。
