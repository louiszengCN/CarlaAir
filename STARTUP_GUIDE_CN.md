# 启动教程

## 在哪里执行

要在 **工程根目录** 执行，也就是和这些文件同级的目录：

- `CarlaAir.ps1`
- `AirSimConfig\settings.json`
- `BuildWindows.ps1`

## 最基本启动方法

在工程根目录打开 PowerShell，执行：

```powershell
powershell -ExecutionPolicy Bypass -File .\CarlaAir.ps1 Town10HD
```

如果你已经在 PowerShell 里，并且当前目录就是工程根目录，也可以执行：

```powershell
.\CarlaAir.ps1 Town10HD
```

如果因为执行策略报错，优先使用前一种写法。

## 建议的排查启动方式

第一次排查时，建议先不要拉起自动交通：

```powershell
powershell -ExecutionPolicy Bypass -File .\CarlaAir.ps1 Town10HD --no-traffic
```

这样能先确认主程序和 `AirSim` 端口是否正常，再看交通流。

## 常用命令

### 启动默认地图

```powershell
powershell -ExecutionPolicy Bypass -File .\CarlaAir.ps1 Town10HD
```

### 启动并关闭自动交通

```powershell
powershell -ExecutionPolicy Bypass -File .\CarlaAir.ps1 Town10HD --no-traffic
```

### 指定分辨率

```powershell
powershell -ExecutionPolicy Bypass -File .\CarlaAir.ps1 Town10HD --res 1920x1080
```

### 指定 CARLA 主端口

```powershell
powershell -ExecutionPolicy Bypass -File .\CarlaAir.ps1 Town10HD --port 2000
```

### 指定画质

```powershell
powershell -ExecutionPolicy Bypass -File .\CarlaAir.ps1 Town10HD --quality High
```

### 前台模式

```powershell
powershell -ExecutionPolicy Bypass -File .\CarlaAir.ps1 Town10HD --fg --no-traffic
```

说明：

- `--fg` 会让当前 PowerShell 会话直接挂在游戏进程上
- 前台模式下默认不会自动拉起 `auto_traffic.py`
- 适合看即时报错，不适合日常使用

### 查看日志

```powershell
powershell -ExecutionPolicy Bypass -File .\CarlaAir.ps1 --log
```

### 停止 CarlaAir

```powershell
powershell -ExecutionPolicy Bypass -File .\CarlaAir.ps1 --kill
```

## 2000 和 41451 怎么判断

- `2000` 是 `CARLA` 主端口
- `41451` 是 `AirSim` 端口

判断方式：

1. 如果 `2000` 不正常
   先按主程序启动问题排查
2. 如果 `2000` 正常，但 `41451` 不正常
   先按 `AirSim` 初始化问题排查

## 最常见问题：41451 不正常

如果 `2000` 正常，但 `41451` 不正常，优先检查这几项。

### 1. 是否通过 CarlaAir.ps1 启动

必须确认是用：

```powershell
powershell -ExecutionPolicy Bypass -File .\CarlaAir.ps1 Town10HD --no-traffic
```

而不是直接双击 exe。

### 2. 检查真实 Documents 路径

在 PowerShell 中执行：

```powershell
[Environment]::GetFolderPath('MyDocuments')
```

记下输出路径。

### 3. 检查 AirSim 实际配置文件是否存在

然后检查这个真实路径下是否存在：

```text
<真实Documents路径>\AirSim\settings.json
```

### 4. 如果不存在，就手动复制

从工程根目录的：

```text
AirSimConfig\settings.json
```

复制到：

```text
<真实Documents路径>\AirSim\settings.json
```

然后重新通过 `CarlaAir.ps1` 启动。