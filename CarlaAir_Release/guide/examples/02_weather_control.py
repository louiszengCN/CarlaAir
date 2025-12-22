#!/usr/bin/env python3
"""CarlaAir Quick Start - Step 2: 天气控制"""
import carla
import time

client = carla.Client('localhost', 2000)
world = client.get_world()

presets = {
    "晴天": {"sun_altitude_angle": 70, "cloudiness": 10},
    "多云": {"sun_altitude_angle": 50, "cloudiness": 80},
    "暴雨": {"precipitation": 100, "cloudiness": 90, "wetness": 100,
             "precipitation_deposits": 80, "wind_intensity": 50},
    "大雾": {"fog_density": 80, "fog_distance": 10, "cloudiness": 60},
    "日落": {"sun_altitude_angle": 5, "cloudiness": 30},
}

for name, params in presets.items():
    weather = carla.WeatherParameters()
    for k, v in params.items():
        setattr(weather, k, v)
    world.set_weather(weather)
    print(f"天气切换: {name}")
    time.sleep(3)

world.set_weather(carla.WeatherParameters.ClearNoon)
print("天气已恢复为晴天")
