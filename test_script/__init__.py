"""
test_script — CarlaAir integration test scripts.

Requires a running CarlaAir server (./carlaAir.sh) and AirSim (port 41451).

Available scripts
-----------------
diagnostic_test      — non-interactive pass/fail diagnostic for all features
drone_traj_col       — collect drone trajectories with AirSim control
drone_traj_playback  — replay recorded drone trajectories in CARLA
human_traj_col       — collect human-controlled walker trajectories
human_traj_playback  — replay recorded walker trajectories
test_drone_in_carla  — fly AirSim drone through CARLA Town01
"""

from __future__ import annotations
