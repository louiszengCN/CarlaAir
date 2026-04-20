#!/bin/bash
# CARLA-Air × ROS 2 launcher — source this before running examples/ros2/*.py
#
# Strategy: ROS 2 Humble uses system Python 3.10 at /opt/ros/humble.
# Conda env `carlaAir` (python 3.10) has `carla` and `airsim` installed.
# We keep ROS's system interpreter and prepend the conda env's site-packages
# to PYTHONPATH so carla/airsim imports resolve without contaminating conda.

if [ -z "$BASH_VERSION" ] && [ -z "$ZSH_VERSION" ]; then
    echo "Please source this file with bash/zsh: source ros2_env.sh"
    return 1 2>/dev/null || exit 1
fi

# ---------- ROS 2 Humble ----------
if [ -f /opt/ros/humble/setup.bash ]; then
    source /opt/ros/humble/setup.bash
else
    echo "[ERR] /opt/ros/humble/setup.bash not found — is ROS 2 Humble installed?"
    return 1 2>/dev/null || exit 1
fi

# ---------- conda env packages (carla, airsim) ----------
CARLA_ENV_SITE="$HOME/miniconda3/envs/carlaAir/lib/python3.10/site-packages"
if [ ! -d "$CARLA_ENV_SITE" ]; then
    # fall back to anaconda
    CARLA_ENV_SITE="$HOME/anaconda3/envs/carlaAir/lib/python3.10/site-packages"
fi
if [ -d "$CARLA_ENV_SITE" ]; then
    export PYTHONPATH="$CARLA_ENV_SITE:$PYTHONPATH"
else
    echo "[WARN] carlaAir conda env site-packages not found — carla/airsim imports will fail"
    echo "       expected at: \$HOME/miniconda3/envs/carlaAir/lib/python3.10/site-packages"
fi

# Ensure RMW DDS fallback is the default (some hosts have no custom DDS)
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"

# IMPORTANT: rclpy C extension is compiled for Python 3.10 on Humble, but some
# shells default to a conda / pyenv python3 that breaks it. Always use the
# system python3.10. Export CARLAAIR_PY so scripts know which interpreter to use.
export CARLAAIR_PY=/usr/bin/python3.10
alias py-ros2='/usr/bin/python3.10'

echo "[OK] ROS 2: humble (\$ROS_DISTRO=$ROS_DISTRO)"
echo "[OK] Python: $CARLAAIR_PY"
echo "[OK] PYTHONPATH includes: $CARLA_ENV_SITE"
echo ""
echo "     Run scripts as:  \$CARLAAIR_PY examples/ros2/<script>.py"
