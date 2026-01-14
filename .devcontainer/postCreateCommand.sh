#!/usr/bin/env bash
set -euo pipefail

WORKSPACE_DIR="${PWD}"

# Ensure ROS env is sourced for interactive shells.
BASHRC="$HOME/.bashrc"
if ! grep -q "/opt/ros/humble/setup.bash" "$BASHRC"; then
  {
    echo ""
    echo "# ROS 2 Humble"
    echo "source /opt/ros/humble/setup.bash"
    echo "# Source workspace overlay if it exists"
    echo "if [ -f \"${WORKSPACE_DIR}/install/setup.bash\" ]; then source \"${WORKSPACE_DIR}/install/setup.bash\"; fi"
  } >> "$BASHRC"
fi

# Mirror out-of-tree packages into the ROS workspace src/ so colcon discovers them.
mkdir -p "${WORKSPACE_DIR}/src"
if [ -d "${WORKSPACE_DIR}/duckietown_msgs" ] && [ ! -e "${WORKSPACE_DIR}/src/duckietown_msgs" ]; then
  ln -s "${WORKSPACE_DIR}/duckietown_msgs" "${WORKSPACE_DIR}/src/duckietown_msgs"
fi
if [ -d "${WORKSPACE_DIR}/tof_autonomous_example/tof_reader" ] && [ ! -e "${WORKSPACE_DIR}/src/tof_reader" ]; then
  ln -s "${WORKSPACE_DIR}/tof_autonomous_example/tof_reader" "${WORKSPACE_DIR}/src/tof_reader"
fi

# Install system dependencies for all packages (best-effort; don't fail hard on missing rosdep keys).
source /opt/ros/humble/setup.bash

if command -v rosdep >/dev/null 2>&1; then
  rosdep update || true
  rosdep install --from-paths "${WORKSPACE_DIR}/src" -i -y --rosdistro humble || true
fi

# Build workspace (generates duckietown_msgs Python modules and installs ament_python packages).
if command -v colcon >/dev/null 2>&1; then
  colcon build --symlink-install
fi

# Create a stable path for Pylance to find generated ROS Python modules.
# We link .ros_python -> first dist-packages/site-packages found under install/.
rm -f "${WORKSPACE_DIR}/.ros_python" || true
PY_SITE_PATH=""
if [ -d "${WORKSPACE_DIR}/install" ]; then
  PY_SITE_PATH=$(find "${WORKSPACE_DIR}/install" -type d \( -name "dist-packages" -o -name "site-packages" \) 2>/dev/null | head -n 1 || true)
fi
if [ -n "${PY_SITE_PATH}" ]; then
  ln -s "${PY_SITE_PATH}" "${WORKSPACE_DIR}/.ros_python"
else
  # Keep the path present so python.analysis.extraPaths doesn't point to a missing entry.
  mkdir -p "${WORKSPACE_DIR}/.ros_python"
fi

# Install your Python ROS packages as editable too (helps Pylance even before colcon overlay is sourced).
python3 -m pip install -e "${WORKSPACE_DIR}/src/blank_package" || true
python3 -m pip install -e "${WORKSPACE_DIR}/tof_autonomous_example/tof_reader" || true

echo "Devcontainer setup complete. Open a new terminal to pick up ROS env." 
