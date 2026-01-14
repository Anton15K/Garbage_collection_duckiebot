# Dev Container (ROS 2 Humble)

Open this repo in VS Code and run **Dev Containers: Reopen in Container**.

What you get:
- ROS 2 Humble base environment
- `colcon`, `rosdep`, common build tooling
- Python tooling (`ruff`, `black`, `isort`, `pytest`)
- Pylance configured to resolve:
  - your ament_python packages (`blank_package`, `tof_reader`)
  - generated Python modules from built interface packages (via `.ros_python` symlink)

Notes:
- The first build may take a while because `rosdep update` runs in the image build.
- Workspace is built in `postCreateCommand.sh` using `colcon build --symlink-install`.
