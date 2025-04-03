# UWU Blobfish Scripts

## Key Scripts

- `install.sh`: Prepares ROS2 environment and necessary tools that are beyond the scope of the project dependencies or `rosdep`.

## Convenience Scripts

- `build.sh`: Installs project dependencies, the `requirements.txt` hack, and builds all packages.
  - Specify package names to build only those packages.
  - Or specify `--full`, `--thirdparty` or `--src` to build all packages, `/src` packages, or `/thirdparty` packages.
  - See `build.sh --help` for more options.
- `activate.sh`: Sources the project ROS overlay in a new subshell.
  - `deactivate`/`exit` to exit the subshell.
  - `launch` is shortcut for `ros2 launch`.
  - `reload` to reload the overlay after changes.
  - `build` to run `build.sh` and `reload` in one command. Package names and build set can be specified.
