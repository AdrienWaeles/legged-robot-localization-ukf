# Lab session

Sources of the first lab session (`TP1`): an Unscented Kalman Filter fed by the Go2 odometry, and a controller that
drives the robot through test trajectories (forward and back, rotations).

| Path | Content |
|---|---|
| `TP1/include/` | Headers: `ukf_filter.hpp`, `odometry_tracker.hpp`, `unitree_controller.hpp` |
| `TP1/src/` | Implementation and the `main.cpp` test sequence |

## Build

Requires [`unitree_sdk2`](https://github.com/unitreerobotics/unitree_sdk2) built on your machine, plus Eigen 3.

```bash
cd lab-session/TP1
cmake -S . -B build -DUNITREE_SDK_PATH=<path to unitree_sdk2>
cmake --build build
```

## Run

Pass the network interface connected to the robot:

```bash
./build/unitree_go2_localization <network_interface>
```

The controller does not read the robot state yet: the tracked pose stays at zero, so the reported errors are not
meaningful until state feedback is implemented.

The program moves the robot. Use a clear area and keep the remote control within reach.
