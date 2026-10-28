# SDK examples

Standalone programs built on [unitree_sdk2](https://github.com/unitreerobotics/unitree_sdk2).

| File | Description |
|---|---|
| `go2_comp.cpp` | Complementary filter running on the robot |
| `go2_obstacle.cpp` | Obstacle avoidance and return to the initial position |
| `go2_compstacle.cpp` | Both combined: returns to the start using the filtered pose |

## Build

Copy the files into `unitree_sdk2/example/` and register each one in its `CMakeLists.txt`:

```cmake
add_executable(go2_comp go2_comp.cpp)
target_link_libraries(go2_comp unitree_sdk2)
```

Then, from `unitree_sdk2/build`:

```bash
cmake ..
make
```

The binaries are in `unitree_sdk2/build/bin`. Pass the network interface connected to the robot as the first argument.
