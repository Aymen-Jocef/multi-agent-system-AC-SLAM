# Multi-Robot Collaborative SLAM

ROS2 packages for multi-robot autonomous exploration and SLAM using the Chinese Postman Problem algorithm.

## Packages

- **robot_sim_gz**: Gazebo simulation, SLAM toolbox, and Nav2 launch files
- **cpp_explorer**: Chinese Postman Problem algorithm for autonomous exploration

## Installation

```bash
# Clone into your ROS2 workspace src folder
cd ~/ros2_ws/src
git clone https://github.com/YOUR_USERNAME/multi_robot_slam.git

# Install dependencies
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y

# Build
colcon build --packages-select cpp_explorer robot_sim_gz

# Source
source install/setup.bash
```

## Usage

### Launch all robots at once (recommended)
```bash
ros2 launch robot_sim_gz multi_robot_bringup.launch.py
```

This spawns 3 robots in Gazebo, each with their own SLAM, Nav2, and CPP Explorer nodes.

### Launch individual robots separately
```bash
# Terminal 1: Spawn robots in Gazebo
ros2 launch robot_sim_gz gz_multi_robot_launch.py

# Terminal 2: SLAM for robot1
ros2 launch robot_sim_gz online_async.launch.py namespace:=robot1

# Terminal 3: Nav2 for robot1
ros2 launch robot_sim_gz navigation_launch.py namespace:=robot1

# Terminal 4: Explorer for robot1
ros2 launch cpp_explorer cpp_explore.launch.py namespace:=robot1 graph_yaml_path:=/path/to/graph.yaml
```

Repeat for robot2, robot3 with different namespaces.

## Parameters

### cpp_explore.launch.py
| Parameter | Default | Description |
|-----------|---------|-------------|
| `namespace` | `robot1` | Robot namespace |
| `graph_yaml_path` | `config/graph.yaml` | Path to CPP graph file |
| `start_node` | `1` | Starting node ID |
| `use_sim_time` | `true` | Use simulation time |

### online_async.launch.py (SLAM)
| Parameter | Default | Description |
|-----------|---------|-------------|
| `namespace` | `robot1` | Robot namespace |
| `use_sim_time` | `true` | Use simulation time |

### navigation_launch.py (Nav2)
| Parameter | Default | Description |
|-----------|---------|-------------|
| `namespace` | `robot1` | Robot namespace |
| `params_file` | `config/nav2_params.yaml` | Nav2 parameters file |

## Dependencies

- ROS2 (Humble/Iron)
- Gazebo
- Nav2
- SLAM Toolbox
- Python packages: `networkx`, `pyyaml`, `numpy`
