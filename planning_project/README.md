# Planning Project

ROS Noetic implementation for Scenario B: Target Rescue.
Uses grid-based A* pathfinding, orienteering optimization, and Dubins curve path planning to rescue victims and reach the gate within a time limit.

## Build

```bash
catkin_make
source devel/setup.bash
```

## Launch

```bash
# Terminal 1: Start simulation
roslaunch loco_planning multiple_robots.launch

# Terminal 2: Run planning node
rosrun planning_project planning_node [--debug] [--mode combinatorial|sampling]
```

Flags:
- `--debug`: Enable debug output
- `--mode combinatorial`: Use combinatorial approach (default)
- `--mode sampling`: Use sampling-based approach

For fixed map testing:
```bash
roslaunch loco_planning multiple_robots.launch generate_new_config:=false
```

## Visualize Results

Grid map with Dubins trajectory:
```bash
cd plots
python3 plot_map.py ../results/map.json --trajectory ../results/trajectory.json
```

RRT tree and path:
```bash
cd plots
python3 plot_rrt.py ../results/rrt_tree.json --trajectory ../results/trajectory.json
```
