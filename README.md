# WATonomous ASD Training - Autonomous Navigation

## Objective

Implement a complete autonomous navigation system for a differential drive robot using ROS2. The robot navigates from point A to point B while avoiding obstacles using only LiDAR sensor data.

## System Architecture

The navigation system consists of 4 nodes:

1. **Costmap Node** - Converts LiDAR scans into local occupancy grids with obstacle inflation
2. **Map Memory Node** - Aggregates local costmaps into a persistent global map
3. **Planner Node** - Generates collision-free paths using A* algorithm
4. **Control Node** - Follows planned paths using Pure Pursuit controller

## How to Run

### Prerequisites
- Docker installed
- Linux, macOS, or WSL2

### Build and Launch

```bash
# Build the system
./watod build

# Start all nodes and simulation
./watod up

# Stop the system
./watod down
```

### Set Navigation Goals

1. Open Foxglove at http://localhost:8080
2. In the 3D panel, click the "Publish" button
3. Select "Publish point"
4. Hold **Shift** and **click** on the map to set a goal
5. Watch the robot navigate autonomously!

## Goal Coordinates

Valid goal range: **-20m to +20m** in both X and Y

Example goals:
- `(5, 5)`
- `(10, -10)`
- `(-15, 15)`

## Topics

- `/lidar` - LiDAR sensor data
- `/costmap` - Local occupancy grid
- `/map` - Global map
- `/goal_point` - Navigation goals
- `/path` - Planned path
- `/cmd_vel` - Velocity commands

## Implementation

All navigation logic is implemented in `src/robot/`:
- `costmap/` - Obstacle detection and inflation
- `map_memory/` - Global map management
- `planner/` - A* pathfinding with adaptive planning
- `control/` - Pure Pursuit path following
