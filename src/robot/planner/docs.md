# Planner Node

Should contain the following ROS constructs:

1 Subscriber that subscribes to the '/map' topic for nav_msgs::msg::OccupancyGrid messages
1 Subscriber that subscribes to the '/goal_point' topic for geometry_msgs::msg::PointStamped messages
1 Subscriber that subscribes to the '/odom/filtered' topic for nav_msgs::msg::Odometry messages
1 Publisher that publishes nav_msgs::msg::Path messages to a '/path' topic
1 Timer that tracks when we have reached the goal, or timedout, and we need to replan

The planner node corrosponds to a combination of configuration and action. Because the robots sole purpose is to move to a specified point, its configuration is very simple. Within the planner, you can model its configurator as a simple state machine that switches between two states: waiting for a goal, waiting for robot to reach the goal. These two states are sufficient enough for the task at hand. Furthermore, the planning node also produces a global plan from the robot's starting position to the final goal. It does this by using the discretized grid of the map and iterating through the grid to reach an optimal path. This plan is updated everytime the map updates.

To plan from one point in an incomplete map to another, there are a multiple of algorithms you can use. The simplest is Breadth First Search, which will inflate outwards from the robot's starting position, and end when it reaches the goal point. For this assignment, we suggest you make your life more interesting and learn how to use A\*.

<details>

<summary>
To give you an easier time with building A\*, here's some structs that could help you with building and traversing the graph:
</summary>

```cpp
// ------------------- Supporting Structures -------------------

// 2D grid index
struct CellIndex
{
  int x;
  int y;

  CellIndex(int xx, int yy) : x(xx), y(yy) {}
  CellIndex() : x(0), y(0) {}

  bool operator==(const CellIndex &other) const
  {
    return (x == other.x && y == other.y);
  }

  bool operator!=(const CellIndex &other) const
  {
    return (x != other.x || y != other.y);
  }
};

// Hash function for CellIndex so it can be used in std::unordered_map
struct CellIndexHash
{
  std::size_t operator()(const CellIndex &idx) const
  {
    // A simple hash combining x and y
    return std::hash<int>()(idx.x) ^ (std::hash<int>()(idx.y) << 1);
  }
};

// Structure representing a node in the A* open set
struct AStarNode
{
  CellIndex index;
  double f_score;  // f = g + h

  AStarNode(CellIndex idx, double f) : index(idx), f_score(f) {}
};

// Comparator for the priority queue (min-heap by f_score)
struct CompareF
{
  bool operator()(const AStarNode &a, const AStarNode &b)
  {
    // We want the node with the smallest f_score on top
    return a.f_score > b.f_score;
  }
};
```

</details>

## Node Behavior and Design

The planner operates as a state machine with two states:

Waiting for Goal: The node waits for a new goal point from the /goal_point topic.
Waiting for Robot to Reach Goal: Once a goal is received, the node plans a path using A\* and monitors the robot's progress toward the goal.

## Key Functionalities

### Global Plan

Uses the A\* algorithm to find the optimal path from the robot's current position to the goal on the occupancy grid (/map).
Pathfinding accounts for obstacle constraints and generates a path that minimizes the cost.

### State Transition

Transition from "waiting for goal" to "waiting for robot to reach the goal" occurs when a valid goal is received.
Transition back occurs when:
The goal is reached.
A timeout is reached.
The map updates, requiring a replan.

### Timer for Replanning

Checks for timeout or completion conditions periodically.
Replans if the robot fails to progress or if a new map update invalidates the previous plan.

## Steps to Implement

### Subscribers

/map: Receives nav_msgs::msg::OccupancyGrid messages.
Used as the grid for A\* pathfinding.
/goal_point: Receives geometry_msgs::msg::PointStamped messages.
Specifies the goal point.
/odom/filtered: Receives nav_msgs::msg::Odometry messages.
Tracks the robot's current position and progress toward the goal.

### Publisher

/path: Publishes nav_msgs::msg::Path messages.
Contains the planned path from the robot's position to the goal.

### Timer

Periodically checks if the goal is reached or if replanning is required.
Ensures efficient operation by limiting unnecessary updates.

### A\* Implementation

Use the occupancy grid as the search space.
Initialize start and goal points based on the robot’s position and the goal point.
Implement A\*:
Maintain an open list of nodes to be evaluated and a closed list of evaluated nodes.
Use a cost function: [ f(n) = g(n) + h(n) ]
( g(n) ): Cost to reach the node.
( h(n) ): Heuristic estimate of cost to reach the goal (e.g., Euclidean or Manhattan distance).
Expand nodes until the goal is reached or no valid path exists.

### State Machine

Define and handle transitions between "waiting for goal" and "waiting for robot to reach goal."

## Key Points

State Machine: Clear separation of "waiting for goal" and "waiting for robot to reach goal."
A\* Planning: Finds an optimal path on the discretized occupancy grid using heuristic cost estimation.
Replanning: Automatically triggered by timer-based checks for progress or map updates.
