# Map Memory Node

Should contain the following ROS constructs:

1 Subscriber that subscribes to the '/costmap' topic for nav_msgs::msg::OccupancyGrid messages
1 Subscriber that subscribes to the '/odom/filtered' topic for nav_msgs::msg::Odometry messages
1 Publisher that publishes nav_msgs::msg::OccupancyGrid messages to a '/map' topic
1 Timer that is used to limit the number of times the map is updated (this is for optimization, we don't need to update the map everytime we get a costmap)

This node is a combination of world modeling and memory. Given that we know where we can and cannot go at a given time, we can infer the inehrent structure of the entire map as we move the robot around. This is useful because we can stop the robot from retracing its steps and planning a new route that goes through obstacles that we previously detected.

To reduce the frequency of map updates, you can design the mapping node to only fuse maps when the robot has moved a set distance. For the explanations below, we use 1.5m.

## Steps to Build and Update the Map

### Subscribe to /costmap Topic

Receive nav_msgs::msg::OccupancyGrid messages from the /costmap topic.
Each costmap represents the local environment as detected by the robot at a specific moment.

### Subscribe to /odom/filtered Topic

Receive nav_msgs::msg::Odometry messages from the /odom/filtered topic.
Use this data to track the robot’s movement and determine when it has moved 5 meters since the last map update.

### Track Robot Movement

Store the robot’s initial position and periodically compute its distance from the last update position:
distance = sqrt((x_current - x_last)² + (y_current - y_last)²)

### Aggregate Costmaps into the Global Map

Maintain a global OccupancyGrid that represents the map of the environment.
When the robot moves 1.5 meters, integrate the most recent costmap into the global map:
Transform the costmap into the global frame using the robot’s current position and orientation.
Update the global map by merging the transformed costmap into it, prioritizing new data over old data.

### Optimize with a Timer

Use a timer to limit the frequency of map updates to a reasonable rate (e.g., every 1 second).
Only perform a map update if the robot has moved 1.5 meters since the last update position.

### Publish the Map

Publish the aggregated global map as a nav_msgs::msg::OccupancyGrid message to the /map topic.

## Logic for Map Update

Check if the robot has moved 5 meters since the last map update.
If yes, aggregate the most recent costmap into the global map.
Publish the updated map.

## Linear Fusion of Costmaps

When integrating costmaps:

If a cell in the new costmap has a known value (occupied or free), overwrite the corresponding cell in the global map.
If a cell in the new costmap is unknown, retain the previous value in the global map.

## Key Points

The global map is updated only when the robot moves at least 5 meters and at a controlled frequency.
The node uses odometry to track movement and costmaps to update the environment representation.
A timer ensures that updates do not occur too frequently, optimizing computational resources.
