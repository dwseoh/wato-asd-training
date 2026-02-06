# Control Node

Should contain the following ROS constructs:

1 Subscriber that subscribes to the '/path' topic for nav_msgs::msg::Path messages
1 Subscriber that subscribes to the '/odom/filtered' topic for nav_msgs::msg::Odometry messages
1 Publisher that publishes geometry_msgs::msg::Twist messages to a '/cmd_vel' topic
1 Timer to follow a stored path

The control node is the final piece of the puzzle. It is the final action layer that is between your robotics software, and moving the robot (well, there's also the hardware abstraction layer that actually takes what this node outputs and converts it into actual electrical signals to move the motors of the robot).

The field of controls is vast, and there are multiple different types of control schemes. As you go through university, you will probably learn about PID control. This form of control is used EVERYWHERE.

As of writing this assignment I was telling my dad about how I learned PID control, and how we can optimize signals in the Laplace domain, find parameters of a system with frequency analysis, blah, blah, blah. And he responded by telling me how he learned the same thing back in China, and uses it all the time to control temperatures and electrical signals in oil refineries.

Turns out you can use PID controls for following the path you just made. You simply calculate how far your robot deviates from the path (and its discrete derivative and integral), and use that to adjust your robot back onto the path. HOWEVER, PID controls is quite finnicky, so lets teach you a different control scheme.

Pure Pursuit Control is a geometric algorithm used to guide a robot along a predefined path. The robot selects a "lookahead point" on the path at a fixed distance ahead, and the algorithm calculates the steering angle needed to reach this point. This angle is based on the curvature of a circular arc connecting the robot’s current position to the target point.

The lookahead distance controls how far ahead the target point is chosen. A larger distance makes the motion smoother but less precise, while a smaller distance improves accuracy but can lead to sharper, less smooth movements. As the robot moves, the target point and steering angle are updated continuously, allowing the robot to follow the path dynamically.

You can read more about Pure Pursuit here.

## Node Behavior and Design

The Pure Pursuit Controller node is responsible for taking a path and odometry data as inputs and generating velocity commands to guide the robot along the path. The node operates in a closed-loop fashion: it continuously checks the robot's current position, calculates the desired heading based on the "lookahead point," and generates appropriate linear and angular velocity commands.

The node subscribes to:

/path: Receives the global path as a series of waypoints.
/odom/filtered: Tracks the robot's current position and orientation.
The node publishes:

/cmd_vel: Outputs velocity commands in the form of linear and angular velocities.
Additionally, the node employs a timer to ensure regular updates of the velocity commands.

## Key Functionalities

### Path Tracking

Extract the current lookahead point from the path based on the robot’s position and a fixed lookahead distance.
Compute the steering angle required to reach the lookahead point.

### Velocity Commands

Calculate linear and angular velocities using the steering angle and the robot’s dynamics.

### Error Handling

Handle edge cases like when the path is empty or the robot is close to the final goal.

### Timer for Updates

The node updates the velocity commands at a fixed rate (e.g., 10 Hz) for smooth and consistent control.

## Key Details

Lookahead Distance: The controller selects a point on the path at a fixed distance ahead of the robot. This distance must be tuned for the robot's dynamics and desired path-following performance.

Linear Speed: The robot’s speed is typically kept constant, while the angular velocity adjusts based on the curvature.

Edge Cases: Ensure the node handles scenarios like an empty path or reaching the final goal. You can stop the robot when the goal is within the goal_tolerance.

Yaw Calculation: Converting the robot’s quaternion orientation to yaw is essential for computing the steering angle.

This implementation provides a simple yet effective way to integrate Pure Pursuit Control in ROS for smooth path following.
