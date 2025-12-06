This ROS 2 project consists of two nodes: one that calculates distances and safety conditions, and another that controls the movement of two turtles based on those conditions.<br>

distance_check node: Monitors the positions of turtle1 and turtle2. It calculates the distance between them and checks if they are too close to the window borders. It publishes a safety status (0 = Safe, 1 = Too Close, 2 = Wall Warning).<br>

ui node: Subscribes to the safety status and drives the turtles. It moves them autonomously and executes evasive maneuvers (stop, reverse, turn) if a safety warning is received.<br>

To run the code you have to open 4 terminal and launch the sequent commands:
- ros2 run turtlesim turtlesim_node
- ros2 run assignment1_rt turtle_spawn
- ros2 run assignment1_rt ui
- ros2 run assignment1_rt distance <br>
There is a folder contining a __init__.py file necessary to make the turtle_spawn.py file works in a c_make file.<br>
enjoy...
