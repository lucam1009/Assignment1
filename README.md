I have done two different nodes, the ui one that let me choose the turtle i want to control and the angular and linear velocity to give to that turtle. It is a node that subscribes to the second node I made and pushes to the two turtle.
The second node, the distance one, subscribe to the position of the two turtle and pushes the distance to the ui node so that node can evaluete wich state the turtle is in ( close to the other turtle, close to the wall, normal case).
To run the code you have to open 4 terminal and launch the sequent commands:
ros2 run turtlesim turtlesim_node
ros2 run assignment1_rt turtle_spawn
ros2 run assignment1_rt ui
ros2 run assignment1_rt distance
enjoy...
