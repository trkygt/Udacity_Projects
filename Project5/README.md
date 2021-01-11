In this project, a home service robot is simulated on gazebo using a TURTLEBOT.

There is a list of shell scripts (See "shell\_scipts" folder on the main project directory.) to follow. 

Firstly, using turtlebot\_gazebo which calls turtlebot_navigation with gmapping packages, we created a map of the world that was created in the previous projects. Using SLAM and teleop\_twist\_keyboard package, the robot is navigated in the environment to complete a full map of the world. 

After that, navigation capabilities of the robot in the created map is tested by publishing 2D Navigation Goals on the rviz tool. This procedure is, then, automated by publishing multiple target positions by a "pick\_objects" node that generates "MoveBaseAction"s. We picked two different positions in the map for pick-up and drop-off locations and sent positions to the robot to navigate. Later, two virtual markers were visualized at those positions using "visualtion\_msgs".

Finally, the position of the robot by "amcl" has been subscribed to the "add_markers" node to simulate pickup/dropoff package scnerio as instructed. 
