#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <string>
#include <geometry_msgs/Vector3.h>

// Define a client for to send goal requests to the move_base server through a SimpleActionClient
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

void send_goal(double x, double y, std::string target) {

  //tell the action client that we want to spin a thread by default
  MoveBaseClient ac("move_base", true);

  // Wait 5 sec for move_base action server to come up
  while(!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
  }

  move_base_msgs::MoveBaseGoal goal;
  
  // set up the frame parameters
  goal.target_pose.header.frame_id = "map";
  goal.target_pose.header.stamp = ros::Time::now();

  // Define a position and orientation for the robot to reach
  goal.target_pose.pose.position.x = x;
  goal.target_pose.pose.position.y = y;
  goal.target_pose.pose.orientation.w = 1.0;

  // Send the goal position and orientation for the robot to reach
  ROS_INFO("Sending goal");
  ac.sendGoal(goal);

  // Wait an infinite time for the results
  ac.waitForResult();

  // Check if the robot reached its goal
  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    ROS_INFO("Robot successfully reached %s\n", target.c_str());
  else
    ROS_INFO("The base failed to reach goal");
}


int main(int argc, char** argv){
	
	// Initialize the pick_objects node
	ros::init(argc, argv, "pick_objects");

	ros::NodeHandle n;

	// publish the location of the object on a topic
	ros::Publisher marker_pub = n.advertise<geometry_msgs::Vector3>("marker_position", 1);
	geometry_msgs::Vector3 marker_msg;
	
	// pickup and dropoff locations
	double pickup_x = 0.0;
	double pickup_y = 7.25;
	double pickup_z = 0.0;

  	double dropoff_x = -4.25;
  	double dropoff_y = 7.25;
	double dropoff_z = 0.0;

	marker_msg.x = pickup_x;
	marker_msg.y = pickup_y;
	marker_msg.z = pickup_z;
	
	// block until there is a subscriber to marker_position topic
	ROS_INFO("Waiting for subscriber to marker_position topic");
	while (marker_pub.getNumSubscribers() < 1) {
		ROS_INFO("Waiting for the marker_position subscriber");
	}

	// publish pick-up position
	marker_pub.publish(marker_msg);

	// go to the pick-up position
  	send_goal(pickup_x, pickup_y, "pick-up position");

	ROS_INFO("Waiting for 5 seconds for robot the pick-up the package");
		
	//update message to the drop-off position
	marker_msg.x = dropoff_x;
	marker_msg.y = dropoff_y;
	marker_msg.z = dropoff_z;

	// publish drop-off position
	marker_pub.publish(marker_msg);

	ros::Duration(5).sleep();

  	// go to the dropoff location
  	send_goal(dropoff_x, dropoff_y, "drop-off position");


  	ros::Duration(5).sleep();  

  return 0;
}
