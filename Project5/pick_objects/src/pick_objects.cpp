#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

// Define a client for to send goal requests to the move_base server through a SimpleActionClient
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;


double pick_goal[3] = {0.0, 7.25, 1.0};
double drop_goal[3] = {-4.25, 7.25, 1.0};

int main(int argc, char** argv){
  // Initialize the simple_navigation_goals node
  ros::init(argc, argv, "pick_objects");

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

  // Define a position and orientation for the robot to reach as its pickup pose
  goal.target_pose.pose.position.x = pick_goal[0];
  goal.target_pose.pose.position.y = pick_goal[1];
  goal.target_pose.pose.orientation.w = pick_goal[2];

   // Send the goal position and orientation for the robot to reach
  ROS_INFO("Sending pick_goal");
  ac.sendGoal(goal);

  // Wait an infinite time for the results
  ac.waitForResult();
  bool pick_status = false;
  bool drop_status = false;
  // Check if the robot reached its goal
  if (!pick_status){ 
    if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
      ROS_INFO("The robot reached the pick-up position, waiting for five seconds to pick it up.");
      ros::Duration(5.0).sleep();
      pick_status = true; 
      
      // Define a position and orientation for the robot to reach as its dropoff pose
      goal.target_pose.pose.position.x = pick_goal[0];
      goal.target_pose.pose.position.y = pick_goal[1];
      goal.target_pose.pose.orientation.w = pick_goal[2];
      
      // Send the goal position and orientation for the robot to reach
      ROS_INFO("Sending drop_goal");
      ac.sendGoal(goal);

      // Wait an infinite time for the results
      ac.waitForResult();

      if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
        ROS_INFO("The robot reached the dropoff location ");
      else
        ROS_INFO("The robot failed to reach dropoff location for some reason");
    }
    else
      ROS_INFO("The robot failed to go to the pick-up position for some reason");
  }
  
  return 0;
}
