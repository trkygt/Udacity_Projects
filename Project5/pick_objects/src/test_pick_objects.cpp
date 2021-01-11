#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

// Define a client for to send goal requests to the move_base server through a SimpleActionClient
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;


int main(int argc, char** argv){
  // Initialize the simple_navigation_goals node
  ros::init(argc, argv, "test_pick_objects");

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
  goal.target_pose.pose.position.x = 0.0;
  goal.target_pose.pose.position.y = 7.25;
  goal.target_pose.pose.orientation.w = 1;

   // Send the goal position and orientation for the robot to reach
  ROS_INFO("Sending pick_goal");
  ac.sendGoal(goal);

  // Wait an infinite time for the results
  ac.waitForResult();
  
  // Check if the robot reached its goal   
  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
  {
    ROS_INFO("The robot reached the pick-up position, waiting for five seconds to pick it up.");
    ros::Duration(5.0).sleep();
        
    // Define a position and orientation for the robot to reach as its dropoff pose
    goal.target_pose.pose.position.x = -4.25;
    goal.target_pose.pose.position.y = 7.25;
    goal.target_pose.pose.orientation.w = 1;
    
    // Send the goal position and orientation for the robot to reach
    ROS_INFO("Five seconds completed, sending the drop-off goal.");
    ac.sendGoal(goal);

    // Wait an infinite time for the results
    ac.waitForResult();

    if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
      ROS_INFO("The robot reached the drop-off position ");
    else
      ROS_INFO("The robot failed to reach drop-off position for some reason");
  }
  else
    ROS_INFO("The robot failed to go to the pick-up position for some reason");
  
  return 0;
}
