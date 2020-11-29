#include "ros/ros.h"
#include "ball_chaser/DriveToTarget.h"
#include <sensor_msgs/Image.h>

// Define a global client that can request services
ros::ServiceClient client;

// This function calls the command_robot service to drive the robot in the specified direction
void drive_robot(float lin_x, float ang_z)
{
  // TODO: Request a service and pass the velocities to it to drive the robot
    ball_chaser::DriveToTarget srv;
    srv.request.linear_x = lin_x;
    srv.request.angular_z = ang_z;
    ROS_INFO("Moving the robot to the white ball with lin_x : %1.2f, and ang_z : %1.2f", lin_x, ang_z);

    if (!client.call(srv))
        ROS_ERROR("Failed to call service drive_to_target");
}

// This callback function continuously executes and reads the image data
void process_image_callback(const sensor_msgs::Image img)
{

    int white_pixel = 255;
  	bool ball_found = false;
  	int ball_location = -1;

    // TODO: Loop through each pixel in the image and check if there's a bright white one
    // Then, identify if this pixel falls in the left, mid, or right side of the image
    // Depending on the white ball position, call the drive_bot function and pass velocities to it
    // Request a stop when there's no white ball seen by the camera

    // Loop through each pixel in the image and check if its equal to the first one
    for (int i = 0; i < (img.height * img.step); i=i+3)
    {
        if (img.data[i] == white_pixel &&
            img.data[i+1] == white_pixel &&
            img.data[i+2] == white_pixel)
        {
            ball_found = true;
          	ball_location = int(i/3) % img.width;
            break;
        }
    }


    if (!ball_found){
        // the ball is not found
    	drive_robot(0.0, 0.0);
    	ROS_ERROR("No white ball detected - Did you insert the white ball in your world?");
    } else
    {
      // Ball Location - Split the image window into 3 regions
      if (ball_location < int(img.width/3))
      {

        drive_robot(0, 0.5); //turning left since the ball is located on the left side
        ROS_INFO("The white ball is detected on the left side of the robot.");

      } else if (ball_location >= int(img.width/3) && ball_location <= 2*int(img.width/3))
      {

      	drive_robot(1, 0.0); //moving straight since the ball is in front
        ROS_INFO("The white ball is detected in front of the robot.");

      } else if (ball_location > 2*int(img.width/3))
      {

        drive_robot(0.0, -0.5); //turning right since the ball is located on the right side
        ROS_INFO("The white ball is detected on the right side of the robot.");

      }

    }

}

int main(int argc, char** argv)
{
    // Initialize the process_image node and create a handle to it
    ros::init(argc, argv, "process_image");
    ros::NodeHandle n;

    // Define a client service capable of requesting services from command_robot
    client = n.serviceClient<ball_chaser::DriveToTarget>("/ball_chaser/command_robot");

    // Subscribe to /camera/rgb/image_raw topic to read the image data inside the process_image_callback function
    ros::Subscriber sub1 = n.subscribe("/camera/rgb/image_raw", 10, process_image_callback);

    // Handle ROS communication events
    ros::spin();

    return 0;
}
