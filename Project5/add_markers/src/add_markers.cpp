#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Vector3.h>
#include <cmath>
#include <string>


class handle_add_markers {
	public:
		handle_add_markers() {
			marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);
			marker_sub = n.subscribe("marker_position", 1, &handle_add_markers::marker_position_callback, this);
			amcl_pose_sub = n.subscribe("amcl_pose", 1, &handle_add_markers::amcl_pose_callback, this);
		}
		
		// function to place or delete a visual marker
		void marker_action(double x, double y, std::string action) {

			// Marker type cube
		  uint32_t shape = visualization_msgs::Marker::CUBE;

			visualization_msgs::Marker marker;
			// Set the frame ID and timestamp.
			marker.header.frame_id = "map";
			marker.header.stamp = ros::Time::now();

			// Set the namespace and id for this marker.
			marker.ns = "basic_shapes";
			marker.id = 0;

			// Set the marker type.
			marker.type = shape;

			// Set the marker action
			if (action == "show") {
				marker.action = visualization_msgs::Marker::ADD;
			}
			else if (action == "hide") {
				marker.action = visualization_msgs::Marker::DELETE;
			}

			// Set the pose of the marker
			marker.pose.position.x = x;
			marker.pose.position.y = y;
			marker.pose.position.z = 0;
			marker.pose.orientation.x = 0.0;
			marker.pose.orientation.y = 0.0;
			marker.pose.orientation.z = 0.0;
			marker.pose.orientation.w = 1.0;

			// Set the scale of the marker
			marker.scale.x = 0.25;
			marker.scale.y = 0.25;
			marker.scale.z = 0.25;

			// Set the color
			marker.color.r = 0.0f;
			marker.color.g = 1.0f;
			marker.color.b = 0.0f;
			marker.color.a = 1.0;

			// publish to the visualization_marker topic
			marker.lifetime = ros::Duration();
			marker_pub.publish(marker);
  	}

		// function to decide if robot in range of destination/marker position
		bool is_robot_close() {
	
			// range for robot and destination positions to show the marker
			double eps = 0.10;

			if (sqrt(pow(amcl_x - marker_x, 2) + pow(amcl_y - marker_y, 2)) < eps) {
				return true;
			}
			else {
				return false;
			}
		}

		// amcl pose subscriber callback function
		void amcl_pose_callback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg) {

			amcl_x = (double)msg->pose.pose.position.x;
			amcl_y = (double)msg->pose.pose.position.y;	

			// handle the pick-up position
			if (target_type == 1) {
				if ( !is_robot_close() ) {
					marker_action(marker_x, marker_y, "show");
				}
				else {
					marker_action(marker_x, marker_y, "hide");
				}
			}
      
			// handle the drop-off position
			if (target_type == 2) {
				if ( is_robot_close() ) {
					marker_action(marker_x, marker_y, "show");
				}
				else {
					marker_action(marker_x, marker_y, "hide");
				}
			}
		}
		
		// marker position subscriber callback function 
		void marker_position_callback(const geometry_msgs::Vector3::ConstPtr& msg) {
			
			marker_x = (double)msg->x;
			marker_y = (double)msg->y;
			target_type += 1;
			if (target_type ==1){
				ROS_INFO("Marker position is received at pick-up position");
			}
			else if (target_type==2){
				ROS_INFO("Marker position is received at drop-off position");			
			}
		}

	private:
		ros::NodeHandle n;
		ros::Publisher marker_pub;
		ros::Subscriber marker_sub;
		ros::Subscriber amcl_pose_sub;
		double amcl_x;
		double amcl_y;
		double marker_x;
		double marker_y;
		int target_type = 0;
	};


int main( int argc, char** argv )
{
	ros::init(argc, argv, "add_markers");

	handle_add_markers add_markers;

	ros::spin();

	return 0;
  
}
