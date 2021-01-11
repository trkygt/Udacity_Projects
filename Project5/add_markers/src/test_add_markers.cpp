#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Vector3.h>
#include <cmath>
#include <string>


class pub_add_marker {
	public:
		pub_add_marker() {
			marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);
		}
		
			// function to place or delete a visual marker
			void add_marker(double x, double y, std::string action) {

			// Marker type cube
		        uint32_t shape = visualization_msgs::Marker::CUBE;

			visualization_msgs::Marker marker;
			// Set the frame ID and timestamp.
			marker.header.frame_id = "map";
			marker.header.stamp = ros::Time::now();

			// Set the namespace and id for this marker.
			marker.ns = "add_markers";
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

		void test() {
			
			// wait for subscriber to the "visualization_marker" topic
			while (marker_pub.getNumSubscribers() < 1) {
				// wait
			}
			ROS_INFO("Showing marker at the pick_up position");
			add_marker(0.0, 7.25, "show");
			ros::Duration(5.0).sleep();

			ROS_INFO("Hiding the marker");
			add_marker(0.0, 7.25, "hide");
			ros::Duration(5.0).sleep();

			ROS_INFO("Showing marker at the drop-off position");
			add_marker(-4.25, 7.25, "show");
			ros::Duration(5.0).sleep();
		}

	private:
		ros::NodeHandle n;
		ros::Publisher marker_pub;
	};


int main( int argc, char** argv )
{
	ros::init(argc, argv, "test_add_markers");

	pub_add_marker add_markers;
	
	// add markers as described
	add_markers.test();

	ros::spin();

	return 0;
  
}
