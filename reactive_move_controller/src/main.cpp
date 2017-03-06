#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/MarkerArray.h>
#include <vector>
#include <math.h>
#include <angles/angles.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <kobuki_msgs/Sound.h>

ros::Subscriber laser_subscriber;
ros::Subscriber goal_subscriber;
ros::Publisher visualization_publisher;
ros::Publisher velocity_publisher;
ros::Publisher sound_publisher;

tf::TransformListener* tfl;

void goal_callback(geometry_msgs::PoseStampedConstPtr msg)
{
}


void laser_callback(sensor_msgs::LaserScanConstPtr msg)
{
	visualization_msgs::MarkerArray vis_msg;
//	// create behavior
//	msg->angle_min; // starting angle
//	msg->angle_max; // ending angle
//	msg->angle_increment; // angular resolution
//	msg->ranges; // sensor data list

	geometry_msgs::Twist velocity;
	//velocity.linear.x = // forward
	//velocity.angular.z = // turn
	
	velocity_publisher.publish(velocity);

	visualization_publisher.publish(vis_msg);
}


int main(int argc, char** argv)
{
	// initialize
	ros::init(argc, argv, "reactive_move_controller");
	ros::NodeHandle nh;
	tfl = new tf::TransformListener(ros::Duration(60));

	goal_subscriber = nh.subscribe("move_base_simple/goal", 1, goal_callback);
	ROS_INFO("subscribing to laser topic...");
	laser_subscriber = nh.subscribe("base_scan_filtered", 1, laser_callback);
	ROS_INFO("advertizing sound topic...");
	sound_publisher = nh.advertise<kobuki_msgs::Sound>("mobile_base/commands/sound", 1, false);
	ROS_INFO("advertizing command velocity topic...");
	velocity_publisher = nh.advertise<geometry_msgs::Twist>("/cmd_vel_mux/input/navi", 1, false);
	ROS_INFO("advertizing visualization topic...");
	visualization_publisher = nh.advertise<visualization_msgs::MarkerArray>("visualization", 1, false);
	ROS_INFO("move controller initialized.");

	// loop
	ros::spin();
}
