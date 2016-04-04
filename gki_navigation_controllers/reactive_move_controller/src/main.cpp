#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/MarkerArray.h>
#include <vector>
#include <math.h>

ros::Subscriber laser_subscriber;
ros::Publisher visualization_publisher;
ros::Publisher velocity_publisher;

void laser_callback(sensor_msgs::LaserScanConstPtr msg)
{
	visualization_msgs::MarkerArray vis_msg;
    // TODO: create behavior
    msg->angle_min; // starting angle
    msg->angle_max; // ending angle
    msg->angle_increment; // angular resolution
    msg->ranges; // sensor data list


    vis_msg.markers.push_back(visualization_msgs::Marker());
    visualization_msgs::Marker& marker = vis_msg.markers.back();
    marker.type = visualization_msgs::Marker::POINTS;
    marker.header.frame_id = msg->header.frame_id;
    marker.color.a = 1.0;
    marker.color.r = 0.2;
    marker.color.g = 1.0;
    marker.color.b = 0.2;
    marker.action = visualization_msgs::Marker::ADD;
    marker.ns = "xy laser";
    marker.pose.orientation.w = 1;
    marker.scale.x = 0.05;
    marker.scale.y = 0.05;
    marker.scale.z = 0.05;

    std::vector<geometry_msgs::Point> points;
    for (int index = 0; index < msg->ranges.size(); index++) {
    	double range = msg->ranges[index];
    	if (range < msg->range_max && range > msg->range_min) {
			double angle = msg->angle_min + msg->angle_increment * index;
			double x = cos(angle) * range;
			double y = sin(angle) * range;
			geometry_msgs::Point p;
			p.x = x;
			p.y = y;
			points.push_back(p);
			marker.points.push_back(p);
    	}
    }
    // useful funcitons:
    // sin(0.5);
    // cos(0.5);
    // hypot(a, b); hypotenuse in triangle abc, length of c
    // atan2(b, a); angle in triangle abc, angle alpha
    //ROS_INFO_STREAM("min angle: "<<msg->angle_min);

    double half_robot_width = 0.35;
    double length = 10;
    for(int index = 0; index < points.size(); index++) {
    	geometry_msgs::Point p = points[index];
    	if (p.y < half_robot_width && p.y > -half_robot_width) {
    		if (p.x < length) {
    			length = p.x;
    		}
    	}
    }

    geometry_msgs::Twist velocity;
	ROS_INFO_STREAM("length = "<<length);
    if (length > 0.5){
    	ROS_INFO_STREAM("fahren ");
    	velocity.linear.x = 0.2;
    }

    //velocity.linear.x = // forward
    //velocity.angular.z = // turn
    velocity_publisher.publish(velocity);
    visualization_publisher.publish(vis_msg);
}

int main(int argc, char** argv)
{
    // initialize
    ros::init(argc, argv, "reactive_move_controller");
    ros::NodeHandle nh;\
    //ROS_INFO_STREAM(0/0);


    ROS_INFO("subscribing to laser topic...");
    laser_subscriber = nh.subscribe("base_scan_filtered", 1, laser_callback);
    ROS_INFO("advertizing command velocity topic...");
    velocity_publisher = nh.advertise<geometry_msgs::Twist>("command_velocity", 1, false);
    visualization_publisher = nh.advertise<visualization_msgs::MarkerArray>("visualization", 1, false);
    ROS_INFO("reactive move controller initialized.");

    // loop
    ros::spin();
}
