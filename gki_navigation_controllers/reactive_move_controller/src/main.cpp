#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Point.h>
#include <visualization_msgs/MarkerArray.h>
#include <vector>
#include <math.h>
#include "Channel.h"

ros::Subscriber laser_subscriber;
ros::Publisher velocity_publisher;
ros::Publisher visualization_publisher;
int stuck = 0;
int zufall = 0;

std::vector <Channel> laser_channels;

void laser_channel_callback(sensor_msgs::LaserScanConstPtr msg)
{
	visualization_msgs::MarkerArray vis_msg;

	// all channels update
	for (size_t index = 0; index < laser_channels.size(); index++)
    {
	    laser_channels[index].laser_update(msg);
	}
	// find best channel
	double score = 0;
	size_t channel_index = 0;
	for(size_t index = 0; index < laser_channels.size(); index++)
	{
		if(laser_channels[index].compute_score()>score)
		{
			score = laser_channels[index].compute_score();
			channel_index = index;
		}
	}
	// set velocity
	geometry_msgs::Twist velocity;

	laser_channels[channel_index].get_velocity(velocity);
	// visualize
	for (size_t index = 0; index < laser_channels.size(); index++)
	{
		laser_channels[index].visualize(vis_msg, msg->header.frame_id, index);
	}

    velocity_publisher.publish(velocity);
    visualization_publisher.publish(vis_msg);
}

void laser_callback(sensor_msgs::LaserScanConstPtr msg)
{
	visualization_msgs::MarkerArray vis_msg;
    // TODO: create behavior
    msg->angle_min; // starting angle
    msg->angle_max; // ending angle
    msg->angle_increment; // angular resolution
    msg->ranges; // sensor data list
    // useful funcitons:
    // sin(0.5);
    // cos(0.5);
    // hypot(a, b); hypotenuse in triangle abc, length of c
    // atan2(b, a); angle in triangle abc, angle alpha
    //ROS_INFO_STREAM("min angle: "<<msg->angle_min);
    std::vector<geometry_msgs::Point> laser_points;
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


    for (size_t index = 0; index < msg->ranges.size(); index++)
    {
    	laser_points.push_back(geometry_msgs::Point());
    	geometry_msgs::Point& point = laser_points.back();
    	if (msg->ranges[index]>msg->range_max || msg->ranges[index]<msg->range_min)
    	{
    		//ROS_INFO_STREAM("Fehler range:"<<msg->ranges[index]<<" range_min: "<<msg->range_min<<" range_max: "<<msg->range_max);
    	}
    	else
    	{
			point.y = sin(msg->angle_min + msg->angle_increment * index) * msg->ranges[index];
			point.x = cos(msg->angle_min + msg->angle_increment * index) * msg->ranges[index];
			//ROS_INFO_STREAM("point: "<<point.x<<" "<<point.y<<" "<<point.z);
			marker.points.push_back(point);
			marker.colors.push_back(marker.color);
    	}


    }
    bool links = false;
    bool rechts = false;
    bool vorne = true;
    double b = 0.3;
    double a = -0.3;
    double c = -10;
    double d = 10;
    for (size_t index = 0; index < laser_points.size(); index++)
    {
    	geometry_msgs::Point& point = laser_points[index];
    	if (point.y < 0.3 && point.y > 0 && point.x<0.5 && point.x > 0)
    	{
    		links=true;
    		if (point.y < b)
    		{
    			b = point.y;
    		}

    	}
    	if(point.y > 0.3 && point.x<0.5 && point.x > -0.5)
    	{
    		if (point.y < d)
    		{
    		    d = point.y;
    		}
    	}
    	if(point.y > -0.3 && point.y < 0 && point.x<0.5 && point.x > 0)
    	{
    		rechts=true;
    		if (point.y > a)
    		{
    		    a = point.y;
    		}
    	}

    	if(point.y < -0.3 && point.x<0.5 && point.x > -0.5)
    	{
    	    if (point.y > c)
    	    {
    	    	c = point.y;
    	   	}
     	}
    	if(point.x <0.5 && point.x > 0 && point.y < 0.3 && point.y > -0.3)
    	{
    		vorne=false;
    	}
    }

    geometry_msgs::Twist velocity;
    // translational velocity
    if(links)
    {
    	velocity.angular.z = -0.5;
    }
    if(rechts)
    {
    	velocity.angular.z = 0.5;
    }
    if(links==false && rechts==false)
    {
    	velocity.angular.z = 0.0;
    }
    if(links==true && rechts==true)
    {
    	ROS_INFO_STREAM("links und rechts sind true");
    	if(-a<b)
    	{
    		ROS_INFO_STREAM("1");
    		velocity.angular.z = 0.5;
    		if (!vorne)
    		{
    		   stuck = rand() % 30 +10;
    		   zufall = rand() % 2;
    		}
    	}
    	else
    	{
    		ROS_INFO_STREAM("2");
    		velocity.angular.z = -0.5;
    		if (!vorne)
    		{
    			stuck = rand() % 30 + 10;
    			zufall = rand() % 2;
    		}
    	}
    	if(-a==b)
    	{
    		ROS_INFO_STREAM("3");
    		velocity.angular.z = 0.0;
    	}
    }
    if(vorne)
    {
    	velocity.linear.x = 0.1;
    }
    else if(!links && !rechts)
    {
    	velocity.linear.x = 0.0;
    	if(-c<d)
    	{
    		velocity.angular.z = 0.5;
    	}
    	else
    	{
    		velocity.angular.z = -0.5;
    	}
    }
    if(velocity.angular.z==0 && velocity.linear.x==0)
    {

    	stuck = rand() % 30 + 10;
    	zufall = rand() % 2;

    }
    if (stuck>0)
    {
    	velocity.linear.x = 0.0;
    	if (zufall == 0)
    	{
    		velocity.angular.z = 0.5;
    	}
    	else
    	{
    		velocity.angular.z = -0.5;
    	}
    	stuck -= 1;
    	ROS_INFO_STREAM("stuck: "<<stuck);
    }
    // rotational velocity
    velocity_publisher.publish(velocity);
    visualization_publisher.publish(vis_msg);
}

int main(int argc, char** argv)
{
    // initialize
    ros::init(argc, argv, "reactive_move_controller");
    ros::NodeHandle nh;


	laser_channels.push_back(Channel(0.0, 0.5));
    for (size_t index = 1; index < 8; index++)
    {
    	double angle = index*15.0/180.0*M_PI;
    	laser_channels.push_back(Channel(angle, 0.5));
    	laser_channels.push_back(Channel(-angle, 0.5));
    }
    for (size_t index = 0; index < 3; index++)
    {
    	double angle = (index*15.0+7.5)/180.0*M_PI;
    	laser_channels.push_back(Channel(angle, 0.5));
    	laser_channels.push_back(Channel(-angle, 0.5));
    }
    ROS_INFO_STREAM("Channels: "<<laser_channels.size());

    ROS_INFO("subscribing to laser topic...");
    laser_subscriber = nh.subscribe("base_scan", 1, laser_channel_callback);
    ROS_INFO("advertizing command velocity topic...");
    velocity_publisher = nh.advertise<geometry_msgs::Twist>("command_velocity", 1, false);
    visualization_publisher = nh.advertise<visualization_msgs::MarkerArray>("visualization", 1, false);
    ROS_INFO("reactive move controller initialized.");

    // loop
    ros::spin();
}
