/*
 * Channel.cpp
 *
 *  Created on: Mar 24, 2015
 *      Author: turtlebot
 */

#include "Channel.h"
#include <tf/tf.h>

double degree_to_radians(double degree)
{
	return (degree / 180) * M_PI;
}

double radians_to_degree(double radians)
{
	return (radians / M_PI) * 180;
}

Channel::Channel(double angle, double width) {
	length = 0;
	this->width = width;
	this->angle = angle;
	max_velocity = 1;
	max_length = 3;
	max_angular_velocity = 0.75;
}

Channel::~Channel() {
}

void Channel::laser_update(sensor_msgs::LaserScanConstPtr msg)
{
	length = max_length;
	std::vector<geometry_msgs::Point> laser_points;
	for (size_t index = 0; index < msg->ranges.size(); index++)
	{

		if (msg->ranges[index]>msg->range_max || msg->ranges[index]<msg->range_min)
		{
			//ROS_INFO_STREAM("Fehler range:"<<msg->ranges[index]<<" range_min: "<<msg->range_min<<" range_max: "<<msg->range_max);
		}
		else
		{

				laser_points.push_back(geometry_msgs::Point());
				geometry_msgs::Point& point = laser_points.back();
				point.y = sin(msg->angle_min + msg->angle_increment * index - angle) * msg->ranges[index];
				point.x = cos(msg->angle_min + msg->angle_increment * index - angle) * msg->ranges[index];
				//ROS_INFO_STREAM("point: "<<point.x<<" "<<point.y<<" "<<point.z);
		}
	}
	for (size_t index = 0 ; index < laser_points.size(); index++)
	{
		geometry_msgs::Point& point = laser_points[index];
		if (point.x < length && point.y <= width / 2.0 && point.y >= -width / 2.0 && point.x > 0)
		{
			length = point.x;
		}
	}
	//ROS_INFO_STREAM("length: "<<length);
}

double Channel::compute_score()
{
	double score = 0.0;
	double length_score = 0.0;
	if(length < 0.35)
	{
		length_score = 0.0;
	}
	else if(length > 3)
	{
		length_score = 1;
	}
	else
	{
		length_score = (length - 0.35) / 2.65;
	}
	double angle_score = 0.0;
	if (fabs(angle) > M_PI/2.0)
	{
		angle_score = 0.5;
	}
	else
	{
		angle_score = 1 - 0.5/90 *radians_to_degree(angle);
	}
	score = length_score * angle_score;
	return score;
}

void Channel::get_velocity(geometry_msgs::Twist& velocity)
{
	if(radians_to_degree(fabs(angle))>45)
	{
		velocity.linear.x = 0.0;
	}
	else
	{
		double angle_limit = -max_velocity/45.0 * radians_to_degree(fabs(angle)) + max_velocity;
		double length_limit = max_velocity * length/max_length;
		velocity.linear.x = fmin(angle_limit, length_limit);

	}
	velocity.angular.z = fmin(2*angle, max_angular_velocity);
}

void Channel::visualize(visualization_msgs::MarkerArray& vis_msg, const std::string& frame_id, int marker_id)
{
    vis_msg.markers.push_back(visualization_msgs::Marker());
    visualization_msgs::Marker& marker = vis_msg.markers.back();
    marker.ns = "channels";
    marker.id = marker_id;
    marker.header.frame_id = frame_id;
	if (length < 0.01)
	{
		//ROS_INFO_STREAM("length: "<<length);
	    marker.action = visualization_msgs::Marker::DELETE;
	    return;
	}
    marker.action = visualization_msgs::Marker::ADD;
    marker.type = visualization_msgs::Marker::ARROW;
    marker.color.a = 1.0;
    double score = compute_score();
    marker.color.r = 1.0 - score;
    marker.color.g = score;
    marker.color.b = 0.2;
    tf::Quaternion quat = tf::createQuaternionFromYaw(angle);
    tf::quaternionTFToMsg(quat, marker.pose.orientation);
    marker.scale.x = length;
    marker.scale.y = 0.05;
    marker.scale.z = 0.05;
}
