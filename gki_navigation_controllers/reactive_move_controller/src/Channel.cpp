/*
 * Channel.cpp
 *
 *  Created on: Mar 24, 2015
 *      Author: turtlebot
 */

#include "reactive_move_controller/Channel.h"
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
	robot_length = 0.35;
	this->goal_distance = 0;
}

Channel::~Channel() {
}

void Channel::laser_update(const sensor_msgs::LaserScan& laser_scan, double goal_distance, bool goal_reached)
{
	this->goal_distance = goal_distance;
	length = max_length;


	std::vector<geometry_msgs::Point> laser_points;
	for (size_t index = 0; index < laser_scan.ranges.size(); index++)
	{

		if (laser_scan.ranges[index]>laser_scan.range_max || laser_scan.ranges[index]<laser_scan.range_min)
		{
			//ROS_INFO_STREAM("Fehler range:"<<laser_scan.ranges[index]<<" range_min: "<<laser_scan.range_min<<" range_max: "<<laser_scan.range_max);
		}
		else
		{

				laser_points.push_back(geometry_msgs::Point());
				geometry_msgs::Point& point = laser_points.back();
				point.y = sin(laser_scan.angle_min + laser_scan.angle_increment * index - angle) * laser_scan.ranges[index];
				point.x = cos(laser_scan.angle_min + laser_scan.angle_increment * index - angle) * laser_scan.ranges[index];
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

double Channel::compute_score(const double & goal_angle, bool goal_reached)
{
	double score = 0.0;
	double length_score = 0.0;
	if(length < robot_length)
	{
		length_score = 0.0;
	}
	else if(length > max_length)
	{
		length_score = 1.0;
	}
	else
	{
		length_score = (length - robot_length) / (max_length - robot_length);
	}
	double angle_score = 0.0;
	if (fabs(angle) > M_PI/2.0)
	{
		angle_score = 0.5;
	}
	else
	{
		angle_score = 1.0 - 0.5/90.0 *radians_to_degree(angle);
	}
	double goal_score = 0.0;
	if(fabs(goal_angle-angle) > M_PI/2.0)
	{
		goal_score = 0.5;
	}
	else
	{
		goal_score = (1.0 - 0.5/90.0 *radians_to_degree(goal_angle-angle));
	}
	if (goal_reached == true )
	{
		goal_score = 1.0;
	}
	score = length_score * angle_score * goal_score;
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
		double length_limit = max_velocity / (max_length-0.2) * (length - 0.2);
		if (goal_distance < length)
			length_limit = max_velocity * length/max_length;

		velocity.linear.x = fmin(angle_limit, length_limit);

	}
	velocity.angular.z = fmin(2*angle, max_angular_velocity);
}

void Channel::visualize(visualization_msgs::MarkerArray& vis_msg, const std::string& frame_id, int marker_id, const double & goal_angle, bool goal_reached)
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
    double score = compute_score(goal_angle, goal_reached);
    marker.color.r = 1.0 - score;
    marker.color.g = score;
    marker.color.b = 0.2;
    tf::Quaternion quat = tf::createQuaternionFromYaw(angle);
    tf::quaternionTFToMsg(quat, marker.pose.orientation);
    marker.scale.x = length;
    marker.scale.y = 0.05;
    marker.scale.z = 0.05;
}
void Channel::to_string()
{
	//ROS_INFO_STREAM("length, angle, goal_distance: "<<length<<"  "<<angle<<" "<<goal_distance);
}
