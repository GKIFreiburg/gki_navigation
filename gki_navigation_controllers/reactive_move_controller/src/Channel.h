/*
 * Channel.h
 *
 *  Created on: Mar 24, 2015
 *      Author: turtlebot
 */

#ifndef SRC_CHANNEL_H_
#define SRC_CHANNEL_H_

#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>
#include <visualization_msgs/MarkerArray.h>

double degree_to_radians(double degree);
double radians_to_degree(double radians);

class Channel {
public:
	Channel(double angle, double width);
	virtual ~Channel();

	void laser_update(sensor_msgs::LaserScanConstPtr msg);
	double compute_score();
	void get_velocity(geometry_msgs::Twist& velocity);
	void visualize(visualization_msgs::MarkerArray& vis_msg, const std::string& frame_id, int marker_id);

private:
	double length;
	double width;
	double angle;
	double max_velocity;
	double max_length;
	double max_angular_velocity;
};

#endif /* SRC_CHANNEL_H_ */
