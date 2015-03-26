/*
 * ReactiveController.h
 *
 *  Created on: Mar 26, 2015
 *      Author: turtlebot
 */

#ifndef SRC_REACTIVECONTROLLER_H_
#define SRC_REACTIVECONTROLLER_H_
#include <nav_core/base_local_planner.h>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/MarkerArray.h>
#include <vector>
#include <math.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include "reactive_move_controller/Channel.h"

class ReactiveController : public nav_core::BaseLocalPlanner {
private:
	ros::Subscriber laser_subscriber;
	ros::Publisher visualization_publisher;
	tf::TransformListener* listener;
	sensor_msgs::LaserScan laser_scan;

	std::vector <Channel> laser_channels;
	std::vector<geometry_msgs::PoseStamped> plan;
	bool goal_reached;
	std::string plan_frame_id;
	double min_goal_distance;
	geometry_msgs::PoseStamped goal;
	double goal_angle;
	double goal_distance;

public:
	ReactiveController();
	virtual ~ReactiveController();

	void laser_channel_callback(sensor_msgs::LaserScanConstPtr msg);
	void compute_goal_angle(double& angle, double& distance, const std::string& laser_frame_id);
	void compute_robot_pose_in_plan(const std::string& laser_frame_id);

    /**
     * @brief  Given the current position, orientation, and velocity of the robot, compute velocity commands to send to the base
     * @param cmd_vel Will be filled with the velocity command to be passed to the robot base
     * @return True if a valid velocity command was found, false otherwise
     */
    virtual bool computeVelocityCommands(geometry_msgs::Twist& cmd_vel);

    /**
     * @brief  Check if the goal pose has been achieved by the local planner
     * @return True if achieved, false otherwise
     */
    virtual bool isGoalReached();

    /**
     * @brief  Set the plan that the local planner is following
     * @param orig_global_plan The plan to pass to the local planner
     * @return True if the plan was updated successfully, false otherwise
     */
    virtual bool setPlan(const std::vector<geometry_msgs::PoseStamped>& plan);

    /**
     * @brief  Constructs the local planner
     * @param name The name to give this instance of the local planner
     * @param tf A pointer to a transform listener
     * @param costmap The cost map to use for assigning costs to local plans
     */
    virtual void initialize(std::string name, tf::TransformListener* tf, costmap_2d::Costmap2DROS* costmap_ros);
};

#endif /* SRC_REACTIVECONTROLLER_H_ */
