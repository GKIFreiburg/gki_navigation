/*
 * ReactiveController.cpp
 *
 *  Created on: Mar 26, 2015
 *      Author: turtlebot
 */
#include <pluginlib/class_list_macros.h>
#include "reactive_move_controller/ReactiveController.h"

PLUGINLIB_EXPORT_CLASS(ReactiveController, nav_core::BaseLocalPlanner);

ReactiveController::ReactiveController()
{
	listener = NULL;
	goal_reached = true;
	min_goal_distance = 0.5;
}

ReactiveController::~ReactiveController()
{
}

void ReactiveController::laser_channel_callback(sensor_msgs::LaserScanConstPtr msg)
{

	laser_scan = *msg;
}

void ReactiveController::compute_goal_angle(double& angle, double& distance, const std::string& laser_frame_id)
{
	tf::Pose goal_tf;
	geometry_msgs::PoseStamped goal;
	tf::poseMsgToTF(goal.pose, goal_tf);
	tf::StampedTransform robot_pose;
	tf::Pose goal_from_robot_tf;
	try
	{
		//ROS_INFO_STREAM("frameid: "<<laser_frame_id<<" frameid2: "<<goal.header.frame_id);
		listener->lookupTransform(goal.header.frame_id, laser_frame_id, ros::Time(0), robot_pose);
		goal_from_robot_tf = robot_pose.inverseTimes(goal_tf);
		//geometry_msgs::Pose transformed_goal;
		//tf::poseTFToMsg(robot_pose, transformed_goal);
		//ROS_INFO_STREAM("transformed goal: "<<transformed_goal);
	}
	catch (tf::TransformException& ex)
	{
		ROS_ERROR("%s", ex.what());
		//ros::Duration(1.0).sleep();
	}
	//geometry_msgs::Pose transformed_goal;
	//tf::poseTFToMsg(goal_from_robot_tf, transformed_goal);
	//ROS_INFO_STREAM("transformed goal: "<<transformed_goal);
	tf::Vector3 point = goal_from_robot_tf.getOrigin();
	angle = atan2(point.y(), point.x());
	distance = hypot(point.x(), point.y());

}

bool ReactiveController::computeVelocityCommands(geometry_msgs::Twist& cmd_vel)
{
	visualization_msgs::MarkerArray vis_msg;
	double goal_angle = 0;
	double goal_distance = 0;
	if (goal_reached == false)
		compute_goal_angle(goal_angle, goal_distance, laser_scan.header.frame_id);
	if (goal_distance < 0.25)
	{
		vis_msg.markers.push_back(visualization_msgs::Marker());
		visualization_msgs::Marker& marker = vis_msg.markers.back();
		marker.action = visualization_msgs::Marker::DELETE;
		marker.ns = "goal";
		marker.id = 0;
		marker.header.frame_id = "/odom";
	    goal_reached = true;

	}
	// all channels update
	for (size_t index = 0; index < laser_channels.size(); index++)
    {
	    laser_channels[index].laser_update(laser_scan, goal_distance, goal_reached);
	    laser_channels[index].to_string();
    }
	// find best channel
	double score = 0;
	size_t channel_index = 0;
	for(size_t index = 0; index < laser_channels.size(); index++)
	{
		if(laser_channels[index].compute_score(goal_angle, goal_reached)>score)
		{
			score = laser_channels[index].compute_score(goal_angle, goal_reached);
			channel_index = index;

		}
	}

	//ROS_INFO_STREAM("Score: "<<score);
	// set velocity
	laser_channels[channel_index].get_velocity(cmd_vel);
	// visualize
	for (size_t index = 0; index < laser_channels.size(); index++)
	{
		laser_channels[index].visualize(vis_msg, laser_scan.header.frame_id, index, goal_angle, goal_reached);
	}

    visualization_publisher.publish(vis_msg);
    return true;
}

bool ReactiveController::isGoalReached()
{
	return goal_reached;
}

bool ReactiveController::setPlan(const std::vector<geometry_msgs::PoseStamped>& plan)
{
	this->plan = plan;
	goal_reached = false;
	if (! plan.empty())
		plan_frame_id = plan[0].header.frame_id;
	return true;
}

void ReactiveController::initialize(std::string name, tf::TransformListener* tf, costmap_2d::Costmap2DROS* costmap_ros)
{
    ros::NodeHandle nh;\
    //ROS_INFO_STREAM(0/0);


	laser_channels.push_back(Channel(0.0, 0.55));
    for (size_t index = 1; index < 8; index++)
    {
    	double angle = index*15.0/180.0*M_PI;
    	laser_channels.push_back(Channel(angle, 0.55));
    	laser_channels.push_back(Channel(-angle, 0.55));
    }
    for (size_t index = 0; index < 3; index++)
    {
    	double angle = (index*15.0+7.5)/180.0*M_PI;
    	laser_channels.push_back(Channel(angle, 0.55));
    	laser_channels.push_back(Channel(-angle, 0.55));
    }
    ROS_INFO_STREAM("Channels: "<<laser_channels.size());
    listener = tf;
    ROS_INFO("subscribing to laser topic...");
    laser_subscriber = nh.subscribe("base_scan", 1, &ReactiveController::laser_channel_callback, this);
    ROS_INFO("subscribing to laser topic...");
    visualization_publisher = nh.advertise<visualization_msgs::MarkerArray>("visualization", 1, false);
    ROS_INFO("reactive move controller initialized.");
}

void ReactiveController::compute_robot_pose_in_plan(const std::string& laser_frame_id)
{
	tf::Pose plan_tf;
		tf::poseMsgToTF(plan[0].pose, plan_tf);
		tf::StampedTransform robot_pose;
		tf::Pose robot_from_plan_tf;
		try
		{
			//ROS_INFO_STREAM("frameid: "<<laser_frame_id<<" frameid2: "<<goal.header.frame_id);
			listener->lookupTransform(plan_frame_id, laser_frame_id, ros::Time(0), robot_pose);
			robot_from_plan_tf = plan_tf.inverseTimes(robot_pose);
			//geometry_msgs::Pose transformed_goal;
			//tf::poseTFToMsg(robot_pose, transformed_goal);
			//ROS_INFO_STREAM("transformed goal: "<<transformed_goal);
		}
		catch (tf::TransformException& ex)
		{
			ROS_ERROR("%s", ex.what());
			//ros::Duration(1.0).sleep();
		}
		for (int index = plan.size() - 1; index >= 0; index --)
		{
			double distance = 0;
			distance = hypot(plan[index].pose.position.x, plan[index].pose.position.y);
			if (distance < min_goal_distance)
			{
				//goal = plan[index].pose;
			}

		}

}
