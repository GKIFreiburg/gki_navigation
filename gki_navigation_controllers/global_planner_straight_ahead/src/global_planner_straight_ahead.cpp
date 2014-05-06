#include "global_planner_straight_ahead/global_planner_straight_ahead.h"
#include <pluginlib/class_list_macros.h>
#include <tf/tf.h>
#include <nav_msgs/Path.h>

namespace global_planner_straight_ahead
{

PLUGINLIB_EXPORT_CLASS(global_planner_straight_ahead::GlobalPlannerStraightAhead, nav_core::BaseGlobalPlanner);

void GlobalPlannerStraightAhead::initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros)
{
    // Name is probably something like channel_controller::ChannelController
    // And our param then should be ChannelController
    size_t colon = name.find_last_of(":");
    std::string class_name = name.substr(colon + 1);
    if(colon == std::string::npos)
        class_name = name;

    ROS_INFO("Initializing ChannelController from ~/%s", class_name.c_str());
    ros::NodeHandle nhPriv("~/" + class_name);    // ~ = /move_base, our config should be in /move_base/name

    pub_path_ = nhPriv.advertise<nav_msgs::Path>("global_plan", 1);
}

bool GlobalPlannerStraightAhead::makePlan(const geometry_msgs::PoseStamped & start, 
        const geometry_msgs::PoseStamped & goal,
        std::vector<geometry_msgs::PoseStamped> & plan)
{
    plan.push_back(start);
    tf::Pose startTF;
    tf::poseMsgToTF(start.pose, startTF);
    tf::Pose oneMeter(tf::Quaternion(0, 0, 0, 1), tf::Vector3(1.0, 0.0, 0.0));
    tf::Pose oneMeterAhead = startTF * oneMeter;
    geometry_msgs::PoseStamped psOneMeterAhead;
    psOneMeterAhead.header = start.header;
    tf::poseTFToMsg(oneMeterAhead, psOneMeterAhead.pose);
    plan.push_back(psOneMeterAhead);

    ROS_INFO("GlobalPlannerStraightAhead, new goal: %f %f -> %f %f",
            start.pose.position.x, start.pose.position.y,
            plan.back().pose.position.x, plan.back().pose.position.y
            );

    nav_msgs::Path pathMsg;
    pathMsg.header = plan.back().header;
    pathMsg.poses = plan;
    pub_path_.publish(pathMsg);

    return true;
}

}


