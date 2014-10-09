#include "retreat_controller.h"
#include <geometry_msgs/Twist.h>
#include <boost/foreach.hpp>
#include <visualization_msgs/Marker.h>
#define forEach BOOST_FOREACH

RetreatController::RetreatController(const std::string & action_name, const std::string & fixed_frame) :
    action_name_(action_name), fixed_frame_(fixed_frame),
    as_(nh_, action_name, boost::bind(&RetreatController::executeCB, this, _1), false)
{
    //sub_laser_ = nh_.subscribe("base_scan_filtered", 2, &RetreatController::laserCallback, this);
    sub_odom_ = nh_.subscribe("odom", 1, &RetreatController::odomCallback, this);
    pub_vel_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel_mux/input/navi", 1);

    ros::NodeHandle nhPriv("~");
    retreat_dist_ = 0.4;
    nhPriv.param("retreat_dist", retreat_dist_, retreat_dist_);

    as_.start();
}

void RetreatController::executeCB(const move_base_msgs::MoveBaseGoalConstPtr & goal)
{
    goal_pose_ = goal->target_pose;
    start_pose_.header.stamp = goal_pose_.header.stamp;

    {
        boost::unique_lock<boost::mutex> scoped_lock(laser_mutex_);
        start_pose_.pose = last_odom_pose_;
    }

    ros::Rate rate(10.0);
    while(!as_.isPreemptRequested() && ros::ok())
    {
        double dt = 0.0;
        {
            boost::unique_lock<boost::mutex> scoped_lock(laser_mutex_);
            dt = hypot(start_pose_.pose.position.x - last_odom_pose_.position.x,
                    start_pose_.pose.position.y - last_odom_pose_.position.y);
        }
        ROS_INFO_THROTTLE(0.5, "Retreat dist: %f", dt);
        if(dt > retreat_dist_) {
            as_.setSucceeded();
            return;
        }

        publishVel(-0.1, 0.0);
        rate.sleep();
    }

    if(as_.isPreemptRequested()) {
        as_.setPreempted();
    } else {
        as_.setAborted();
    }
}

void RetreatController::publishVel(double tv, double rv)
{
    geometry_msgs::Twist cmd_vel;
    cmd_vel.linear.x = tv;
    cmd_vel.angular.z = rv;
    pub_vel_.publish(cmd_vel);
}

void RetreatController::odomCallback(const nav_msgs::Odometry & odom)
{
    last_odom_pose_ = odom.pose.pose;
}

void RetreatController::laserCallback(const sensor_msgs::LaserScan & laser)
{
    last_laser_ = laser;

    tf::StampedTransform transform;

    double dt = (ros::Time::now() - last_laser_.header.stamp).toSec();
    //printf("LASER AGE: %f\n", dt);
    if(dt > 0.2) {
        ROS_ERROR("%s: Laser too old - age is: %f", __func__, dt);
        return;
    }
    try {
        if(!tf_.waitForTransform("/base_footprint",
                    last_laser_.header.frame_id, last_laser_.header.stamp, ros::Duration(0.1))) {
            ROS_ERROR("Current Laser TF not available");
            return;
        }
        tf_.lookupTransform("/base_footprint",
                last_laser_.header.frame_id, last_laser_.header.stamp, transform);
    } catch(tf::TransformException & e) {
        ROS_ERROR("%s: TF Error: %s", __func__, e.what());
        return;
    }

    boost::unique_lock<boost::mutex> scoped_lock(laser_mutex_);
    base_laser_points_.clear();
    for(unsigned int i = 0; i < last_laser_.ranges.size(); i++) {
        if(last_laser_.ranges[i] <= last_laser_.range_min || last_laser_.ranges[i] >= last_laser_.range_max)
            continue;
        double da = last_laser_.angle_min + last_laser_.angle_increment * i;
        //ROS_INFO("rr %f", last_laser_.ranges[i]);
        tf::Vector3 pt(last_laser_.ranges[i] * cos(da), last_laser_.ranges[i] * sin(da), 0.0);
        tf::Vector3 ptBase = transform * pt;
        if(fabs(ptBase.y()) < 0.3)  // keep points inside robot width
            base_laser_points_.push_back(ptBase);
        //ROS_INFO_STREAM(ptBase.x() << " " << ptBase.y());
    }
}

