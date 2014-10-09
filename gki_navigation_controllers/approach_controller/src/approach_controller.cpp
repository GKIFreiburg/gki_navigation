#include "approach_controller.h"
#include <geometry_msgs/Twist.h>
#include <boost/foreach.hpp>
#include <visualization_msgs/Marker.h>
#define forEach BOOST_FOREACH

ApproachController::ApproachController(const std::string & action_name, const std::string & fixed_frame) :
    action_name_(action_name), fixed_frame_(fixed_frame),
    as_(nh_, action_name, boost::bind(&ApproachController::executeCB, this, _1), false)
{
    sub_laser_ = nh_.subscribe("base_scan_filtered", 2, &ApproachController::laserCallback, this);
    pub_vel_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel_mux/input/navi", 1);

    pub_vis_ = nh_.advertise<visualization_msgs::Marker>("approach_vis", 10);

    as_.start();
}

void ApproachController::executeCB(const move_base_msgs::MoveBaseGoalConstPtr & goal)
{
    goal_pose_ = goal->target_pose;

    ros::Rate rate(10.0);
    while(!as_.isPreemptRequested() && ros::ok())
    {
        double minX = HUGE_VAL;
        tf::Vector3 ptMiddle;
        {
            boost::unique_lock<boost::mutex> scoped_lock(laser_mutex_);
            if(base_laser_points_.empty())
                continue;
            forEach(const tf::Vector3 pt, base_laser_points_) {
                if(pt.x() < minX)
                    minX = pt.x();
            }
        }
        printf("MD %f \n", minX);
        if(minX < 0.5) {
            as_.setSucceeded();
            return;
        }

        publishVel(0.1, 0.0);
        rate.sleep();
    }

    if(as_.isPreemptRequested()) {
        as_.setPreempted();
    } else {
        as_.setAborted();
    }
}

void ApproachController::publishVel(double tv, double rv)
{
    geometry_msgs::Twist cmd_vel;
    cmd_vel.linear.x = tv;
    cmd_vel.angular.z = rv;
    pub_vel_.publish(cmd_vel);
}

void ApproachController::laserCallback(const sensor_msgs::LaserScan & laser)
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

    visualization_msgs::Marker marker;
    marker.header.frame_id = "/base_footprint";
    marker.header.stamp = last_laser_.header.stamp;
    marker.ns = "base_points";
    marker.type = visualization_msgs::Marker::POINTS;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.orientation.w = 1;
    marker.scale.x = 0.03;
    marker.scale.y = 0.03;
    marker.scale.z = 0.03;
    marker.color.b = 1.0;
    marker.color.a = 1.0;
    forEach(const tf::Vector3 & ptBase, base_laser_points_) {
        geometry_msgs::Point pt;
        tf::pointTFToMsg(ptBase, pt);
        marker.points.push_back(pt);
    }
    pub_vis_.publish(marker);
}

tf::Pose ApproachController::getTargetPose()
{

}

