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
    sub_line_features_ = nh_.subscribe("isolated_lines", 2, &ApproachController::lineFeaturesCallback, this);
    sub_line_features2_ = nh_.subscribe("connected_lines", 2, &ApproachController::lineFeaturesCallback, this);
    pub_vel_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel_mux/input/navi", 1);

    pub_vis_ = nh_.advertise<visualization_msgs::Marker>("approach_vis", 10);

    approach_dist_ = 0.4;
    ros::NodeHandle nhPriv("~");
    nhPriv.param("approach_dist", approach_dist_, approach_dist_);

    as_.start();
}

void ApproachController::executeCB(const move_base_msgs::MoveBaseGoalConstPtr & goal)
{
    goal_pose_ = goal->target_pose;
    // TODO goal_pose_ needs to go to fixed_frame_

    {
        boost::unique_lock<boost::mutex> scoped_lock(line_feature_pose_mutex_);
        line_feature_pose_.header.stamp = ros::Time(0);
    }

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
        if(minX < approach_dist_) {
            as_.setSucceeded();
            return;
        }

        tf::Pose targetPose = getTargetPose();
        // This is empty in failure, warn and go straight
        if(targetPose.getOrigin().x() == 0) {
            ROS_ERROR_THROTTLE(1.0, "Invalid approach target: Going straight.");
            publishVel(0.1, 0.0);
        } else {
            driveToPose(targetPose);
        }

        rate.sleep();
    }

    if(as_.isPreemptRequested()) {
        as_.setPreempted();
    } else {
        as_.setAborted();
    }
}

void ApproachController::driveToPose(const tf::Pose & pose)
{
    // otherwise:
    // - position:
    // - if too far away in y: go perpedicular
    // - if not really aligned: go more towards y then necessary
    // - if well aligned position:
    // - if angle too bad: turn towards first
    // - if position and angle OK: steer towards!
    // TODO steer towards that.

    // OK So the pose is in the wall, pointing outwards
    // we wanna go 40cm ahead of that
    // and come in perpendicular.

}

void ApproachController::publishVel(double tv, double rv)
{
    geometry_msgs::Twist cmd_vel;
    cmd_vel.linear.x = tv;
    cmd_vel.angular.z = rv;
    pub_vel_.publish(cmd_vel);
}

void ApproachController::lineFeaturesCallback(const geometry_msgs::PoseArray & lineFeatures)
{
    // This collects isolated and connected lines likewise.
    // if one of those isn't centered we can still not subscribe if not trustworthy
    // TODO for now assuming laser frame!!!

    // Determine a good/best line feature.
    // It must be in front of us, e.g. within a side deviation and not too far away.
    geometry_msgs::PoseStamped best_pose;
    best_pose.header = lineFeatures.header;
    bool poseFound = false;
    forEach(const geometry_msgs::Pose & pose, lineFeatures.poses) {
        if(fabs(pose.position.y) > 0.6)     // not where we're pointing
            continue;
        if(pose.position.x < 0) // behind us, shouldnt happen
            continue;
        if(pose.position.x > 2.5)   // too far away
            continue;

        if(!poseFound)
            best_pose.pose = pose;
        else {
            // Take the one that's closed in y coord
            if(fabs(pose.position.y) < fabs(best_pose.pose.position.y)) {
                best_pose.pose = pose;
            }
        }
        poseFound = true;
    }
    if(poseFound) {
        geometry_msgs::PoseStamped best_pose_fixed;
        {
            boost::unique_lock<boost::mutex> scoped_lock_tf(tf_mutex_);
            try {
                if(!tf_.waitForTransform(fixed_frame_,
                            best_pose.header.frame_id, best_pose.header.stamp, ros::Duration(0.1))) {
                    ROS_ERROR("Current Line features TF not available");
                    return;
                }
                tf_.transformPose(fixed_frame_,
                        best_pose, best_pose_fixed);
            } catch(tf::TransformException & e) {
                ROS_ERROR("%s: TF Error: %s", __func__, e.what());
                return;
            }
        }

        boost::unique_lock<boost::mutex> scoped_lock(line_feature_pose_mutex_);
        // FIXME dont overwrite a better one if we already have one and it's not too old
        // problem would need to transform the old one back to laser frame.
        // ignore for now
        line_feature_pose_ = best_pose_fixed;
    }

    if(poseFound) {
        visualization_msgs::Marker marker;
        marker.header = line_feature_pose_.header;
        marker.ns = "line_features_target";
        marker.type = visualization_msgs::Marker::ARROW;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose = line_feature_pose_.pose;
        marker.scale.x = approach_dist_;
        marker.scale.y = 0.1;
        marker.scale.z = 0.1;
        marker.color.b = 1.0;
        marker.color.a = 1.0;
        pub_vis_.publish(marker);
    }
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
    {
        boost::unique_lock<boost::mutex> scoped_lock_tf(tf_mutex_);
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
    ros::Time now = ros::Time::now();
    geometry_msgs::PoseStamped bestPose;
    bool bestPoseFound = false;

    double dtLineFeaturePose = HUGE_VAL;
    {
        boost::unique_lock<boost::mutex> scoped_lock(line_feature_pose_mutex_);
        dtLineFeaturePose = (now - line_feature_pose_.header.stamp).toSec();
        if(dtLineFeaturePose < 30.0) {
            bestPose = line_feature_pose_;
            bestPoseFound = true;
        }
    }


    if(!bestPoseFound) {
        // TODO let's assume the goal is good,
        // otherwise use a fake pose for "go straight ahead"
        bestPose = goal_pose_;
    }

    geometry_msgs::PoseStamped poseRobot;
    geometry_msgs::PoseStamped poseTarget = bestPose;
    poseTarget.header.stamp = ros::Time(0);
    {
        boost::unique_lock<boost::mutex> scoped_lock_tf(tf_mutex_);
        try {
            if(!tf_.waitForTransform("/base_footprint",
                        poseTarget.header.frame_id, poseTarget.header.stamp, ros::Duration(0.1))) {
                ROS_ERROR("Current target pose not available");
                return tf::Pose();
            }
            tf_.transformPose("/base_footprint",
                    poseTarget, poseRobot);
        } catch(tf::TransformException & e) {
            ROS_ERROR("%s: TF Error: %s", __func__, e.what());
            return tf::Pose();
        }
    }

    tf::Pose tfPose;
    tf::poseMsgToTF(poseRobot.pose, tfPose);

    visualization_msgs::Marker marker;
    marker.header = poseRobot.header;
    marker.ns = "approach_target";
    marker.type = visualization_msgs::Marker::ARROW;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose = poseRobot.pose;
    marker.scale.x = approach_dist_;
    marker.scale.y = 0.15;
    marker.scale.z = 0.15;
    marker.color.g = 1.0;
    marker.color.a = 0.5;
    pub_vis_.publish(marker);

    return tfPose;
}

