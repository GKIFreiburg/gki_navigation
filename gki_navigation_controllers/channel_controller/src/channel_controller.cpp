#include "channel_controller/channel_controller.h"
#include "channel_controller/bresenham.h"
#include <pluginlib/class_list_macros.h>
#include <string>
#include <nav_msgs/Path.h>
#include <angles/angles.h>
#include <visualization_msgs/MarkerArray.h>
#include <boost/foreach.hpp>
#define forEach BOOST_FOREACH

namespace channel_controller
{

PLUGINLIB_EXPORT_CLASS(channel_controller::ChannelController, nav_core::BaseLocalPlanner);

ChannelController::ChannelController() : tf_(NULL), costmap_ros_(NULL), current_waypoint_(0)
{
}

ChannelController::~ChannelController()
{
}

void ChannelController::initialize(std::string name,
        tf::TransformListener* tf, costmap_2d::Costmap2DROS* costmap_ros)
{
    tf_ = tf;
    costmap_ros_ = costmap_ros;

    voronoi_.initializeEmpty(costmap_ros_->getSizeInCellsX(),
            costmap_ros_->getSizeInCellsY(), true);

    // Name is probably something like channel_controller::ChannelController
    // And our param then should be ChannelController
    size_t colon = name.find_last_of(":");
    std::string class_name = name.substr(colon + 1);
    if(colon == std::string::npos)
        class_name = name;

    ROS_INFO("Initializing ChannelController from ~/%s", class_name.c_str());
    ros::NodeHandle nhPriv("~/" + class_name);    // ~ = /move_base, our config should be in /move_base/name
    ros::NodeHandle nh;

    nhPriv.param("waypoint_reached_dist", waypoint_reached_dist_, 0.3);
    nhPriv.param("waypoint_reached_angle", waypoint_reached_angle_, 7.0);
    nhPriv.param("goal_reached_dist", goal_reached_dist_, 0.1);
    nhPriv.param("goal_reached_angle", goal_reached_angle_, 0.22);

    nhPriv.param("min_tv", min_tv_, 0.1);
    nhPriv.param("max_tv", max_tv_, 0.6);
    nhPriv.param("min_rv", min_rv_, 0.0);
    nhPriv.param("max_rv", max_rv_, 0.9);
    nhPriv.param("min_inplace_rv", min_inplace_rv_, 0.1);

    nhPriv.param("stopped_tv", stopped_tv_, 0.05);
    nhPriv.param("stopped_rv", stopped_rv_, 0.1);

    nhPriv.param("max_accel_tv", max_accel_tv_, 2.0);
    nhPriv.param("max_accel_rv", max_accel_rv_, 1.5);

    nhPriv.param("vis_max_dist", vis_max_dist_, 1.0);
    nhPriv.param("visualize_voronoi", visualize_voronoi_, false);

    ROS_INFO("waypoint_reached_dist: %f", waypoint_reached_dist_);
    ROS_INFO("waypoint_reached_angle: %f", waypoint_reached_angle_);
    ROS_INFO("goal_reached_dist: %f", goal_reached_dist_);
    ROS_INFO("goal_reached_angle: %f", goal_reached_angle_);
    ROS_INFO("min_tv: %f", min_tv_);
    ROS_INFO("max_tv: %f", max_tv_);
    ROS_INFO("min_rv: %f", min_rv_);
    ROS_INFO("max_rv: %f", max_rv_);
    ROS_INFO("min_inplace_rv: %f", min_inplace_rv_);
    ROS_INFO("stopped_tv: %f", stopped_tv_);
    ROS_INFO("stopped_rv: %f", stopped_rv_);
    ROS_INFO("max_accel_tv: %f", max_accel_tv_);
    ROS_INFO("max_accel_rv: %f", max_accel_rv_);
    ROS_INFO("vis_max_dist: %f", vis_max_dist_);
    ROS_INFO("visualize_voronoi: %d", visualize_voronoi_);

    pub_markers_ = nhPriv.advertise<visualization_msgs::MarkerArray>("channel_markers", 1);
    pub_local_plan_ = nhPriv.advertise<nav_msgs::Path>("local_plan", 1);

    sub_odom_ = nh.subscribe("odom", 1, &ChannelController::odometryCallback, this);

    updateVoronoi();
}
// TODO recovery - how triggered

void ChannelController::updateVoronoi()
{
    costmap_ros_->getCostmapCopy(costmap_);     // srsly we have to copy that to get to the data??????
    ROS_ASSERT(costmap_.getSizeInCellsX() == voronoi_.getSizeX());
    ROS_ASSERT(costmap_.getSizeInCellsY() == voronoi_.getSizeY());

    std::vector<IntPoint> obstacles;
    for(unsigned int x = 0; x < costmap_.getSizeInCellsX(); x++) {
        for(unsigned int y = 0; y < costmap_.getSizeInCellsY(); y++) {
            if(costmap_.getCost(x, y) >= costmap_2d::LETHAL_OBSTACLE) { // lethal and unknown
                obstacles.push_back(IntPoint(x, y));
            }
        }
    }
    voronoi_.exchangeObstacles(obstacles);
    voronoi_.update(true);
    //voronoi_.prune(); // FIXME This only does voronoi stuff, not distance related.
    if(visualize_voronoi_)
        visualizeVoronoi();
}

void ChannelController::visualizeVoronoi()
{
    ROS_ASSERT(costmap_.getSizeInCellsX() == voronoi_.getSizeX());
    ROS_ASSERT(costmap_.getSizeInCellsY() == voronoi_.getSizeY());

    // nothing to send to no one
    if(pub_markers_.getNumSubscribers() == 0)
        return;

    visualization_msgs::MarkerArray channelMarkers;
    visualization_msgs::Marker voronoiMarker;
    voronoiMarker.header.frame_id = costmap_ros_->getGlobalFrameID();
    voronoiMarker.header.stamp = ros::Time(0);
    voronoiMarker.ns = "voronoi";
    voronoiMarker.id = 0;
    voronoiMarker.type = visualization_msgs::Marker::SPHERE_LIST;
    voronoiMarker.action = visualization_msgs::Marker::ADD;
    voronoiMarker.pose.orientation.w = 1.0;
    voronoiMarker.scale.x = costmap_.getResolution() * sqrt(2.0);
    voronoiMarker.scale.y = costmap_.getResolution() * sqrt(2.0);
    voronoiMarker.scale.z = costmap_.getResolution() * sqrt(2.0);
    voronoiMarker.frame_locked = false;
    geometry_msgs::Point cellPoint;
    cellPoint.z = - costmap_.getResolution() * sqrt(2.0)/2.0;
    std_msgs::ColorRGBA cellColor;
    cellColor.a = 1.0;
    for(unsigned int x = 0; x < costmap_.getSizeInCellsX(); x++) {
        for(unsigned int y = 0; y < costmap_.getSizeInCellsY(); y++) {
            float dist = voronoi_.getDistance(x, y);
            dist *= costmap_.getResolution();   // now in meters
            costmap_.mapToWorld(x, y, cellPoint.x, cellPoint.y);
            voronoiMarker.points.push_back(cellPoint);

            if(dist == -INFINITY) {
                cellColor.r = 1.0;
                cellColor.g = 0.0;
                cellColor.b = 1.0;
            } else if(dist == INFINITY) {
                cellColor.r = 0.0;
                cellColor.g = 1.0;
                cellColor.b = 0.0;
            } else {
                if(dist > vis_max_dist_) {
                    cellColor.r = cellColor.g = cellColor.b = 1.0;
                } else {
                    // make those slightly darker then max dist to distinguish
                    // vis max dist regions
                    cellColor.r = cellColor.g = cellColor.b = 0.9 * dist / vis_max_dist_;
                }
            }

            voronoiMarker.colors.push_back(cellColor);
        }
    }
    channelMarkers.markers.push_back(voronoiMarker);
    pub_markers_.publish(channelMarkers);
}

bool ChannelController::isGoalReached()
{
    if(ros::Time::now() - last_odom_.header.stamp > ros::Duration(5.0)) {
        ROS_ERROR_THROTTLE(1.0, "isGoalReached:: Last odom is too old: %f - not at goal.",
                (ros::Time::now() - last_odom_.header.stamp).toSec());
        return false;
    }
    double cur_tv = last_odom_.twist.twist.linear.x;
    double cur_rv = last_odom_.twist.twist.angular.z;
    if(current_waypoint_ >= global_plan_.size() &&
            fabs(cur_tv) < stopped_tv_ && fabs(cur_rv) < stopped_rv_)
        return true;
    return false;
}

bool ChannelController::setPlan(const std::vector<geometry_msgs::PoseStamped> & plan)
{
    global_plan_ = plan;
    current_waypoint_ = 0;

    // localize now to verify this makes sense somehow and return upon that.
    return localizeGlobalPlan(current_waypoint_);
}

bool ChannelController::currentWaypointReached() const
{
    if(local_plan_.empty())
        return true;

    tf::Stamped<tf::Pose> robot_pose;
    if(!costmap_ros_->getRobotPose(robot_pose))
        return false;

    tf::Stamped<tf::Pose> currentWaypoint = local_plan_.front();
    tf::Pose toWaypoint = robot_pose.inverseTimes(currentWaypoint);

    double distThreshold = waypoint_reached_dist_;
    double angleThreshold = waypoint_reached_angle_;
    if(local_plan_.size() == 1) {   // only 1 wp left -> goal wp
        distThreshold = goal_reached_dist_;
        angleThreshold = goal_reached_angle_;
    }

    if(toWaypoint.getOrigin().length() > distThreshold)
        return false;
    if(fabs(tf::getYaw(toWaypoint.getRotation())) > angleThreshold)
        return false;

    return true;
}

bool ChannelController::localizeGlobalPlan(unsigned int start_index)
{
    // TODO test this with different frames for map/odom
    nav_msgs::Path localPlanMsg;
    localPlanMsg.header.stamp = ros::Time(0);
    localPlanMsg.header.frame_id = costmap_ros_->getGlobalFrameID();

    local_plan_.clear();
    if(global_plan_.empty()) {
        pub_local_plan_.publish(localPlanMsg);
        return false;
    }

    tf::StampedTransform transform;
    // purposely ignoreing the global_plan_.header.stamp here as we assume that the global_plan_
    // frame is a fixed frame and the controller's something like odom.
    // We thus want the most current transformation into our frame for the global_plan_.
    try {
        tf_->lookupTransform(costmap_ros_->getGlobalFrameID(),
                global_plan_.front().header.frame_id, ros::Time(0), transform);
    } catch(tf::TransformException & e) {
        ROS_ERROR("%s: TF Error: %s", __func__, e.what());
        pub_local_plan_.publish(localPlanMsg);
        return false;
    }

    // we'll assume all frame_id are the same in global plan
    for(unsigned int i = start_index; i < global_plan_.size(); i++) {
        tf::Pose global_pose;
        tf::poseMsgToTF(global_plan_.at(i).pose, global_pose);

        tf::Stamped<tf::Pose> local_pose;
        local_pose.frame_id_ = costmap_ros_->getGlobalFrameID();
        local_pose.stamp_ = transform.stamp_;
        local_pose.setData(transform * global_pose);
        local_plan_.push_back(local_pose);

        if(pub_local_plan_.getNumSubscribers() > 0) {
            geometry_msgs::PoseStamped local_pose_msg;
            tf::poseStampedTFToMsg(local_pose, local_pose_msg);
            localPlanMsg.poses.push_back(local_pose_msg);
        }
    }

    pub_local_plan_.publish(localPlanMsg);
    return true;
}

DriveChannel ChannelController::computeChannel(tf::Pose from_pose, tf::Pose to_pose, double clearance_dist) const
{
    ROS_ASSERT(costmap_.getSizeInCellsX() == voronoi_.getSizeX());
    ROS_ASSERT(costmap_.getSizeInCellsY() == voronoi_.getSizeY());

    DriveChannel channel;
    channel.from_pose_ = from_pose;
    channel.min_dist_ = HUGE_VAL;

    int start_x, start_y;
    int target_x, target_y;
    costmap_.worldToMapNoBounds(from_pose.getOrigin().x(), from_pose.getOrigin().y(),
            start_x, start_y);
    costmap_.worldToMapNoBounds(to_pose.getOrigin().x(), to_pose.getOrigin().y(),
            target_x, target_y);
    int trace_x = start_x;
    int trace_y = start_y;
    for(Bresenham b(start_x, start_y, target_x, target_y); b.hasNext(); b.advance()) {
        if(b.cur_x() < 0 || b.cur_y() < 0)
            break;
        if(b.cur_x() >= (int)voronoi_.getSizeX() || b.cur_y() >= (int)voronoi_.getSizeY())
            break;
        float dist = voronoi_.getDistance(b.cur_x(), b.cur_y());
        dist *= costmap_.getResolution();
        if(dist == -INFINITY)   // +INFINITY Ok
            break;
        if(dist < clearance_dist)
            break;
        if(dist < channel.min_dist_)
            channel.min_dist_ = dist;
        trace_x = b.cur_x();
        trace_y = b.cur_y();
    }
    double dx = costmap_.getResolution() * (trace_x - start_x);
    double dy = costmap_.getResolution() * (trace_y - start_y);

    tf::Pose deltaPose = from_pose.inverseTimes(to_pose);
    tf::Pose deltaChannelLength(deltaPose.getRotation(),
            deltaPose.getOrigin().normalized() * hypot(dx, dy));
    channel.to_pose_ = from_pose * deltaChannelLength;

    return channel;
}

visualization_msgs::Marker ChannelController::createChannelMarkers(
        const std::vector<DriveChannel> & channels, double min_good_dist,
        int best_idx) const
{
    visualization_msgs::Marker channelMarker;
    channelMarker.header.stamp = ros::Time(0);
    channelMarker.header.frame_id = costmap_ros_->getGlobalFrameID();
    channelMarker.ns = "channels";
    channelMarker.id = 2;
    channelMarker.type = visualization_msgs::Marker::LINE_LIST;
    channelMarker.action = visualization_msgs::Marker::ADD;
    channelMarker.pose.orientation.w = 1.0;
    channelMarker.scale.x = 0.01;
    channelMarker.lifetime = ros::Duration(0);
    channelMarker.frame_locked = false;

    int index = 0;
    forEach(const DriveChannel & channel, channels) {
        // rotation of 90 for half of min_dist_ and the end to show width
        tf::Pose channelDelta = channel.from_pose_.inverseTimes(channel.to_pose_);
        double dir = atan2(channelDelta.getOrigin().y(), channelDelta.getOrigin().x());
        tf::Vector3 half_width(0.0, channel.min_dist_/2.0, 0.0);
        tf::Vector3 half_width_other(0.0, -channel.min_dist_/2.0, 0.0);
        tf::Pose side_offset(tf::createQuaternionFromYaw(0.0), half_width);
        tf::Pose side_offset_other(tf::createQuaternionFromYaw(0.0), half_width_other);
        tf::Pose channel_dir(tf::createQuaternionFromYaw(dir));
        tf::Pose z = tf::Pose(channel.from_pose_.getRotation()) * channel_dir * side_offset;
        tf::Pose z2 = tf::Pose(channel.from_pose_.getRotation()) * channel_dir * side_offset_other;

        geometry_msgs::Point pt;
        pt.x = channel.from_pose_.getOrigin().x();
        pt.y = channel.from_pose_.getOrigin().y();
        pt.z = 0.0;
        channelMarker.points.push_back(pt);
        pt.x = channel.to_pose_.getOrigin().x();
        pt.y = channel.to_pose_.getOrigin().y();
        channelMarker.points.push_back(pt);
        pt.x = channel.to_pose_.getOrigin().x() + z2.getOrigin().x();
        pt.y = channel.to_pose_.getOrigin().y() + z2.getOrigin().y();
        channelMarker.points.push_back(pt);
        pt.x = channel.to_pose_.getOrigin().x() + z.getOrigin().x();
        pt.y = channel.to_pose_.getOrigin().y() + z.getOrigin().y();
        channelMarker.points.push_back(pt);
        //ROS_INFO_STREAM("CHANNEL TO " << pt << " LENGTH " << channel_dist);
        std_msgs::ColorRGBA col;
        col.r = 0.7;
        col.g = 0.0;
        col.b = 0.0;
        col.a = 1.0;
        if(channel.length() >= min_good_dist) {
            col.r = 0.0;
            col.g = 0.7;
            col.b = 0.0;
        }
        if(index == best_idx) {
            col.r = 0.0;
            col.g = 0.0;
            col.b = 1.0;
        }
        channelMarker.colors.push_back(col);
        channelMarker.colors.push_back(col);
        channelMarker.colors.push_back(col);
        channelMarker.colors.push_back(col);
        index++;
    }

    return channelMarker;
}

visualization_msgs::Marker ChannelController::createPoseMarker(const tf::Pose & pose,
        double r, double g, double b,
        const std::string & ns, int id) const
{
    visualization_msgs::Marker poseMarker;
    poseMarker.header.stamp = ros::Time(0);
    poseMarker.header.frame_id = costmap_ros_->getGlobalFrameID();
    poseMarker.ns = ns;
    poseMarker.id = id;
    poseMarker.type = visualization_msgs::Marker::SPHERE;
    poseMarker.action = visualization_msgs::Marker::ADD;
    tf::poseTFToMsg(pose, poseMarker.pose);
    poseMarker.scale.x = 0.1;
    poseMarker.scale.y = 0.1;
    poseMarker.scale.z = 0.1;
    poseMarker.color.r = r;
    poseMarker.color.g = g;
    poseMarker.color.b = b;
    poseMarker.color.a = 1.0;
    poseMarker.lifetime = ros::Duration(0);
    poseMarker.frame_locked = false;

    return poseMarker;
}

double ChannelController::straight_up(double x, double a, double b) const
{
    // linearly go up from 0 to 1 between a and b
    if(x <= a)
        return 0.0;
    if(x >= b)
        return 1.0;
    return (x - a)/(b - a);
}

double ChannelController::straight_down(double x, double a, double b) const
{
    // linearly go down from 1 to 0 between a and b
    if(x <= a)
        return 1.0;
    if(x >= b)
        return 0.0;
    return (b - x)/(b - a);
}

void ChannelController::odometryCallback(const nav_msgs::Odometry & odom)
{
    last_odom_ = odom;
}

void ChannelController::limitTwist(geometry_msgs::Twist & cmd_vel) const
{
    double cur_tv = last_odom_.twist.twist.linear.x;
    double cur_rv = last_odom_.twist.twist.angular.z;

    double dt = (ros::Time::now() - last_cmd_vel_time_).toSec();  // estimated dt for commands
    // If we haven't send commands for a while, we'll get an initial "kick"
    // with a large dt
    double dx_max = max_accel_tv_ * dt;
    double dth_max = max_accel_rv_ * dt;

    // FIXME later: stopping for obstacle? -> test and increase accel limits or not limit
    // if the channel is too short.

    double twist_scale = 1.0;
    if(fabs(cmd_vel.linear.x - cur_tv) > dx_max) {
        twist_scale = dx_max/fabs(cmd_vel.linear.x - cur_tv);
    }
    if(fabs(cmd_vel.angular.z - cur_rv) > dth_max) {
        twist_scale = std::min(twist_scale, dth_max/fabs(cmd_vel.angular.z - cur_rv));
    }

    // scale both for same trajectory
    if(twist_scale < 1.0) {
        geometry_msgs::Twist new_cmd_vel = cmd_vel;
        new_cmd_vel.linear.x = twist_scale * cmd_vel.linear.x;
        new_cmd_vel.angular.z = twist_scale * cmd_vel.angular.z;
        //// check mins
        //// FIXME Not relevant when scaling here?
        //// -> we'd have a large delta anyways...
        //if(fabs(cmd_vel.linear.x) < stopped_tv_) {
        //    if(fabs(new_cmd_vel.angular.z) < min_inplace_rv_) {
        //        new_cmd_vel.angular.z = sign(cmd_vel.angular.z) * min_inplace_rv_;
        //    }
        //} else {
        //    if(fabs(new_cmd_vel.linear.x) < min_tv_)
        //        new_cmd_vel.linear.x = sign(cmd_vel.linear.x) * min_tv_;
        //    if(fabs(new_cmd_vel.angular.z) < min_rv_)
        //        new_cmd_vel.angular.z = sign(cmd_vel.angular.z) * min_rv_;
        //}
        ROS_INFO("Scaling cmd vel from %f, %f -> %f %f", cmd_vel.linear.x, cmd_vel.angular.z,
                new_cmd_vel.linear.x, new_cmd_vel.angular.z);
        cmd_vel = new_cmd_vel;
    }
}

bool ChannelController::computeVelocityForChannel(const DriveChannel & channel, geometry_msgs::Twist & cmd_vel) const
{
    ROS_ASSERT(!local_plan_.empty());
    cmd_vel = geometry_msgs::Twist();   // init to 0

    tf::Pose relPose = channel.from_pose_.inverseTimes(channel.to_pose_);
    double channel_dir = atan2(relPose.getOrigin().y(), relPose.getOrigin().x());
    double channel_length = relPose.getOrigin().length();
    double channel_width = channel.min_dist_;
    double cur_tv = last_odom_.twist.twist.linear.x;
    //double cur_rv = last_odom_.twist.twist.angular.z;

    // the channel should have originated at robot_pose
    tf::Pose relToWp = channel.from_pose_.inverseTimes(local_plan_.front());
    double dist_to_wp = relToWp.getOrigin().length();
    double angle_to_wp = atan2(relToWp.getOrigin().y(), relToWp.getOrigin().x());
    // FIXME later: maybe look ahead if next waypoints are on the same channels: GO!

    // channel way different, stop first, then turn in place
    if(fabs(channel_dir) > angles::from_degrees(90)) {
        if(fabs(cur_tv) < stopped_tv_) {
            // Just turn full. We're stopped and way in the wrong direction
            cmd_vel.angular.z = max_rv_ * sign(channel_dir);
        }
        // else: stop (0)
        return true;
    }

    // kinda steer towards channel
    cmd_vel.angular.z = sign(channel_dir) * (min_rv_ + (max_rv_ - min_rv_) *
            straight_up(fabs(channel_dir), angles::from_degrees(10.0), angles::from_degrees(90.0)));

    cmd_vel.linear.x = min_tv_ + (max_tv_ - min_tv_) *
        straight_up(channel_length, 0.3, 1.0);

    // go slower forward if we aren't pointing in the right direction
    double bad_direction_tv_scale =
        straight_down(fabs(channel_dir), angles::from_degrees(10.0), angles::from_degrees(90.0));

    // go slower when narrow FIXME later: maybe only when narrow within where we are (not at end of channel)
    double close_to_obst_tv_scale = straight_up(channel_width, 0.5, 1.0);

    // for now ignore close to wp unless goal.
    double close_to_goal_tv_scale = 1.0;
    double close_to_goal_rv_scale = 1.0;
    if(local_plan_.size() == 1) {   // going for goal
        close_to_goal_tv_scale = straight_up(dist_to_wp, 0.2, 1.0);
        close_to_goal_rv_scale = straight_up(fabs(angle_to_wp), angles::from_degrees(10.0),
                angles::from_degrees(90.0));
        // Maybe do "states" in here and separate
        // TODO stop if reached, turn to goal, etc. --> other behviaor, here we go towards goal
        // -> should be in main fn
        //
        // Do we have a problem with goal reached navigation, when very close to goal?
        // Overshooting gives large da.
    }

    double tv_scale = bad_direction_tv_scale;
    tv_scale = std::min(tv_scale, close_to_obst_tv_scale);
    tv_scale = std::min(tv_scale, close_to_goal_tv_scale);

    double rv_scale = close_to_goal_rv_scale;

    cmd_vel.linear.x *= 0.1 + 0.9 * tv_scale;
    cmd_vel.angular.z *= 0.1 + 0.9 * rv_scale;

    // TODO min values.

    // straightup/down for racing/approaching
    // limit twist -> scale both if one is too much to get same movement, check min vels
    // stop first if wrong direction
    // TODO what info do I need here:
    // channel width/length
    // relative channel direction
    // dist to next target
    // cur velocities
    //
    // What is derived from that?
    //

    limitTwist(cmd_vel);

    // TODO turn to goal -> safe to do here if at pose
    // intermediate points will jump away before
    return true;
}


bool ChannelController::computeVelocityCommands(geometry_msgs::Twist & cmd_vel)
{
    cmd_vel = geometry_msgs::Twist();   // init to 0

    // True if a valid velocity command was found, false otherwise
    updateVoronoi();

    if(!localizeGlobalPlan(current_waypoint_)) {
        return false;
    }
    if(ros::Time::now() - last_odom_.header.stamp > ros::Duration(5.0)) {
        ROS_ERROR("Last odom is too old: %f - cannot produce commands.",
                (ros::Time::now() - last_odom_.header.stamp).toSec());
        return false;
    }

    // FIXME later: we should be able to skip/advance to waypoints furhter in the plan if they are reached.
    // No need to follow the plan backwards if we somehow got ahead towards goal - only that counts.
    while(!local_plan_.empty() && currentWaypointReached()) {
        current_waypoint_++;
        if(!localizeGlobalPlan(current_waypoint_)) {
            return false;
        }
    } // FIXME later: more efficient, esp for skipping: wayPOintReache(idx)

    if(local_plan_.empty()) {
        return true;    // FIXME At goal, but move_base wants us to say we did a velocity command always
    }
    tf::Stamped<tf::Pose> robot_pose;
    if(!costmap_ros_->getRobotPose(robot_pose)) {
        return false;
    }

    // TODO if there is no channel as our start is in obst: get out of that!
    // Minimize channels, smaller padding, recovery behaviors - go anywhere.
    // -> What is configured as recovery behaviors? Maybe something implemented already

    // TODO we must reject this if the local target is not easily reachable/too far away

    visualization_msgs::MarkerArray channelMarkers;

    tf::Pose currentTarget = local_plan_.front();

    channelMarkers.markers.push_back(createPoseMarker(currentTarget, 1.0, 1.0, 0.0, "current_target"));

    tf::Pose relativeTarget = robot_pose.inverseTimes(currentTarget);
    double distToTarget = relativeTarget.getOrigin().length();

    // need DriveChannel struct (from, to, min_dist, length)
    // Compute DA from that or store?
    // + this = fun computeChannels

    // TODO turn to goal seems to go only positive - normalize?
    // TODO 2 I don't know why we even turn to goal already? Is there code that does this.
    std::vector<DriveChannel> channels;
    int best_idx = -1;
    double best_da = HUGE_VAL;
    ROS_INFO("dist to target is: %f", distToTarget);
    for(double da = -M_PI; da <= M_PI - angles::from_degrees(5.0); da += angles::from_degrees(5.0)) {
        tf::Pose rotDir(tf::createQuaternionFromYaw(da));
        // point in da, along up to the whole costmap length
        tf::Pose relativeTargetDir(relativeTarget.getRotation(),
                relativeTarget.getOrigin().normalized() *
                    (costmap_.getSizeInMetersX() + costmap_.getSizeInMetersY()));
        tf::Pose rotTarget = robot_pose * rotDir * relativeTargetDir;  // FIXME rotDir * relativeTarget?

        DriveChannel channel = computeChannel(robot_pose, rotTarget,
                costmap_ros_->getInscribedRadius() + 0.05);
        channels.push_back(channel);
        if(channel.length() >= distToTarget) {   //  valid
            //ROS_INFO("Valid channel found with da %f at %zu", da, channels.size() - 1);
            if(fabs(da) < fabs(best_da)) {
                best_da = da;
                best_idx = channels.size() - 1;
                //ROS_INFO("Better channel found with da %f at %d", best_da, best_idx);
            }
        }
    }
    // Channel selection: Everything that is long enough to reach a waypoint ahead
    // free dist at waypoint should be >= dist along channel length to wpt (i.e. no split path)
    //
    // Maybe split this between: Drivable/safe channels and channels that have goal in reach
    // Then select from goal channels - warn if only drivable left (shouldnt need to turn to much)
    // -> sleect channel based on da + width/length --> SCORE
    // -> Maybe channels should have distToWp and da as vars?
    // -> small da waaay important when small distToTarget
    // -> with larget distToTarget da small-ish -> look at width, da large-ish look at da
    // -> Usually prefer large width channels
    // --> if aiming correctly free dist, otherwise aim for da.

    // DEBUG: Check channel selection and drive to channel independently.

    // FIXME really distToTarget, what if tartget is behind corner? There should be a waypoint ;)
    // Maybe better score like: low da, waypoint(S) - latest waypoint, distance
    channelMarkers.markers.push_back(createChannelMarkers(channels, distToTarget, best_idx));
    pub_markers_.publish(channelMarkers);

    // why no channel found sometimes???

    // no valid channel found -> replan global please
    if(best_idx < 0) {
        ROS_WARN_THROTTLE(0.5, "No valid channel found.");
        return false;
    }

    ROS_INFO("Best channel found with da %f at %d", best_da, best_idx);

    bool foundChannelCmd = computeVelocityForChannel(channels[best_idx], cmd_vel);

    last_cmd_vel_time_ = ros::Time::now();
    return foundChannelCmd;

    // Now: How would I use/judge channels as input interface
    // -> Read this and decide on channel fns I need.
    // -> discuss and Do/impl
    // -> visualize all
    //
    // Primary channel:
    // - Drive to waypoint directly
    // - Free at least until waypoint (more unnec?)
    // - if not: Take free channel with least angle diff to wpt
    // -> how long must that be? Or mix angle to wpt + dist?
    // - chan must reach wpt, if too short: problem.
    // - what if too short, but still has some length? GO there or use side-channel?
    // -> Problem with wide-spreach wpts around corner.
    //
    // Secondary channel:
    // - must "reach" same waypoint (how defined with not-straight-to-wp channels?) as primary
    // - "reach" = dist to target wpt at end must be free dist (i.e. don't have obstacle in between), no "other" channel/gap
    // - shouldn't divert too much (don't drive backwards, because there it's free)
    // - judges: more free space to sides, longer (after waypoint?)
    // - judge channel width (not min clearnec)
    //

    // if no good channel to next nicely distant waypoint: return false for new global plan
    // maybe we'd be OK if we can't reach the next waypoint, but a later on and manually skip the in between ones
    // What is our next target waypoint? Do we need something different than the
    // waypoint_reached_dist that adcanves waypoints OR is this the same logic/idea?
    // Does it make sense to "not have reached" a locally near waypoint, but
    // still aim for ones that's further away?

    // - adjust params to allow replanning
    // - we shuold at some point say that we can't computeVelocityCommands
    // if the global plan/next local plan waypoint is too far away from us and its not easily reachable

    // Now: Compute all minimal-width channels (w/ padding, etc.) and display them.

    // Define what a channel is
    // - local/min channel with braking? - Start simple, but extensible
    // - the max channel that steers towards min channel and will basically expand around min channel
    //  to speed up if poss -> but always guarantee min channel reachable if poss
    //
    //  Main goal: everything reachable with braking/slowest speed is something that we can reach (if slowly)
    //  Secondary: If there is connected space around our min channel we speed up
    //  Ternary: Maybe drive a "bend" around the min channel (e.g. farther away from wall) to get more speed even if we leave the min channel
    //
    //  Define what makes a min/max channel parameter-wise -> What is input, what is output param of channel?
    // Then impl and display the channel computations.
    //
    // Must judge the "best" channels given those impls.
    // Finally - derive control comamnds for driving to channel.

    return true;
}

}

