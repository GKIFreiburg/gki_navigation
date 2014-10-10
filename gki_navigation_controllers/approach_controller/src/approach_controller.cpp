#include "approach_controller.h"
#include <utility>
#include <image_geometry/pinhole_camera_model.h>
#include <angles/angles.h>
#include <geometry_msgs/Twist.h>
#include <boost/foreach.hpp>
#include <visualization_msgs/Marker.h>
#define forEach BOOST_FOREACH

double straight_up(double x, double a, double b)
{
    // linearly go up from 0 to 1 between a and b
    if(x <= a)
        return 0.0;
    if(x >= b)
        return 1.0;
    return (x - a)/(b - a);
}

double straight_down(double x, double a, double b)
{
    // linearly go down from 1 to 0 between a and b
    if(x <= a)
        return 1.0;
    if(x >= b)
        return 0.0;
    return (b - x)/(b - a);
}


ApproachController::ApproachController(const std::string & action_name, const std::string & fixed_frame) :
    action_name_(action_name), fixed_frame_(fixed_frame),
    as_(nh_, action_name, boost::bind(&ApproachController::executeCB, this, _1), false)
{
    sub_odom_ = nh_.subscribe("odom", 2, &ApproachController::odomCallback, this);
    sub_laser_ = nh_.subscribe("base_scan_filtered", 2, &ApproachController::laserCallback, this);
    sub_line_features_ = nh_.subscribe("isolated_lines", 2, &ApproachController::lineFeaturesCallback, this);
    sub_line_features2_ = nh_.subscribe("connected_lines", 2, &ApproachController::lineFeaturesCallback, this);
    sub_line_list_ = nh_.subscribe("laser_lines", 2, &ApproachController::lineListCallback, this);
    sub_marker_ = nh_.subscribe("/worldmodel/marker_percept", 3, &ApproachController::imagePerceptCallback, this);

    pub_vel_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel_mux/input/navi", 1);

    pub_vis_ = nh_.advertise<visualization_msgs::Marker>("approach_vis", 30);

    approach_dist_ = 0.4;
    succeeded_dist_ = 0.15;
    marker_max_height_ = 0.75;
    marker_max_dist_ = 1.5;

    ros::NodeHandle nhPriv("~");
    nhPriv.param("approach_dist", approach_dist_, approach_dist_);
    nhPriv.param("succeeded_dist", succeeded_dist_, succeeded_dist_);
    nhPriv.param("marker_max_height", marker_max_height_, marker_max_height_);
    nhPriv.param("marker_max_dist", marker_max_dist_, marker_max_dist_);

    ROS_INFO("ApproachController: approach_dist: %f", approach_dist_);
    ROS_INFO("ApproachController: succeeded_dist: %f", succeeded_dist_);
    ROS_INFO("ApproachController: marker_max_height: %f", marker_max_height_);
    ROS_INFO("ApproachController: marker_max_dist: %f", marker_max_dist_);

    as_.start();
}

void ApproachController::executeCB(const move_base_msgs::MoveBaseGoalConstPtr & goal)
{
    {
        boost::unique_lock<boost::mutex> scoped_lock_tf(tf_mutex_);
        try {
            if(!tf_.waitForTransform(fixed_frame_,
                        goal->target_pose.header.frame_id, goal->target_pose.header.stamp, ros::Duration(1.0))) {
                ROS_ERROR("Current goal pose TF not available");
                as_.setAborted();
                return;
            }
            tf_.transformPose(fixed_frame_,
                    goal->target_pose, goal_pose_);
            goal_pose_.pose.position.z = goal->target_pose.pose.position.z; // keep the z value set from outside
        } catch(tf::TransformException & e) {
            ROS_ERROR("%s: TF Error: %s", __func__, e.what());
            as_.setAborted();
            return;
        }
    }
    {
        boost::unique_lock<boost::mutex> scoped_lock(odom_mutex_);
        approach_start_pose_ = last_odom_pose_;
    }

    visualization_msgs::Marker marker;
    marker.header = goal_pose_.header;
    marker.ns = "goal_target";
    marker.type = visualization_msgs::Marker::ARROW;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose = goal_pose_.pose;
    marker.scale.x = 0.25;
    marker.scale.y = 0.1;
    marker.scale.z = 0.1;
    marker.color.g = 1.0;
    marker.color.b = 1.0;
    marker.color.a = 1.0;
    marker.lifetime = ros::Duration(30.0);
    pub_vis_.publish(marker);

    {
        boost::unique_lock<boost::mutex> scoped_lock(line_feature_pose_mutex_);
        line_feature_pose_.header.stamp = ros::Time(0);
    }

    {
        boost::unique_lock<boost::mutex> scoped_lock(marker_pose_mutex_);
        marker_poses_.clear();
    }

    ros::Rate rate(10.0);
    while(!as_.isPreemptRequested() && ros::ok())
    {
        double minX = HUGE_VAL;
        {
            boost::unique_lock<boost::mutex> scoped_lock(laser_mutex_);
            if(base_laser_points_.empty()) {
                ROS_WARN_THROTTLE(1.0, "ApproachController: no laser points in front of robot. stopping.");
                rate.sleep();
                continue;
            }
            forEach(const tf::Vector3 pt, base_laser_points_) {
                if(pt.x() < minX)
                    minX = pt.x();
            }
        }
        ROS_INFO_THROTTLE(1.0, "ApproachController: min laser dist: %f", minX);

        geometry_msgs::PoseStamped cur_odom_pose;
        {
            boost::unique_lock<boost::mutex> scoped_lock(odom_mutex_);
            cur_odom_pose = last_odom_pose_;
        }
        tf::Pose startPoseTf;
        tf::Pose curPoseTf;
        tf::poseMsgToTF(approach_start_pose_.pose, startPoseTf);
        tf::poseMsgToTF(cur_odom_pose.pose, curPoseTf);
        tf::Pose deltaPose = startPoseTf.inverseTimes(curPoseTf);
        if(deltaPose.getOrigin().length() > 2.0) {    // driving for 2m is not good
            as_.setAborted();
            return;
        }
        // turning away more than 45 deg is also not good
        if(tf::getYaw(deltaPose.getRotation()) > angles::from_degrees(45.0)) {
            as_.setAborted();
            return;
        }
        // FIXME: Maybe wait until we have a marker observation
        // Or is it better to just go, changing perspective.

        // TODO maybe always succeed?
        bool poseOK = false;
        tf::Pose targetPose = getTargetPose(poseOK);
        //ROS_INFO("TP: %f %f", targetPose.getOrigin().x(), targetPose.getOrigin().y());
        // This is empty in failure, warn and go straight
        if(!poseOK) {
            ROS_ERROR_THROTTLE(1.0, "Invalid approach target: Going straight.");

            if(minX < approach_dist_) {
                as_.setAborted();
                return;
            }

            publishVel(0.08, 0.0);
        } else {
            double dist = hypot(targetPose.getOrigin().y(), targetPose.getOrigin().x());
            ROS_INFO_THROTTLE(1.0, "Dist to target: %f", dist);
            if(dist < 0.01) {
                ROS_INFO("ApproachController: Success: within close target distance: %f", dist);
                as_.setSucceeded();
                return;
            }
            if(minX < approach_dist_) {
                if(dist < succeeded_dist_) {   // close enough
                    ROS_INFO("ApproachController: Success: within target distance: %f", dist);
                    as_.setSucceeded();
                }
                else {
                    ROS_WARN("ApproachController: Failure: outside target distance: %f", dist);
                    as_.setAborted();
                }
                return;
            }

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
    double tv = 0;
    double rv = 0;

    // OK So the pose is in the wall, pointing outwards by 25cm, i.e. ring centner
    // The frame of the pose is the cube holder, i.e. what we wanna position
    // Basically drive right at that thing!

    double dist = hypot(pose.getOrigin().y(), pose.getOrigin().x());
    double da = atan2(pose.getOrigin().y(), pose.getOrigin().x());
    ROS_INFO_THROTTLE(0.5, "target rel: %f %f %f da: %f", pose.getOrigin().x(), pose.getOrigin().y(),
            angles::to_degrees(tf::getYaw(pose.getRotation())), angles::to_degrees(da));
    //tf::Pose pinv = pose.inverse();
    //ROS_INFO("drive rel: %f %f %f", pinv.getOrigin().x(), pinv.getOrigin().y(),
    //        angles::to_degrees(tf::getYaw(pinv.getRotation())));

    // OK need to steer this pose so its zero
    if(fabs(da) > angles::from_degrees(15.0)) {
        rv = 0.18 + 0.6 * straight_up(fabs(da), angles::from_degrees(15), angles::from_degrees(90));
        if(da < 0)
            rv = -rv;
    } else {
        tv = 0.08;
        rv = 0.4 * straight_up(fabs(da), angles::from_degrees(0), angles::from_degrees(20));
        if(da < 0)
            rv = -rv;
    }
    if(fabs(dist) < 0.01)   // da will be incorrect/noisy so close
        rv = 0;
    publishVel(tv, rv);
}

void ApproachController::publishVel(double tv, double rv)
{
    geometry_msgs::Twist cmd_vel;
    cmd_vel.linear.x = tv;
    cmd_vel.angular.z = rv;
    pub_vel_.publish(cmd_vel);
}

void ApproachController::lineListCallback(const laser_line_detection::LineList & lineList)
{
    line_list_ = lineList;
}

void ApproachController::odomCallback(const nav_msgs::Odometry & odom)
{
    boost::unique_lock<boost::mutex> scoped_lock(odom_mutex_);
    last_odom_pose_.header = odom.header;
    last_odom_pose_.pose = odom.pose.pose;
}

void ApproachController::imagePerceptCallback(const hector_worldmodel_msgs::ImagePercept & imagePercept)
{
    image_geometry::PinholeCameraModel model;
    if(!model.fromCameraInfo(imagePercept.camera_info)) {
        ROS_ERROR_THROTTLE(1.0, "ApproachController: imagePerceptCallback: could not build camera model.");
        return;
    }

    cv::Point3d ray = model.projectPixelTo3dRay(cv::Point2d(imagePercept.x, imagePercept.y));
    tf::Vector3 rayTf(ray.x, ray.y, ray.z);
    rayTf.normalize();
    tf::Vector3 rayEnd = rayTf * marker_max_dist_;
    // the point 0,0,0 and rayEnd are both in the camera frame.
    // We can build a new ray in the laser frame to match to the lines.
    geometry_msgs::PointStamped cameraStart;
    geometry_msgs::PointStamped cameraRayEnd;
    cameraStart.header = imagePercept.header;
    cameraRayEnd.header = imagePercept.header;
    // cameraStart is at 0, 0, 0
    tf::pointTFToMsg(rayEnd, cameraRayEnd.point);
    geometry_msgs::PointStamped cameraLaserStart;
    geometry_msgs::PointStamped cameraLaserEnd;
    // transform to get start and end in laser frame
    {
        boost::unique_lock<boost::mutex> scoped_lock_tf(tf_mutex_);
        try {
            // FIXME correct would be tiem travel to laser lines
            // this should be close enough
            if(!tf_.waitForTransform(line_list_.header.frame_id,
                        imagePercept.header.frame_id, imagePercept.header.stamp, ros::Duration(0.2))) {
                ROS_ERROR("Current image percept to laser lines TF not available");
                return;
            }
            tf_.transformPoint(line_list_.header.frame_id,
                    cameraStart, cameraLaserStart);
            tf_.transformPoint(line_list_.header.frame_id,
                    cameraRayEnd, cameraLaserEnd);
        } catch(tf::TransformException & e) {
            ROS_ERROR("%s: TF Error: %s", __func__, e.what());
            return;
        }
    }

    visualization_msgs::Marker marker;
    marker.header = line_list_.header;
    marker.ns = "marker_ray";
    marker.type = visualization_msgs::Marker::ARROW;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.orientation.w = 1;
    marker.scale.x = 0.1;
    marker.scale.y = 0.2;
    marker.scale.z = 0.2;
    marker.color.r = 1.0;
    marker.color.a = 1.0;
    marker.lifetime = ros::Duration(2.0);
    marker.points.push_back(cameraLaserStart.point);
    marker.points.push_back(cameraLaserEnd.point);
    pub_vis_.publish(marker);

    tf::Vector3 cameraLaserStartTf;
    tf::Vector3 cameraLaserEndTf;
    tf::pointMsgToTF(cameraLaserStart.point, cameraLaserStartTf);
    tf::pointMsgToTF(cameraLaserEnd.point, cameraLaserEndTf);
    std::vector< std::pair<tf::Vector3, tf::Vector3> > intersections;
    forEach(const laser_line_detection::Line & line, line_list_.lines) {
        tf::Vector3 lineStartTf;
        tf::Vector3 lineEndTf;
        tf::pointMsgToTF(line.start, lineStartTf);
        tf::pointMsgToTF(line.end, lineEndTf);
        tf::Vector3 intersection;
        if(lineIntersects(
                    lineStartTf, lineEndTf,
                    cameraLaserStartTf, cameraLaserEndTf,
                    intersection
                    )) {
            if(intersection.z() > marker_max_height_) {
                continue;
            }
            // this is laser frame, so no points behind:
            if(intersection.x() <= 0)
                continue;

            // found something
            intersections.push_back(std::make_pair(intersection, lineEndTf - lineStartTf));
        }
    }
    if(intersections.empty()) {
        ROS_WARN_THROTTLE(1.0, "Found no valid line intersection with marker percept");
        return;
    }

    tf::Vector3 bestIntersection = intersections.front().first;
    tf::Vector3 bestIntersectionLineDirection = intersections.front().second;
    // best = closest to laser...
    for(std::vector< std::pair<tf::Vector3, tf::Vector3> >::iterator it = intersections.begin();
            it != intersections.end(); it++) {
        if(hypot(bestIntersection.x(), bestIntersection.y()) > hypot(it->first.x(), it->first.y())) {
            bestIntersection = it->first;
            bestIntersectionLineDirection = it->second;
        }
    }
    tf::Vector3 lineNormalA(-bestIntersectionLineDirection.y(), bestIntersectionLineDirection.x(), 0.0);
    tf::Vector3 lineNormalB(bestIntersectionLineDirection.y(), -bestIntersectionLineDirection.x(), 0.0);
    lineNormalA.normalize();
    lineNormalB.normalize();
    tf::Vector3 laserDirection(1.0, 0.0, 0.0);
    double dotA = lineNormalA.dot(laserDirection);
    double dotB = lineNormalB.dot(laserDirection);
    // take the normal more pointing towards us, i.e. against the laser
    tf::Vector3 normal;
    if(dotA < dotB) {   // negative ideally
        normal = lineNormalA;
    } else {
        normal = lineNormalB;
    }

    // we have the intersection point and normal.
    // put them into a pose in the laser frame.
    tf::Quaternion rot = tf::shortestArcQuat(normal, laserDirection);
    geometry_msgs::PoseStamped intersectionPose;
    intersectionPose.header = line_list_.header;
    tf::quaternionTFToMsg(rot, intersectionPose.pose.orientation);
    tf::pointTFToMsg(bestIntersection, intersectionPose.pose.position);

    // convert to fixed_frame_
    geometry_msgs::PoseStamped intersectionPoseFixed;
    {
        boost::unique_lock<boost::mutex> scoped_lock_tf(tf_mutex_);
        try {
            // FIXME correct would be tiem travel to laser lines
            // this should be close enough
            if(!tf_.waitForTransform(fixed_frame_,
                        intersectionPose.header.frame_id, intersectionPose.header.stamp, ros::Duration(0.2))) {
                ROS_ERROR("Current intersection pose to fixed TF not available");
                return;
            }
            tf_.transformPose(fixed_frame_,
                    intersectionPose, intersectionPoseFixed);
        } catch(tf::TransformException & e) {
            ROS_ERROR("%s: TF Error: %s", __func__, e.what());
            return;
        }
    }

    // visualize
    usleep(20*1000);
    visualization_msgs::Marker markerTarget;
    markerTarget.header = intersectionPoseFixed.header;
    markerTarget.ns = "marker_target";
    markerTarget.type = visualization_msgs::Marker::ARROW;
    markerTarget.action = visualization_msgs::Marker::ADD;
    markerTarget.pose = intersectionPoseFixed.pose;
    markerTarget.scale.x = 0.25;
    markerTarget.scale.y = 0.1;
    markerTarget.scale.z = 0.1;
    markerTarget.color.r = 1.0;
    markerTarget.color.b = 1.0;
    markerTarget.color.a = 1.0;
    markerTarget.lifetime = ros::Duration(1.0);
    pub_vis_.publish(markerTarget);

    // store it!
    boost::unique_lock<boost::mutex> scoped_lock(marker_pose_mutex_);
    marker_poses_.push_back(intersectionPoseFixed);
}

bool ApproachController::lineIntersects(const tf::Vector3 & s1, const tf::Vector3 & e1,
        const tf::Vector3 & s2, const tf::Vector3 & e2, tf::Vector3 & intersection) 
{
    tf::Vector3 d1 = e1 - s1;
    tf::Vector3 d2 = e2 - s2;
    tf::Vector3 b = s1 - s2;

    // first check the parallelism
    tf::Vector3 d1proj = tf::Vector3(d1.x(), d1.y(), 0.0);
    d1proj.normalize();
    tf::Vector3 d2proj = tf::Vector3(d2.x(), d2.y(), 0.0);
    d2proj.normalize();
    double da = acos(fabs(d1proj.dot(d2proj)));
    if(fabs(da) < angles::from_degrees(20)) {   // too steep approach angle. This isn't it.
        return false;
    }

    double denom = d1.y() * d2.x() - d1.x() * d2.y();
    if(fabs(denom) < 0.001) // shouldnt happen from preivous cond
        return false;

    double lambda1 = (b.x() * d2.y() - b.y() * d2.x())/denom;
    // lambda is where the intersection is on line1
    // if it is between 0 and 1, it is between s1 and e1
    // i.e. we have an intersection on the line.
    if(lambda1 < 0.0 || lambda1 > 1.0) {
        return false;
    }
    // we want to hit both lines
    double lambda2 = (b.x() * d1.y() - b.y() * d1.x())/denom;
    if(lambda2 < 0.0 || lambda2 > 1.0) {
        return false;
    }

    ROS_INFO("s1: %f %f, e1: %f %f, d1: %f %f", s1.x(), s1.y(), e1.x(), e1.y(), d1.x(), d1.y());
    ROS_INFO("s2: %f %f, e2: %f %f, d2: %f %f", s2.x(), s2.y(), e2.x(), e2.y(), d2.x(), d2.y());
    ROS_INFO("lambda1: %f", lambda1);

    intersection = s1 + lambda1 * d1;
    return true;
}

void ApproachController::lineFeaturesCallback(const geometry_msgs::PoseArray & lineFeatures)
{
    // This collects isolated and connected lines likewise.
    // if one of those isn't centered we can still not subscribe if not trustworthy
    // FIXME for now assuming laser frame!!!

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
        if(pose.position.x > 2.0)   // too far away
            continue;

        if(!poseFound)
            best_pose.pose = pose;
        else {
            // Take the one that's closed //    in y coord
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
                            best_pose.header.frame_id, best_pose.header.stamp, ros::Duration(0.2))) {
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
        marker.lifetime = ros::Duration(1.0);
        marker.scale.x = 0.25;
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
    marker.lifetime = ros::Duration(1.0);
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

tf::Pose ApproachController::getTargetPose(bool & valid)
{
    valid = false;
    ros::Time now = ros::Time::now();
    geometry_msgs::PoseStamped bestPose;
    bool bestPoseFound = false;

    // a hardcoded z value for the goal means the following:
    // 1.0 - approach a number
    // -1.0 - allows line_feature_pose_
    if(goal_pose_.pose.position.z < 0) {
        boost::unique_lock<boost::mutex> scoped_lock(line_feature_pose_mutex_);
        double dtLineFeaturePose = (now - line_feature_pose_.header.stamp).toSec();
        //printf("AGE: %f\n", dtLineFeaturePose);
        if(dtLineFeaturePose < 30.0) {
            bestPose = line_feature_pose_;
            bestPoseFound = true;
        }
    }
    {
        boost::unique_lock<boost::mutex> scoped_lock(marker_pose_mutex_);
        // Integrate markers
        // TODO what if one is off way
        if(!marker_poses_.empty()) {
            double dtMarkerPose = (now - marker_poses_.back().header.stamp).toSec();
            //printf("MARKER AGE: %f\n", dtMarkerPose);
            if(dtMarkerPose < 30.0) {
                bestPose.pose.position.x = 0;
                bestPose.pose.position.y = 0;
                bestPose.pose.position.z = 0;
                forEach(const geometry_msgs::PoseStamped & pose, marker_poses_) {
                    bestPose.pose.position.x += pose.pose.position.x;
                    bestPose.pose.position.y += pose.pose.position.y;
                    bestPose.pose.position.z += pose.pose.position.z;
                    bestPoseFound = true;
                }
                bestPose.pose.position.x /= (double)marker_poses_.size();
                bestPose.pose.position.y /= (double)marker_poses_.size();
                bestPose.pose.position.z /= (double)marker_poses_.size();
                bestPose.pose.orientation = marker_poses_.back().pose.orientation;
            }
        }
    }

    if(!bestPoseFound) {
        // let's assume the goal is good,
        bestPose = goal_pose_;
        bestPose.pose.position.z = 0.0;
    }

    // shift the pose out by 0.25 cm std for circles
    geometry_msgs::PoseStamped poseTarget;// = bestPose;
    poseTarget.header = bestPose.header;
    tf::Pose targetPose;
    tf::poseMsgToTF(bestPose.pose, targetPose);
    tf::Pose ringOffset(tf::Quaternion(0,0,0,1), tf::Vector3(0.25, 0.0, 0.0));
    tf::Pose targetRingPose = targetPose * ringOffset;
    tf::poseTFToMsg(targetRingPose, poseTarget.pose);
    //ROS_INFO_STREAM(bestPose);
    //ROS_INFO_STREAM(poseTarget);

    geometry_msgs::PoseStamped poseRobot;
    poseTarget.header.stamp = ros::Time(0);
    {
        boost::unique_lock<boost::mutex> scoped_lock_tf(tf_mutex_);
        try {
            if(!tf_.waitForTransform("/cube_holder_link",
                        poseTarget.header.frame_id, poseTarget.header.stamp, ros::Duration(0.2))) {
                ROS_ERROR("Current target pose TF not available");
                return tf::Pose();
            }
            tf_.transformPose("/cube_holder_link",
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
    marker.lifetime = ros::Duration(5.0);
    marker.scale.x = approach_dist_;
    marker.scale.y = 0.15;
    marker.scale.z = 0.15;
    marker.color.g = 1.0;
    marker.color.a = 0.5;
    pub_vis_.publish(marker);

    valid = true;
    return tfPose;
}

