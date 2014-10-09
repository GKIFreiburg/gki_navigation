#ifndef APPROACH_CONTROLLER_H
#define APPROACH_CONTROLLER_H

#include <string>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/server/simple_action_server.h>
#include <geometry_msgs/Pose.h>
#include <sensor_msgs/LaserScan.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <vector>
#include <boost/thread/mutex.hpp>

class ApproachController
{
    public:
        ApproachController(const std::string & action_name, const std::string & fixed_frame);

    private:
        void executeCB(const move_base_msgs::MoveBaseGoalConstPtr & goal);

        // TODO how is this with threading. Do we need to mutex that with the main spin?
        // Or do we need our own spin inside the exec cb?
        void laserCallback(const sensor_msgs::LaserScan & laser);

        /// Get the best estimate what we should steer towards
        tf::Pose getTargetPose();

        void publishVel(double tv, double rv);
    private:
        std::string action_name_;
        std::string fixed_frame_;

        ros::NodeHandle nh_;
        actionlib::SimpleActionServer<move_base_msgs::MoveBaseAction> as_; 

        tf::TransformListener tf_;

        ros::Subscriber sub_laser_;
        ros::Subscriber sub_marker_;
        ros::Subscriber sub_line_features_;

        ros::Publisher pub_vel_;
        ros::Publisher pub_vis_;

        geometry_msgs::PoseStamped goal_pose_;
        geometry_msgs::PoseStamped marker_pose_;
        geometry_msgs::PoseStamped line_feature_pose_;
        // also maybe lines to get perpendicular/project?

        boost::mutex laser_mutex_;
        sensor_msgs::LaserScan last_laser_;

        /// laser points in base frame
        std::vector<tf::Vector3> base_laser_points_;
        // laser scan for not crashing/distance.
};

#endif

