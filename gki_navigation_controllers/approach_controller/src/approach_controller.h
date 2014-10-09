#ifndef APPROACH_CONTROLLER_H
#define APPROACH_CONTROLLER_H

#include <string>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/server/simple_action_server.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>
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

        void laserCallback(const sensor_msgs::LaserScan & laser);

        void lineFeaturesCallback(const geometry_msgs::PoseArray & lineFeatures);

        /// Get the best estimate what we should steer towards
        /**
         * In the /base_footprint frame.
         * Empty pose if failure.
         */
        tf::Pose getTargetPose();

        void publishVel(double tv, double rv);

        void driveToPose(const tf::Pose & pose);

    private:
        std::string action_name_;
        std::string fixed_frame_;

        double approach_dist_;

        ros::NodeHandle nh_;
        actionlib::SimpleActionServer<move_base_msgs::MoveBaseAction> as_; 

        tf::TransformListener tf_;

        ros::Subscriber sub_laser_;
        ros::Subscriber sub_marker_;
        ros::Subscriber sub_line_features_;
        ros::Subscriber sub_line_features2_;

        ros::Publisher pub_vel_;
        ros::Publisher pub_vis_;

        // those need to be in fixed frame
        geometry_msgs::PoseStamped goal_pose_;
        geometry_msgs::PoseStamped marker_pose_;
        boost::mutex line_feature_pose_mutex_;
        geometry_msgs::PoseStamped line_feature_pose_;
        // also maybe lines to get perpendicular/project?

        boost::mutex laser_mutex_;
        sensor_msgs::LaserScan last_laser_;

        /// laser points in base frame
        std::vector<tf::Vector3> base_laser_points_;
        // laser scan for not crashing/distance.
};

#endif

