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
#include <laser_line_detection/LineList.h>
#include <hector_worldmodel_msgs/ImagePercept.h>
#include <boost/thread/mutex.hpp>

class ApproachController
{
    public:
        ApproachController(const std::string & action_name, const std::string & fixed_frame);

    private:
        void executeCB(const move_base_msgs::MoveBaseGoalConstPtr & goal);

        void laserCallback(const sensor_msgs::LaserScan & laser);

        void lineFeaturesCallback(const geometry_msgs::PoseArray & lineFeatures);

        void lineListCallback(const laser_line_detection::LineList & lineList);

        void imagePerceptCallback(const hector_worldmodel_msgs::ImagePercept & imagePercept);

        bool lineIntersects(const tf::Vector3 & p1s, const tf::Vector3 & p1e,
                const tf::Vector3 & p2s, const tf::Vector3 & p2e, tf::Vector3 & intersection); 

        /// Get the best estimate what we should steer towards
        /**
         * In the /cube_holder_link frame.
         * Empty pose if failure.
         */
        tf::Pose getTargetPose(bool & valid);

        void publishVel(double tv, double rv);

        void driveToPose(const tf::Pose & pose);

    private:
        std::string action_name_;
        std::string fixed_frame_;

        double approach_dist_;
        double succeeded_dist_;
        double marker_max_dist_;
        double marker_max_height_;

        ros::NodeHandle nh_;
        actionlib::SimpleActionServer<move_base_msgs::MoveBaseAction> as_; 

        boost::mutex tf_mutex_;
        tf::TransformListener tf_;

        ros::Subscriber sub_laser_;
        ros::Subscriber sub_line_features_;
        ros::Subscriber sub_line_features2_;
        ros::Subscriber sub_line_list_;
        ros::Subscriber sub_marker_;

        ros::Publisher pub_vel_;
        ros::Publisher pub_vis_;

        // those need to be in fixed frame
        geometry_msgs::PoseStamped goal_pose_;
        boost::mutex marker_pose_mutex_;
        geometry_msgs::PoseStamped marker_pose_;
        boost::mutex line_feature_pose_mutex_;
        geometry_msgs::PoseStamped line_feature_pose_;
        // also maybe lines to get perpendicular/project?

        laser_line_detection::LineList line_list_;

        boost::mutex laser_mutex_;
        sensor_msgs::LaserScan last_laser_;

        /// laser points in base frame
        std::vector<tf::Vector3> base_laser_points_;
        // laser scan for not crashing/distance.
};

#endif

