#ifndef CHANNEL_CONTROLLER_H
#define CHANNEL_CONTROLLER_H

#include <nav_core/base_local_planner.h>
#include <nav_msgs/Odometry.h>
#include <visualization_msgs/MarkerArray.h>
#include "dynamicvoronoi/dynamicvoronoi.h"

namespace channel_controller
{
    class DriveChannel
    {
        public:
            tf::Pose from_pose_;
            tf::Pose to_pose_;
            double min_dist_;      ///< min width along the whole channel
            double da_;            ///< angle delta to next waypoint

            double length() const { return from_pose_.inverseTimes(to_pose_).getOrigin().length(); }

    };

    class ChannelController : public nav_core::BaseLocalPlanner
    {
        public:
            /// Simple class that reports on the current status for computeVelocityCommands.
            class ScopedVelocityStatus
            {
                public:
                    ScopedVelocityStatus(geometry_msgs::Twist & cmdVel,
                            ros::Publisher & pubStatus, const costmap_2d::Costmap2DROS* costmap);
                    ~ScopedVelocityStatus();

                    // goal approach
                    void setAtGoalPosStopToTurn(double angle_to_goal, double cur_tv);
                    void setAtGoalPosTurnToGoal(double angle_to_goal, double cur_tv);

                    // default channel behaviors
                    void setChannelStopToTurn(double rel_channel_dir, double cur_tv);
                    void setChannelTurnToChannel(double rel_channel_dir, double cur_tv);

                    void setChannelFollowChannel(double rel_channel_dir,
                            const std::string & tv_scale, const std::string & rv_scale);

                    // failures/recoveries
                    void setNoValidChannel();

                private:
                    void publishStatus();

                private:
                    geometry_msgs::Twist & cmd_vel;
                    ros::Publisher & pub_markers;
                    const costmap_2d::Costmap2DROS* costmap;

                    std::stringstream status;
            };

        public:
            ChannelController();
            virtual ~ChannelController();

            virtual void initialize(std::string name,
                    tf::TransformListener* tf, costmap_2d::Costmap2DROS* costmap_ros);

            virtual bool isGoalReached();

            virtual bool setPlan(const std::vector<geometry_msgs::PoseStamped> & plan);

            virtual bool computeVelocityCommands(geometry_msgs::Twist & cmd_vel);

        protected:
            void updateVoronoi();
            void visualizeVoronoi();

            /// Transform global_plan_ to the controller frame starting at start_index.
            bool localizeGlobalPlan(unsigned int start_index);

            bool currentWaypointReached() const;

            /// Compute max length of channel in a direction angle that has guaranteed clearance_dist.
            DriveChannel computeChannel(tf::Pose from_pose, tf::Pose to_pose, double clearance_dist) const;

            std::vector<DriveChannel> computeChannels(const tf::Pose & robotPose,
                    const tf::Pose & relativeTarget) const;

            /// Evaluate channels and return the index of the best one.
            int evaluateChannels(const std::vector<DriveChannel> & channels, double distToTarget) const;

            bool computeVelocityForChannel(const DriveChannel & channel, geometry_msgs::Twist & cmd_vel, ScopedVelocityStatus & status) const;

            void limitTwist(geometry_msgs::Twist & cmd_vel) const;

            bool handleGoalApproach(geometry_msgs::Twist & cmd_vel,
                    const tf::Stamped<tf::Pose> & robotPose, double distToTarget,
                    ScopedVelocityStatus & status) const;

            visualization_msgs::Marker createChannelMarkers(
                    const std::vector<DriveChannel> & channels, double min_good_dist,
                    int best_idx) const;

            visualization_msgs::Marker createPoseMarker(const tf::Pose & pose,
                    double r, double g, double b,
                    const std::string & ns, int id = 0) const;

            inline double sign(double x) const {
                if(x < 0)
                    return -1.0;
                return 1.0;
            }

            void odometryCallback(const nav_msgs::Odometry & odom);

            double straight_up(double x, double a, double b) const;
            double straight_down(double x, double a, double b) const;

        protected:
            tf::TransformListener* tf_;
            costmap_2d::Costmap2DROS* costmap_ros_;
            costmap_2d::Costmap2D costmap_;
            DynamicVoronoi voronoi_;

            /// Current waypoint that we are approaching in the global plan.
            unsigned int current_waypoint_;
            /// Global plan as set externally
            std::vector<geometry_msgs::PoseStamped> global_plan_;
            /// Localized global plan in the cost map's global frame
            /// starting at the current_waypoint_
            std::vector< tf::Stamped<tf::Pose> > local_plan_;

            ros::Subscriber sub_odom_;
            ros::Publisher pub_markers_;
            ros::Publisher pub_status_marker_;
            ros::Publisher pub_local_plan_;

            nav_msgs::Odometry last_odom_;

            ros::Time last_cmd_vel_time_;   ///< last time we send a command

            // Controller Parameters

            /// Waypoints within this are considered reached (unless goal wpt)
            double waypoint_reached_dist_;
            double waypoint_reached_angle_;
            /// Goal waypoint is considered reached
            double goal_reached_dist_;
            double goal_reached_angle_;

            double min_tv_;         ///< min tv that keeps the robot moving
            double max_tv_;         ///< max tv that the robot can do
            double min_rv_;         ///< min rv that keeps rotating when also driving with tv
            double max_rv_;         ///< max rv that the robot can do
            double min_inplace_rv_; ///< min rv that makes the robot rotate when not driving with tv

            double stopped_tv_;     ///< Trans vel smaller than this - consider stopped
            double stopped_rv_;     ///< Rot vel smaller than this - consider stopped

            // TODO limit this somewhere, but also consider braking for obstacles hard!
            double max_accel_tv_;   ///< Max change in tv per second
            double max_accel_rv_;   ///< Max change in rv per second

            bool visualize_voronoi_;
            double vis_max_dist_;
    };

}

#endif

