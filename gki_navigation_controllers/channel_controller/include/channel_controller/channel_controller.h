#ifndef CHANNEL_CONTROLLER_H
#define CHANNEL_CONTROLLER_H

#include <nav_core/base_local_planner.h>
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

            double length() const { return from_pose_.inverseTimes(to_pose_).getOrigin().length(); }

    };

    class ChannelController : public nav_core::BaseLocalPlanner
    {
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

            visualization_msgs::Marker createChannelMarkers(
                    const std::vector<DriveChannel> & channels, double min_good_dist,
                    int best_idx) const;

            visualization_msgs::Marker createPoseMarker(const tf::Pose & pose,
                    double r, double g, double b,
                    const std::string & ns, int id = 0) const;

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

            ros::Publisher pub_markers_;
            ros::Publisher pub_local_plan_;

            /// Waypoints within this are considered reached (unless goal wpt)
            double waypoint_reached_dist_;
            double waypoint_reached_angle_;
            /// Goal waypoint is considered reached
            double goal_reached_dist_;
            double goal_reached_angle_;

            double min_vel_lin_;
            double max_vel_lin_;
            double min_vel_th_;
            double max_vel_th_;

            bool visualize_voronoi_;
            double vis_max_dist_;
    };

}

#endif

