#ifndef CHANNEL_CONTROLLER_H
#define CHANNEL_CONTROLLER_H

#include <nav_core/base_local_planner.h>
#include "dynamicvoronoi/dynamicvoronoi.h"

namespace channel_controller
{
    // TODO min setup
    // Upon that: What fns will I need in general?
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

        protected:
            tf::TransformListener* tf_;
            costmap_2d::Costmap2DROS* costmap_ros_;
            costmap_2d::Costmap2D costmap_;
            DynamicVoronoi voronoi_;

            ros::Publisher pub_markers_;

            double min_vel_lin_;
            double max_vel_lin_;
            double min_vel_th_;
            double max_vel_th_;

            bool visualize_voronoi_;
            double vis_max_dist_;
    };

}

#endif

