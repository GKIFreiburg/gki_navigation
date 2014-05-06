#ifndef GLOBAL_PLANNER_STRAIGHT_AHEAD_H
#define GLOBAL_PLANNER_STRAIGHT_AHEAD_H

#include <nav_core/base_global_planner.h>

namespace global_planner_straight_ahead
{
    class GlobalPlannerStraightAhead : public nav_core::BaseGlobalPlanner
    {
        public:
            virtual bool makePlan(const geometry_msgs::PoseStamped & start, 
                    const geometry_msgs::PoseStamped & goal,
                    std::vector<geometry_msgs::PoseStamped> & plan);

            virtual void initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros);

        private:
            ros::Publisher pub_path_;
    };
}

#endif

