#include "channel_controller/channel_controller.h"
#include <pluginlib/class_list_macros.h>
#include <string>
#include <visualization_msgs/MarkerArray.h>

namespace channel_controller
{

PLUGINLIB_EXPORT_CLASS(channel_controller::ChannelController, nav_core::BaseLocalPlanner);

ChannelController::ChannelController() : tf_(NULL), costmap_ros_(NULL)
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
    ssize_t colon = name.find_last_of(":");
    std::string class_name = name.substr(colon + 1);
    if(colon == std::string::npos)
        class_name = name;

    ROS_INFO("Initializing ChannelController from ~/%s", class_name.c_str());
    ros::NodeHandle nhPriv("~/" + class_name);    // ~ = /move_base, our config should be in /move_base/name

    nhPriv.param("max_vel_lin", max_vel_lin_, 0.6);
    nhPriv.param("max_vel_th", max_vel_th_, 0.9);
    nhPriv.param("min_vel_lin", min_vel_lin_, 0.1);
    nhPriv.param("min_vel_th", min_vel_th_, 0.0);
    nhPriv.param("vis_max_dist", vis_max_dist_, 1.0);
    nhPriv.param("visualize_voronoi", visualize_voronoi_, false);

    pub_markers_ = nhPriv.advertise<visualization_msgs::MarkerArray>("channel_markers", 1);

    updateVoronoi();
}

void ChannelController::updateVoronoi()
{
    costmap_ros_->getCostmapCopy(costmap_);     // srsly we have to copy that to get to the data??????
    std::vector<IntPoint> obstacles;
    // TODO everywhere assert voronoi size == costmap size
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
    cellPoint.z = 0.0;
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
                    // make those slightly darker then max dist as max to distinguish
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
    return false;
}

bool ChannelController::setPlan(const std::vector<geometry_msgs::PoseStamped> & plan)
{
    // TODO return value means what?
}

bool ChannelController::computeVelocityCommands(geometry_msgs::Twist & cmd_vel)
{
    updateVoronoi();    // TODO how horrible is this performance-wise?
}

}

