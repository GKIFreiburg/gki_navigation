#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>

pcl::PointCloud<pcl::PointXYZ> cloud;
ros::Publisher pubCloud;
tf::TransformListener* tfl;
std::string map_frame_id;

void publish_obstacle_cloud()
{
    if(cloud.points.size() <= 0)
        return;

    tf::StampedTransform baseToMap;
    try {
        tfl->waitForTransform("base_footprint", map_frame_id, ros::Time(0), ros::Duration(10.0));
        tfl->lookupTransform("base_footprint", map_frame_id, ros::Time(0), baseToMap);
    } catch (tf::TransformException ex){
        ROS_ERROR("%s",ex.what());
    }

    pcl::PointCloud<pcl::PointXYZ> cloudOut;
    cloudOut.header.frame_id = "base_footprint";
    cloudOut.header.stamp = baseToMap.stamp_;
    for(unsigned int i = 0; i < cloud.points.size(); i++) {
        tf::Point pin(cloud.points[i].x, cloud.points[i].y, cloud.points[i].z);
        tf::Point pout = baseToMap * pin;
        cloudOut.points.push_back(pcl::PointXYZ(pout.x(), pout.y(), pout.z()));
    }
    pubCloud.publish(cloudOut);
}

void map_callback(const nav_msgs::OccupancyGrid & map)
{
    ROS_INFO("static_map_to_obstacle_cloud: Got new map.");
    map_frame_id = map.header.frame_id;
    tf::Pose mapOrigin;
    tf::poseMsgToTF(map.info.origin, mapOrigin);

    pcl::PointCloud<pcl::PointXYZ> cloudNew;
    for(unsigned int i = 0; i < map.data.size(); i++) {
        if(map.data[i] > 50) {  // occupied
            int ix = i % map.info.width;
            int iy = i / map.info.width;
            //ROS_INFO("Map pt at %d %d", ix, iy);
            tf::Point pt(ix * map.info.resolution + 0.5 * map.info.resolution,
                    iy * map.info.resolution + 0.5 * map.info.resolution, 0.0);
            tf::Point ptGlobal = mapOrigin * pt;

            cloudNew.points.push_back(pcl::PointXYZ(ptGlobal.x(), ptGlobal.y(), ptGlobal.z()));
        }
    }

    cloud = cloudNew;
    cloud.header = map.header;
    ROS_INFO("Current cloud has %zu points", cloud.points.size());
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "static_map_to_obstacle_cloud");
    ros::NodeHandle nh;
    tfl = new tf::TransformListener();

    ros::Subscriber subMap = nh.subscribe("map", 1, map_callback);
    pubCloud = nh.advertise<pcl::PointCloud<pcl::PointXYZ> >("obstacle_cloud", 1);

    ros::Rate rate(2.0);
    while(ros::ok())
    {
        ros::spinOnce();
        publish_obstacle_cloud();
        rate.sleep();
    }
}
