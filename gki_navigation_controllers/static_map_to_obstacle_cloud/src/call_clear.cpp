#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <std_msgs/Empty.h>

bool trigger_call = false;

void call_clear_callback(const std_msgs::Empty & e)
{
    trigger_call = true;
}

void check_call_clear()
{
    if(trigger_call) {
        std_srvs::Empty e;
        ROS_INFO("Calling clear_costmaps");
        ros::service::call("move_base/clear_costmaps", e);
        ROS_INFO("Called clear_costmaps");
        trigger_call = false;
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "call_clear");
    ros::NodeHandle nh;

    ros::Subscriber sub = nh.subscribe("call_clear", 1, call_clear_callback);

    ros::Rate rate(10.0);
    while(ros::ok()) {
        ros::spinOnce();
        check_call_clear();
        rate.sleep();
    }
}

