#include <ros/ros.h>
#include "retreat_controller.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "retreat_controller");
    ros::NodeHandle nh;

    ros::NodeHandle nhPriv("~");
    std::string fixed_frame = "/odom";
    nhPriv.param("fixed_frame", fixed_frame, fixed_frame);

    RetreatController ac("retreat_action", fixed_frame);

    ros::spin();
}

