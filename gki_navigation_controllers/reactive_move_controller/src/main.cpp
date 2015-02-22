#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>

ros::Subscriber laser_subscriber;
ros::Publisher velocity_publisher;

void laser_callback(sensor_msgs::LaserScanConstPtr msg)
{
    // TODO: create behavior
    msg->angle_min; // starting angle
    msg->angle_max; // ending angle
    msg->angle_increment; // angular resolution
    msg->ranges; // sensor data list
    // useful funcitons:
    // sin(0.5);
    // cos(0.5);
    // hypot(a, b); hypotenuse in triangle abc, length of c
    // atan2(b, a); angle in triangle abc, angle alpha


    geometry_msgs::Twist velocity;
    // translational velocity
    velocity.linear.x = 0.0;
    // rotational velocity
    velocity.angular.z = 0.0;
    velocity_publisher.publish(velocity);
}

int main(int argc, char** argv)
{
    // initialize
    ros::init(argc, argv, "reactive_move_controller");
    ros::NodeHandle nh;
    ROS_INFO("subscribing to laser topic...");
    laser_subscriber = nh.subscribe("base_scan", 1, laser_callback);
    ROS_INFO("advertizing command velocity topic...");
    velocity_publisher = nh.advertise<geometry_msgs::Twist>("command_velocity", 1, false);
    ROS_INFO("reactive move controller initialized.");

    // loop
    ros::spin();
}
