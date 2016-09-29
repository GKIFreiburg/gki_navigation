#include <ros/ros.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Odometry.h>

std::string base_frame;
std::string odom_frame;
double x_stddev = 0.002;
double y_stddev = 0.002;
double rotation_stddev = 0.017;
double cov_xy = 0.0;
double cov_xrotation = 0.0;
double cov_yrotation = 0.0;

ros::Publisher odom_publisher;
nav_msgs::Odometry odom;
geometry_msgs::Pose2D pose;

template<typename T>
void paramCached(const ros::NodeHandle& nh, const std::string& param_name, T& param_val, const T& default_val)
{
    if (! nh.getParamCached(param_name, param_val))
    {
    	param_val = default_val;
        ROS_WARN_STREAM("could not read "<<param_name<<" parameter. assuming '"<<param_val<<"'");
    }
}

void updateParams()
{
    ros::NodeHandle n("~");
    // covariance matrix
	//	xx	xy	0	0	0	xth
	//	xy	yy	0	0	0	yth
	//	0	0	999	0	0	0
	//	0	0	0	999	0	0
	//	0	0	0	0	999	0
	//	xth	yth	0	0	0	thth
    paramCached(n, "x_stddev", odom.pose.covariance[0+0*6], 0.002);
    paramCached(n, "y_stddev", odom.pose.covariance[1+1*6], 0.002);
    paramCached(n, "rotation_stddev", odom.pose.covariance[5+5*6], 0.017);
    paramCached(n, "cov_xy", odom.pose.covariance[1+0*6], 0.0);
    odom.pose.covariance[0+1*6] = odom.pose.covariance[1+0*6];
    paramCached(n, "cov_xrotation", odom.pose.covariance[5+0*6], 0.0);
    odom.pose.covariance[0+5*6] = odom.pose.covariance[5+0*6];
    paramCached(n, "cov_yrotation", odom.pose.covariance[5+1*6], 0.0);
    odom.pose.covariance[1+5*6] = odom.pose.covariance[5+1*6];
    odom.pose.covariance[2+2*6] = 999;
    odom.pose.covariance[2+2*6] = 999;
    odom.pose.covariance[2+2*6] = 999;

    paramCached(n, "base_frame", odom.child_frame_id, std::string("base_link"));
    paramCached(n, "odom_frame", odom.header.frame_id, std::string("odom"));
}

void velocity_callback(const geometry_msgs::TwistStampedConstPtr msg)
{
	updateParams();

	const geometry_msgs::Twist& velocity = msg->twist;

    double dt = (odom.header.stamp - msg->header.stamp).toSec();
    double delta_x = (velocity.linear.x * cos(pose.theta) - velocity.linear.y * sin(pose.theta)) * dt;
    double delta_y = (velocity.linear.x * sin(pose.theta) + velocity.linear.y * cos(pose.theta)) * dt;
    double delta_yaw = velocity.angular.z * dt;

    pose.x += delta_x;
    pose.y += delta_y;
    pose.theta += delta_yaw;

    odom.header.stamp = msg->header.stamp;
    odom.header.frame_id = odom_frame;

    // set the position
    odom.pose.pose.position.x = pose.x;
    odom.pose.pose.position.y = pose.y;
    odom.pose.pose.position.z = 0.0;
	tf2::Quaternion quaternion;
	quaternion.setRPY(0, 0, pose.theta);
	tf2::convert(quaternion, odom.pose.pose.orientation);

    // copy the velocity
    odom.child_frame_id = base_frame;
    odom.twist.twist = velocity;

    odom_publisher.publish(odom);
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "odometry_publisher");

	ros::NodeHandle n;

	odom_publisher = n.advertise<nav_msgs::Odometry>("odom", 50);
	ros::Subscriber velocity_subscriber = n.subscribe("velocity", 50, &velocity_callback);

	ros::spin();
}

