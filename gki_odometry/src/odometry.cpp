#include <ros/ros.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/utils.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <message_filters/cache.h>
#include <message_filters/subscriber.h>
#include <angles/angles.h>

std::string base_frame;
std::string odom_frame;

ros::Publisher odom_publisher;
nav_msgs::Odometry odom;
geometry_msgs::Pose2D pose;
typedef message_filters::Cache<sensor_msgs::Imu> ImuCache;
boost::shared_ptr<ImuCache> imu_cache;

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
    // covariance matrix pose
	//	xx	xy	0	0	0	xth
	//	xy	yy	0	0	0	yth
	//	0	0	999	0	0	0
	//	0	0	0	999	0	0
	//	0	0	0	0	999	0
	//	xth	yth	0	0	0	thth
    double high_covariance = std::numeric_limits<double>::max();
    double x_stddev;
    double y_stddev;
    double xy_cov;
    double theta_stddev;
    double xtheta_cov;
    double ytheta_cov;
    paramCached(n, "x_stddev", x_stddev, 0.002);
    paramCached(n, "y_stddev", y_stddev, 0.002);
    paramCached(n, "theta_stddev", theta_stddev, 0.017);
    paramCached(n, "xy_cov", xy_cov, 0.0);
    paramCached(n, "xtheta_cov", xtheta_cov, 0.0);
    paramCached(n, "ytheta_cov", ytheta_cov, 0.0);
    odom.pose.covariance[0+0*6] = pow(x_stddev, 2);
    odom.pose.covariance[1+1*6] = pow(y_stddev, 2);
    odom.pose.covariance[1+0*6] = pow(xy_cov, 2);
    odom.pose.covariance[0+1*6] = odom.pose.covariance[1+0*6];
    odom.pose.covariance[5+0*6] = pow(xtheta_cov, 2);
    odom.pose.covariance[0+5*6] = odom.pose.covariance[5+0*6];
    odom.pose.covariance[5+1*6] = pow(ytheta_cov, 2);
    odom.pose.covariance[1+5*6] = odom.pose.covariance[5+1*6];
    odom.pose.covariance[2+2*6] = high_covariance;
    odom.pose.covariance[2+2*6] = high_covariance;
    odom.pose.covariance[2+2*6] = high_covariance;
    odom.pose.covariance[5+5*6] = pow(theta_stddev, 2);
    // covariance matrix velocity
    //	xx	0	0	0	0	0
    //	0	999	0	0	0	0
    //	0	0	999	0	0	0
    //	0	0	0	999	0	0
    //	0	0	0	0	999	0
    //	0	0	0	0	0	thth
    odom.twist.covariance[0+0*6] = pow(x_stddev, 2);
    odom.twist.covariance[1+1*6] = high_covariance;
    odom.twist.covariance[2+2*6] = high_covariance;
    odom.twist.covariance[3+3*6] = high_covariance;
    odom.twist.covariance[4+4*6] = high_covariance;
    odom.twist.covariance[5+5*6] = pow(theta_stddev, 2);

    paramCached(n, "base_frame", odom.child_frame_id, std::string("base_link"));
    paramCached(n, "odom_frame", odom.header.frame_id, std::string("odom"));
}

void velocity_callback(const geometry_msgs::TwistStampedConstPtr msg)
{
	updateParams();
    sensor_msgs::ImuConstPtr previous_imu = imu_cache->getElemBeforeTime(odom.header.stamp);
    sensor_msgs::ImuConstPtr current_imu = imu_cache->getElemBeforeTime(msg->header.stamp);

    // without imu
	geometry_msgs::Twist velocity = msg->twist;
    if (current_imu && previous_imu)
    {
    	// use imu, if we have data
    	double previous_theta = tf2::getYaw(previous_imu->orientation);
    	double current_theta = tf2::getYaw(current_imu->orientation);
    	double delta_yaw = current_theta - previous_theta;
    	velocity.angular.z = delta_yaw / (current_imu->header.stamp - previous_imu->header.stamp).toSec();
    }
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
	odom.twist.twist = velocity;
    odom_publisher.publish(odom);
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "odometry_publisher");

	ros::NodeHandle n;

	odom_publisher = n.advertise<nav_msgs::Odometry>("odom", 50);
	ros::Subscriber velocity_subscriber = n.subscribe("velocity", 50, &velocity_callback);
	message_filters::Subscriber<sensor_msgs::Imu> imu_subscriber(n, "imu", 1);
	imu_cache.reset(new  ImuCache(imu_subscriber, 100));

	ros::spin();
}

