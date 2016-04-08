#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/MarkerArray.h>
#include <vector>
#include <math.h>
#include <angles/angles.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <kobuki_msgs/Sound.h>

ros::Subscriber laser_subscriber;
ros::Subscriber goal_subscriber;
ros::Publisher visualization_publisher;
ros::Publisher velocity_publisher;
ros::Publisher sound_publisher;

tf::TransformListener* tfl;

ros::Time last_forward_time;

double half_robot_width = 0.28;
bool has_goal = false;

double interpolate(double min, double max, double x, double min_value = 0, double max_value = 1)
{
	if (x < min)
	{
		return min_value;
	}
	if (x > max)
	{
		return max_value;
	}
	return ((x - min) / (max - min)) * (max_value - min_value) + min_value;
}

struct Channel
{
	double angle;
	double length;
	double score;
	std::vector<geometry_msgs::Point> points;

	Channel(double angle)
	{
		this->angle = angle;
		this->length = 10;
		score = 0.0;
	}

	void compute_length(sensor_msgs::LaserScanConstPtr msg)
	{
		points.clear();
		for (int index = 0; index < msg->ranges.size(); index++)
		{
			double range = msg->ranges[index];
			if (range < msg->range_max && range > msg->range_min)
			{
				double angle = msg->angle_min + msg->angle_increment * index;
				double x = cos(angle - this->angle) * range;
				double y = sin(angle - this->angle) * range;
				geometry_msgs::Point p;
				p.x = x;
				p.y = y;
				points.push_back(p);
			}
		}
		length = 3;
		for (int index = 0; index < points.size(); index++)
		{
			geometry_msgs::Point p = points[index];
			if (p.y < half_robot_width && p.y > -half_robot_width && p.x > 0)
			{
				if (p.x < length)
				{
					length = p.x;
				}
			}
		}

		//ROS_INFO_STREAM("length "<<length);
	}

	void compute_score(double distance_to_goal, double angle_to_goal)
	{
		// compute score
		if (has_goal)
		{
			double min_length = fmin(length, distance_to_goal);
			double angle_score = interpolate(0, 45, fabs(angles::to_degrees(angle)), 1, 0.5);
			double length_score = interpolate(0.35, 3, length, 0, 1);
			double goal_distance_score = interpolate(0, 3, distance_to_goal, 0, 1);
			double angle_difference = angles::shortest_angular_distance(angle, angle_to_goal);
			double goal_score = interpolate(0, 180, fabs(angles::to_degrees(angle_difference)), 1, 0);
//	    score = length_score * angle_score;
			if (distance_to_goal > length)
			{
				// goal behind obstacle
				score =  1.2 * goal_score + 1.4 *fmin(length_score, goal_distance_score) + angle_score;
			}
			else
			{
				// goal ahead
				score =  1.75 * goal_score + 1.45 *fmin(length_score, goal_distance_score) + angle_score;
			}
		}
		else
		{
			//double min_length = fmin(length, distance_to_goal);
			double angle_score = interpolate(0, 45, fabs(angles::to_degrees(angle)), 1, 0.5);
			double length_score = interpolate(0.5 , 3, length,  0, 1);
			double angle_difference = angles::shortest_angular_distance(angle, angle_to_goal);
			//double goal_score = interpolate(0, 45, fabs(angles::to_degrees(angle_difference)), 1, 0.5 );
			score = length_score * angle_score;
			//score = goal_score * length_score * angle_score;
		}

	}

	visualization_msgs::Marker create_marker(const std::string& frame_id)
	{
		visualization_msgs::Marker marker;
		marker.type = visualization_msgs::Marker::LINE_STRIP;
		marker.header.frame_id = frame_id;
		marker.color.a = 1.0;
		if (has_goal) {
			marker.color.r = 0.2;
			marker.color.g = 0.8;
			marker.color.b = 0.2;
		} else
		{
			marker.color.r = 0.8;
			marker.color.g = 0.2;
			marker.color.b = 0.2;
		}
		marker.action = visualization_msgs::Marker::ADD;
		marker.ns = "channel";
		marker.pose.orientation = tf::createQuaternionMsgFromYaw(angle);
		marker.scale.x = 0.005;

		geometry_msgs::Point p1;
		p1.x = 0;
		p1.y = -half_robot_width;

		geometry_msgs::Point p3;
		p3.x = length;
		p3.y = -half_robot_width;

		geometry_msgs::Point p2;
		p2.x = 0;
		p2.y = half_robot_width;

		geometry_msgs::Point p4;
		p4.x = length;
		p4.y = half_robot_width;

		marker.points.push_back(p1);
		marker.points.push_back(p3);
		marker.points.push_back(p4);
		marker.points.push_back(p2);

		return marker;
	}
};

std::vector<Channel> channels;

std::vector<geometry_msgs::PoseStamped> goal_list;
//geometry_msgs::PoseStamped goal;
void goal_callback(geometry_msgs::PoseStampedConstPtr msg)
{
	ROS_INFO_STREAM("received new goal...");
	goal_list.push_back(*msg);
	if (goal_list.size() > 5) {
		goal_list.erase(goal_list.begin());
	}
	has_goal = ! goal_list.empty();
}

geometry_msgs::PoseStamped transform_goal(sensor_msgs::LaserScanConstPtr msg)
{
	if (goal_list.empty()) {
		return geometry_msgs::PoseStamped();
	}
	geometry_msgs::PoseStamped& goal = goal_list.front();
	geometry_msgs::PoseStamped goal_in_robot_frame;
	if (!goal.header.frame_id.empty())
	{
		try
		{
//			goal.header.stamp = msg->header.stamp;
			goal.header.stamp = ros::Time(0);
			tfl->waitForTransform(msg->header.frame_id, goal.header.frame_id, goal.header.stamp, ros::Duration(1.0));
			tfl->transformPose(msg->header.frame_id, goal, goal_in_robot_frame);
			//ROS_INFO_STREAM("goal: "<<goal_in_robot_frame);
		} catch (tf::LookupException& e)
		{
			ROS_INFO_STREAM(e.what());
		} catch (tf::InvalidArgument& e)
		{
			ROS_INFO_STREAM(e.what());
		} catch (tf::TransformException& e)
		{
			ROS_INFO_STREAM(e.what());
		}
	}
	return goal_in_robot_frame;
}

void laser_channels_callback(sensor_msgs::LaserScanConstPtr msg)
{
	visualization_msgs::MarkerArray vis_msg;

	double distance_to_goal = 0;
	double angle_to_goal = 0;
	if (has_goal)
	{
		geometry_msgs::PoseStamped goal = transform_goal(msg);
		angle_to_goal = atan2(goal.pose.position.y, goal.pose.position.x);
		distance_to_goal = hypot(goal.pose.position.y, goal.pose.position.x);
		if (distance_to_goal < 0.2) {
			// goal reached
			goal_list.erase(goal_list.begin());
			has_goal = ! goal_list.empty();

			kobuki_msgs::Sound sound_msg;
			sound_msg.value = sound_msg.CLEANINGEND;
			sound_publisher.publish(sound_msg);
		}
		else
		{
			ROS_INFO_STREAM(" goal: "<< distance_to_goal<<", "<< angles::to_degrees(angle_to_goal));
		}
	}


	for (int index = 0; index < channels.size(); index += 1)
	{
		Channel& c = channels[index];
		c.compute_length(msg);
		c.compute_score(distance_to_goal, angle_to_goal);
		visualization_msgs::Marker m = c.create_marker(msg->header.frame_id);
		m.id = vis_msg.markers.size();
		vis_msg.markers.push_back(m);
	}

	int best_index = 0;
	for (int index = 0; index < channels.size(); index += 1)
	{
		Channel& c = channels[index];
		if (channels[best_index].score < c.score)
		{
			best_index = index;
		}
	}
	visualization_msgs::Marker m = channels[best_index].create_marker(msg->header.frame_id);
	m.id = vis_msg.markers.size();
	m.color.r = 0.2;
	m.color.g = 0.2;
	m.color.b = 1;
	m.scale.x = 0.05;
	m.ns = "best channel";
	vis_msg.markers.push_back(m);

	geometry_msgs::Twist velocity;
	double max_angular_velocity = angles::from_degrees(45);
	double max_linear_velocity = 0.75;
	velocity.angular.z = interpolate(angles::from_degrees(-45), angles::from_degrees(45), channels[best_index].angle, -max_angular_velocity,
			max_angular_velocity);
	velocity.linear.x = interpolate(0.35, 2, channels[best_index].length, 0, max_linear_velocity);
	velocity.linear.x = interpolate(0, angles::from_degrees(45), fabs(channels[best_index].angle), velocity.linear.x, 0);

	if (velocity.linear.x > 0.03)
	{
		last_forward_time = msg->header.stamp;
	}
	if (msg->header.stamp - last_forward_time > ros::Duration(3.0))
	{
		// stuck
		ROS_INFO_STREAM("stuck");
		kobuki_msgs::Sound sound_msg;
		sound_msg.value = sound_msg.BUTTON;
		sound_publisher.publish(sound_msg);
		velocity.angular.z = angles::from_degrees(45);
	}

	//velocity.linear.x = // forward
	//velocity.angular.z = // turn
	//ROS_INFO_STREAM("velocity: "<<velocity.linear.x<<" "<<angles::to_degrees(velocity.angular.z));
	velocity_publisher.publish(velocity);
	//ROS_INFO_STREAM("goal: "<<goal.pose.position);
	if (has_goal)
	{
		for (int index = 0; index < goal_list.size(); index++)
		{
			geometry_msgs::PoseStamped& goal = goal_list[index];
			visualization_msgs::Marker marker;
			marker.type = visualization_msgs::Marker::SPHERE;
			marker.header.frame_id = goal.header.frame_id;
			marker.color.a = 1.0;
			marker.color.r = interpolate(0, 5, index, 1, 0);
			marker.color.g = interpolate(0, 5, index, 0.66, 0);
			marker.color.b = 0;
			marker.action = visualization_msgs::Marker::ADD;
			marker.ns = "goal";
			marker.id = index;
			marker.pose.orientation.w = 1;
			marker.scale.x = 0.1;
			marker.scale.y = 0.1;
			marker.scale.z = 0.1;
			marker.pose = goal.pose;
			vis_msg.markers.push_back(marker);
		}
	}
	else
	{
		//ROS_INFO_STREAM(" goal: "<< distance_to_goal<<", "<< angles::to_degrees(angle_to_goal));
		//ROS_INFO_STREAM("goal: "<<goal.pose.position);
		visualization_msgs::Marker marker;
		marker.type = visualization_msgs::Marker::CUBE;
		marker.header.frame_id = msg->header.frame_id;
		marker.action = visualization_msgs::Marker::DELETE;
		marker.ns = "goal";
		marker.pose.orientation.w = 1;
		vis_msg.markers.push_back(marker);
	}

	visualization_publisher.publish(vis_msg);
}

//void laser_callback(sensor_msgs::LaserScanConstPtr msg)
//{
//	visualization_msgs::MarkerArray vis_msg;
//	// create behavior
//	msg->angle_min; // starting angle
//	msg->angle_max; // ending angle
//	msg->angle_increment; // angular resolution
//	msg->ranges; // sensor data list
//
//	vis_msg.markers.push_back(visualization_msgs::Marker());
//	visualization_msgs::Marker& marker = vis_msg.markers.back();
//	marker.type = visualization_msgs::Marker::POINTS;
//	marker.header.frame_id = msg->header.frame_id;
//	marker.color.a = 1.0;
//	marker.color.r = 0.2;
//	marker.color.g = 1.0;
//	marker.color.b = 0.2;
//	marker.action = visualization_msgs::Marker::ADD;
//	marker.ns = "xy laser";
//	marker.pose.orientation.w = 1;
//	marker.scale.x = 0.05;
//	marker.scale.y = 0.05;
//	marker.scale.z = 0.05;
//
//	std::vector<geometry_msgs::Point> points;
//	for (int index = 0; index < msg->ranges.size(); index++)
//	{
//		double range = msg->ranges[index];
//		if (range < msg->range_max && range > msg->range_min)
//		{
//			double angle = msg->angle_min + msg->angle_increment * index;
//			double x = cos(angle) * range;
//			double y = sin(angle) * range;
//			geometry_msgs::Point p;
//			p.x = x;
//			p.y = y;
//			points.push_back(p);
//			marker.points.push_back(p);
//		}
//	}
//	// useful funcitons:
//	// sin(0.5);
//	// cos(0.5);
//	// hypot(a, b); hypotenuse in triangle abc, length of c
//	// atan2(b, a); angle in triangle abc, angle alpha
//	//ROS_INFO_STREAM("min angle: "<<msg->angle_min);
//
//	double length = 10;
//	double rotation_direction = 1;
//	for (int index = 0; index < points.size(); index++)
//	{
//		geometry_msgs::Point p = points[index];
//		if (p.y < half_robot_width && p.y > -half_robot_width)
//		{
//			if (p.x < length)
//			{
//				length = p.x;
//				if (p.y > 0)
//				{
//					rotation_direction = -1;
//				}
//				else
//				{
//					rotation_direction = 1;
//				}
//			}
//		}
//	}
//
//	geometry_msgs::Twist velocity;
//	if (length > 0.5)
//	{
//		ROS_INFO_STREAM("fahren ");
//		velocity.linear.x = interpolate(0.5, 3, length, 0.08, 0.8);
//	}
//	else
//	{
//		ROS_INFO_STREAM("drehen ");
//		velocity.angular.z = angles::from_degrees(45) * rotation_direction;
//	}
//	ROS_INFO_STREAM("velocity: "<<velocity.linear.x<<" "<<angles::to_degrees(velocity.angular.z));
//	if (velocity.linear.x > 0.08)
//	{
//		last_forward_time = msg->header.stamp;
//	}
//	if (msg->header.stamp - last_forward_time > ros::Duration(5.0))
//	{
//		// stuck
//		ROS_INFO_STREAM("stuck");
//		kobuki_msgs::Sound sound_msg;
//		sound_msg.value = sound_msg.BUTTON;
//		sound_publisher.publish(sound_msg);
//		velocity.angular.z = angles::from_degrees(45);
//	}
//
//	//velocity.linear.x = // forward
//	//velocity.angular.z = // turn
//	velocity_publisher.publish(velocity);
//	visualization_publisher.publish(vis_msg);
//}

int main(int argc, char** argv)
{
	// initialize
	ros::init(argc, argv, "reactive_move_controller");
	ros::NodeHandle nh;\
	//ROS_INFO_STREAM(0/0);
	tfl = new tf::TransformListener(ros::Duration(60));

//    for (double x = 0; x < 3.5; x +=0.1){
//    	double value = interpolate(0.5, 3, x, 3, 1);
//    	ROS_INFO_STREAM(x<< " -> "<<value);
//    }
//    return 0;

	ROS_INFO("creating channels...");
	for (double x = 90; x > -91; x -= 2)
	{
		channels.push_back(Channel(angles::from_degrees(x)));
	}
	//channels.push_back(Channel(angles::from_degrees(-90)));
	ROS_INFO_STREAM(channels.size()<< " channels created.");

	goal_subscriber = nh.subscribe("move_base_simple/goal", 1, goal_callback);
	ROS_INFO("subscribing to laser topic...");
	laser_subscriber = nh.subscribe("base_scan_filtered", 1, laser_channels_callback);
	ROS_INFO("advertizing command velocity topic...");
	sound_publisher = nh.advertise<kobuki_msgs::Sound>("mobile_base/commands/sound", 1, false);
	velocity_publisher = nh.advertise<geometry_msgs::Twist>("/cmd_vel_mux/input/navi", 1, false);
	visualization_publisher = nh.advertise<visualization_msgs::MarkerArray>("visualization", 1, false);
	ROS_INFO("reactive move controller initialized.");

	// loop
	ros::spin();
}
