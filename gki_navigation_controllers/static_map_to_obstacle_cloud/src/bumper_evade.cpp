#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <std_srvs/Empty.h>
#include <std_msgs/Empty.h>
#include <kobuki_msgs/BumperEvent.h>
#include <vector>
#include <deque>
#include <utility>

using namespace std;

// params
int min_bumps_to_trigger = 2;
double min_bumps_trigger_time = 30.0;
double min_time_between_evades = 30.0;

double evade_time = 5.0;
double evade_tv = 0.8;
double evade_rv = 0.6;

ros::Time evade_start_time;
ros::Time last_evade_end_time;
bool evade_active = false;

ros::Publisher pubCmdVel;
bool trigger_call = false;

deque<pair<kobuki_msgs::BumperEvent, ros::Time> > bumper_buffer;

void bumper_callback(const kobuki_msgs::BumperEvent & e)
{
    if(e.state == kobuki_msgs::BumperEvent::PRESSED)
        bumper_buffer.push_back(make_pair(e, ros::Time::now()));
}

void execute_evade()
{
    geometry_msgs::Twist cmd_vel;
    cmd_vel.linear.x = -evade_tv;
    cmd_vel.angular.z = 0.0;
    if(!bumper_buffer.empty()) {
        kobuki_msgs::BumperEvent last_bump = bumper_buffer.back().first;
        if(last_bump.bumper == kobuki_msgs::BumperEvent::LEFT) {
            cmd_vel.angular.z = evade_rv;
        } else if(last_bump.bumper == kobuki_msgs::BumperEvent::RIGHT) {
            cmd_vel.angular.z = -evade_rv;
        }
    }
    pubCmdVel.publish(cmd_vel);
}

void check_evade()
{
    ros::Time now = ros::Time::now();
    if(evade_active) {
        if(now - evade_start_time <= ros::Duration(evade_time)) {
            execute_evade();
        } else {   // evade timeout = finished
            evade_active = false;
            last_evade_end_time = now;
        }
        return;
    }
    // check if activate
    if(now - last_evade_end_time > ros::Duration(min_time_between_evades))
        return;

    // remove too old ones from buffer
    while(!bumper_buffer.empty() &&
            now - bumper_buffer.front().second > ros::Duration(min_bumps_trigger_time)) {
        bumper_buffer.pop_front();
    }

    if((int)bumper_buffer.size() >= min_bumps_to_trigger) {
        evade_start_time = now;
        evade_active = true;
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "bumper_evade");
    ros::NodeHandle nh;
    ros::NodeHandle nhPriv("~");
    nhPriv.param("min_bumps_to_trigger", min_bumps_to_trigger, min_bumps_to_trigger);
    nhPriv.param("min_bumps_trigger_time", min_bumps_trigger_time, min_bumps_trigger_time);
    nhPriv.param("min_time_between_evades", min_time_between_evades, min_time_between_evades);
    nhPriv.param("evade_time", evade_time, evade_time);
    nhPriv.param("evade_tv", evade_tv, evade_tv);
    nhPriv.param("evade_rv", evade_rv, evade_rv);

    ROS_INFO("min_bumps_to_trigger: %d", min_bumps_to_trigger);
    ROS_INFO("min_bumps_trigger_time: %f", min_bumps_trigger_time);
    ROS_INFO("min_time_between_evades: %f", min_time_between_evades);
    ROS_INFO("evade_time: %f", evade_time);
    ROS_INFO("evade_tv: %f", evade_tv);
    ROS_INFO("evade_rv: %f", evade_rv);

    ros::Subscriber sub = nh.subscribe("mobile_base/events/bumper", 1, bumper_callback);
    pubCmdVel = nh.advertise<geometry_msgs::Twist>("cmd_vel_mux/input/bumper_evade", 1);

    ros::Rate rate(10.0);
    while(ros::ok()) {
        ros::spinOnce();
        check_evade();
        rate.sleep();
    }
}

