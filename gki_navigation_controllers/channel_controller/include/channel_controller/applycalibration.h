#ifndef APPLYCALIBRATION_H
#define APPLYCALIBRATION_H

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <vector>

class ApplyCalibration
{
public:
    ApplyCalibration();
    void init();
    geometry_msgs::Twist lookup(geometry_msgs::Twist input);

private:
    double start_tv, start_rv, res_tv, res_rv;
    std::vector<double> data_rv, data_tv;
    std::vector<double> values_rv, values_tv;
    bool loadSuccessful;
};

inline double StraightUp(double x, double min, double max)
{
   if(max == min) {
      return(0.0);
   }

   if(x < min) {
      return(0.0);
   }

   if(x > max) {
      return(1.0);
   }

   return((x - min) / (max - min));
}

inline double StraightDown(double x, double min, double max)
{
   if(max == min) {
      return(0.0);
   }

   if(x < min) {
      return(1.0);
   }

   if(x > max) {
      return(0.0);
   }

   return((max - x) / (max - min));
}

#endif // APPLYCALIBRATION_H
