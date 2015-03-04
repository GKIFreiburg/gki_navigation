#include "channel_controller/applycalibration.h"

ApplyCalibration::ApplyCalibration():loadSuccessful(false)
{
}

void ApplyCalibration::init()
{
    ros::NodeHandle nhPriv("~");
    XmlRpc::XmlRpcValue parameter_list;

    nhPriv.getParamCached("tv/resolution", res_tv);
    nhPriv.getParamCached("tv/start_value", start_tv);
    nhPriv.getParamCached("tv/data", parameter_list);
    if(parameter_list.getType() != XmlRpc::XmlRpcValue::TypeArray)
    {
        ROS_WARN("failed to initalize Calibration! Parameter not an array");
        return;
    }
    for(int x = 0; x < parameter_list.size(); x++)
    {
        if(parameter_list[x].getType() != XmlRpc::XmlRpcValue::TypeDouble)
        {
            ROS_WARN("failed to initalize Calibration! Parameter not of type doulbe");
            return;
        }
        data_tv.push_back(static_cast<double>(parameter_list[x]));
        values_tv.push_back(start_tv+x*res_tv);
    }

    nhPriv.getParamCached("rv/resolution", res_rv);
    nhPriv.getParamCached("rv/start_value", start_rv);
    nhPriv.getParamCached("rv/data", parameter_list);
    if(parameter_list.getType() != XmlRpc::XmlRpcValue::TypeArray)
    {
        ROS_WARN("failed to initalize Calibration! Parameter not an array");
        return;
    }
    for(int x = 0; x < parameter_list.size(); x++)
    {
        if(parameter_list[x].getType() != XmlRpc::XmlRpcValue::TypeDouble)
        {
            ROS_WARN("failed to initalize Calibration! Parameter not of type doulbe");
            return;
        }
        data_rv.push_back(static_cast<double>(parameter_list[x]));
        values_rv.push_back(start_rv+x*res_rv);
    }

    if(data_tv.size() == 0)
    {
        ROS_WARN("failed to initalize Calibration!");
        return;
    }
    loadSuccessful = true;
}

geometry_msgs::Twist ApplyCalibration::lookup(geometry_msgs::Twist input)
{
    std::vector<double>::iterator up;
    int pos_up;
    double valueInterpolated;
    double outTv, outRv;
    double inTv = input.linear.x;
    double inRv = input.angular.z;    
    geometry_msgs::Twist calibratedTwist;

    // check if parameter load was successful
    if(!loadSuccessful)
        return input;

    // get tv value
    up = std::upper_bound(values_tv.begin(), values_tv.end(), inTv);
    pos_up = (up - values_tv.begin());

    if(up == values_tv.begin())
    {
        // found the first element -> input is out of lower bound but may still be close by: interpolate between 1 and last calibration value
        valueInterpolated = StraightDown(inTv, (*up-res_tv), *up) * 1 + StraightUp(inTv, (*up-res_tv), *up) * data_tv[pos_up];
    }
    else if(up == values_tv.end())
    {
        pos_up--; // decrement by one, as end points beyond the last value
        up--; // decrement iterator by one to point at last value instead of end
        // it's possible that the input is out the upper bound, check if is out of upper bound
        if(inTv > *up)
        {
            // yes, value is out of upper bound
            valueInterpolated = StraightDown(inTv, *up, (*up + res_tv)) * data_tv[pos_up] + StraightUp(inTv, *up, (*up + res_tv)) * 1;
        }
        else
        {
            // no value is still in bound, interpolate normally
            valueInterpolated = StraightDown(inTv, *(up-1), *up)* data_tv[pos_up-1] + StraightUp(inTv, *(up-1), *up) * data_tv[pos_up];
        }
    }
    else
    {
        // interpolate: using distance to adjacent steps normalized to 1 as percentage of the corresponding calibration value
        // we know that the iterator always points to next bigger value than the given one
        //valueInterpolated = (((*up - inTv) / res_tv) * data_tv[pos_up]) + (((inTv - *(up-1)) / res_tv) * data_tv[pos_up-1]);
        valueInterpolated = StraightDown(inTv, *(up-1), *up)* data_tv[pos_up-1] + StraightUp(inTv, *(up-1), *up) * data_tv[pos_up];
    }
    outTv = inTv * valueInterpolated;

    // get rv value
    up = std::upper_bound(values_rv.begin(), values_rv.end(), inRv);
    pos_up = (up- values_rv.begin());

    if(up == values_rv.begin())
    {
        // found the first element -> input is out of lower bound but may still be close by: interpolate between 1 and last calibration value
        valueInterpolated = StraightDown(inRv, (*up-res_rv), *up) * 1 + StraightUp(inRv, (*up-res_rv), *up) * data_rv[pos_up];
    }
    else if(up == values_rv.end())
    {
        pos_up--; // decrement by one, as end points beyond the last value
        up--; // decrement iterator by one to point at last value instead of end

        // it's possible that the input is out the upper bound, check if is out of upper bound
        if(inTv > *up)
        {
            // yes, value is out of upper bound
            valueInterpolated = StraightDown(inRv, *up, (*up + res_rv)) * data_rv[pos_up] + StraightUp(inRv, *up, (*up + res_rv)) * 1;
        }
        else
        {
            // no value is still in bound, interpolate normally
            valueInterpolated = StraightDown(inRv, *(up-1), *up)* data_rv[pos_up-1] + StraightUp(inRv, *(up-1), *up) * data_rv[pos_up];
        }
    }
    else
    {
        // interpolate: using distance to adjacent steps normalized to 1 as percentage of the corresponding calibration value
        // we know that the iterator always points to next bigger value than the given one
        valueInterpolated = StraightDown(inRv, *(up-1), *up)* data_rv[pos_up-1] + StraightUp(inRv, *(up-1), *up) * data_rv[pos_up];
    }
    outRv = inRv * valueInterpolated;

    // return properly calibrated value by use of data vector
    calibratedTwist.linear.x = outTv;
    calibratedTwist.angular.z = outRv;

    return calibratedTwist;
}
