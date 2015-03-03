#include "channel_controller/applycalibration.h"

ApplyCalibration::ApplyCalibration()
{
}

void ApplyCalibration::init()
{
    ros::NodeHandle nhPriv("~");
    nhPriv.getParamCached("tv/data", data_tv);
    nhPriv.getParamCached("tv/resolution", res_tv);
    nhPriv.getParamCached("tv/start_value", start_tv);

    nhPriv.getParamCached("rv/data", data_rv);
    nhPriv.getParamCached("rv/resolution", res_rv);
    nhPriv.getParamCached("rv/start_value", start_rv);

    if(data_tv.size() == 0)
    {
        loadSuccessful = false;
        return;
    }

    for(size_t i = 0; i< data_rv.size();i++)
    {
        values_rv.push_back(start_rv+i*res_rv);
        values_tv.push_back(start_tv+i*res_tv);
    }

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
