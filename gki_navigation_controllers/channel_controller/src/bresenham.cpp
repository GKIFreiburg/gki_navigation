/*********************************************************************
*
* Software License Agreement (BSD License)
*
*  Copyright (c) 2008, Willow Garage, Inc.
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Willow Garage nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*
* Author: Eitan Marder-Eppstein
*********************************************************************/

/* Adapted from costmap_2d/costmap_2d.h */

#include <limits.h>
#include <cmath>
#include <stdlib.h>
#include <algorithm>
#include <stdio.h>

inline int sign(int x)
{
    return x > 0 ? 1.0 : -1.0;
}

unsigned int size_x_ = 20;

class Print
{
    public:
        void operator()(int off) const
        {
            printf("%d %d\n", off % size_x_, off/size_x_);
        }
};

// TODO negative coords

template <class ActionType>
inline void raytraceLine(ActionType at, unsigned int x0, unsigned int y0, unsigned int x1, unsigned int y1, unsigned int max_length = UINT_MAX)
{
    int dx = x1 - x0;
    int dy = y1 - y0;

    unsigned int abs_dx = abs(dx);
    unsigned int abs_dy = abs(dy);

    int offset_dx = sign(dx);
    int offset_dy = sign(dy) * size_x_;

    int cur_x = x0;
    int cur_y = y0;

    //we need to chose how much to scale our dominant dimension, based on the maximum length of the line
    double dist = sqrt((x0 - x1) * (x0 - x1) + (y0 - y1) * (y0 - y1));
    double scale = std::min(1.0,  max_length / dist);

    //if x is dominant
    if(abs_dx >= abs_dy){
        int error_y = abs_dx / 2;
        bresenham2D(at, abs_dx, abs_dy, error_y, offset_dx, offset_dy, cur_x, cur_y, (unsigned int)(scale * abs_dx));
        return;
    }

    //otherwise y is dominant
    int error_x = abs_dy / 2;
    bresenham2D(at, abs_dy, abs_dx, error_x, offset_dy, offset_dx, cur_x, cur_y, (unsigned int)(scale * abs_dy));
}

template <class ActionType>
inline void bresenham2D(ActionType at, unsigned int abs_da, unsigned int abs_db, int error_b, int offset_a, int offset_b, 
        int cur_x, int cur_y, unsigned int max_length){
    unsigned int offset = cur_y * size_x_ + cur_x;
    unsigned int end = std::min(max_length, abs_da);
    for(unsigned int i = 0; i < end; ++i){
        at(offset);
        offset += offset_a;
        error_b += abs_db;
        if((unsigned int)error_b >= abs_da){
            offset += offset_b;
            error_b -= abs_da;
        }
    }
    at(offset);
}

int main(int argc, char** argv)
{
    raytraceLine(Print(), 0, 0, 10, 5);
    raytraceLine(Print(), 0, 0, 5, 10);
}
