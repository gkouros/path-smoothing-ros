/*********************************************************************
*
* Software License Agreement (BSD License)
*
*  Copyright (c) 2016, George Kouros.
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
*   * Neither the name of the the copyright holder nor the names of its
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
* Author:  George Kouros
*********************************************************************/

#include "path_smoothing_ros/cubic_spline_interpolator.h"
#include <tf/tf.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>





int main(int argc, char** argv)
{
    ros::init(argc, argv, "demo_generator");
    ros::NodeHandle nh("~");
    ros::Publisher PosePub = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("/waypoint", 1, true);
    ros::Rate loop_rate(10);
    geometry_msgs::PoseWithCovarianceStamped pose;
    pose.header.frame_id = "map";
    while(ros::ok())
    {
        int i,j;
      for (i=-5; i<=5; i++)
      {
        for (j=-5; j<=5; j++)
        {
            pose.pose.pose.position.x=j;
            pose.pose.pose.position.y=i;
            pose.pose.pose.orientation.x=0;
            pose.pose.pose.orientation.y=0;
            pose.pose.pose.orientation.z=0;
            pose.pose.pose.orientation.w=1;
            PosePub.publish(pose);
            ROS_INFO("%f , %f",pose.pose.pose.position.x,pose.pose.pose.position.y);
            loop_rate.sleep();
        }
        i=i+1;
        for (j=5; j>=-5; j--)
        {
            pose.pose.pose.position.x=j;
            pose.pose.pose.position.y=i;
            pose.pose.pose.orientation.x=0;
            pose.pose.pose.orientation.y=0;
            pose.pose.pose.orientation.z=0;
            pose.pose.pose.orientation.w=1;
            PosePub.publish(pose);
            ROS_INFO("%f , %f",pose.pose.pose.position.x,pose.pose.pose.position.y);
            loop_rate.sleep();
        }
        i+1;

      }
      return 0;
    }
    return 0;
}
