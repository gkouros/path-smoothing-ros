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


ros::Publisher initialPosePub,finalPosePub,pathPub,smoothedPathPub;
ros::Subscriber waypoint_sub;

geometry_msgs::PoseStamped pose;
nav_msgs::Path path, smoothedPath;

double pointsPerUnit;
int skipPoints;
bool useEndConditions,useMiddleConditions; 

void path_generator_callback(const geometry_msgs::PoseWithCovarianceStamped wp)
{
  path.header.frame_id = "map";
  pose.header.frame_id = "map";

  // pose.pose.position.x = wp.data.pose.pose.position.x;
  // pose.pose.position.y = wp.data.pose.pose.position.y;
  pose.pose.position=wp.pose.pose.position;
  pose.pose.orientation = wp.pose.pose.orientation;
  path.poses.push_back(pose);
  // create a cubic spline interpolator if path >1
  if(path.poses.size() >1)
  {
    path_smoothing::CubicSplineInterpolator csi(pointsPerUnit, skipPoints, useEndConditions, useMiddleConditions);

      // pointsPerUnit, skipPoints, useEndConditions, useMiddleConditions);
    csi.interpolatePath(path, smoothedPath);
    finalPosePub.publish(path.poses.back());
    pathPub.publish(path);
    smoothedPathPub.publish(smoothedPath);
  }
  initialPosePub.publish(path.poses.front());
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "path_smoothing_ros_wrapper");
  ros::NodeHandle nh("~");
  //ROS_INFO_STREAM("Namespace:" << nh.getNamespace());
  nh.param<double>("points_per_unit", pointsPerUnit, 5.0);
  nh.param<int>("skip_points", skipPoints, 0);
  nh.param<bool>("use_end_conditions", useEndConditions, false);
  nh.param<bool>("use_middle_conditions", useMiddleConditions, false);


  initialPosePub = nh.advertise<geometry_msgs::PoseStamped>("initial_pose", 1, true);
  finalPosePub = nh.advertise<geometry_msgs::PoseStamped>("final_pose", 1, true);
  pathPub = nh.advertise<nav_msgs::Path>("initial_path", 1, true);
  smoothedPathPub = nh.advertise<nav_msgs::Path>("smoothed_path", 1, true);


  waypoint_sub = nh.subscribe("/waypoint", 10, path_generator_callback);

  // int pointsPerUnit, skipPoints;
  // bool useEndConditions, useMiddleConditions;


  ros::spin();
  return 0;
}
