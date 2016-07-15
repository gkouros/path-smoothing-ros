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

int main(int argc, char** argv)
{
  ros::init(argc, argv, "path_smoothing_ros_demo");
  ros::NodeHandle nh("~");
  ROS_INFO_STREAM("Namespace:" << nh.getNamespace());

  ros::Publisher initialPosePub = nh.advertise<geometry_msgs::PoseStamped>("initial_pose", 1, true);
  ros::Publisher finalPosePub = nh.advertise<geometry_msgs::PoseStamped>("final_pose", 1, true);
  ros::Publisher pathPub = nh.advertise<nav_msgs::Path>("initial_path", 1, true);
  ros::Publisher smoothedPathPub = nh.advertise<nav_msgs::Path>("smoothed_path", 1, true);

  int pointsPerUnit, skipPoints;
  bool useEndConditions, useMiddleConditions;

  nh.param<int>("points_per_unit", pointsPerUnit, 5);
  nh.param<int>("skip_points", skipPoints, 0);
  nh.param<bool>("use_end_conditions", useEndConditions, false);
  nh.param<bool>("use_middle_conditions", useMiddleConditions, false);

  XmlRpc::XmlRpcValue poseList;
  if (!nh.getParam("path_poses", poseList))
  {
    ROS_FATAL("Failed to load path point list");
    exit(EXIT_FAILURE);
  }

  nav_msgs::Path path, smoothedPath;
  path.header.frame_id = "map";
  geometry_msgs::PoseStamped pose;
  pose.header.frame_id = "map";

  for (int i = 0; i < poseList.size(); i++)
  {
    pose.pose.position.x = static_cast<double>(poseList[i]["x"]);
    pose.pose.position.y = static_cast<double>(poseList[i]["y"]);
    pose.pose.orientation = tf::createQuaternionMsgFromYaw(poseList[i]["yaw"]);
    path.poses.push_back(pose);
  }

  // create a cubic spline interpolator
  path_smoothing::CubicSplineInterpolator csi("lala");
    // pointsPerUnit, skipPoints, useEndConditions, useMiddleConditions);
  csi.interpolatePath(path, smoothedPath);

  initialPosePub.publish(path.poses.front());
  finalPosePub.publish(path.poses.back());
  pathPub.publish(path);
  smoothedPathPub.publish(smoothedPath);

  ros::Time currTime = ros::Time::now();

  while (ros::ok() && ros::Time::now().toSec() - currTime.toSec() < 2.0)
  {
    ros::spinOnce();
  }

  return 0;
}
