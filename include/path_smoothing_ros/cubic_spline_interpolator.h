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

#ifndef PATH_SMOOTHING_ROS_CUBIC_SPLINE_INTERPOLATOR_H
#define PATH_SMOOTHING_ROS_CUBIC_SPLINE_INTERPOLATOR_H

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>

namespace path_smoothing
{

  class CubicSplineInterpolator
  {
    public:

      CubicSplineInterpolator(
        double pointsPerUnit = 5.0,
        unsigned int skipPoints = 0,
        bool useEndConditions = true,
        bool useMiddleConditions = false);

      ~CubicSplineInterpolator();

      void interpolatePath(
        const nav_msgs::Path& path, nav_msgs::Path& smoothedPath);

      void interpolatePath(
        const std::vector<geometry_msgs::PoseStamped>& path,
        std::vector<geometry_msgs::PoseStamped>& smoothedPath);

      void interpolatePoint(
        const std::vector<geometry_msgs::PoseStamped>& path,
        const std::vector<double>& cummulativeDistances,
        geometry_msgs::PoseStamped& point,
        double pointCummDist);

      void calcCummulativeDistances(
        const std::vector<geometry_msgs::PoseStamped> path,
        std::vector<double>& cummulativeDistances);

      double calcTotalDistance(const std::vector<geometry_msgs::PoseStamped>& path);

      double calcDistance(
        const std::vector<geometry_msgs::PoseStamped>& path,
        unsigned int idx);

      double calcAlphaCoeff(
        const std::vector<geometry_msgs::PoseStamped> path,
        const std::vector<double> cummulativeDistances,
        unsigned int idx,
        double input);

      double calcBetaCoeff(
        const std::vector<geometry_msgs::PoseStamped> path,
        const std::vector<double> cummulativeDistances,
        unsigned int idx,
        double input);

      double calcGammaCoeff(
        const std::vector<geometry_msgs::PoseStamped> path,
        const std::vector<double> cummulativeDistances,
        unsigned int idx,
        double input);

      double calcDeltaCoeff(
        const std::vector<geometry_msgs::PoseStamped> path,
        const std::vector<double> cummulativeDistances,
        unsigned int idx,
        double input);

      double calcRelativeDistance(
        const std::vector<double>& cummulativeDistances,
        unsigned int idx,
        double input);

      void calcPointGradient(
        const std::vector<geometry_msgs::PoseStamped>& path,
        const std::vector<double>& cummulativeDistances,
        unsigned int idx, std::vector<double>& gradient);

      unsigned int findGroup(
        const std::vector<double>& cummulativeDistances,
        double pointCummDist);

      double getPointsPerUnit() {return pointsPerUnit_;}
      unsigned int skipPoints() {return skipPoints_;}
      bool getUseEndConditions() {return useEndConditions_;}
      bool getUseMiddleConditions() {return useMiddleConditions_;}

      void setPointsPerUnit(double ppu) {pointsPerUnit_ = ppu;}
      void setSkipPoints(unsigned int sp) {skipPoints_ = sp;}
      void setUseEndConditions(bool uec) {useEndConditions_ = uec;}
      void setUseMiddleConditions(bool umc) {useMiddleConditions_ = umc;}

    private:
      double pointsPerUnit_;
      unsigned int skipPoints_;
      bool useEndConditions_;
      bool useMiddleConditions_;
  };

}  // namespace path_smoothing

#endif  // PATH_SMOOTHING_ROS_CUBIC_SPLINE_INTERPOLATOR_H
