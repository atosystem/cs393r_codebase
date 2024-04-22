//========================================================================
//  This software is free: you can redistribute it and/or modify
//  it under the terms of the GNU Lesser General Public License Version 3,
//  as published by the Free Software Foundation.
//
//  This software is distributed in the hope that it will be useful,
//  but WITHOUT ANY WARRANTY; without even the implied warranty of
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//  GNU Lesser General Public License for more details.
//
//  You should have received a copy of the GNU Lesser General Public License
//  Version 3 in the file COPYING that came with this distribution.
//  If not, see <http://www.gnu.org/licenses/>.
//========================================================================
/*!
\file    slam.cc
\brief   SLAM Starter Code
\author  Joydeep Biswas, (C) 2019
*/
//========================================================================

#include <algorithm>
#include <cmath>
#include <iostream>
#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Geometry"
#include "gflags/gflags.h"
#include "glog/logging.h"
#include "shared/math/geometry.h"
#include "shared/math/math_util.h"
#include "shared/util/timer.h"

#include "slam.h"

#include "vector_map/vector_map.h"
#include "config_reader/config_reader.h"

using namespace math_util;
using Eigen::Affine2f;
using Eigen::Rotation2Df;
using Eigen::Translation2f;
using Eigen::Vector2f;
using Eigen::Vector2i;
using std::cout;
using std::endl;
using std::string;
using std::swap;
using std::vector;
using vector_map::VectorMap;

CONFIG_DOUBLE(scanner_range, "scanner_range");
CONFIG_DOUBLE(trans_range, "trans_range");
CONFIG_DOUBLE(low_res, "low_res");
CONFIG_DOUBLE(high_res, "high_res");

namespace slam {

SLAM::SLAM() :
    prev_odom_loc_(0, 0),
    prev_odom_angle_(0),
    odom_initialized_(false),
    matcher(
      CONFIG_scanner_range,
      CONFIG_trans_range,
      CONFIG_low_res,
      CONFIG_high_res) {}

void SLAM::GetPose(Eigen::Vector2f* loc, float* angle) const {
  // Return the latest pose estimate of the robot.
  *loc = Vector2f(0, 0);
  *angle = 0;
}

void SLAM::ObserveLaser(const vector<float>& ranges,
                        float range_min,
                        float range_max,
                        float angle_min,
                        float angle_max) {
  // A new laser scan has been observed. Decide whether to add it as a pose
  // for SLAM. If decided to add, align it to the scan from the last saved pose,
  // and save both the scan and the optimized pose.
}

void SLAM::ObserveOdometry(const Vector2f& odom_loc, const float odom_angle) {
  if (!odom_initialized_) {
    prev_odom_angle_ = odom_angle;
    prev_odom_loc_ = odom_loc;
    odom_initialized_ = true;
    return;
  }
  // Keep track of odometry to estimate how far the robot has moved between 
  // poses.
}

vector<Vector2f> SLAM::GetMap() {
  vector<Vector2f> map;
  // Reconstruct the map as a single aligned point cloud from all saved poses
  // and their respective scans.
  return map;
}

void SLAM::ScanMatch(PgNode &base_node, PgNode &match_node,
                     pair<pose_2d::Pose2Df, Eigen::Matrix3f> &result) {
  pair<Trans, Eigen::Matrix3f> trans_and_uncertainty =
    matcher.GetTransAndUncertainty(base_node.getPointCloud(), match_node.getPointCloud());
  const Trans &trans = trans_and_uncertainty.first;

  result.first = pose_2d::Pose2Df(trans.second, trans.first);
  result.second = trans_and_uncertainty.second;
}

}  // namespace slam
