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
\file    slam.h
\brief   SLAM Interface
\author  Joydeep Biswas, (C) 2018
*/
//========================================================================

#include <algorithm>
#include <vector>

#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Geometry"

#include "./CorrelativeScanMatcher.h"

#ifndef SRC_SLAM_H_
#define SRC_SLAM_H_

namespace slam {

struct PgNode {
  std::vector<Eigen::Vector2f> getPointCloud() { return std::vector<Eigen::Vector2f>(0); }
};

class SLAM {
 public:
  // Default Constructor.
  SLAM();

  // Observe a new laser scan.
  void ObserveLaser(const std::vector<float>& ranges,
                    float range_min,
                    float range_max,
                    float angle_min,
                    float angle_max);

  // Observe new odometry-reported location.
  void ObserveOdometry(const Eigen::Vector2f& odom_loc,
                       const float odom_angle);

  // Get latest map.
  std::vector<Eigen::Vector2f> GetMap();

  // Get latest robot pose.
  void GetPose(Eigen::Vector2f* loc, float* angle) const;

  /**
   * Run Correlative Scan Matching on two sets of laser scans to estimate
   * the relative transformation of two nodes.
   *
   * @param base_node[in]         Base node.
   * @param match_node[in]        Match node.
   * @param result[out]           Pair of estimated pose and covariance of
   *                              match_node w.r.t. base_node.
   */
  void ScanMatch(PgNode &base_node, PgNode &match_node,
                 pair<Trans, Eigen::Matrix3f> &result);

 private:

  // Previous odometry-reported locations.
  Eigen::Vector2f prev_odom_loc_;
  float prev_odom_angle_;
  bool odom_initialized_;
  CorrelativeScanMatcher matcher;
};
}  // namespace slam

#endif   // SRC_SLAM_H_
