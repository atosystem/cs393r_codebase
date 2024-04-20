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

#ifndef SRC_SLAM_H_
#define SRC_SLAM_H_

namespace slam {

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

  // === Pose Graph Functions === //
  
  /**
   * @return true if the robot has moved far enough.
   */
  bool shouldAddPgNode();
  
  /**
   * @brief update pose graph with latest scans. 
   *  1. add node.
   *  2. add edges with all previous nodes.
   */
  void updatePoseGraph();

  /**
   * @brief 
   * 1. successive scan 
   * 2. non_successive_scan_constraints_: connect new node with all previous nodes.
   * @param new_node 
   */
  void updatePoseGraphObsConstraints(PgNode &new_node);
  
  /**
   * @brief Add edge to pose graph for gtsam optimization.
   * 
   */
  void addObservationConstraint(const size_t &from_node_num, const size_t &to_node_num,
                                      std::pair<std::pair<Eigen::Vector2f, float>, Eigen::MatrixXd> &constraint_info);

  /**
   * Optimize the pose graph and update the estimated poses in the nodes.
   *
   * @param new_node_init_estimates Initial pose estimated for nodes that have been added to the graph since the
   * last optimization.
   */
  void optimizePoseGraph(gtsam::Values &new_node_init_estimates);

  
  /**
   * Run CSM on the measurements of the two nodes to get the estimated position of node 2 in the frame of node 1.
   *
   * @param base_node[in]         Base node.
   * @param match_node[in]        Match node. Find the relative position with respect to base node.
   * @param csm_results[out]      Pair with first entry as estimated position of node 2 relative to node 1 based on
   *                          scan alignment and second entry as estimated covariance.
   */
  void runCSM(PgNode &base_node, PgNode &match_node, std::pair<std::pair<Eigen::Vector2f, float>, Eigen::MatrixXd> &csm_results);
    
  // Utility functions.
  void convertLidar2PointCloud(
    const std::vector<float>& ranges,
      float range_min,
      float range_max,
      float angle_min,
      float angle_max
      )

 private:

  // Previous odometry-reported locations.
  Eigen::Vector2f prev_odom_loc_;
  float prev_odom_angle_;
  bool odom_initialized_;

  vector<Eigen::Vector2f> recent_point_cloud_;
  
};
}  // namespace slam

#endif   // SRC_SLAM_H_
