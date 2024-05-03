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

#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/geometry/Pose2.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/nonlinear/ISAM2.h>
#include "gtsam/nonlinear/GaussNewtonOptimizer.h"

#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Geometry"
#include "pg_node.h"
#include "shared/math/poses_2d.h"

#include "shared/math/poses_2d.h"
#include "./CorrelativeScanMatcher.h"

#ifndef SRC_SLAM_H_
#define SRC_SLAM_H_

namespace slam
{

  class SLAM
  {
  public:
    // Default Constructor.
    SLAM();

    // Observe a new laser scan.
    void ObserveLaser(const std::vector<float> &ranges,
                      float range_min,
                      float range_max,
                      float angle_min,
                      float angle_max);

    // Observe new odometry-reported location.
    void ObserveOdometry(const Eigen::Vector2f &odom_loc,
                         const float odom_angle);

    // Get latest map.
    std::vector<Eigen::Vector2f> GetMap();

    // Get latest robot pose.
    void GetPose(Eigen::Vector2f *loc, float *angle);

    // Get pg_nodes
    std::vector<PgNode> GetPgNodes() const;

    // === Pose Graph Functions === //
    /**
     * Run Correlative Scan Matching on two sets of laser scans to estimate
     * the relative transformation of two nodes.
     *
     * @param base_node[in]         Base node.
     * @param match_node[in]        Match node.
     * @param result[out]           Pair of estimated pose and covariance of
     *                              match_node w.r.t. base_node.
     * @return true if csm was successful, false otherwise.
     */
    bool ScanMatch(PgNode &base_node, PgNode &match_node,
                   pair<pose_2d::Pose2Df, Eigen::Matrix3f> &result);

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
     * @brief build edges of new node with previous nodes by observation constraints.
     *        can include successive and non-successive nodes.
     *        call addObservationConstraint() to add edges.
     *        call optimizePoseGraph() to optimize the whole graph.
     * @param new_node  a new node that should be connected with all previous nodes by observation constraint
     */
    void updatePoseGraphObsConstraints(PgNode &new_node);

    /**
     * @brief Add edge to pose graph for gtsam optimization.
     *
     */
    void addObservationConstraint(const size_t &from_node_num, const size_t &to_node_num,
                                  std::pair<pose_2d::Pose2Df, Eigen::Matrix3f> &constraint_info);

    /**
     * Optimize the pose graph and update the estimated poses in the nodes.
     *
     * @param new_node_init_estimates Initial pose estimated for nodes that have been added to the graph since the
     * last optimization.
     */
    void optimizePoseGraph(gtsam::Values &new_node_init_estimates);

    void offlineOptimizePoseGraph();

    /**
     * Run CSM on the measurements of the two nodes to get the estimated position of node 2 in the frame of node 1.
     *
     * @param base_node[in]         Base node.
     * @param match_node[in]        Match node. Find the relative position with respect to base node.
     * @param csm_results[out]      Pair with first entry as estimated position of node 2 relative to node 1 based on
     *                          scan alignment and second entry as estimated covariance.
     */
    void runCSM(PgNode &base_node, PgNode &match_node, std::pair<pose_2d::Pose2Df, Eigen::MatrixXd> &csm_results);

    // Utility functions.
    void convertLidar2PointCloud(
        const std::vector<float> &ranges,
        float range_min,
        float range_max,
        float angle_min,
        float angle_max);

    pose_2d::Pose2Df transformPoseFromSrc2Map(const pose_2d::Pose2Df &pose_rel_src_frame,
                                              const pose_2d::Pose2Df &src_frame_pose_rel_map_frame);

    pose_2d::Pose2Df transformPoseFromMap2Target(const pose_2d::Pose2Df &pose_rel_map_frame,
                                                 const pose_2d::Pose2Df &target_frame_pose_rel_map_frame);

    // stop front end SLAM
    void stop_frontend();

  private:
    // Previous odometry-reported locations.
    Eigen::Vector2f prev_odom_loc_;
    float prev_odom_angle_;
    bool odom_initialized_;

    std::vector<Eigen::Vector2f> recent_point_cloud_;

    bool first_scan;

    pose_2d::Pose2Df last_node_odom_pose_;

    float last_node_cumulative_dist_;

    gtsam::NonlinearFactorGraph *graph_;

    gtsam::ISAM2 *isam_;

    std::vector<PgNode> pg_nodes_;

    CorrelativeScanMatcher matcher;

    bool stopSlamCmdRecv_;
  };
} // namespace slam

#endif // SRC_SLAM_H_
