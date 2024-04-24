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
#include <ros/ros.h>
#include <gtsam/nonlinear/ISAM2.h>
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

#include "config_reader/config_reader.h"

using namespace gtsam;
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


// config from .lua file
CONFIG_FLOAT(min_angle_diff_between_nodes, "min_angle_diff_between_nodes");
CONFIG_FLOAT(min_trans_diff_between_nodes, "min_trans_diff_between_nodes");
double scanner_range = 30.0;
double trans_range = 2.0;
double low_res = 0.3;
double high_res = 0.03;
float k1 = 0.1;
float k2 = 0.05;
float k3 = 0.1;
float k4 = 0.1;

// PoseGraph Parameters
CONFIG_BOOL(considerOdomConstraint, "considerOdomConstraint");
CONFIG_FLOAT(new_node_x_std,"new_node_x_std");
CONFIG_FLOAT(new_node_y_std,"new_node_y_std");
CONFIG_FLOAT(new_node_theta_std,"new_node_theta_std");
CONFIG_FLOAT(max_factors_per_node,"max_factors_per_node");
CONFIG_FLOAT(maximum_node_dis_scan_comparison,"maximum_node_dis_scan_comparison");
CONFIG_BOOL(non_successive_scan_constraints,"non_successive_scan_constraints");


// Motion Model Parameters
CONFIG_FLOAT(motion_model_trans_err_from_trans,"motion_model_trans_err_from_trans");
CONFIG_FLOAT(motion_model_trans_err_from_rot,"motion_model_trans_err_from_rot");
CONFIG_FLOAT(motion_model_rot_err_from_trans,"motion_model_rot_err_from_trans");
CONFIG_FLOAT(motion_model_rot_err_from_rot,"motion_model_rot_err_from_rot");




namespace slam
{

  // load config from file
  config_reader::ConfigReader config_reader_({"config/slam.lua"});

  SLAM::SLAM() : prev_odom_loc_(0, 0),
                 prev_odom_angle_(0),
                 odom_initialized_(false),
                 first_scan(true),
                 last_node_cumulative_dist_(0), 
                 matcher(
                  scanner_range, trans_range,
                  low_res, high_res,
                  k1, k2, k3, k4)
                {
                  graph_ = new NonlinearFactorGraph();
                  isam_ = new ISAM2(); 
                 }
  
  // return global frame
  void SLAM::GetPose(Eigen::Vector2f *loc, float *angle) 
  {
    // Return the latest pose estimate of the robot.
    // *loc = Vector2f(0, 0);
    // *angle = 0;
    if (pg_nodes_.size() == 0) {
      *loc = Vector2f(0, 0);
      *angle = 0;
    } else {
      // Transform odomoetry pose from map frame to odometry frame. Get M(i, i-1) = M(i, odom) * M(i-1, odom)^-1
      pose_2d::Pose2Df rel_pos_to_last_node_odom_pose = transformPoseFromMap2Target(
                                                          pose_2d::Pose2Df(prev_odom_angle_,prev_odom_loc_), 
                                                          last_node_odom_pose_);
      
      // Transform rel_pos_to_last_node_odom_pose to global frame. Get M(i, global_frame) = M(i, i-1) * M(i-1, global_frame)
      pose_2d::Pose2Df pos_map = transformPoseFromSrc2Map(rel_pos_to_last_node_odom_pose, pg_nodes_.back().getEstimatedPose());

      *angle = pos_map.angle;
      *loc = pos_map.translation;
    
    }
  }

  
  std::vector<PgNode> SLAM::GetPgNodes() const 
  {
    return pg_nodes_;
  }

  void SLAM::ObserveLaser(const vector<float> &ranges,
                          float range_min,
                          float range_max,
                          float angle_min,
                          float angle_max)
  {
    // A new laser scan has been observed. Decide whether to add it as a pose
    // for SLAM. If decided to add, align it to the scan from the last saved pose,
    // and save both the scan and the optimized pose.

    if (shouldAddPgNode())
    {
      ROS_INFO_STREAM("Adding new node...");
      // convert recent lidar scan to recent_point_cloud_
      convertLidar2PointCloud(ranges, range_min, range_max, angle_min, angle_max);
      updatePoseGraph();
    }
  }

  bool SLAM::shouldAddPgNode()
  {
    // Check if odom has changed enough since the last node.
    float angle_diff = math_util::AngleDist(prev_odom_angle_, last_node_odom_pose_.angle);

    if ((last_node_cumulative_dist_ > CONFIG_min_trans_diff_between_nodes)
    || (angle_diff > CONFIG_min_angle_diff_between_nodes)) {
        last_node_cumulative_dist_ = 0.0;
        return true;
    }
    return false;
  }


  void SLAM::updatePoseGraph()
  {
    // TODO: update pose graph.
    if (first_scan)
    {
      ROS_INFO_STREAM("first scan");
      first_scan = false;
      // first scan

      // Add prior instead of odom
      // We set global frame as (0, 0, 0).
      pose_2d::Pose2Df _pose(0.0, Vector2f(0.0, 0.0));
      uint32_t node_number = pg_nodes_.size();
      PgNode new_node(_pose, node_number, recent_point_cloud_);

      Pose2 init_pos(0.0, 0.0, 0.0);
      noiseModel::Diagonal::shared_ptr init_noise =
          noiseModel::Diagonal::Sigmas(Vector3(CONFIG_new_node_x_std,
                                               CONFIG_new_node_y_std,
                                               CONFIG_new_node_theta_std));
      graph_->add(PriorFactor<Pose2>(new_node.getNodeNumber(), init_pos, init_noise));
      
      // odom_only_estimates_.emplace_back(std::make_pair(prev_odom_loc_, prev_odom_angle_));
      last_node_odom_pose_.Set(
        prev_odom_angle_,
        prev_odom_loc_
      );
      
      // TODO: add a node without observation constraints
      pg_nodes_.push_back(new_node);
      gtsam::Values init_estimate_for_new_node;
      init_estimate_for_new_node.insert(new_node.getNodeNumber(), Pose2(new_node.getEstimatedPose().translation.x(),
                                                                        new_node.getEstimatedPose().translation.y(),
                                                                        new_node.getEstimatedPose().angle));
      optimizePoseGraph(init_estimate_for_new_node);

    }
    else
    {
      // not first scan

      // Transform odomoetry pose from map frame to odometry frame. Get M(i, i-1) = M(i, odom) * M(i-1, odom)^-1
      // transform prev odom change from map frame to last node's frame
      pose_2d::Pose2Df rel_pos_to_last_node_odom_pose = transformPoseFromMap2Target(
                                                          pose_2d::Pose2Df(prev_odom_angle_,prev_odom_loc_), 
                                                          last_node_odom_pose_);
      
      // Transform rel_pos_to_last_node_odom_pose to global frame. Get M(i, global_frame) = M(i, i-1) * M(i-1, global_frame)
      pose_2d::Pose2Df pos_map = transformPoseFromSrc2Map(rel_pos_to_last_node_odom_pose, pg_nodes_.back().getEstimatedPose());

      // TODO: create new pgnode
     
      uint32_t node_number = pg_nodes_.size();
      // TODO: not sure what frame to use here
      // PgNode new_node(rel_pos_to_last_node_odom_pose, node_number, recent_point_cloud_);
      PgNode new_node(pos_map, node_number, recent_point_cloud_);

      
      if (CONFIG_considerOdomConstraint)
      {
        // Add an odometry constraint
        float transl_std = (CONFIG_motion_model_trans_err_from_trans * (rel_pos_to_last_node_odom_pose.translation.norm())) +
                             (CONFIG_motion_model_trans_err_from_rot * fabs(rel_pos_to_last_node_odom_pose.angle));
        float rot_std = (CONFIG_motion_model_rot_err_from_trans * (rel_pos_to_last_node_odom_pose.translation.norm())) +
                            (CONFIG_motion_model_rot_err_from_rot * fabs(rel_pos_to_last_node_odom_pose.angle));
        noiseModel::Diagonal::shared_ptr odometryNoise =
            noiseModel::Diagonal::Sigmas(Vector3(transl_std, transl_std, rot_std));
        Pose2 odometry_offset_est(rel_pos_to_last_node_odom_pose.translation.x(), rel_pos_to_last_node_odom_pose.translation.y(), rel_pos_to_last_node_odom_pose.angle);

        graph_->add(BetweenFactor<Pose2>(pg_nodes_.back().getNodeNumber(), new_node.getNodeNumber(), odometry_offset_est, odometryNoise));
      }

      // odom_only_estimates_.emplace_back(std::make_pair(prev_odom_loc_, prev_odom_angle_));
      last_node_odom_pose_.Set(
        prev_odom_angle_,
        prev_odom_loc_
      );
      
      // Add observation constraints
      updatePoseGraphObsConstraints(new_node);
    }

    ROS_INFO_STREAM("Num edges " << graph_->size());
    ROS_INFO_STREAM("Num nodes " << graph_->keys().size());
  }


  void SLAM::addObservationConstraint(const size_t &from_node_num, const size_t &to_node_num,
                                      std::pair<pose_2d::Pose2Df, Eigen::Matrix3f> &constraint_info)
  {

    Pose2 factor_translation(constraint_info.first.translation.x(), constraint_info.first.translation.y(), constraint_info.first.angle);
    noiseModel::Gaussian::shared_ptr factor_noise = noiseModel::Gaussian::Covariance(constraint_info.second.cast<double>());
    //        ROS_INFO_STREAM("Adding constraint from node " << from_node_num << " to node " << to_node_num <<" factor " << factor_transl.x() << ", " << factor_transl.y() << ", " << factor_transl.theta());
    graph_->add(BetweenFactor<Pose2>(from_node_num, to_node_num, factor_translation, factor_noise));
  }

  void SLAM::ObserveOdometry(const Vector2f &odom_loc, const float odom_angle)
  {
    if (!odom_initialized_)
    {
      odom_initialized_ = true;
    }

    // Keep track of cumulative distance between each odemetry observation
    last_node_cumulative_dist_ += (odom_loc - prev_odom_loc_).norm();
    prev_odom_angle_ = odom_angle;
    prev_odom_loc_ = odom_loc;
  }

  void SLAM::runCSM(PgNode &base_node, PgNode &match_node, std::pair<pose_2d::Pose2Df, Eigen::MatrixXd> &csm_results)
  {
    // TODO: need revision to real CSM
    // using fake CSM right now

    // pose_2d::Pose2Df rel_pose = match_node.getEstimatedPose() - base_node.getEstimatedPose();
    // match node's pose relative to base node's pose
    pose_2d::Pose2Df rel_pose = transformPoseFromMap2Target(match_node.getEstimatedPose(), 
                                                                                  base_node.getEstimatedPose());

    Eigen::Matrix3d est_cov;
      est_cov << 1.0, 0, 0,
              0, 1.0, 0,
              0, 0, 1.0;

    csm_results = std::make_pair(rel_pose, est_cov);
  }

  void SLAM::convertLidar2PointCloud(const std::vector<float> &ranges,
        float range_min,
        float range_max,
        float angle_min,
        float angle_max)
  {
    // clear 
    recent_point_cloud_.clear();
    const Vector2f kLaserLoc(0.2, 0);
    float angle_increment = (angle_max - angle_min) / (ranges.size() - 1.0);
    float current_angle = angle_min - angle_increment;
    unsigned int N = floor(( angle_max - angle_min) / angle_increment);
    for (unsigned int i = 0 ;i < N; ++i )
    {
      current_angle += angle_increment;
      if (ranges[i] >= range_max || ranges[i] <= range_min) {
        // out of range -> should discard
        continue;
      }
      // convert to euclidean space
      Vector2f _point( ranges[i] * cos(current_angle) , ranges[i] * sin(current_angle));
      // consider the shift between lidar and base_link
      _point = _point + kLaserLoc;
      recent_point_cloud_.push_back(_point);
    }
  }
void SLAM::updatePoseGraphObsConstraints(PgNode &new_node) {
  
  ROS_INFO_STREAM("Updateing PoseGraphObsConstraints");
  
  PgNode preceding_node = pg_nodes_.back();

  // Add laser factor for previous pose and this node
  std::pair<pose_2d::Pose2Df, Eigen::Matrix3f> successive_scan_offset;
  ScanMatch(preceding_node, new_node, successive_scan_offset); 
  // build edge of observation constraint
  addObservationConstraint(preceding_node.getNodeNumber(), new_node.getNodeNumber(), successive_scan_offset);

  // Add constraints for non-successive scans
  if (CONFIG_non_successive_scan_constraints && pg_nodes_.size() > 2) {
      // TODO: specify skip_count and start_num
      int skip_count = 1;
      size_t start_num = 0;
      int num_added_factors = 0;
      // for every non-successive scan
      for (size_t i = start_num; i < (pg_nodes_.size() - 2); i+= skip_count) {
        if (num_added_factors >= CONFIG_max_factors_per_node) {
            break;
        }
        
        PgNode node = pg_nodes_[i];

        float node_dist = (node.getEstimatedPose().translation -
                            preceding_node.getEstimatedPose().translation).norm();
        
        if (node_dist <= CONFIG_maximum_node_dis_scan_comparison) {
            std::pair<pose_2d::Pose2Df, Eigen::Matrix3f> non_successive_scan_offset;
            ScanMatch(node, preceding_node, non_successive_scan_offset);
            // build edge of observation constraint
            addObservationConstraint(node.getNodeNumber(), preceding_node.getNodeNumber(),
                                      non_successive_scan_offset);
            num_added_factors++;
        }
      }
    }
  
  // TODO: should we put it in the beginning?
  pg_nodes_.push_back(new_node);
  
  gtsam::Values init_estimate_for_new_node;
  init_estimate_for_new_node.insert(new_node.getNodeNumber(), Pose2(new_node.getEstimatedPose().translation.x(),
                                                                    new_node.getEstimatedPose().translation.y(),
                                                                    new_node.getEstimatedPose().angle));
  optimizePoseGraph(init_estimate_for_new_node);

}

void SLAM::optimizePoseGraph(gtsam::Values &new_node_init_estimates) {
  // Optimize the trajectory and update the nodes' position estimates
  // TODO do we need other params here?
  isam_->update(*graph_, new_node_init_estimates);
  Values result = isam_->calculateEstimate();

  // update each node int the graph using the optimized values
  for (PgNode &pg_node : pg_nodes_) {

      // Node number is the key, so we'll access the results using that
      Pose2 estimated_pose = result.at<Pose2>(pg_node.getNodeNumber());
      pg_node.setPose(Vector2f(estimated_pose.x(), estimated_pose.y()), estimated_pose.theta());
  }
}



vector<Eigen::Vector2f> SLAM::GetMap() {
  vector<Eigen::Vector2f> map;
  // Reconstruct the map as a single aligned point cloud from all saved poses
  // and their respective scans.
  for (size_t i = 0; i < pg_nodes_.size(); i++) {
      PgNode node = pg_nodes_[i];
      std::vector<Eigen::Vector2f> point_cloud = node.getPointCloud();

      pose_2d::Pose2Df node_pose = node.getEstimatedPose();
      for (Eigen::Vector2f point : point_cloud) {
          map.push_back(transformPoseFromSrc2Map(pose_2d::Pose2Df(0, point), node_pose).translation);
      }
  }
  return map;
}

  // Utility functions
  // trasfrom a 2D pose from src frame to map frame
  pose_2d::Pose2Df SLAM::transformPoseFromSrc2Map(const pose_2d::Pose2Df & pose_rel_src_frame, const pose_2d::Pose2Df & src_frame_pose_rel_map_frame) {
      // Rotate the point first
      Eigen::Rotation2Df rotation_mat(src_frame_pose_rel_map_frame.angle);
      Eigen::Vector2f rotated_still_src_transl = rotation_mat * pose_rel_src_frame.translation;

      // Then translate
      Eigen::Vector2f rotated_and_translated = src_frame_pose_rel_map_frame.translation + rotated_still_src_transl;
      float target_angle = AngleMod(src_frame_pose_rel_map_frame.angle + pose_rel_src_frame.angle);

      return pose_2d::Pose2Df(target_angle, rotated_and_translated);

  }

    // trasfrom a 2D pose from map frame to target frame
    pose_2d::Pose2Df SLAM::transformPoseFromMap2Target(const pose_2d::Pose2Df & pose_rel_map_frame, const pose_2d::Pose2Df & target_frame_pose_rel_map_frame) {
        // Translate the point
        Eigen::Vector2f trans = pose_rel_map_frame.translation - target_frame_pose_rel_map_frame.translation;

        // Then rotate
        Eigen::Rotation2Df rot_mat(-target_frame_pose_rel_map_frame.angle);
        Eigen::Vector2f final_trans = rot_mat * trans;

        float final_angle = AngleMod(pose_rel_map_frame.angle - target_frame_pose_rel_map_frame.angle);

        return pose_2d::Pose2Df(final_angle,final_trans);
    }

void SLAM::ScanMatch(PgNode &base_node, PgNode &match_node,
                     pair<pose_2d::Pose2Df, Eigen::Matrix3f> &result) {
  // Calculate initial guess of the relative pose from odometry.
const pose_2d::Pose2Df &base_pose = base_node.getEstimatedPose();
  const pose_2d::Pose2Df &match_pose = match_node.getEstimatedPose();
  const Trans odom(
    match_pose.translation - base_pose.translation,
    AngleDiff(match_pose.angle, base_pose.angle));

  // Run the scan matcher to get the relative pose and uncertainty.
  const pair<Trans, Eigen::Matrix3f> trans_and_uncertainty =
    matcher.GetTransAndUncertainty(base_node.getPointCloud(), match_node.getPointCloud(), odom);
  const Trans &trans = trans_and_uncertainty.first;
  // result.first = pose_2d::Pose2Df(trans.second, trans.first);
  // result.second = trans_and_uncertainty.second;

  result = std::make_pair(pose_2d::Pose2Df(trans.second, trans.first), trans_and_uncertainty.second);
}

}  // namespace slam
