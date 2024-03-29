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
\file    navigation.h
\brief   Interface for reference Navigation class.
\author  Joydeep Biswas, (C) 2019
*/
//========================================================================

#include <vector>
#include <deque>

#include "eigen3/Eigen/Dense"

#include "vector_map/vector_map.h"

#ifndef NAVIGATION_H
#define NAVIGATION_H

#define CAR_LENGTH 0.535
#define CAR_WIDTH 0.281
#define CAR_BASE 0.324
#define CAR_CMAX 1
#define SAFETY_MARGIN 0.03

// heuristic 
#define PENALTY_CURVE  2

using std::vector;

namespace ros {
  class NodeHandle;
}  // namespace ros

namespace navigation {

// car params
// const float CAR_LENGTH = 0.535;
// const float CAR_WIDTH = 0.281;
// const float CAR_BASE = 0.324;
// const float CAR_CMAX = 1;
// const float SAFETY_MARGIN = 0.3;

struct PathOption {
  float curvature;
  float clearance;
  float free_path_length;
  Eigen::Vector2f obstruction;
  Eigen::Vector2f closest_point;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
};

struct Control {
  float curvature;
  float velocity;
};

struct Position {
  float x;
  float y;

  Position(float x, float y) : x(x), y(y) {}

  bool operator==(const Position& other) const {
    return x == other.x && y == other.y;
  }

  bool operator!=(const Position& other) const {
    return !(*this == other);
  }
};

class Navigation {
 public:

   // Constructor
  explicit Navigation(const std::string& map_file, ros::NodeHandle* n);

  // Used in callback from localization to update position.
  void UpdateLocation(const Eigen::Vector2f& loc, float angle);

  // Used in callback for odometry messages to update based on odometry.
  void UpdateOdometry(const Eigen::Vector2f& loc,
                      float angle,
                      const Eigen::Vector2f& vel,
                      float ang_vel);

  // Updates based on an observed laser scan
  void ObservePointCloud(const std::vector<Eigen::Vector2f>& cloud,
                         double time);

  // Main function called continously from main
  void Run();
  // Used to set the next target pose.
  void SetNavGoal(const Eigen::Vector2f& loc, float angle);

  void ObstacleAvoidance();

  void GlobalPlanner(vector<Position> &path);

  void LocalPlanner(Position &goal);

  bool CheckNavComplete();

  PathOption ChoosePath(const vector<float> &candidate_curvs, const Position &goal);

  float ComputeFreePathLength(float curvature, Position &endpoint);

  float ComputeClearance(float free_path_len, float curv);
 
  float LatencyCompensation(size_t queue_size = 3);

  // Sample candidate curvatures. Only needed for the first time
  void GenerateCurvatures(int num_samples);

  // Compute control commands based on free path length
  float ComputeTOC(float free_path_length, float velocity);

  // Run sine wave velocity for calculating latency (peroid = T)
  void RunSineWave(float T);

  // for testing
  // visualization
  void drawCar(bool withMargin);

  void drawPointCloud();
 private:

  // Whether odometry has been initialized.
  bool odom_initialized_;
  // Whether localization has been initialized.
  bool localization_initialized_;
  // Current robot location.
  Eigen::Vector2f robot_loc_;
  // Current robot orientation.
  float robot_angle_;
  // Current robot velocity.
  Eigen::Vector2f robot_vel_;
  // Current robot angular speed.
  float robot_omega_;
  // Odometry-reported robot location.
  Eigen::Vector2f odom_loc_;
  // Odometry-reported robot angle.
  float odom_angle_;
  // Odometry-reported robot starting location.
  Eigen::Vector2f odom_start_loc_;
  // Odometry-reported robot starting angle.
  float odom_start_angle_;
  // Latest observed point cloud.
  std::vector<Eigen::Vector2f> point_cloud_;

  // Whether navigation is complete.
  bool nav_complete_;
  // Navigation goal location.
  Eigen::Vector2f nav_goal_loc_;
  // Navigation goal angle.
  float nav_goal_angle_;
  // Map of the environment.
  vector_map::VectorMap map_;

  // Generated curvatures
  vector<float> curvatures_;

  // Configuration
  static constexpr float dt = 0.05f;
  static constexpr float max_speed = 1.0f;
  static constexpr float max_curvature = 1.f / 0.98f;
  static constexpr float max_acceleration = 4.f;
  
  // Control queue for latency compensation
  std::deque<Control> control_queue;
};

}  // namespace navigation

class Graph {
  vector<vector<int>>& graph;
  std::unordered_map<GridLocation, GridLocation> came_from;
  std::unordered_map<GridLocation, double> cost_so_far;

  void GenerateGraph();
  void AStarSearch();

}
#endif  // NAVIGATION_H
