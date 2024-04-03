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
#include <unordered_map>
#include <queue>
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

struct GridLocation {
  int x;
  int y;
  GridLocation(int x,int y): x(x),y(y) {}
  // Default constructor
  GridLocation() : x(0), y(0) {}
  
  GridLocation operator-(const GridLocation& other) {
      x = x - other.x;
      y = y - other.y;
      return *this;
  }

  bool operator==(const GridLocation& other) const {
        return x == other.x && y == other.y;
  }

  bool operator!=(const GridLocation& other) const {
      return !(*this == other);
  }

  // Comparison operator for < (This is dummy for PriorityQueue)
  bool operator<(const GridLocation& other) const {
      if (x < other.x)
          return true;
      if (x > other.x)
          return false;
      return y < other.y;
  }
};

// implement a Priority Queue with re-prioritize option 
template<typename T, typename priority_t>
struct PriorityQueue {
  typedef std::pair<priority_t, T> PQElement;
  std::priority_queue<PQElement, std::vector<PQElement>,
                 std::greater<PQElement>> elements;

  inline bool empty() const {
     return elements.empty();
  }

  inline void put(T item, priority_t priority) {
    elements.emplace(priority, item);
  }

  T get() {
    T best_item = elements.top().second;
    elements.pop();
    return best_item;
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

  PathOption ChoosePath(const vector<float> &candidate_curvs, const Eigen::Vector2f &goal);

  float ComputeFreePathLength(float curvature, const Eigen::Vector2f &goal, Eigen::Vector2f &endpoint, float &distance_to_goal);

  float ComputeClearance(float free_path_len, float curv);
 
  float LatencyCompensation(size_t queue_size = 3);

  // Sample candidate curvatures. Only needed for the first time
  void GenerateCurvatures(int num_samples);

  // Compute control commands based on free path length
  float ComputeTOC(float free_path_length, float velocity);

  // Run sine wave velocity for calculating latency (peroid = T)
  void RunSineWave(float T);

  // navigation
  void GlobalPlanner(vector<Eigen::Vector2f> &path);

  void LocalPlanner(Eigen::Vector2f &goal);

  bool CheckNavComplete();

  // for testing
  // visualization
  void drawCar(bool withMargin);

  void drawPointCloud();

  void drawGraph();

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

class MapGraph {
  public:
    MapGraph(const vector_map::VectorMap& );
    
    // check if the location is free of obstacle
    bool isFree(const GridLocation&);

    // convert a given point(map frame) to GridLocation
    // a quantization is performed
    GridLocation point2GridLoc(const Eigen::Vector2f&);

    // convert GridLoc to coordinate in map frame
    Eigen::Vector2f gridLoc2point(const GridLocation&);

    // draw gridlines (dark blue), cross (dark blue) as obstacles
    void drawObstacleGrid();
    
    // A* Search on a grid, return the found path
    void aStarSearch(const GridLocation& start, 
                          const GridLocation& goal, 
                          vector<GridLocation>& path); 
    
    
  private:  
    // check if point _p is on line segment _p0->_p1
    bool pointOnLineSegment(const GridLocation& _p, const GridLocation& _p0, const GridLocation& _p1 );
    // backtrack from goal to the start, reconstruct the path 
    void backtrackPath(const GridLocation& start, 
                        const GridLocation& goal, 
                        std::map<GridLocation, GridLocation>& came_from,
                        vector<GridLocation>& path); 
    
    double getEdgeCost(const GridLocation& p1, const GridLocation& p2);
    
    double getHeuristic(const GridLocation& p1, const GridLocation& p2);
    
    vector<GridLocation> getNeighbors(GridLocation current);
    
    // true: obstacle, false: free
    vector<vector<bool>> obstacle_grid;
    int grid_num_x = 0;
    int grid_num_y = 0;
    float map_max_x = 0;
    float map_min_x = 0;
    float map_max_y = 0;
    float map_min_y = 0;
  

};

}  // namespace navigation

#endif  // NAVIGATION_H
