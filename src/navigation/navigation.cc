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
\file    navigation.cc
\brief   Starter code for navigation.
\author  Joydeep Biswas, (C) 2019
*/
//========================================================================

#include "gflags/gflags.h"
#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Geometry"
#include "amrl_msgs/AckermannCurvatureDriveMsg.h"
#include "amrl_msgs/Pose2Df.h"
#include "amrl_msgs/VisualizationMsg.h"
#include "glog/logging.h"
#include "ros/ros.h"
#include "ros/package.h"
#include "shared/math/math_util.h"
#include "shared/util/timer.h"
#include "shared/ros/ros_helpers.h"
#include "config_reader/config_reader.h"
#include "navigation.h"
#include "visualization/visualization.h"

using Eigen::Vector2f;
using amrl_msgs::AckermannCurvatureDriveMsg;
using amrl_msgs::VisualizationMsg;
using std::string;
using std::vector;
using geometry::line2f;

using namespace math_util;
using namespace ros_helpers;

namespace {
ros::Publisher drive_pub_;
ros::Publisher viz_pub_;
VisualizationMsg local_viz_msg_;
VisualizationMsg global_viz_msg_;
AckermannCurvatureDriveMsg drive_msg_;
// Epsilon value for handling limited numerical precision.
const float kEpsilon = 1e-5;
} //namespace

// Hyperparameters
CONFIG_FLOAT(grid_dx, "grid_dx"); // dx for grid construction
CONFIG_FLOAT(grid_dy, "grid_dy"); // dy for grid construction

#define DIST_NAV_COMPLETE 0.5     // max distance to goal to be considered complete
#define DIST_REPLAN 10            // min distance to intermediate goal to replan
#define SCORE_GOAL 1              // weight for distance to goal

namespace navigation {

config_reader::ConfigReader config_reader_({"config/navigation.lua"});

string GetMapFileFromName(const string& map) {
  string maps_dir_ = ros::package::getPath("amrl_maps");
  return maps_dir_ + "/" + map + "/" + map + ".vectormap.txt";
}

Navigation::Navigation(const string& map_name, ros::NodeHandle* n) :
    odom_initialized_(false),
    localization_initialized_(false),
    robot_loc_(0, 0),
    robot_angle_(0),
    robot_vel_(0, 0),
    robot_omega_(0),
    nav_complete_(true),
    nav_goal_loc_(0, 0),
    nav_goal_angle_(0) {
  map_.Load(GetMapFileFromName(map_name));
  drive_pub_ = n->advertise<AckermannCurvatureDriveMsg>(
      "ackermann_curvature_drive", 1);
  viz_pub_ = n->advertise<VisualizationMsg>("visualization", 1);
  local_viz_msg_ = visualization::NewVisualizationMessage(
      "base_link", "navigation_local");
  global_viz_msg_ = visualization::NewVisualizationMessage(
      "map", "navigation_global");
  InitRosHeader("base_link", &drive_msg_.header);
}

void Navigation::SetNavGoal(const Vector2f& loc, float angle) {
  nav_goal_loc_ = loc;
  nav_goal_angle_ = angle;
  nav_complete_ = false;
}

void Navigation::UpdateLocation(const Eigen::Vector2f& loc, float angle) {
  localization_initialized_ = true;
  robot_loc_ = loc;
  robot_angle_ = angle;
}

void Navigation::UpdateOdometry(const Vector2f& loc,
                                float angle,
                                const Vector2f& vel,
                                float ang_vel) {
  robot_omega_ = ang_vel;
  robot_vel_ = vel;
  if (!odom_initialized_) {
    odom_start_angle_ = angle;
    odom_start_loc_ = loc;
    odom_initialized_ = true;
    odom_loc_ = loc;
    odom_angle_ = angle;
    return;
  }
  odom_loc_ = loc;
  odom_angle_ = angle;
}

void Navigation::ObservePointCloud(const vector<Vector2f>& cloud,
                                   double time) {
  point_cloud_ = cloud;                                     
}

float Navigation::ComputeClearance(float free_path_len, float curv) {
  float min_clearance = 10.0;
  float r = 1.0 / std::abs(curv);
  float turning_angle = free_path_len / r; // len = r * turning_angle
  float c_max = 3;

  float car_inner_y = CAR_WIDTH / 2.0 + SAFETY_MARGIN;
  float car_outter_y = -car_inner_y;
  float car_front_x = (CAR_BASE + CAR_LENGTH) / 2.0 + SAFETY_MARGIN;
  
  const Vector2f center_pt = Vector2f(0,r);  // turning instant center
  const Vector2f car_outter_front_pt = Vector2f(car_front_x,car_outter_y);
  
  const float r_min = r - (CAR_WIDTH / 2.0 + SAFETY_MARGIN);
  const float r_max = (center_pt - car_outter_front_pt ).norm();
  
  // Calculate clearance (distance to base_link)
  if (curv == 0) {    // straight line
    for (auto point : point_cloud_) {
      // filter out large clearance
      if (std::abs(point.y()) > c_max) continue;
      // filter out collision points
      if (point.y() <= car_inner_y && point.y() >= car_outter_y && point.x() >= car_front_x) {
        continue;
      }
      float cur_clearance = std::abs(point.y());
      min_clearance = std::min(min_clearance,cur_clearance);
    }

  } else { // turing 
    
    for (auto point : point_cloud_) {
      // for right turn
      if (curv < 0) point.y() = -point.y();

      float cur_clearance;
      float r_p = (center_pt - point).norm();
      // cout << "r_p, r: " << r_p << ", " << r << endl;
      float p_angle = atan2(point.x(), r - point.y()); // angle: init baselink, center, p
      // cout << "turning_angle, p_angle: " << turning_angle << ", " << p_angle << endl;
      // filter out collision 
      if (r_min <= r_p && r_p <= r_max) continue;
      // filter out far point
      if (std::abs(r_p - r) > c_max || p_angle < 0 || p_angle > turning_angle) { // TODO: condition on turning_angle is an assumption
        continue;
      }
      cur_clearance = std::abs(r_p - r); // |c-p| - rmax
      // cout << "curv, x, clearance: " << curv << ", " << point.x() << ", " << cur_clearance << endl;
      // if (cur_clearance < min_clearance) {
      //     // cout << "min_clearance, x, r_p: " << min_clearance << ", " << point.x() << ", " << r_p << endl;
      // }
      min_clearance = std::min(min_clearance,cur_clearance);
    }
  }
  // cout << "============/=======\n";
  return min_clearance;
}

PathOption Navigation::ChoosePath(const vector<float> &candidate_curvs, const Eigen::Vector2f &goal) {
 
  // PathOption return_path;

  float highest_score = -1;
  PathOption best_path;
  float score_clearance = 0; // hyper-param
  float score_curv = 0;
  float score_goal = SCORE_GOAL;
  for (auto _curv : candidate_curvs) {
    // draw options (gray)
    // visualization::DrawPathOption(_curv,1,5,0x808080,false,local_viz_msg_);

    Eigen::Vector2f endpoint(-1, -1);
    float free_path_len = ComputeFreePathLength(_curv, endpoint);
    float clearance = ComputeClearance(free_path_len, _curv);
    // float score = free_path_len + score_w * clearance - PENALTY_CURVE * std::abs(_curv);
    // visualization::DrawPathOption(_curv, free_path_len, clearance, 0xFF0000, false, local_viz_msg_);
    float score = free_path_len + 
                  score_clearance * clearance +
                  score_curv * std::abs(_curv) +
                  score_goal * (goal - endpoint).norm();
    if (score > highest_score) {
      highest_score = score;
      best_path.curvature = _curv;
      best_path.clearance = clearance;
      best_path.free_path_length = free_path_len;
    }
  }

  //std::cout<<"Score "<<highest_score<<"\n";
  // draw best option (blue)
  // visualization::DrawPathOption(best_path.curvature,best_path.free_path_length,best_path.clearance,0x0F03FC,false,local_viz_msg_);
  std::cout<<"Best C="<<best_path.curvature<<" Free Path Length="<<best_path.free_path_length<<"\n";
  
  // just for an example
  // return_path.curvature = best_curv;
  // return_path.clearance = 1.4;
  // return_path.free_path_length = best_option;
  // return_path.obstruction = Vector2f(0,0)
  // return_path.closest_point = Vector2f(0,0);

  return best_path;
    
}

float Navigation::ComputeFreePathLength(float curvature, Eigen::Vector2f &endpoint) {
   /* notation
  Angle
    theta: turing angle
  Radius
    r: turning radius
  
  inner_rear        inner_front(r1)
    *----------------*
    |                |
    |  *(0,0)        |>>
    |                |
    *----------------*
   outer_rear(r2)    outer_front(rmax)
  */
  // Car points (baselink frame)
  float car_inner_y = CAR_WIDTH / 2.0 + SAFETY_MARGIN;
  float car_outter_y = -car_inner_y;
  float car_front_x = (CAR_BASE + CAR_LENGTH) / 2.0 + SAFETY_MARGIN;
  //float car_rear_x = -(CAR_LENGTH - CAR_BASE) / 2.0 - SAFETY_MARGIN;
  
  



  float free_path_length = 10.0;

  if (curvature==0) {
    // go straight
    for (auto point : point_cloud_) {
      if (point.y() <= car_inner_y && point.y() >= car_outter_y && point.x() >= car_front_x) {
        free_path_length = std::min(free_path_length, point.x() - car_front_x );
      }
    }
  } else {
    // go curve
    const float r = 1.0 / std::abs(curvature);
    const Vector2f center_pt = Vector2f(0,r);  // turning instant center
    const Vector2f car_inner_front_pt = Vector2f(car_front_x,car_inner_y);
    const Vector2f car_outter_front_pt = Vector2f(car_front_x,car_outter_y);
    //const Vector2f car_inner_rear_pt = Vector2f(car_rear_x,car_inner_y);
    //const Vector2f car_outter_rear_pt = Vector2f(car_rear_x,car_outter_y);



    const float r_min = r - (CAR_WIDTH / 2.0 + SAFETY_MARGIN);
    const float r_max = (center_pt - car_outter_front_pt ).norm();
    const float r_1 = (center_pt - car_inner_front_pt ).norm();
    // const float r_2 = (center_pt - car_outter_rear_pt ).norm();


    for (auto point : point_cloud_) {
      if (curvature < 0)
        point.y() = -point.y();
        
      //visualization::DrawCross(point,0.2,2,local_viz_msg_); 
      // whether the car will hit the point
      bool collision = true;
      
      // r_p point to center
      float r_p = (center_pt - point).norm();

      // angle: point angle, init base_link -> point
      float theta = atan2(point.x() ,  (r - point.y())); // atan2(x / (r-y))
      // angle: new base_link -> point
      float omega; 

      if (theta > 0 && r_p >= r_1 && r_p <= r_max && point.x() > 0) { 
        // front side
        omega = asin(car_front_x / r_p); // asin(h/rp)
      } else if (theta > 0 && r_p >= r_min && r_p <= r_1 && point.x() > 0) {
        // inner side
        omega = acos((r - (CAR_WIDTH / 2.0 + SAFETY_MARGIN)) / r_p); // acos(r-w/2 / rp)
      } else {
        // case 3, or no collision
        collision = false;
      }

      if (collision) {
        //  turning angle: init base_link -> new base_link = theta - omega
        float cur_free_path_length = (theta - omega) * r; // turning_angle * r

        //std::cout<<"cur_free_path_length="<<cur_free_path_length<<" free_path_length="<<free_path_length<<"\n";
        if (cur_free_path_length < free_path_length) {
          free_path_length = cur_free_path_length;
          endpoint = point;
        }
      } 
    }
    
  }

  // draw free path length along the curve (cyan)
  // visualization::DrawPathOption(curvature,free_path_length,5,0x00FFFF,false,local_viz_msg_);
  return free_path_length;

}

void Navigation::ObstacleAvoidance() {
  static Eigen::Vector2f intermediate_goal(-1, -1);
  LocalPlanner(intermediate_goal);

  float cur_velocity = LatencyCompensation();

  // 1. Generate possible curvatures (kinemetic constraint)
  int num_samples = 10; // ?
  if (curvatures_.empty()) GenerateCurvatures(num_samples);
  // 2. For each possible path:
      // a. Compute Free Path Length
      // b. Compute Clearance
      // c. Compute Distance To Goal
      // d. Compute total “score”
  // 3. From all paths, pick path with best score
  PathOption chosen_path = ChoosePath(curvatures_, intermediate_goal);
  // path.curvature, path.free_path_length

  // 4. Implement 1-D TOC on the chosen arc.
  float velocity = ComputeTOC(chosen_path.free_path_length, cur_velocity); // calculate velocity

  // std::cout << "Curvature: " << chosen_path.curvature << "; velocity: " << velocity << "\n";
  drive_msg_.curvature = chosen_path.curvature;
  drive_msg_.velocity = velocity;

  // add to the control queue
  Control latest_control;
  latest_control.curvature = chosen_path.curvature;
  latest_control.velocity = velocity;
  control_queue.push_back(latest_control);
}

void Navigation::GenerateCurvatures(int num_samples = 100) {
  if (num_samples % 2 == 0) {
    num_samples += 1;
  }
  curvatures_.resize(num_samples);
  std::cout<<"Total samples="<<num_samples<<"\n";
  // half of samples
  num_samples = (num_samples - 1) / 2;
  std::cout<<"half samples="<<num_samples<<"\n";
  float _delta = CAR_CMAX * 1.0 / num_samples;
  std::cout<<"delta="<<_delta<<"\n";
  // straight line
  curvatures_[0] = 0;
  for (int i = 0; i < num_samples; ++i) {
    curvatures_[i*2+1] = _delta*(i+1);
    curvatures_[i*2+2] = -curvatures_[i*2+1];
  }
  return;
}

float Navigation::ComputeTOC(float free_path_length, float cur_velocity) {
  // float velocity = robot_vel_.norm();
  float velocity = cur_velocity;
  float min_dist = velocity * velocity / (2 * max_acceleration);

  if (free_path_length <= min_dist) {
    return std::max(velocity - dt * max_acceleration, 0.f);
  }

  if (velocity < max_speed) {
    return std::min(velocity + dt * max_acceleration, max_speed);
  }

  return max_speed;
}

void Navigation::drawCar(bool withMargin=true) {
  // draw car (black)
  float car_inner_y = CAR_WIDTH / 2.0;
  float car_outter_y = -car_inner_y;
  float car_front_x = (CAR_BASE + CAR_LENGTH) / 2.0;
  float car_rear_x = -(CAR_LENGTH - CAR_BASE) / 2.0;

  Vector2f car_inner_front_pt = Vector2f(car_front_x,car_inner_y);
  Vector2f car_outter_front_pt = Vector2f(car_front_x,car_outter_y);
  Vector2f car_inner_rear_pt = Vector2f(car_rear_x,car_inner_y);
  Vector2f car_outter_rear_pt = Vector2f(car_rear_x,car_outter_y);

  visualization::DrawLine(car_inner_front_pt,car_outter_front_pt,0, local_viz_msg_);
  visualization::DrawLine(car_outter_rear_pt,car_outter_front_pt,0, local_viz_msg_);
  visualization::DrawLine(car_inner_front_pt,car_inner_rear_pt,0, local_viz_msg_);
  visualization::DrawLine(car_inner_rear_pt,car_outter_rear_pt,0, local_viz_msg_);

  // draw margin (orange)
  float car_inner_y_m = CAR_WIDTH / 2.0 + SAFETY_MARGIN;
  float car_outter_y_m = -car_inner_y_m;
  float car_front_x_m = (CAR_BASE + CAR_LENGTH) / 2.0 + SAFETY_MARGIN;
  float car_rear_x_m = -(CAR_LENGTH - CAR_BASE) / 2.0 - SAFETY_MARGIN;

  Vector2f car_inner_front_pt_m = Vector2f(car_front_x_m,car_inner_y_m);
  Vector2f car_outter_front_pt_m = Vector2f(car_front_x_m,car_outter_y_m);
  Vector2f car_inner_rear_pt_m = Vector2f(car_rear_x_m,car_inner_y_m);
  Vector2f car_outter_rear_pt_m = Vector2f(car_rear_x_m,car_outter_y_m);

  visualization::DrawLine(car_inner_front_pt_m,car_outter_front_pt_m,0xFFC116, local_viz_msg_);
  visualization::DrawLine(car_outter_rear_pt_m,car_outter_front_pt_m,0xFFC116, local_viz_msg_);
  visualization::DrawLine(car_inner_front_pt_m,car_inner_rear_pt_m,0xFFC116, local_viz_msg_);
  visualization::DrawLine(car_inner_rear_pt_m,car_outter_rear_pt_m,0xFFC116, local_viz_msg_);


}

void Navigation::drawPointCloud() {
  // olive color
  for (auto point : point_cloud_) {
    // visualization::DrawCross(point,0.01,0x808000,local_viz_msg_);
    visualization::DrawPoint(point,0x808000,local_viz_msg_);
	}
}

void Navigation::RunSineWave(float T) {

  static float time_accum = 0;

  // print <drive_msg.vel>,<odemtry_vel>

  drive_msg_.curvature = 0;
  drive_msg_.velocity = sin(2* M_PI/ T * time_accum);
  time_accum += dt;
  if (time_accum>T)
    time_accum -= T;
  
  std::cout<<drive_msg_.velocity<<","<<robot_vel_.norm()<<"\n";
}

float Navigation::LatencyCompensation(size_t queue_size) {
  /*
    1.  Take the previous sent controls (not applied yet) to
        calculate where the car will be located and oriented at
        after the controls are applied.

    2.  Use the displacement and orientation change to compute a
        coordinate transformation matrix. Then apply the inverse of
        this matrix to the point cloud.

    3.  Return the latest velocity in the control queue.
  */

  if (control_queue.size() < queue_size) {
    return robot_vel_.norm();
  }

  // location and orientation w.r.t. original base_link
  float x = 0.f, y = 0.f;
  float theta = 0.f;

  for (const auto &control : control_queue) {
    if (std::abs(control.curvature) < 1E-3f) {
      // straight line
      x += control.velocity * std::cos(theta) * dt;
      y += control.velocity * std::sin(theta) * dt;
    } else {
      // curve
      float theta_prime = theta + control.velocity * control.curvature * dt;
      x += (-std::sin(theta) + std::sin(theta_prime)) / control.curvature;
      y += (std::cos(theta) - std::cos(theta_prime)) / control.curvature;
      theta = theta_prime;
    }
  }
  cout << "forward-predict: (x', y', theta'): (" << x << ", " << y << ", " << theta << ')' << endl;

  // rotation and translation matrix
  Eigen::Matrix3f transform;
  transform <<
        std::cos(theta), -std::sin(theta), x * std::cos(theta) - y * std::sin(theta),
        std::sin(theta), std::cos(theta), x * std::sin(theta) + y * std::cos(theta),
        0.f, 0.f, 1.f;

  // draw an example
  // Eigen::Vector3f example_pt1(0,0,1);
  // Eigen::Vector3f example_pt2(1,0,1);
  // example_pt1 = transform * example_pt1;
  // example_pt2 = transform * example_pt2;
  // visualization::DrawLine(Vector2f(example_pt1.x(), example_pt1.y()), Vector2f(example_pt2.x(), example_pt2.y()), 0xEB4EED ,local_viz_msg_);

  // Transform the lidar points using translation and rotation
  for (auto& point : point_cloud_) {
      // homogeneous coordinates
      Eigen::Vector3f p(point.x(), point.y(), 1);
      p = transform.inverse() * p;
      point.x() = p.x() / p.z();
      point.y() = p.y() / p.z();
  }

  // pop out the oldest control and return the lastest velocity
  control_queue.pop_front();
  return control_queue.back().velocity;
}

// void Navigation::GlobalPlanner(vector<Position> &path) {
void Navigation::GlobalPlanner(vector<Eigen::Vector2f> &path) {
  static MapGraph mapGraph(map_);
  mapGraph.drawObstacleGrid();
  // nav goal: nav_goal_loc_, nav_goal_angle_
  // robot_loc_: robot_loc_, robot_angle_
  /*

  */
  // int h = map_h / fix_res;
  // int w = map_w / fix_res;
  // vector<vector<int>>& graph; // 1: -1: not available

  //  graph (bool: Obstacle->False)
  // static vector<vector<bool>> obstacle_graph;
  // this->GenerateGraph(obstacle_graph);

  // frontier = PriorityQueue() // 
  // frontier.put(start, 0)
  // came_from = dict()
  // cost_so_far = dict()
  // came_from[start] = None
  // cost_so_far[start] = 0

  // while not frontier.empty():
  //   current = frontier.get()

  //   if current == goal:
  //       break
    
  //   for next in graph.neighbors(current):
  //       new_cost = cost_so_far[current] + graph.cost(current, next)
  //       if next not in cost_so_far or new_cost < cost_so_far[next]:
  //         cost_so_far[next] = new_cost
  //         priority = new_cost + heuristic(goal, next)
  //         frontier.put(next, priority)
  //         came_from[next] = current

}

void Navigation::LocalPlanner(Eigen::Vector2f &goal) {
  static vector<Eigen::Vector2f> path;

  // // check if we need to replan
  // if (goal == Position(-1, -1) || (robot_loc_ - goal).norm() > DIST_REPLAN) {
  //   GlobalPlanner(path);
  // }

  // // TODO: run through the path and find the farthest valid point
  // goal = path[0];
}

bool Navigation::CheckNavComplete() {
  if (nav_complete_) {
    return true;
  }
  nav_complete_ = (robot_loc_ - nav_goal_loc_).norm() < DIST_NAV_COMPLETE;
  return nav_complete_;
}

void Navigation::Run() {
  // This function gets called 20 times a second to form the control loop.
  
  // Clear previous visualizations.
  visualization::ClearVisualizationMsg(local_viz_msg_);
  visualization::ClearVisualizationMsg(global_viz_msg_);

  // If odometry has not been initialized, we can't do anything.
  if (!odom_initialized_) return;

  // if (CheckNavComplete()) return;
  vector<Eigen::Vector2f> pp;
  GlobalPlanner(pp);


  // ObstacleAvoidance();

  // Add timestamps to all messages.
  local_viz_msg_.header.stamp = ros::Time::now();
  global_viz_msg_.header.stamp = ros::Time::now();
  drive_msg_.header.stamp = ros::Time::now();
  // Publish messages.
  viz_pub_.publish(local_viz_msg_);
  viz_pub_.publish(global_viz_msg_);
  drive_pub_.publish(drive_msg_);
}

MapGraph::MapGraph(const vector_map::VectorMap& map_) {
  // initilize graph
  cout<<"Initialize Grid:"<<endl;
  cout<<"CONFIG_grid_dx: "<<CONFIG_grid_dx<<endl;
  cout<<"CONFIG_grid_dy: "<<CONFIG_grid_dy<<endl;

  // get bounradies of the map
  for (size_t i = 0; i < map_.lines.size(); ++i) {
    const line2f map_line = map_.lines[i];
    this->map_max_x = std::max(this->map_max_x,map_line.p0.x());
    this->map_max_x = std::max(this->map_max_x,map_line.p1.x());
    this->map_min_x = std::min(this->map_min_x,map_line.p0.x());
    this->map_min_x = std::min(this->map_min_x,map_line.p1.x());

    this->map_max_y = std::max(this->map_max_y,map_line.p0.y());
    this->map_max_y = std::max(this->map_max_y,map_line.p1.y());
    this->map_min_y = std::min(this->map_min_y,map_line.p0.y());
    this->map_min_y = std::min(this->map_min_y,map_line.p1.y());



  }

  cout<<"map_max_x="<<this->map_max_x<<endl;
  cout<<"map_min_x="<<this->map_min_x<<endl;
  cout<<"map_max_y="<<this->map_max_y<<endl;
  cout<<"map_min_y="<<this->map_min_y<<endl;

  // initialize obstacle_grid
  obstacle_grid.clear();
  // obstacle_grid [x][y]
  // only define in first quadrant (x>0, y>0)

  grid_num_x = std::round( (map_max_x - map_min_x) / CONFIG_grid_dx );
  grid_num_y = std::round( (map_max_y - map_min_y) / CONFIG_grid_dy );

  cout<<"grid width(x)="<<grid_num_x<<endl;
  cout<<"grid height(y)="<<grid_num_y<<endl;

  obstacle_grid.resize(grid_num_x);
  for (int i = 0; i < grid_num_x; ++i) {
    obstacle_grid[i].resize(grid_num_y);
  }

  for (int i_x = 0; i_x < grid_num_x; ++i_x) {
    for (int i_y = 0; i_y < grid_num_y; ++i_y) {
      GridLocation p(i_x,i_y);
      obstacle_grid[i_x][i_y] = false;
      for (size_t line_i = 0; line_i < map_.lines.size(); ++line_i) {
        const line2f map_line = map_.lines[line_i];

        GridLocation p0 = point2GridLoc(map_line.p0);
        GridLocation p1 = point2GridLoc(map_line.p1);

        if (pointOnLineSegment(p,p0,p1)) {
          obstacle_grid[i_x][i_y] = true;
          break;
        }
      }
    }
  }

}

void MapGraph::drawObstacleGrid() {
  // draw gridlines (dark blue), cross (dark blue) as obstacles

  for (int i = 0; i < grid_num_x; ++i) {
    Vector2f pt_a(i * CONFIG_grid_dx + map_min_x, map_min_y);
    Vector2f pt_b(i * CONFIG_grid_dx + map_min_x, grid_num_y * CONFIG_grid_dy + map_min_y);
    visualization::DrawLine(pt_a,pt_b,0x0502e2, global_viz_msg_);
  }
  for (int i = 0; i < grid_num_y; ++i) {
    Vector2f pt_a(map_min_x, i * CONFIG_grid_dy + map_min_y);
    Vector2f pt_b(grid_num_x * CONFIG_grid_dx + map_min_x, i * CONFIG_grid_dy + map_min_y);
    visualization::DrawLine(pt_a,pt_b,0x0502e2, global_viz_msg_);
  }

  for (int i_x = 0; i_x < grid_num_x; ++i_x) {
    for (int i_y = 0; i_y < grid_num_y; ++i_y) {
      if (obstacle_grid[i_x][i_y]) {
        Vector2f pt = gridLoc2point(GridLocation(i_x,i_y));
        visualization::DrawCross(
          pt,
          CONFIG_grid_dx / 2.0,
          0x0502e2,
          global_viz_msg_
        );
      }
    }
  }

}

GridLocation MapGraph::point2GridLoc(const Eigen::Vector2f& _point) {
  return GridLocation(
    std::round( (_point.x() - this->map_min_x) / CONFIG_grid_dx),
    std::round( (_point.y() - this->map_min_y) / CONFIG_grid_dy)
  );
}

Eigen::Vector2f MapGraph::gridLoc2point(const GridLocation& _gridLoc) {
  Eigen::Vector2f _pt(
    _gridLoc.x * CONFIG_grid_dx + this->map_min_x,
    _gridLoc.y * CONFIG_grid_dy + this->map_min_y
  );
  return _pt;
}

bool MapGraph::isFree(const GridLocation& _gridLoc) {
  return !(obstacle_grid[_gridLoc.x][_gridLoc.y]);
}

bool MapGraph::pointOnLineSegment(const GridLocation& _p, const GridLocation& _p0, const GridLocation& _p1 ) {
  // check if in bounded box
  if ( !((_p.x >= _p1.x && _p.x <= _p0.x)  ||  (_p.x <= _p1.x && _p.x >= _p0.x))  ) return false;
  if ( !((_p.y >= _p1.y && _p.y <= _p0.y)  ||  (_p.y <= _p1.y && _p.y >= _p0.y))  ) return false;

  // check if on line segment
  Eigen::Vector3f vec_p0_p( _p.x - _p0.x, _p.y - _p0.y , 0);
  Eigen::Vector3f vec_p0_p1( _p1.x - _p0.x, _p1.y - _p0.y, 0 );

  float cross_product = vec_p0_p.cross(vec_p0_p1).z();

  if (cross_product == 0) {
    return true;
  } else {
    return false;
  }
}

}  // namespace navigation

