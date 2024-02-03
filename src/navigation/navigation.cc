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
#include "navigation.h"
#include "visualization/visualization.h"

using Eigen::Vector2f;
using amrl_msgs::AckermannCurvatureDriveMsg;
using amrl_msgs::VisualizationMsg;
using std::string;
using std::vector;

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

namespace navigation {

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


float ComputeClearance(float free_path_len, float curv) {
  float min_clearance = 1000.0;
  // float c_max = ??
  float r = 1.0 / curv;
  float car_outter_y = -car_inner_y;
  float car_front_x = (CAR_BASE + CAR_LENGTH) / 2.0 + SAFETY_MARGIN;
  
  const Vector2f center_pt = Vector2f(0,r);  // turning instant center
  const Vector2f car_outter_front_pt = Vector2f(car_front_x,car_outter_y);
  
  const float r_min = r - (CAR_WIDTH / 2.0 + SAFETY_MARGIN);
  const float r_max = (center_pt - car_outter_front_pt ).norm();

  if (curv == 0) {    // straight line
    for (auto point : point_cloud_) {
      float cur_clearance = point.y() - CAR_WIDTH;
      min_clearance = min(min_clearance,cur_clearance);
    }

  } else { // turing 
    for (auto point : point_cloud_) {
      // TODO: if the point will affect the clearance
      float cur_clearance;
      float center_to_point_dist = (center_pt - point).norm();
      if (center_to_point_dist > r) { // |c-p| > r
        cur_clearance = center_to_point_dist - r_max; // |c-p| - rmax
      } else {
        cur_clearance = r_min - center_to_point_dist;
      }
      min_clearance = min(min_clearance,cur_clearance);
    }
  }

  return min_clearance;
}
PathOption Navigation::ChoosePath(vector<PathOption> candidate_paths) {
 
  // PathOption return_path;

  float highest_score = -1;
  PathOption best_path;
  float score_w = 0.5; // hyper-param

  for (auto path : candidate_paths) {
    float _curv = path.curvature;
    float free_path_len = ComputeFreePathLength(_curv);
    float clearance = ComputeClearance(free_path_len, _curv);
    float score = free_path_len + score_w * clearance;
    if (score > highest_score) {
      highest_score = score;
      best_path.curvature = _curv;
      best_path.clearance = clearance;
      best_path.free_path_len = free_path_len;
    }
  }

  // just for an example
  // return_path.curvature = best_curv;
  // return_path.clearance = 1.4;
  // return_path.free_path_length = best_option;
  // return_path.obstruction = Vector2f(0,0)
  // return_path.closest_point = Vector2f(0,0);

  return best_path;
    
}

float Navigation::ComputeFreePathLength(float curvature) {
  """notation
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
  """
  // Car points (baselink frame)
  float car_inner_y = CAR_WIDTH / 2.0 + SAFETY_MARGIN
  float car_outter_y = -car_inner_y;
  float car_front_x = (CAR_BASE + CAR_LENGTH) / 2.0 + SAFETY_MARGIN;
  float car_rear_x = -(CAR_LENGTH - CAR_BASE) / 2.0 - SAFETY_MARGIN;
  
  const float r = 1.0 / curvature;

  const Vector2f center_pt = Vector2f(0,r);  // turning instant center
  const Vector2f car_inner_front_pt = Vector2f(car_front_x,car_inner_y);
  const Vector2f car_outter_front_pt = Vector2f(car_front_x,car_outter_y);
  const Vector2f car_inner_rear_pt = Vector2f(car_rear_x,car_inner_y);
  const Vector2f car_outter_rear_pt = Vector2f(car_rear_x,car_outter_y);


  const float r_min = r - (CAR_WIDTH / 2.0 + SAFETY_MARGIN);
  const float r_max = (center_pt - car_outter_front_pt ).norm();
  const float r_1 = (center_pt - car_inner_front_pt ).norm();
  const float r_2 = (center_pt - car_outter_rear_pt ).norm();

  float free_path_length = 1000.0;
  for (auto point : point_cloud_) {
    
    // whether the car will hit the point
    bool collision = true;
    
    // r_p point to center
    float r_p = (center_pt - point).norm();

    // angle: point angle, init base_link -> point
    float theta = atan2(point.x() / (r - point.y())); // atan2(x / (r-y))
    // angle: new base_link -> point
    float omega; 

    if (theta > 0 && r_p >= r_1 && r_p <= r_max) { 
      // front side
      omega = asin2(car_front_x / r_p); // asin(h/rp)
    } else if (theta > 0 && r_p >= r_min && r_p <= r_1) {
      // inner side
      omega = acos2((r - (CAR_WIDTH / 2 + SAFETY_MARGIN)) / r_p); // acos(r-w/2 / rp)
    } else {
      // case 3, or no collision
      collision = false;
    }

    if (collision) {
      //  turning angle: init base_link -> new base_link = theta - omega
      float cur_free_path_length = (theta - omega) * r; // turning_angle * r
      free_path_length = min(free_path_length, cur_free_path_length);
    } 

  }
  return free_path_length;


}
// float Navigation::ComputeFreePathLength(float curvature) {

//   // assume all points have free path length og +inf first
//   float free_path_lengths[point_cloud_.size()] = { std::numeric_limits<float>::infinity() };

//   float car_inner_y = this->car_w / 2.0 + m;
//   float car_outter_y = -car_inner_y;
//   float car_front_x = (this->car_b + this->car_l) / 2.0 + m;
//   float car_rear_x = -(this->car_l - this->car_b) / 2.0 - m;
  
//   const float r = 1.0 / curvature

//   const Vector2f center_pt = Vector2f(0,r);
//   const Vector2f car_inner_front_pt = Vector2f(car_front_x,car_inner_y);
//   const Vector2f car_outter_front_pt = Vector2f(car_front_x,car_outter_y);
//   const Vector2f car_inner_rear_pt = Vector2f(car_rear_x,car_inner_y);
//   const Vector2f car_outter_rear_pt = Vector2f(car_rear_x,car_outter_y);


//   const float r_min = r - (this->car_w / 2.0 + m);
//   const float r_max = (center_pt - car_outter_front_pt ).norm();
//   const float r1 = (center_pt - car_inner_front_pt ).norm();
//   const float r2 = (center_pt - car_outter_rear_pt ).norm();

//   // for (auto _pt : point_cloud_)
//   for (unsigned int i=0; i< point_cloud_.size(); ++i)
//   {
//     float r_p = (center_pt - point_cloud_[i] ).norm();
    
//     // the coressponding point on car
//     Vector2f p_car;
//     bool collision = true;
//     // determine cases
//     if (
//       point_cloud_[i].x() >=0 &&
//       point_cloud_[i].y() >= car_inner_y &&
//       r_p >= r_min &&
//       r_p <=r1
//     ) {
//       // case1 : first hit inner side
//       p_car.x() = sqrt( r_p^2 - r_min^2);
//       p_car.y() = car_inner_y;
//     }else if (
//       point_cloud_[i].x() >=0 &&
//       point_cloud_[i].y() >= car_outter_y &&
//       r_p >= r_1 &&
//       r_p <= r_max
//     ) {
//       // case2 : first hit front side
//       p_car.x() = car_front_x;
//       p_car.y() = r - sqrt(r_p^2 - car_front_x^2);
//     } else if (
//       point_cloud_[i].x() >= car_rear_x &&
//       point_cloud_[i].y() <= car_outter_y &&
//       r_p >= r + car_inner_y &&
//       r_p <= r_2
//     ){
//       // case3 : first hit outter side
//       p_car.x() = -sqrt( r_p^2 - (r + car_inner_y)^2 );
//       p_car.y() = car_outter_y;
    
//     } else {
//       // won't collide
//       collision = false;
//     }

//     if (collision) {
//       float theta = asin( (p_car - point_cloud_[i]).norm() / 2.0 / r_p  );
//       _free_path_length = r * sin( 2 * theta  );
//       free_path_lengths.push_back(_free_path_length);
//     }
    
//   }

//   return *std::min_element(free_path_lengths, free_path_lengths + point_cloud_.size());
  
// }


void Navigation::RunAssign1() {

  // 1. Generate possible curvatures (kinemetic constraint)
  vector<float> curvatures = GenerateCurvatures();
  // 2. For each possible path:
      // a. Compute Free Path Length
      // b. Compute Clearance
      // c. Compute Distance To Goal
      // d. Compute total “score”
  // 3. From all paths, pick path with best score
  path = ChoosePath(curvatures);
  // path.curvature, path.free_path_length

  // 4. Implement 1-D TOC on the chosen arc.
  control = ComputeTOC(path); // calculate velocity

  drive_msg_.curvature = control.curvature;
  drive_msg_.velocity = control.velocity;

}
void Navigation::Run() {
  // This function gets called 20 times a second to form the control loop.
  
  // Clear previous visualizations.
  visualization::ClearVisualizationMsg(local_viz_msg_);
  visualization::ClearVisualizationMsg(global_viz_msg_);

  // If odometry has not been initialized, we can't do anything.
  if (!odom_initialized_) return;

  // The control iteration goes here. 
  // Feel free to make helper functions to structure the control appropriately.
  
  // The latest observed point cloud is accessible via "point_cloud_"


  // Eventually, you will have to set the control values to issue drive commands:
  // drive_msg_.curvature = ...;
  // drive_msg_.velocity = ...;

  // Add timestamps to all messages.
  local_viz_msg_.header.stamp = ros::Time::now();
  global_viz_msg_.header.stamp = ros::Time::now();
  drive_msg_.header.stamp = ros::Time::now();
  // Publish messages.
  viz_pub_.publish(local_viz_msg_);
  viz_pub_.publish(global_viz_msg_);
  drive_pub_.publish(drive_msg_);
}

}  // namespace navigation
