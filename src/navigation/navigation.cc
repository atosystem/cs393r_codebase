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
CONFIG_FLOAT(dist_nav_complete, "dist_nav_complete"); // max distance to goal to be considered complete
CONFIG_FLOAT(dist_replan, "dist_replan"); // min distance to intermediate goal to replan
CONFIG_FLOAT(score_goal, "score_goal"); // weight for distance to goal
CONFIG_INT(num_iters_replan, "num_iters_replan");

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
  
  // std::cout<<drive_msg_.velocity<<","<<robot_vel_.norm()<<"\n";
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
    if (std::fabs(control.curvature) < 1E-3f) {
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
  // cout << "forward-predict: (x', y', theta'): (" << x << ", " << y << ", " << theta << ')' << endl;

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

void Navigation::GenerateCurvatures(int num_samples = 100) {
  if (num_samples % 2 == 0) {
    num_samples += 1;
  }
  curvatures_.resize(num_samples);
  // std::cout<<"Total samples="<<num_samples<<"\n";
  // half of samples
  num_samples = (num_samples - 1) / 2;
  // std::cout<<"half samples="<<num_samples<<"\n";
  float _delta = CAR_CMAX * 1.0 / num_samples;
  // std::cout<<"delta="<<_delta<<"\n";
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

float Navigation::ComputeClearance(float free_path_len, float curv) {
  float min_clearance = 10.0;
  float r = 1.0 / std::fabs(curv);
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
      if (std::fabs(point.y()) > c_max) continue;
      // filter out collision points
      if (point.y() <= car_inner_y && point.y() >= car_outter_y && point.x() >= car_front_x) {
        continue;
      }
      float cur_clearance = std::fabs(point.y());
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
      if (std::fabs(r_p - r) > c_max || p_angle < 0 || p_angle > turning_angle) { // TODO: condition on turning_angle is an assumption
        continue;
      }
      cur_clearance = std::fabs(r_p - r); // |c-p| - rmax
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

float Navigation::ComputeFreePathLength(float curvature, const Eigen::Vector2f &goal, Eigen::Vector2f &endpoint) {
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
  
  


  const float range_max = 10.f;
  float free_path_length = range_max;

  if (curvature==0) {
    // go straight
    bool all_no_collision = true;
    for (auto point : point_cloud_) {
      if (point.y() <= car_inner_y && point.y() >= car_outter_y && point.x() >= car_front_x) {
        // collision
        const float cur_free_path_length = point.x() - car_front_x;
        if (cur_free_path_length < free_path_length) {
          free_path_length = cur_free_path_length;
          endpoint = Vector2f(point.x(), 0);
        }
        all_no_collision = false;
      }

      if (all_no_collision) {
        endpoint = Vector2f(range_max, 0);
        free_path_length = range_max;
      }

      // check if the nearest endpoint is in the halfway.
      // nearest endpoint: from it to goal is perpendicular to +x
      if (goal.x() < endpoint.x()) {
        free_path_length = std::fabs(goal.x());
        endpoint = Vector2f(goal.x(), 0);
      }
    }
  } else {
    // go curve
    const float r = 1.0 / std::fabs(curvature);
    const Vector2f center_pt = Vector2f(0,r);  // turning instant center
    const Vector2f car_inner_front_pt = Vector2f(car_front_x,car_inner_y);
    const Vector2f car_outter_front_pt = Vector2f(car_front_x,car_outter_y);
    //const Vector2f car_inner_rear_pt = Vector2f(car_rear_x,car_inner_y);
    //const Vector2f car_outter_rear_pt = Vector2f(car_rear_x,car_outter_y);



    const float r_min = r - (CAR_WIDTH / 2.0 + SAFETY_MARGIN);
    const float r_max = (center_pt - car_outter_front_pt ).norm();
    const float r_1 = (center_pt - car_inner_front_pt ).norm();
    // const float r_2 = (center_pt - car_outter_rear_pt ).norm();


    bool all_no_collision = true;
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
        const float turning_angle = theta - omega;
        float cur_free_path_length = turning_angle * r; // turning_angle * r

        //std::cout<<"cur_free_path_length="<<cur_free_path_length<<" free_path_length="<<free_path_length<<"\n";
        if (cur_free_path_length < free_path_length) {
          free_path_length = cur_free_path_length;

          endpoint.x() = r * std::sin(turning_angle);
          endpoint.y() = std::fabs(r - r * std::cos(turning_angle)) * (curvature < 0 ? -1.f : 1.f);
        }

        all_no_collision = false;
      }
    }
    
    // check if the nearest endpoint is in the halfway.
    // nearest endpoint: the intersection of the circle
    // and the line from the center to the goal
    Vector2f halfway_endpoint;
    float halfway_free_path_length;

    Vector2f goal_in_circle_coordinate;
    if (curvature > 0) {
      goal_in_circle_coordinate.x() = -goal.y() + r;
      goal_in_circle_coordinate.y() = goal.x();
    } else {
      goal_in_circle_coordinate.x() = goal.y() + r;
      goal_in_circle_coordinate.y() = goal.x();
    }

    Vector2f endpoint_in_circle_coordinate = goal_in_circle_coordinate.normalized() * r;
    halfway_endpoint.x() = endpoint_in_circle_coordinate.y();
    halfway_endpoint.y() = std::fabs(-endpoint_in_circle_coordinate.x() + r) * (curvature < 0 ? -1.f : 1.f);

    float turning_angle = std::atan2(goal_in_circle_coordinate.y(), goal_in_circle_coordinate.x());
    if (turning_angle < 0) {
      turning_angle += 2 * M_PI;
    }
    halfway_free_path_length = turning_angle * r;

    if (all_no_collision || halfway_free_path_length < free_path_length) {
      free_path_length = halfway_free_path_length;
      endpoint = halfway_endpoint;
    }
  }

  // draw free path length along the curve (cyan)
  // visualization::DrawPathOption(curvature,free_path_length,5,0x00FFFF,false,local_viz_msg_);
  return free_path_length;

}

PathOption Navigation::ChoosePath(const vector<float> &candidate_curvs, const Eigen::Vector2f &goal) {
  Vector2f diff = goal - robot_loc_;
  Vector2f goal_base_link;
  goal_base_link.x() = diff.x() * std::cos(robot_angle_) + diff.y() * std::sin(robot_angle_);
  goal_base_link.y() = -diff.x() * std::sin(robot_angle_) + diff.y() * std::cos(robot_angle_);
  cout << "goal_base_link: (" << goal_base_link.x() << ", " << goal_base_link.y() << ")" << endl;

  visualization::DrawCross(
    goal_base_link,
    CONFIG_grid_dx / 2.0,
    0xff7f50,
    local_viz_msg_
  );

  float highest_score = -std::numeric_limits<float>::infinity();
  PathOption best_path;
  float score_clearance = 0; // hyper-param
  float score_curv = 0;
  float score_goal = CONFIG_score_goal;
  Vector2f selected_endpoint;
  for (auto _curv : candidate_curvs) {
    Vector2f endpoint;
    float free_path_len = ComputeFreePathLength(_curv, goal_base_link, endpoint);
    float clearance = ComputeClearance(free_path_len, _curv);
    const float distance_to_goal = (goal_base_link - endpoint).norm();

    // draw options (gray)
    visualization::DrawCross(
      endpoint,
      CONFIG_grid_dx / 2.0,
      0x808080,
      local_viz_msg_
    );
    visualization::DrawPathOption(
      _curv,
      free_path_len,
      clearance,
      0x808080,
      false,
      local_viz_msg_
    );

    float score = free_path_len + 
                  score_clearance * clearance +
                  score_curv * std::fabs(_curv) +
                  score_goal * distance_to_goal;

    cout << "=== curvature: " << _curv << " ===\n";
    cout << "score: " << score << "\n";
    cout << "free_path_len: " << free_path_len << "\n";
    cout << "clearance: " << clearance << "\n";
    cout << "curvature: " << std::fabs(_curv) << "\n";
    cout << "endpoint: (" << endpoint.x() << ", " << endpoint.y() << ")\n";
    cout << "dist(endpoint, goal): " << distance_to_goal << " * " << score_goal << " = " << score_goal * distance_to_goal << "\n" << endl;

    if (score > highest_score) {
      highest_score = score;
      best_path.curvature = _curv;
      best_path.clearance = clearance;
      best_path.free_path_length = free_path_len;
      selected_endpoint = endpoint;
    }
  }

  //std::cout<<"Score "<<highest_score<<"\n";
  // draw best option (blue)
  visualization::DrawCross(
    selected_endpoint,
    CONFIG_grid_dx / 2.0,
    0x0F03FC,
    local_viz_msg_
  );
  visualization::DrawPathOption(
    best_path.curvature,
    best_path.free_path_length,
    best_path.clearance,
    0x0F03FC,
    false,
    local_viz_msg_
  );
  std::cout<<"Best C="<<best_path.curvature<<" Free Path Length="<<best_path.free_path_length<<"\n";
  
  // just for an example
  // return_path.curvature = best_curv;
  // return_path.clearance = 1.4;
  // return_path.free_path_length = best_option;
  // return_path.obstruction = Vector2f(0,0)
  // return_path.closest_point = Vector2f(0,0);

  return best_path;
    
}

void Navigation::LocalPlanner() {
  static vector<Eigen::Vector2f> path;
  static int index = 0;
  static int replan_count = 0;

  // check if we need to replan
  if (path.size() == 0 ||
      replan_count % CONFIG_num_iters_replan == 0 
      // intermediate_goal.isApprox(Vector2f(-1, -1)) ||
      // (robot_loc_ - intermediate_goal).norm() > CONFIG_dist_replan
      ) {
    path.clear();
    GlobalPlanner(path);
    index = 0;
    replan_count = 0;
    cout << "Now replanning" << endl;
  }
  replan_count++;

  // draw remaining path (red)
  for (size_t i = 0; i < path.size(); ++i) {
    visualization::DrawCross(
          path[i],
          CONFIG_grid_dx / 2.0,
          0xe20502,
          global_viz_msg_
        );
  }

  // run through the path and find the farthest valid point
  for (size_t i = index; i < path.size(); ++i) {
    Eigen::Vector2f pt = path[i];
    geometry::line2f line(robot_loc_, pt);
    bool collision = false;
    for (const auto& map_line : map_.lines) {
      if (map_line.Intersects(line)) {
        collision = true;
        break;
      }
    }
    if (collision) {
      break;
    }
    index = i;
  }
  intermediate_goal = path[index];
}

void Navigation::ObstacleAvoidance() {
  LocalPlanner();

  // intermediate goal (black)
  visualization::DrawCross(
    intermediate_goal,
    CONFIG_grid_dx / 2.0,
    0x000000,
    global_viz_msg_
  );
  visualization::DrawArc(
    intermediate_goal,
    CONFIG_dist_replan,
    0,
    360,
    0x00000,
    global_viz_msg_
  );

  float cur_velocity = robot_vel_.norm(); // LatencyCompensation();

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

bool Navigation::CheckNavComplete() {
  if (nav_complete_) {
    intermediate_goal = Vector2f(-1, -1);
    return true;
  }
  nav_complete_ = (robot_loc_ - nav_goal_loc_).norm() < CONFIG_dist_nav_complete;
  return nav_complete_;
}

void Navigation::Run() {
  // This function gets called 20 times a second to form the control loop.
  
  // Clear previous visualizations.
  visualization::ClearVisualizationMsg(local_viz_msg_);
  visualization::ClearVisualizationMsg(global_viz_msg_);

  // If odometry has not been initialized, we can't do anything.
  if (!odom_initialized_) return;

  if (CheckNavComplete()) return;

  // visualization
  // car + margin
  drawCar(true);
  drawPointCloud();
  // Eventually, you will have to set the control values to issue drive commands:
  // drive_msg_.curvature = 0.0;
  // drive_msg_.velocity = 1.0;
  RunAssign1();

  // draw nav goal (violet)
  visualization::DrawCross(
    nav_goal_loc_,
    CONFIG_grid_dx / 2.0,
    0x9400d3,
    global_viz_msg_
  );
  visualization::DrawArc(
    nav_goal_loc_,
    CONFIG_dist_nav_complete,
    0,
    360,
    0x9400d3,
    global_viz_msg_
  );

  ObstacleAvoidance();

  // Add timestamps to all messages.
  local_viz_msg_.header.stamp = ros::Time::now();
  global_viz_msg_.header.stamp = ros::Time::now();
  drive_msg_.header.stamp = ros::Time::now();
  // Publish messages.
  // viz_pub_.publish(local_viz_msg_);
  // viz_pub_.publish(global_viz_msg_);
  drive_pub_.publish(drive_msg_);
}

void Navigation::GlobalPlanner(vector<Eigen::Vector2f> &path) {
  static MapGraph mapGraph(map_);
  // mapGraph.drawObstacleGrid();
  // nav goal: nav_goal_loc_, nav_goal_angle_
  // robot_loc_: robot_loc_, robot_angle_
  
  // cout << "Start planning: " << endl;
  // convert coodinates: start and goal to GridLocation
  GridLocation start = mapGraph.point2GridLoc(robot_loc_);
  GridLocation goal = mapGraph.point2GridLoc(nav_goal_loc_);
  // cout << "robot_loc_: " << robot_loc_ << endl;
  // cout << "nav_goal_loc_: " << nav_goal_loc_ << endl;
  // cout << "start: " << start.x << " " << start.y << endl;
  // cout << "goal: " << goal.x << " " << goal.y << endl;
  vector<GridLocation> grid_path;
  mapGraph.aStarSearch(start, goal, grid_path);
  // convert path to real coodinates
  path.clear();
  for (GridLocation pt : grid_path) {
    path.push_back(mapGraph.gridLoc2point(pt));
  }
  // cout << "path.size(): " << path.size() << endl;
  
  // Draw the path
  for (auto pt : path) {
    visualization::DrawCross(
          pt,
          CONFIG_grid_dx / 2.0,
          0xe20502,
          global_viz_msg_
        );
  }

}

MapGraph::MapGraph(const vector_map::VectorMap& map_) {
  // initilize graph
  // cout<<"Initialize Grid:"<<endl;
  // cout<<"CONFIG_grid_dx: "<<CONFIG_grid_dx<<endl;
  // cout<<"CONFIG_grid_dy: "<<CONFIG_grid_dy<<endl;

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

  // cout<<"map_max_x="<<this->map_max_x<<endl;
  // cout<<"map_min_x="<<this->map_min_x<<endl;
  // cout<<"map_max_y="<<this->map_max_y<<endl;
  // cout<<"map_min_y="<<this->map_min_y<<endl;

  // initialize obstacle_grid
  obstacle_grid.clear();
  // obstacle_grid [x][y]
  // only define in first quadrant (x>0, y>0)

  grid_num_x = std::round( (map_max_x - map_min_x) / CONFIG_grid_dx );
  grid_num_y = std::round( (map_max_y - map_min_y) / CONFIG_grid_dy );

  // cout<<"grid width(x)="<<grid_num_x<<endl;
  // cout<<"grid height(y)="<<grid_num_y<<endl;

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

void MapGraph::backtrackPath(const GridLocation& start, 
                        const GridLocation& goal, 
                        std::map<GridLocation, GridLocation>& came_from,
                        vector<GridLocation>& path) 
{

  
  GridLocation current(goal.x, goal.y);
  if (came_from.find(goal) == came_from.end()) {
    return; // no path can be found
  }
  while (current != start) {
    path.push_back(current);
    GridLocation temp;
    temp.x = came_from[current].x;
    temp.y = came_from[current].y;
    current.x = temp.x;
    current.y = temp.y;
  }
  path.push_back(start); // optional
  std::reverse(path.begin(), path.end());

}

double MapGraph::getEdgeCost(const GridLocation& p1, const GridLocation& p2) {
  // Calculate the difference between x and y coordinates
  float dx = (p2.x - p1.x) * 1.0;
  float dy = (p2.y - p1.y) * 1.0;

  // Calculate the L2 norm (Euclidean distance)
  double distance = std::sqrt(dx * dx + dy * dy);

  return distance;
}

double MapGraph::getHeuristic(const GridLocation& p1, const GridLocation& p2) {
  // Calculate the difference between x and y coordinates
  float dx = (p2.x - p1.x) * 1.0;
  float dy = (p2.y - p1.y) * 1.0;

  // Calculate the L2 norm (Euclidean distance)
  double distance = std::sqrt(dx * dx + dy * dy);

  return distance;
}

vector<GridLocation> MapGraph::getNeighbors(const GridLocation current) {
  vector<vector<int>> DIRS = {{0, 1}, {0, -1}, {1, 1}, {1, 0}, {1, -1}, {-1, 1}, {-1, 0}, {-1, -1}};
  vector<GridLocation> neighbors;

  int grid_h = obstacle_grid.size();
  int grid_w = obstacle_grid[0].size();

  for (auto dir : DIRS) {
    GridLocation neighbor(current.x + dir[0], current.y + dir[1]);
    if (neighbor.x >= 0 && neighbor.x < grid_h && neighbor.y >= 0 
        && neighbor.y < grid_w && !obstacle_grid[neighbor.x][neighbor.y]) {
          neighbors.push_back(neighbor);
    }
  }

  return neighbors;

}

void MapGraph::aStarSearch(const GridLocation& start, 
                          const GridLocation& goal, 
                          vector<GridLocation>& path) 
{
    std::map<GridLocation, GridLocation> came_from;
    std::map<GridLocation, double> cost_so_far;

    // PriorityQueue<GridLocation, double> frontier;
    std::priority_queue<std::pair<double, GridLocation>, vector<std::pair<double, GridLocation>>, std::greater<std::pair<double, GridLocation>>> frontier;
    frontier.push({0, start});
    came_from[start] = start;
    cost_so_far[start] = 0;
  
    while (!frontier.empty()) {
      GridLocation current = frontier.top().second;
      frontier.pop();
      // cout << "current: " << current.x << " " << current.y << endl;
      // cout << "start: " << start.x << " " << start.y << endl;
      // cout << "goal: " << goal.x << " " << goal.y << endl;
      if (current == goal) break;

      for (GridLocation next : getNeighbors(current)) {
        double new_cost = cost_so_far[current] + getEdgeCost(current, next);
        if (cost_so_far.find(next) == cost_so_far.end()
            || new_cost < cost_so_far[next]) {
          // cout << "next: " << next.x << " " << next.y << endl;
          cost_so_far[next] = new_cost;
          double priority = new_cost + getHeuristic(next, goal);
          frontier.push({priority, next});
          // frontier.put(next, priority);
          came_from[next] = current;
        }
      }
    }

    backtrackPath(start, goal, came_from, path);
}

}  // namespace navigation

