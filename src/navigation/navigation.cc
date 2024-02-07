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
PathOption Navigation::ChoosePath(const vector<float> &candidate_curvs) {
 
  // PathOption return_path;

  float highest_score = -1;
  PathOption best_path;
  float score_clearance = 0; // hyper-param
  float score_curv = 0;
  for (auto _curv : candidate_curvs) {
    // draw options (gray)
    // visualization::DrawPathOption(_curv,1,5,0x808080,false,local_viz_msg_);

    float free_path_len = ComputeFreePathLength(_curv);
    float clearance = ComputeClearance(free_path_len, _curv);
    // float score = free_path_len + score_w * clearance - PENALTY_CURVE * std::abs(_curv);
    visualization::DrawPathOption(_curv, free_path_len, clearance, 0xFF0000, false, local_viz_msg_);
    float score = free_path_len + score_clearance * clearance + score_curv * std::abs(_curv);
    if (score > highest_score) {
      highest_score = score;
      best_path.curvature = _curv;
      best_path.clearance = clearance;
      best_path.free_path_length = free_path_len;
    }
  }

  //std::cout<<"Score "<<highest_score<<"\n";
  // draw best option (blue)
  visualization::DrawPathOption(best_path.curvature,best_path.free_path_length,best_path.clearance,0x0F03FC,false,local_viz_msg_);
  std::cout<<"Best C="<<best_path.curvature<<" Free Path Length="<<best_path.free_path_length<<"\n";
  
  // just for an example
  // return_path.curvature = best_curv;
  // return_path.clearance = 1.4;
  // return_path.free_path_length = best_option;
  // return_path.obstruction = Vector2f(0,0)
  // return_path.closest_point = Vector2f(0,0);

  return best_path;
    
}

float Navigation::ComputeFreePathLength(float curvature) {
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
        free_path_length = std::min(free_path_length, cur_free_path_length);
      } 
    }
    
  }

  // draw free path length along the curve (cyan)
  // visualization::DrawPathOption(curvature,free_path_length,5,0x00FFFF,false,local_viz_msg_);
  return free_path_length;

}


void Navigation::RunAssign1() {
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
  PathOption chosen_path = ChoosePath(curvatures_);
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



float Navigation::LatencyCompensation() {
  // take the control from latency ago (the control that actually happens now)
  // calculate displacement on x and y axis
  float x_diff = 0.0;
  float y_diff = 0.0;
  float theta_diff = 0.0;
  float queue_size = 3;
  float latency_period = dt * queue_size;

  if (control_queue.size() < queue_size) return robot_vel_.norm();

  // start forward-predict (accumulate displacement) 
  for (auto &control : control_queue) {
    float curvature = control.curvature;
    float velocity = control.velocity;
    
    // Calculate the x, y displacement
    float delta_x = velocity * std::cos(curvature) * latency_period;
    float delta_y = velocity * std::sin(curvature) * latency_period;

    // Accumulate the displacements
    x_diff += delta_x;
    y_diff += delta_y;

    // Calculate the rotation 
    theta_diff +=  velocity * curvature * latency_period; // theta = v / r * time

  }
  cout << "forward-predict: x_diff, y_diff, theta_diff" << x_diff << ", " << y_diff << ", " << theta_diff << endl; 

  // rotation and translation matrix
  Eigen::Matrix3f transformation_matrix;
  transformation_matrix <<
        std::cos(theta_diff), -std::sin(theta_diff), x_diff,
        std::sin(theta_diff), std::cos(theta_diff), y_diff,
        0, 0, 1;

  // draw an example
  Eigen::Vector3f example_pt1(0,0,1);
  Eigen::Vector3f example_pt2(1,0,1);
  example_pt1 = transformation_matrix * example_pt1;
  example_pt2 = transformation_matrix * example_pt2;

  visualization::DrawLine(Vector2f(example_pt1.x(), example_pt1.y()), Vector2f(example_pt2.x(), example_pt2.y()), 0xEB4EED ,local_viz_msg_);

  // Transform the lidar points using translation and rotation
  for (auto& point : point_cloud_) {
      // homogeneous coordinates
      Eigen::Vector3f p(point.x(), point.y(), 1);
      p = transformation_matrix.inverse() * p;
      point.x() = p.x();
      point.y() = p.y();
  }
  float cur_velocity = control_queue[0].velocity;
  // pop out the oldest control
  control_queue.erase(control_queue.begin());
  return cur_velocity;

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

  // visualization
  // car + margin
  drawCar(true);
  drawPointCloud();
  // Eventually, you will have to set the control values to issue drive commands:
  // drive_msg_.curvature = ...;
  // drive_msg_.velocity = ...;
  RunAssign1();

  // Run Sine Wave Velocity with peroid = 10 sec
  // RunSineWave(3);

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

