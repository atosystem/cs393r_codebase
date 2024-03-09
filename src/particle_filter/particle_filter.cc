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
\file    particle-filter.cc
\brief   Particle Filter Starter Code
\author  Joydeep Biswas, (C) 2019
*/
//========================================================================

#include <algorithm>
#include <cmath>
#include <iostream>
#include <mutex>
#include <thread>
#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Geometry"
#include "gflags/gflags.h"
#include "glog/logging.h"
#include "shared/math/geometry.h"
#include "shared/math/line2d.h"
#include "shared/math/math_util.h"
#include "shared/util/timer.h"

#include "config_reader/config_reader.h"
#include "particle_filter.h"

#include "vector_map/vector_map.h"

using geometry::line2f;
using std::cout;
using std::endl;
using std::string;
using std::swap;
using std::vector;
using Eigen::Vector2f;
using Eigen::Vector2i;
using vector_map::VectorMap;
using math_util::AngleDiff;

DEFINE_double(num_particles, 100, "Number of particles");

CONFIG_FLOAT(x_std, "x_std");
CONFIG_FLOAT(y_std, "y_std");
CONFIG_FLOAT(r_std, "r_std");
CONFIG_FLOAT(k1, "k1");
CONFIG_FLOAT(k2, "k2");
CONFIG_FLOAT(k3, "k3");
CONFIG_FLOAT(k4, "k4");
CONFIG_FLOAT(sigma_s, "sigma_s");
CONFIG_FLOAT(gamma_pow, "gamma_pow");
CONFIG_FLOAT(d_short_d_long, "d_short_d_long");

namespace particle_filter {

config_reader::ConfigReader config_reader_({"config/particle_filter.lua"});

ParticleFilter::ParticleFilter() :
    prev_odom_loc_(0, 0),
    prev_odom_angle_(0),
    odom_initialized_(false),
    loss_count_(0),
    loss_sum_(0.f) {}

void ParticleFilter::GetParticles(vector<Particle>* particles) const {
  *particles = particles_;
}

void ParticleFilter::GetPredictedPointCloud(const Vector2f& loc,
                                            const float angle,
                                            int num_ranges,
                                            float range_min,
                                            float range_max,
                                            float angle_min,
                                            float angle_max,
                                            vector<Vector2f>* scan_ptr) {
  vector<Vector2f>& scan = *scan_ptr;
  // Compute what the predicted point cloud would be, if the car was at the pose
  // loc, angle, with the sensor characteristics defined by the provided
  // parameters.
  // This is NOT the motion model predict step: it is the prediction of the
  // expected observations, to be used for the update step.

  // Note: The returned values must be set using the `scan` variable:
  scan.resize(num_ranges);
  /*
  // Fill in the entries of scan using array writes, e.g. scan[i] = ...
  for (size_t i = 0; i < scan.size(); ++i) {
    scan[i] = Vector2f(0, 0);
  }

  // The line segments in the map are stored in the `map_.lines` variable. You
  // can iterate through them as:
  for (size_t i = 0; i < map_.lines.size(); ++i) {
    const line2f map_line = map_.lines[i];
    // The line2f class has helper functions that will be useful.
    // You can create a new line segment instance as follows, for :
    line2f my_line(1, 2, 3, 4); // Line segment from (1,2) to (3.4).
    // Access the end points using `.p0` and `.p1` members:
    // printf("P0: %f, %f P1: %f,%f\n", 
    //        my_line.p0.x(),
    //        my_line.p0.y(),
    //        my_line.p1.x(),
    //        my_line.p1.y());

    // Check for intersections:
    bool intersects = map_line.Intersects(my_line);
    // You can also simultaneously check for intersection, and return the point
    // of intersection:
    Vector2f intersection_point; // Return variable
    intersects = map_line.Intersection(my_line, &intersection_point);
    if (intersects) {
      // printf("Intersects at %f,%f\n", 
      //        intersection_point.x(),
      //        intersection_point.y());
    } else {
      // printf("No intersection\n");
    }
  }*/

  
  // ian ==== (successfully compile)
  // I only consider cases when the obstacle is within r_max and r_min or greater than r_max.
  // 
  // Location of the laser on the robot. Assumes the laser is forward-facing.
  const Vector2f kLaserLoc(0.2, 0);
  Eigen::Rotation2Df r1(angle);

  // lidar center point (map frame)
  Vector2f laser_loc = loc + r1 * kLaserLoc;
  float angle_delta = (angle_max - angle_min) / num_ranges;
  for (size_t i = 0; i < scan.size(); ++i) {
    float _beamAngle = i * angle_delta + angle_min;
    
    line2f my_line(
      range_min * cos(_beamAngle + angle ) + laser_loc.x(), range_min * sin(_beamAngle + angle ) + laser_loc.y(),
      range_max * cos(_beamAngle + angle ) + laser_loc.x(), range_max * sin(_beamAngle + angle ) + laser_loc.y());

    float shortest_range = range_max;
    for (size_t i = 0; i < map_.lines.size(); ++i) {
      const line2f map_line = map_.lines[i];
      // Check for intersections:
      // bool intersects = map_line.Intersects(my_line);
      // You can also simultaneously check for intersection, and return the point
      // of intersection:
      Vector2f intersection_point; // Return variable
      bool intersects = map_line.Intersection(my_line, &intersection_point);
      if (intersects) {
        float r = (intersection_point - laser_loc).norm();
        if (r < shortest_range)
        {
          // we got a shorter distance
          shortest_range = r;
        }
      } else {
        // scan[i] = Vector2f(range_max * cos(_beamAngle + angle ) + laser_loc.x(), range_max * sin(_beamAngle + angle ) + laser_loc.y());
      }
    }
    // predicted lidar scan at _beamAngle
    // 
    
    // uncomment the following line to debug
    // shortest_range = range_max;
    scan[i] = Vector2f(
      shortest_range * cos(_beamAngle + angle ) + laser_loc.x(), 
      shortest_range * sin(_beamAngle + angle ) + laser_loc.y()
    );
  }
}

void ParticleFilter::Update(const vector<float>& ranges,
                            float range_min,
                            float range_max,
                            float angle_min,
                            float angle_max,
                            Particle* p_ptr) {
  // Implement the update step of the particle filter here.
  // You will have to use the `GetPredictedPointCloud` to predict the expected
  // observations for each particle, and assign weights to the particles based
  // on the observation likelihood computed by relating the observation to the
  // predicted point cloud.

  // ian =========
  // Require tuning
  const float sigma_s = CONFIG_sigma_s;
  const float gamma = pow(ranges.size(), CONFIG_gamma_pow); // 1 (pow = 0): uncorrelated, 1/n (pow = -1): perfectly correlated
  vector<Vector2f> scan;
  this->GetPredictedPointCloud( p_ptr->loc,
                                p_ptr->angle ,
                                ranges.size(),
                                range_min,
                                range_max,
                                angle_min,
                                angle_max,
                                &scan);
  float log_prob = 0;
  const Vector2f kLaserLoc(0.2, 0);
  float d_short = CONFIG_d_short_d_long;
  float d_long = CONFIG_d_short_d_long;

  Eigen::Rotation2Df r1(p_ptr->angle);
  Vector2f laser_loc = p_ptr->loc + r1 * kLaserLoc;
  for(size_t i=0; i< scan.size();++i)
  {
  // lidar center point (map frame)
    float r = (scan[i] - laser_loc).norm();
    // cout << " ranges[i] -  r " << ranges[i] -  r << endl;
    float d = r - ranges[i];
    
    d = std::min(d,d_long);
    d = std::max(d, -d_short);
  
    log_prob += -0.5 * pow( d ,2) / pow(sigma_s,2);
  }
  log_prob = log_prob * gamma;
  p_ptr->weight += log_prob;
  // p_ptr->weight = exp(log_prob);
  // ian =========
}

void ParticleFilter::RecordLoss(const vector<float>& ranges,
                                float range_min,
                                float range_max,
                                float angle_min,
                                float angle_max) {

  if (!odom_initialized_) {
    return;
  }

  // Copy & paste from Update()
  Vector2f loc;
  float angle;
  GetLocation(&loc, &angle);
  vector<Vector2f> scan;
  this->GetPredictedPointCloud(loc,
                               angle,
                               ranges.size(),
                               range_min,
                               range_max,
                               angle_min,
                               angle_max,
                               &scan);
  const Vector2f kLaserLoc(0.2, 0);
  const Eigen::Rotation2Df r1(angle);
  const Vector2f laser_loc = loc + r1 * kLaserLoc;

  // distance between predicted point cloud and observations
  for (size_t i = 0; i < scan.size(); ++i) {
    // lidar center point (map frame)
    const float r = (scan[i] - laser_loc).norm();
    const float d = pow(r - ranges[i], 2);
    if (d > 10.f) {
      continue;
    }
    loss_sum_ += d;
    loss_count_++;
  }
}

void ParticleFilter::Report() {
  if (loss_count_ == 0) {
    cout << "\n\nAverage Loss: 0\nLoss Count: 0\n\n";
    return;
  }
  cout << "\n\nAverage Loss: " << loss_sum_ / loss_count_
       << "\nLoss Count: " << loss_count_ << "\n\n";
  loss_sum_ = 0.f;
  loss_count_ = 0;
}

void ParticleFilter::PrintConfigurations() {
  cout << "\n\n===== Configurations ====="
       << "\nx_std: " << CONFIG_x_std
       << "\ny_std: " << CONFIG_y_std
       << "\nr_std: " << CONFIG_r_std
       << "\nk1: " << CONFIG_k1
       << "\nk2: " << CONFIG_k2
       << "\nk3: " << CONFIG_k3
       << "\nk4: " << CONFIG_k4
       << "\nsigma_s: " << CONFIG_sigma_s
       << "\ngamma_pow: " << CONFIG_gamma_pow
       << "\nd_short_d_long: " << CONFIG_d_short_d_long
       << "\n==========================\n\n";
}

void ParticleFilter::Resample() {
  // Resample the particles, proportional to their weights.
  // The current particles are in the `particles_` variable. 
  // Create a variable to store the new particles, and when done, replace the
  // old set of particles:
  // vector<Particle> new_particles';
  // During resampling: 
  //    new_particles.push_back(...)
  // After resampling:
  // particles_ = new_particles;

  // You will need to use the uniform random number generator provided. For
  // example, to generate a random number between 0 and 1:
  // float x = rng_.UniformRandom(0, 1);
  // printf("Random number drawn from uniform distribution between 0 and 1: %f\n",
  //        x);

  // ian ===== (successfully compile)
  // cout<< "Particles cnt: (before,after) " << particles_.size();
  vector<Particle> new_particles;
  new_particles.reserve(particles_.size());

  vector<float> cmf; // cumulative mass function
  cmf.resize(particles_.size()+1);
  cmf[0] = 0;
  for(size_t i=0; i<particles_.size();++i)
  {
    cmf[i+1] = exp(particles_[i].weight) + cmf[i];
    particles_[i].weight = log(1/FLAGS_num_particles); //1.0f / particles_.size();
  }
  // add 0.1 to the last boundary, won't affect the sampling
  cmf[particles_.size()] += 0.1f;

  /*
    Low-variance resampling (L8, P47)
    1. Pick a random number between 0 and 1
    2. Sample at N equidistant locations after it, wrapping around if needed
  */
  // During resampling: 
  const float shift = 1. / particles_.size();
  float r = rng_.UniformRandom() - shift;
  for(size_t i=0; i<particles_.size();++i)
  {
    r += shift;
    if (r > 1) {
      r -= 1;
    }
    auto upper = std::upper_bound(cmf.begin(),cmf.end(),r);
    int idx = std::distance(cmf.begin(), upper) - 1;
    new_particles.push_back(particles_[idx]);
  }
  // After resampling:
  particles_ = new_particles;
  // cout<<"  "<< particles_.size() << endl;
  // ian =====
}

void ParticleFilter::ObserveLaser(const vector<float>& ranges,
                                  float range_min,
                                  float range_max,
                                  float angle_min,
                                  float angle_max) {
  // A new laser scan observation is available (in the laser frame)
  // Call the Update and Resample steps as necessary.

  // Record loss for expected location (tuning purposes only)
  RecordLoss(ranges,
             range_min,
             range_max,
             angle_min,
             angle_max);

  // parallelize the update step
  const int numThreads = std::thread::hardware_concurrency();
  vector<std::thread> workers;
  workers.reserve(numThreads);
  for (int i = 0; i < numThreads; ++i) {
    workers.emplace_back([&](){
      for(size_t j = i; j < particles_.size(); j += numThreads) {
        this->Update( ranges,
                      range_min,
                      range_max,
                      angle_min,
                      angle_max,
                      &particles_[j]);
      }
    });
  }
  for (auto& worker : workers) {
    worker.join();
  }

  this->NormalizeParticlesWeights();

  const int resample_period = 3;
  static int resample_cnt = 0;
  
  resample_cnt ++;
  if (resample_cnt == resample_period) {
    this->Resample();
    resample_cnt = 0;
  }
}

void ParticleFilter::Predict(const Vector2f& odom_loc,
                             const float odom_angle) {
  // Implement the predict step of the particle filter here.
  // A new odometry value is available (in the odom frame)
  // Implement the motion model predict step here, to propagate the particles
  // forward based on odometry.


  // You will need to use the Gaussian random number generator provided. For
  // example, to generate a random number from a Gaussian with mean 0, and
  // standard deviation 2:
  // float x = rng_.Gaussian(0.0, 2.0);
  // printf("Random number drawn from Gaussian distribution with 0 mean and "
  //        "standard deviation of 2 : %f\n", x);
  
  // initialize the odom
  if (!odom_initialized_) {
    prev_odom_loc_ = odom_loc;
    prev_odom_angle_ = odom_angle;
    odom_initialized_ = true;
    return;
  }

  // cout << "predict motion using new odom: " << "(" << odom_loc.x() << ", " << odom_loc.y() << ")" << endl;

  float k1 = CONFIG_k1; // trans error from trans model
  float k2 = CONFIG_k2; // trans error from rotat model
  float k3 = CONFIG_k3; // rotat error from trans model
  float k4 = CONFIG_k4; // rotat error from rotat model

  // In odom frame, compute delta x, y and angle 
  // This is the "car" movement
  Vector2f dxy_odom = odom_loc - prev_odom_loc_;
  float dangle_odom = AngleDiff(odom_angle, prev_odom_angle_); // AngleDiff bound the angle into [-pi, pi]
  // cout << "odom_angle, prev_odom_angle_, dangle_odom, std::abs(dangle_odom) : " << odom_angle << " " <<  prev_odom_angle_ << " " << dangle_odom  << " " << std::std::abs(dangle_odom) << endl;

  // For each particles, transform odom movements to map frame, sample noise
  bool weight_change = false;
  for (auto &_particle : particles_) {

    // Find transformation to transform points from odom frame to map frame
    // rot =  angle in map - angle in odom =  _particle.angle - prev_odom_angle_ 
    Eigen::Rotation2Df R_odom2map(AngleDiff(_particle.angle, prev_odom_angle_));
    
    // Transform dxy, dangle from odom frame to map frame
    Vector2f dxy_map = R_odom2map * dxy_odom; 
    float dangle_map = dangle_odom; 
    
    // sample noise
    float dx_map = rng_.Gaussian(dxy_map.x(), k1*dxy_map.norm() + k2*std::abs(dangle_map));
    float dy_map = rng_.Gaussian(dxy_map.y(), k1*dxy_map.norm() + k2*std::abs(dangle_map));
    dangle_map = rng_.Gaussian(dangle_map, k3*dxy_map.norm() + k4*std::abs(dangle_map));

    const float prev_x = _particle.loc.x();
    const float prev_y = _particle.loc.y();

    // update particles
    _particle.loc.x() = _particle.loc.x() + dx_map;
    _particle.loc.y() = _particle.loc.y() + dy_map;
    _particle.angle = _particle.angle + dangle_map;

    // zero out prob (= assign -inf to log prob) to ``filter out'' the particles that pass through the wall
    const line2f my_line(prev_x, prev_y, _particle.loc.x(), _particle.loc.y());
    for (const auto &map_line : map_.lines) {
      if (map_line.Intersects(my_line)) {
        weight_change = true;
        _particle.weight = -std::numeric_limits<double>::infinity();
        break;
      }
    }
  }
  if (weight_change) {
    this->NormalizeParticlesWeights();
  }

  // update prev odometry to current odom
  prev_odom_loc_ = odom_loc;
  prev_odom_angle_ = odom_angle;
  
  
}

void ParticleFilter::Initialize(const string& map_file,
                                const Vector2f& loc,
                                const float angle) {
  // The "set_pose" button on the GUI was clicked, or an initialization message
  // was received from the log. Initialize the particles accordingly, e.g. with
  // some distribution around the provided location and angle.
  map_.Load(map_file);
  
  // TODO: questions: what frame does loc, angle in???? => map
  // reset the odometry
  odom_initialized_ = false;
  // clear the particles
  particles_.clear();

  // initialize particles 
  float x_std = CONFIG_x_std;
  float y_std = CONFIG_y_std;
  float r_std = CONFIG_r_std;
  for (int i = 0; i < FLAGS_num_particles; i++) {
    Particle p;
    p.loc.x() = rng_.Gaussian(loc.x(), x_std);
    p.loc.y() =  rng_.Gaussian(loc.y(), y_std);
    p.angle =  rng_.Gaussian(angle, r_std);
    p.weight = log(1/FLAGS_num_particles); // TODO: not sure????
    particles_.push_back(p);
  }
  printf("Initialize particles/odom finished.");

}

void ParticleFilter::NormalizeParticlesWeights() {
  
  // normalize max
  double max_prob = -std::numeric_limits<double>::infinity();

  for (int i = 0; i < FLAGS_num_particles; i++) {
    max_prob = std::max(max_prob, particles_[i].weight);
  }

  // normalize sum to 1
  double sum_prob = 0.0;
  for (int i = 0; i < FLAGS_num_particles; i++) {
    particles_[i].weight = exp(particles_[i].weight - max_prob);
    sum_prob += particles_[i].weight;
  }

  for (int i = 0; i < FLAGS_num_particles; i++) {
     particles_[i].weight = particles_[i].weight / sum_prob;
     particles_[i].weight = log(particles_[i].weight);
  }
}

void ParticleFilter::GetLocation(Eigen::Vector2f* loc_ptr, 
                                 float* angle_ptr) const {
  Vector2f& loc = *loc_ptr;
  float& angle = *angle_ptr;
  // Compute the best estimate of the robot's location based on the current set
  // of particles. The computed values must be set to the `loc` and `angle`
  // variables to return them. Modify the following assignments:
  // loc = Vector2f(0, 0);
  // angle = 0;
  
  // Weighted sum over all particles
  Vector2f weighted_sum_loc(0.0, 0.0);
  float weighted_sum_angle = 0.0;
  for (auto& particle : particles_) {
    // cout << "particle: " << particle.loc << endl;
    // cout << "particle weight: " << particle.weight << endl;

    weighted_sum_loc = weighted_sum_loc + exp(particle.weight) * particle.loc;
    weighted_sum_angle = weighted_sum_angle + exp(particle.weight) * particle.angle;
  }

  loc = weighted_sum_loc;
  angle = weighted_sum_angle;
  
  // cout << "Get previous odometry: " << prev_odom_loc_ << endl;
  // cout << "Get Location: " << loc << endl;
  // cout << "Get angle: " << angle / M_2PI * 360 << endl;
}


}  // namespace particle_filter
