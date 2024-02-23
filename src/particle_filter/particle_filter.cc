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

DEFINE_double(num_particles, 50, "Number of particles");

namespace particle_filter {

config_reader::ConfigReader config_reader_({"config/particle_filter.lua"});

ParticleFilter::ParticleFilter() :
    prev_odom_loc_(0, 0),
    prev_odom_angle_(0),
    odom_initialized_(false) {}

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
    float _beamAngle = i * angle_delta - angle_min;
    
    line2f my_line(
      range_min * cos(_beamAngle + angle ) + laser_loc.x(), range_min * sin(_beamAngle + angle ) + laser_loc.y(),
      range_max * cos(_beamAngle + angle ) + laser_loc.x(), range_max * sin(_beamAngle + angle ) + laser_loc.y());


    float shortest_range = range_max;
    for (size_t i = 0; i < map_.lines.size(); ++i) {
      const line2f map_line = map_.lines[i];
      // Check for intersections:
      bool intersects = map_line.Intersects(my_line);
      // You can also simultaneously check for intersection, and return the point
      // of intersection:
      Vector2f intersection_point; // Return variable
      intersects = map_line.Intersection(my_line, &intersection_point);
      if (intersects) {
        // float r = sqrt(
        //   pow( (intersection_point.x()-laser_loc.x()), 2 ) + 
        //   pow( (intersection_point.y()-laser_loc.y()), 2 )
        // );
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
    scan[i] = Vector2f(shortest_range * cos(_beamAngle + angle ) + laser_loc.x(), shortest_range * sin(_beamAngle + angle ) + laser_loc.y());
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
  const float sigma_s = 1.f;
  const float gamma = 1.f; // 1: uncorrelated, 1/n: perfectly correlated
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
  for(size_t i=0; i< scan.size();++i)
  {
    float r = (scan[i] - p_ptr->loc).norm();
    log_prob += -0.5 * pow( r - ranges[i],2) / pow(sigma_s,2);
  }
  log_prob = log_prob * gamma;
  p_ptr->weight = exp(log_prob);
  // ian =========
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
  vector<Particle> new_particles;
  new_particles.resize(particles_.size());

  vector<float> cmf; // cumulative mass function
  cmf.resize(particles_.size()+1);
  cmf[0] = 0;
  for(size_t i=0; i<particles_.size();++i)
  {
    cmf[i+1] = particles_[i].weight + cmf[i];
    particles_[i].weight = 1.0f / particles_.size();
  }
  // add 0.1 to the last boundary, won't affect the sampling
  cmf[particles_.size()] += 0.1f;

  // During resampling: 
  for(size_t i=0; i<particles_.size();++i)
  {
    float x = rng_.UniformRandom(0, cmf[particles_.size()] - 0.1f);
    auto upper = std::upper_bound(cmf.begin(),cmf.end(),x);
    int idx = std::distance(cmf.begin(), upper) - 1;
    new_particles.push_back(particles_[idx]);
  }
  // After resampling:
  particles_ = new_particles;

  // ian =====
}

void ParticleFilter::ObserveLaser(const vector<float>& ranges,
                                  float range_min,
                                  float range_max,
                                  float angle_min,
                                  float angle_max) {
  // A new laser scan observation is available (in the laser frame)
  // Call the Update and Resample steps as necessary.
  // ian =====
  // might need some heuristic to determine whether or not to update
  for(size_t i=0;i<particles_.size();++i)
  {
    this->Update( ranges,
                  range_min,
                  range_max,
                  angle_min,
                  angle_max,
                  &particles_[i]);
  }
  this->Resample();
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

  cout << "predict motion using new odom: " << odom_loc << endl;

  float k1 = 0.1; // trans error from trans model
  float k2 = 0.1; // rot error from  trans model
  float k3 = 0.1; // trans error from rot model
  float k4 = 0.1; // rot error from rot model

  // In odom frame, compute delta x, y and angle 
  // This is the "car" movement
  Vector2f dxy_odom = odom_loc - prev_odom_loc_;
  float dangle_odom = AngleDiff(odom_angle, prev_odom_angle_); // AngleDiff bound the angle into [-pi, pi]

  // For each particles, transform odom movements to map frame, sample noise
  for (auto &_particle : particles_) {

    // Find transformation to transform points from odom frame to map frame
    // rot =  angle in map - angle in odom =  _particle.angle - prev_odom_angle_ 
    Eigen::Rotation2Df R_odom2map(AngleDiff(_particle.angle, prev_odom_angle_));
    
    // Transform dxy, dangle from odom frame to map frame
    Vector2f dxy_map = R_odom2map * dxy_odom; 
    float dangle_map = dangle_odom; 

    // sample noise
    float dx_map = rng_.Gaussian(dxy_map.x(), k1*dxy_map.norm() + k2*abs(dangle_map));
    float dy_map = rng_.Gaussian(dxy_map.y(), k1*dxy_map.norm() + k2*abs(dangle_map));
    dangle_map = rng_.Gaussian(dangle_map, k3*dxy_map.norm() + k4*abs(dangle_map));

    // update particles
    _particle.loc.x() = _particle.loc.x() + dx_map;
    _particle.loc.y() = _particle.loc.y() + dy_map;
    _particle.angle = _particle.angle + dangle_map;
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
  // TODO: require tuning
  float x_std = 0.1 * 0.0;
  float y_std = 0.1 * 0.0;
  float angle_std = M_PI / 9 * 0.0; // 20deg
  for (int i = 0; i < FLAGS_num_particles; i++) {
    Particle p;
    p.loc.x() = rng_.Gaussian(loc.x(), x_std);
    p.loc.y() =  rng_.Gaussian(loc.y(), y_std);
    p.angle =  rng_.Gaussian(angle, angle_std);
    p.weight = 1/FLAGS_num_particles; // TODO: not sure????
    particles_.push_back(p);
  }
  printf("Initialize particles/odom finished.");

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
    cout << "particle: " << particle.loc << endl;
    weighted_sum_loc = weighted_sum_loc + particle.weight * particle.loc;
    weighted_sum_angle = weighted_sum_angle + particle.weight * particle.angle;
  }

  loc = weighted_sum_loc;
  angle = weighted_sum_angle;
  
  cout << "Get previous odometry: " << prev_odom_loc_ << endl;
  cout << "Get Location: " << loc << endl;
}


}  // namespace particle_filter
