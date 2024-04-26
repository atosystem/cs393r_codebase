#include "./CorrelativeScanMatcher.h"

#include <algorithm>
#include <vector>

#include <boost/dynamic_bitset.hpp>
#include "eigen3/Eigen/Dense"
#include <iostream>

#define UNCERTAINTY_USELESS_THRESHOLD log(1e-4)
#define DEBUG false

using Eigen::Vector2f;
using std::pair;
using std::vector;

#define EPSILON 1e-6

LookupTable CorrelativeScanMatcher::GetLookupTable(
    const vector<Vector2f>& pointcloud, double resolution) {
  LookupTable table(scanner_range_, resolution);
  for (const Vector2f& point : pointcloud) {
    table.SetPointValue(point, 1);
  }
  table.GaussianBlur();
  table.normalize();
  return table;
}

LookupTable CorrelativeScanMatcher::GetLookupTableLowRes(
    const LookupTable& high_res_table) {
  LookupTable low_res_table(scanner_range_, low_res_);
  // Run the max filter over the portions of this table.

  for (double x = -scanner_range_ + EPSILON; x < scanner_range_; x += low_res_) {
    for (double y = -scanner_range_ + EPSILON; y < scanner_range_; y += low_res_) {
      // Get the max value for all the cells that this low res
      // cell encompasses.
      double max_area =
          high_res_table.MaxArea(x, y, x + low_res_, y + low_res_);
      low_res_table.SetPointValue(Vector2f(x, y), max_area);
    }
  }

  return low_res_table;
}

LookupTable CorrelativeScanMatcher::GetLookupTableHighRes(
    const vector<Vector2f>& pointcloud) {
  return GetLookupTable(pointcloud, high_res_);
}

vector<Vector2f> RotatePointcloud(const vector<Vector2f>& pointcloud,
                                  const double rotation) {
  const Eigen::Matrix2f rot_matrix =
      Eigen::Rotation2Df(rotation).toRotationMatrix();
  vector<Vector2f> rotated_pointcloud;
  for (const Vector2f& point : pointcloud) {
    rotated_pointcloud.push_back(rot_matrix * point);
  }
  return rotated_pointcloud;
}

double CalculatePointcloudCost(const vector<Vector2f>& pointcloud,
                               const double x_trans, const double y_trans,
                               const LookupTable& cost_table) {
  double probability = 0.0;
  for (const Vector2f& point : pointcloud) {
    double cost = cost_table.GetPointValue(point + Vector2f(x_trans, y_trans));
    // Only count as percentage of points that fall inside the grid.
    probability += log(cost);
  }

  return probability / pointcloud.size();
}

bool LessThanTransProb(TransProb a, TransProb b) {
  return a.first < b.first;
}

vector<TransProb> MemoizeLowRes(
    const vector<Vector2f>& query_scan, const LookupTable& low_res_table,
    const double resolution, const double range,
    const double restrict_rotation_min, const double restrict_rotation_max) {
  vector<TransProb> low_res_costs(
      low_res_table.AbsCoords(range, range) + 1,
      std::make_pair(-INFINITY, std::make_pair(Vector2f(0, 0), 0)));
  for (double rotation = restrict_rotation_min;
       rotation < restrict_rotation_max; rotation += M_PI / 180) {
    // Rotate the pointcloud by this rotation.
    const vector<Vector2f> rotated_query =
        RotatePointcloud(query_scan, rotation);
    for (double x_trans = -range + EPSILON; x_trans < range;
         x_trans += resolution) {
      for (double y_trans = -range + EPSILON; y_trans < range;
           y_trans += resolution) {
        // If this is a negligible amount of the total sum then just use the
        // low res cost, don't worry about the high res cost.
        Trans transformation = std::make_pair(Vector2f(x_trans, y_trans), rotation);
        double low_res_cost = CalculatePointcloudCost(rotated_query, x_trans,
                                                      y_trans, low_res_table);
        TransProb trans_prob = std::make_pair(low_res_cost, transformation);
        size_t abs_coord = low_res_table.AbsCoords(x_trans, y_trans);
        low_res_costs[abs_coord] =
            std::max(low_res_costs[abs_coord], trans_prob, LessThanTransProb);
      }
    }
  }
  return low_res_costs;
}

TransProb CorrelativeScanMatcher::GetProbAndTransformation(
    const vector<Vector2f>& rotated_pointcloud_a,
    const LookupTable& pointcloud_b_cost, double resolution, double x_min,
    double x_max, double y_min, double y_max, double rotation) {
  Trans current_most_likely_trans = std::make_pair(Vector2f(x_min, y_min), rotation);
  double current_most_likely_prob = -INFINITY;

  for (double x_trans = x_min + EPSILON; x_trans < x_max;
       x_trans += resolution) {
    for (double y_trans = y_min + EPSILON; y_trans < y_max;
         y_trans += resolution) {
      // Otherwise, get the probability / cost of this scan.
      double probability = CalculatePointcloudCost(
          rotated_pointcloud_a, x_trans, y_trans, pointcloud_b_cost);
      // If it is the best so far, keep track of it!
      if (probability > current_most_likely_prob) {
        current_most_likely_trans = Trans(Vector2f(x_trans, y_trans), rotation);
        current_most_likely_prob = probability;
      }
    }
  }
  return TransProb(current_most_likely_prob, current_most_likely_trans);
}

TransProb CorrelativeScanMatcher::GetTransformation(
    const vector<Vector2f>& pointcloud_a,
    const vector<Vector2f>& pointcloud_b,
    const double rotation_min,
    const double rotation_max) {
  double current_probability = 1.0;
  double best_probability = -INFINITY;
  Trans best_transformation;
  const LookupTable pointcloud_b_cost_high_res =
      GetLookupTableHighRes(pointcloud_b);
  const LookupTable pointcloud_b_cost_low_res =
      GetLookupTableLowRes(pointcloud_b_cost_high_res);
  vector<TransProb> low_res_costs = MemoizeLowRes(pointcloud_a,
                    pointcloud_b_cost_low_res, low_res_, trans_range_,
                    rotation_min, rotation_max);
#if DEBUG
  std::cout << "Low Res Cost: "
            << CalculatePointcloudCost(RotatePointcloud(pointcloud_a, 3.14),
                                       0.7, -0.2, pointcloud_b_cost_low_res)
            << std::endl;
  std::cout << "High Res Cost: "
            << CalculatePointcloudCost(RotatePointcloud(pointcloud_a, 3.14),
                                       0.7, -0.2, pointcloud_b_cost_high_res)
            << std::endl;
#endif
  while (current_probability >= best_probability) {
    // Evaluate over the low_res lookup table.
    std::vector<TransProb>::iterator max = std::max_element(
      low_res_costs.begin(), low_res_costs.end(), LessThanTransProb);
    // Exclude this scan so we never get it again.
    auto prob_and_trans_low_res = *max;
    max->first = -INFINITY;
    current_probability = prob_and_trans_low_res.first;
    if (current_probability < best_probability) {
      break;
    }
    // double best_probability_low_res = best_prob_and_trans_low_res.first;
    Vector2f best_translation_low_res = prob_and_trans_low_res.second.first;
    double best_rotation_low_res = prob_and_trans_low_res.second.second;

    auto rotated_pointcloud_a =
        RotatePointcloud(pointcloud_a, prob_and_trans_low_res.second.second);

#if DEBUG
    printf("Found Low Res Pose (%f, %f), rotation %f: %f\n",
           best_translation_low_res.x(), best_translation_low_res.y(),
           best_rotation_low_res, best_probability_low_res);
#endif

    double x_min_high_res =
        std::max(best_translation_low_res.cast<double>().x(), -trans_range_);
    double x_max_high_res =
        std::min(best_translation_low_res.x() + low_res_, trans_range_);
    double y_min_high_res =
        std::max(best_translation_low_res.cast<double>().y(), -trans_range_);
    double y_max_high_res =
        std::min(best_translation_low_res.y() + low_res_, trans_range_);

#if DEBUG
    printf("Commencing High Res Search in window (%f, %f) (%f, %f) \n",
           x_min_high_res, y_min_high_res, x_max_high_res, y_max_high_res);
#endif

    CHECK_LT(x_min_high_res, trans_range_);
    CHECK_LT(y_min_high_res, trans_range_);
    CHECK_GT(x_max_high_res, -trans_range_);
    CHECK_GT(y_max_high_res, -trans_range_);
    auto prob_and_trans_high_res = GetProbAndTransformation(
        rotated_pointcloud_a, pointcloud_b_cost_high_res, high_res_,
        x_min_high_res, x_max_high_res, y_min_high_res, y_max_high_res,
        best_rotation_low_res);
#if DEBUG
    if (prob_and_trans_high_res.first > best_probability_low_res) {
      for (double y = y_min_high_res; y < y_max_high_res; y += high_res_) {
        for (double x = x_min_high_res; x < x_max_high_res; x += high_res_) {
          std::cout << CalculatePointcloudCost(
                           RotatePointcloud(pointcloud_a,
                                            best_rotation_low_res),
                           x, y, pointcloud_b_cost_high_res)
                    << " ";
        }
        std::cout << std::endl;
      }
    }
    printf("Found High Res Pose (%f, %f, %f): %f\n",
           prob_and_trans_high_res.second.first.x(),
           prob_and_trans_high_res.second.first.y(),
           prob_and_trans_high_res.second.second,
           prob_and_trans_high_res.first);
#endif

    if (prob_and_trans_high_res.first > best_probability) {
      // This is the new best and we should keep searching to make
      // sure there is nothing better.
      best_probability = prob_and_trans_high_res.first;
      best_transformation = prob_and_trans_high_res.second;
    }
  }
  return std::make_pair(best_probability, best_transformation);
}

TransProb CorrelativeScanMatcher::GetTransformation(
    const vector<Vector2f>& pointcloud_a,
    const vector<Vector2f>& pointcloud_b,
    const double rotation_a,
    const double rotation_b,
    const double rotation_restriction) {
  double angle_dist = math_util::AngleDiff(rotation_a, rotation_b);
  double rotation_min = math_util::AngleMod(angle_dist - rotation_restriction / 2);
  double rotation_max = math_util::AngleMod(angle_dist + rotation_restriction / 2);
  if (rotation_max < rotation_min) {
    rotation_max += 2 * M_PI;
  }

  return GetTransformation(
      pointcloud_a, pointcloud_b,
      rotation_min,
      rotation_max);
}

// Calculates full uncertainty (including rotation)
Eigen::Matrix3f CorrelativeScanMatcher::GetUncertaintyMatrix(
    const vector<Vector2f>& pointcloud_a,
    const vector<Vector2f>& pointcloud_b) {
  // Calculation Method taken from Realtime Correlative Scan Matching
  // by Edward Olsen.
  Eigen::Matrix3f K = Eigen::Matrix3f::Zero();
  Eigen::Vector3f u(0, 0, 0);
  double s = 0;
  const LookupTable pointcloud_b_cost_high_res =
      GetLookupTableHighRes(pointcloud_b);
  const LookupTable pointcloud_b_cost_low_res =
      GetLookupTableLowRes(pointcloud_b_cost_high_res);
  const vector<TransProb> low_res_costs =
      MemoizeLowRes(pointcloud_a, pointcloud_b_cost_low_res, low_res_,
                    trans_range_, 0, 2 * M_PI);
  printf("Calculating Uncertainty...\n");
  for (double rotation = 0; rotation < 2 * M_PI; rotation += M_PI / 180) {
    // Rotate the pointcloud by this rotation.
    const vector<Vector2f> rotated_pointcloud_a =
        RotatePointcloud(pointcloud_a, rotation);
    for (double x_trans = -trans_range_ + EPSILON; x_trans < trans_range_;
         x_trans += low_res_) {
      for (double y_trans = -trans_range_ + EPSILON; y_trans < trans_range_;
           y_trans += low_res_) {
        // If this is a negligible amount of the total sum then just use the
        // low res cost, don't worry about the high res cost.
        size_t abs_coord =
            pointcloud_b_cost_low_res.AbsCoords(x_trans, y_trans);
        double low_res_cost = low_res_costs[abs_coord].first;
        double cost = 0.0;
        if (low_res_cost <= UNCERTAINTY_USELESS_THRESHOLD) {
          cost = low_res_cost;
        } else {
          cost = CalculatePointcloudCost(rotated_pointcloud_a, x_trans, y_trans,
                                         pointcloud_b_cost_high_res);
        }
        cost = exp(cost);
        Eigen::Vector3f x(x_trans, y_trans, rotation);
        K += x * x.transpose() * cost;
        u += x * cost;
        s += cost;
      }
    }
  }
  // Calculate Uncertainty matrix.
  // std::cout << "K: " << std::endl << K << std::endl;
  // std::cout << "u " << std::endl << u << std::endl;
  // std::cout << "s: " << std::endl << s << std::endl;
  Eigen::Matrix3f uncertainty =
      (1.0 / s) * K - (1.0 / (s * s)) * u * u.transpose();
  return uncertainty;
}

// Calculates x-y uncertainty
Eigen::Matrix2f CorrelativeScanMatcher::GetUncertaintyMatrix(
    const vector<Vector2f>& pointcloud_a, const vector<Vector2f>& pointcloud_b,
    const double rotation) {
  // Calculation Method taken from Realtime Correlative Scan Matching
  // by Edward Olsen.
  Eigen::Matrix2f K = Eigen::Matrix2f::Zero();
  Eigen::Vector2f u(0, 0);
  double s = 0;
  const LookupTable pointcloud_b_cost_high_res =
      GetLookupTableHighRes(pointcloud_b);
  const LookupTable pointcloud_b_cost_low_res =
      GetLookupTableLowRes(pointcloud_b_cost_high_res);
#if DEBUG
  printf("Calculating Uncertainty given rotation...\n");
#endif
  // Rotate the pointcloud by this rotation.
  const vector<Vector2f> rotated_pointcloud_a =
      RotatePointcloud(pointcloud_a, rotation);
  auto low_res_costs =
      MemoizeLowRes(pointcloud_a, pointcloud_b_cost_low_res, low_res_,
                    trans_range_, rotation, rotation + EPSILON);

  for (double x_trans = -trans_range_ + EPSILON; x_trans < trans_range_;
       x_trans += low_res_) {
    for (double y_trans = -trans_range_ + EPSILON; y_trans < trans_range_;
         y_trans += low_res_) {
      // If this is a negligible amount of the total sum then just use the
      // low res cost, don't worry about the high res cost.
      size_t abs_coord = pointcloud_b_cost_low_res.AbsCoords(x_trans, y_trans);
      double low_res_cost = low_res_costs[abs_coord].first;
      double cost = 0.0;
      if (low_res_cost <= UNCERTAINTY_USELESS_THRESHOLD) {
        cost = low_res_cost;
      } else {
        cost = CalculatePointcloudCost(rotated_pointcloud_a, x_trans, y_trans,
                                       pointcloud_b_cost_high_res);
      }
      cost = exp(cost);
      Eigen::Vector2f x(x_trans, y_trans);
      K += x * x.transpose() * cost;
      u += x * cost;
      s += cost;
    }
  }
  // Calculate Uncertainty matrix.
  // std::cout << "K: " << std::endl << K << std::endl;
  // std::cout << "u " << std::endl << u << std::endl;
  // std::cout << "s: " << std::endl << s << std::endl;
  Eigen::Matrix2f uncertainty =
      (1.0 / s) * K - (1.0 / (s * s)) * u * u.transpose();
  return uncertainty;
}

std::pair<double, double> CorrelativeScanMatcher::GetLocalUncertaintyStats(
    const vector<vector<Vector2f>>& comparisonClouds,
    const vector<Vector2f> targetCloud) {
  double condition_avg = 0.0;
  double scale_avg = 0.0;

#pragma omp parallel for
  for (size_t i = 0; i < comparisonClouds.size(); i++) {
    vector<Vector2f> comparisonCloud = comparisonClouds[i];
    TransProb result = GetTransformation(targetCloud, comparisonCloud);
    Eigen::Matrix2f uncertainty = GetUncertaintyMatrix(
        targetCloud, comparisonCloud, result.second.second);
    Eigen::EigenSolver<Eigen::Matrix2f> es(uncertainty);
    Eigen::Vector2f eigenvalues = es.eigenvalues().real();
    condition_avg += eigenvalues.maxCoeff() / eigenvalues.minCoeff();
    scale_avg += eigenvalues.maxCoeff();
  }

  condition_avg /= (comparisonClouds.size());
  scale_avg /= (comparisonClouds.size());

  return std::make_pair(condition_avg, scale_avg);
}

pair<Trans, Eigen::Matrix3f> CorrelativeScanMatcher::GetTransAndUncertainty(
    const vector<Vector2f>& pointcloud_a,
    const vector<Vector2f>& pointcloud_b,
    const Trans& odom) {
  // Calculation Method taken from Realtime Correlative Scan Matching
  // by Edward Olsen.
  Eigen::Matrix3f K = Eigen::Matrix3f::Zero();
  Eigen::Vector3f u(0, 0, 0);
  double s = 0;
  const LookupTable pointcloud_b_cost_high_res =
      GetLookupTableHighRes(pointcloud_b);
  const LookupTable pointcloud_b_cost_low_res =
      GetLookupTableLowRes(pointcloud_b_cost_high_res);
  const vector<TransProb> low_res_costs =
      MemoizeLowRes(pointcloud_a, pointcloud_b_cost_low_res, low_res_,
                    trans_range_, 0, 2 * M_PI);
  // printf("Calculating Uncertainty...\n");
  TransProb best;
  best.first = -INFINITY;

  // 
  for (double rotation = 0; rotation < 2 * M_PI; rotation += M_PI / 180) {
    // Rotate the pointcloud by this rotation.
    const vector<Vector2f> rotated_pointcloud_a =
        RotatePointcloud(pointcloud_a, rotation);
    for (double x_trans = -trans_range_ + EPSILON; x_trans < trans_range_;
         x_trans += low_res_) {
      for (double y_trans = -trans_range_ + EPSILON; y_trans < trans_range_;
           y_trans += low_res_) {
        // If this is a negligible amount of the total sum then just use the
        // low res cost, don't worry about the high res cost.
        size_t abs_coord =
            pointcloud_b_cost_low_res.AbsCoords(x_trans, y_trans);
        double low_res_cost = low_res_costs[abs_coord].first;
        double cost = 0.0;
        if (low_res_cost <= UNCERTAINTY_USELESS_THRESHOLD) {
          cost = low_res_cost;
        } else {
          cost = CalculatePointcloudCost(rotated_pointcloud_a, x_trans, y_trans,
                                         pointcloud_b_cost_high_res);
        }
        const Trans trans(Vector2f(x_trans, y_trans), rotation);
        // ---- Remove for debugging ----
        // cost += EvaluateMotionModel(trans, odom);
        // ---- Remove for debugging ----
        if (cost > best.first) {
          best.first = cost;
          best.second = trans;
        }
        cost = exp(cost);
        Eigen::Vector3f x(x_trans, y_trans, rotation);
        K += x * x.transpose() * cost;
        u += x * cost;
        s += cost;
      }
    }
  }
  // Calculate Uncertainty matrix.
  // std::cout << "K: " << std::endl << K << std::endl;
  // std::cout << "u " << std::endl << u << std::endl;
  // std::cout << "s: " << std::endl << s << std::endl;
  
  // ---- Print for debugging ----
  std::cout << "Current odom: " << "("<< odom.first.x() <<","<<odom.first.y() << "," << odom.second << ")" << std::endl;
  std::cout << "Evalaute odom: " << "("<< best.second.first.x() << ","<< best.second.first.y() << ", " << best.second.second << ") , Prob: " << exp(best.first) << std::endl;
  
  Trans trans = std::make_pair(Vector2f(u.x() / s, u.y() / s), u.z() / s);
  Eigen::Matrix3f uncertainty =
      (1.0 / s) * K - (1.0 / (s * s)) * u * u.transpose();
  return std::make_pair(trans, uncertainty);
  
 
}

double CorrelativeScanMatcher::EvaluateMotionModel(
    const Trans &trans, const Trans &odom) {
    const float x_trans = trans.first.x(),
                y_trans = trans.first.y(),
                rotation = trans.second,
                odom_trans = odom.first.norm(),
                odom_rot = std::fabs(odom.second);

    const float trans_error = k1_ * odom_trans + k2_ * odom_rot,
                  rot_error = k3_ * odom_trans + k4_ * odom_rot;

    return (double) std::log(
      ProbabilityDensityGaussian(x_trans, odom.first.x(), trans_error) *
      ProbabilityDensityGaussian(y_trans, odom.first.y(), trans_error) *
      ProbabilityDensityGaussian(rotation, odom.second, rot_error));
}
