#include "./CorrelativeScanMatcher.h"
#include <iostream>

vector<Vector2f> CorrelativeScanMatcher::RotatePointcloud(
    const vector<Vector2f> &pointcloud, const double rotation) {
  const Eigen::Matrix2f rot_matrix =
      Eigen::Rotation2Df(rotation).toRotationMatrix();
  vector<Vector2f> rotated_pointcloud;
  for (const Vector2f &point : pointcloud) {
    rotated_pointcloud.push_back(rot_matrix * point);
  }
  return rotated_pointcloud;
}

double CorrelativeScanMatcher::CalculatePointcloudCost(
    const vector<Vector2f> &pointcloud, const double x_trans,
    const double y_trans, const CostTable &cost_table) {
  double probability = 0.0;
  for (const Vector2f &point : pointcloud) {
    double cost = cost_table.GetPointValue(point + Vector2f(x_trans, y_trans));
    // Only count as percentage of points that fall inside the grid.
    probability += log(cost);
  }

  return probability / pointcloud.size();
}

CostTable CorrelativeScanMatcher::CostTableFromPointCloud(
    const vector<Vector2f> &pointcloud) {
  CostTable table(scanner_range_, resolution);
  for (const Vector2f &point : pointcloud) {
    table.SetPointValue(point, 1);
  }
  table.GaussianBlur();
  table.normalize();
  return table;
}

void CorrelativeScanMatcher::GenerateSearchParams(
    vector<pair<double, double>> &tranlations, vector<double> &rotations,
    const Trans &odom) {
  rotations.clear();
  rotations.reserve(360);
  for (int i = 0; i < 360; i++) {
    rotations.push_back(i * M_PI / 180.);
  }

  int num_translations = std::ceil((trans_range_ * 2 - EPSILON) / resolution);
  tranlations.clear();
  tranlations.reserve(num_translations * num_translations);

  for (int i = 0; i < num_translations; i++) {
    double x_trans = i * resolution - trans_range_ + EPSILON + odom.first.x();
    for (int j = 0; j < num_translations; j++) {
      double y_trans = j * resolution - trans_range_ + EPSILON + odom.first.y();
      tranlations.push_back(std::make_pair(x_trans, y_trans));
    }
  }
}

bool CorrelativeScanMatcher::GetTransform(
    const vector<Vector2f> &pointcloud_a, const vector<Vector2f> &pointcloud_b,
    const Trans &odom, pair<Trans, Eigen::Matrix3f> &transform) {
  vector<double> rotations(360);
  vector<pair<double, double>> translations;
  GenerateSearchParams(translations, rotations, odom);
  const CostTable cost_table = CostTableFromPointCloud(pointcloud_b);

  // Calculation Method taken from Realtime Correlative Scan Matching
  // by Edward Olsen.
  Eigen::Matrix3f K = Eigen::Matrix3f::Zero();
  Eigen::Vector3f u(0, 0, 0);
  double s = 0;
#pragma omp parallel for
  for (double rotation : rotations) {
    const vector<Vector2f> rotated_pointcloud_a =
        RotatePointcloud(pointcloud_a, rotation);
    for (const pair<double, double> &translation : translations) {
      double x_trans = translation.first, y_trans = translation.second;
      double cost = CalculatePointcloudCost(
        rotated_pointcloud_a, x_trans, y_trans, cost_table);
        const Trans trans = std::make_pair(Vector2f(x_trans, y_trans), rotation);
        cost += EvaluateMotionModel(trans, odom);
        Eigen::Vector3f x(x_trans, y_trans, rotation);
        cost = exp(cost);
        #pragma omp critical
        {
          K += x * x.transpose() * cost;
          u += x * cost;
          s += cost;
        }
    }
  }

  std::cout << "K: " << std::endl << K << std::endl;
  std::cout << "u " << std::endl << u << std::endl;
  std::cout << "s: " << std::endl << s << std::endl;
  
  // Check if CSM converged. If s is too small then return the odometry only.
  if (s < EPSILON) {
    // use odometry and fixed uncertainty
    Trans trans = odom;
    Eigen::Matrix3f fixed_uncertainty;
    fixed_uncertainty << 1.0, 0, 0,
                        0, 1.0, 0,
                        0, 0, 1.0;
    transform = std::make_pair(trans, fixed_uncertainty);
    return false;
  }
  
  // Calculate mean by normalizing u
  Trans trans = std::make_pair(Vector2f(u.x() / s, u.y() / s), u.z() / s);
  // Calculate Uncertainty matrix.
  Eigen::Matrix3f uncertainty =
      (1.0 / s) * K - (1.0 / (s * s)) * u * u.transpose();

  // ---- Print for debugging ----
  std::cout << "Odometry: " << '(' << odom.first.x() << ", "
            << odom.first.y() << ", " << odom.second << ')' << std::endl;
  std::cout << "Estimated: " << '(' << trans.first.x() << ", "
            << trans.first.y() << ", " << trans.second << ')' << std::endl;

  transform =  std::make_pair(trans, uncertainty);
  return true;
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
