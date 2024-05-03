#ifndef SRC_SLAM_CSM_H_
#define SRC_SLAM_CSM_H_

#include <vector>
#include "eigen3/Eigen/Dense"
#include "visualization/CImg.h"
#include "shared/math/statistics.h"

#define DEFAULT_GAUSSIAN_SIGMA 1
#define MIN_VALUE_FOR_LOOKUP 1E-10
#define EPSILON 1e-6

using cimg_library::CImg;
using Eigen::Vector2f;
using std::pair;
using std::vector;
using statistics::ProbabilityDensityGaussian;

typedef pair<Vector2f, float> Trans;

struct CostTable {
  uint64_t width;
  uint64_t height;
  double resolution;
  CImg<double> values;
  CostTable(const double range, const double resolution)
      : width(floor((range * 2.0) / resolution)),
        height(floor((range * 2.0) / resolution)),
        resolution(resolution) {
    // Construct a width x height image, with only 1 z level.
    // And, only one double per color with default value 0.0.
    values = CImg<double>(width, height, 1, 1, 0.0);
  }

  CostTable() : width(0), height(0), resolution(1) {}

  inline uint64_t convertX(float x) const {
    return width / 2 + floor(x / resolution);
  }

  inline uint64_t convertY(float y) const {
    return height / 2 + floor(y / resolution);
  }

  inline double GetPointValue(Vector2f point) const {
    uint64_t x = convertX(point.x());
    uint64_t y = convertY(point.y());
    if (x >= width || y >= height || values(x, y) <= MIN_VALUE_FOR_LOOKUP) {
      return MIN_VALUE_FOR_LOOKUP;
    }
    CHECK_LE(values(x, y), 1.0);
    return values(x, y);
  }

  void SetPointValue(Vector2f point, double value) {
    uint64_t x = convertX(point.x());
    uint64_t y = convertY(point.y());
    if (x >= width || y >= height) {
      return;
    }
    values(x, y) = value;
  }

  void normalize() { values = values.normalize(0, 1); }

  void GaussianBlur(const double sigma) {
    values = values.blur(sigma, sigma, 0, true, true);
  }

  void GaussianBlur() { GaussianBlur(DEFAULT_GAUSSIAN_SIGMA); }
};

class CorrelativeScanMatcher {
 public:
  CorrelativeScanMatcher(
    double scanner_range, double trans_range, double resolution,
    float k1, float k2, float k3, float k4) :
        scanner_range_(scanner_range),
        trans_range_(trans_range),
        resolution(resolution),
        k1_(k1), k2_(k2), k3_(k3), k4_(k4) {}

  /**
   * @brief Get the Trans And Uncertainty object
   * 
   * @param pointcloud_a [in]
   * @param pointcloud_b [in]
   * @param odom [in]
   * @param results [out]
   * @return true if csm converged, false otherwise
   */
  bool GetTransform(
    const vector<Vector2f> &pointcloud_a,
    const vector<Vector2f> &pointcloud_b,
    const Trans &odom,
    pair<Trans, Eigen::Matrix3f> &results);

 private:
  static vector<Vector2f> RotatePointcloud(
    const vector<Vector2f> &pointcloud, const double rotation);
  static double CalculatePointcloudCost(
    const vector<Vector2f> &pointcloud, const double x_trans,
    const double y_trans, const CostTable &cost_table);
  CostTable CostTableFromPointCloud(const vector<Vector2f> &pointcloud);
  void GenerateSearchParams(
    vector<pair<double, double>> &tranlations, vector<double> &rotations,
    const Trans &odom);
  double EvaluateMotionModel(const Trans &trans, const Trans &odom);
  double scanner_range_;
  double trans_range_;
  double resolution;
  float k1_, k2_, k3_, k4_;
};

#endif  // SRC_SLAM_CSM_H_
