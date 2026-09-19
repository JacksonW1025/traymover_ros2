#include "lidar_localization/weighted_fitness.hpp"

#include <cmath>
#include <iostream>
#include <limits>
#include <numeric>
#include <vector>

#include <pcl/registration/registration.h>
#include <pcl/search/kdtree.h>

namespace
{
using Cloud = pcl::PointCloud<pcl::PointXYZI>;
using lidar_localization::evaluateWeightedFitness;

bool expect(bool condition, const char * message)
{
  if (!condition) {std::cerr << "FAILED: " << message << '\n';}
  return condition;
}

bool close(double a, double b)
{
  return std::abs(a - b) < 1e-10 * (1.0 + std::abs(b));
}

pcl::PointXYZI point(float x, float y, float z)
{
  pcl::PointXYZI p;
  p.x = x;
  p.y = y;
  p.z = z;
  p.intensity = 7.0f;
  return p;
}

// Exercise the actual PCL duplicate-source scorer as the compatibility oracle.
class FixedRegistration : public pcl::Registration<pcl::PointXYZI, pcl::PointXYZI>
{
protected:
  void computeTransformation(Cloud & output, const Matrix4 & guess) override
  {
    final_transformation_ = guess;
    converged_ = true;
    pcl::transformPointCloud(*input_, output, guess);
  }
};

double duplicateFitness(
  const Cloud & source, const std::vector<int> & weights,
  const Cloud::Ptr & target, const Eigen::Matrix4f & transform)
{
  Cloud::Ptr duplicates(new Cloud);
  for (std::size_t i = 0; i < source.size(); ++i) {
    for (int copy = 0; copy < weights[i]; ++copy) {duplicates->push_back(source[i]);}
  }
  FixedRegistration registration;
  registration.setInputSource(duplicates);
  registration.setInputTarget(target);
  Cloud output;
  registration.align(output, transform);
  return registration.getFitnessScore();
}

struct FailingTree
{
  int calls{0};
  int returned{0};
  float distance{0.0f};
  int nearestKSearch(
    const pcl::PointXYZI &, int, std::vector<int> &, std::vector<float> & distances)
  {
    ++calls;
    distances[0] = distance;
    return returned;
  }
};

bool rejected(const lidar_localization::WeightedFitness & result)
{
  return result.baseline == std::numeric_limits<double>::max() &&
         result.weighted == std::numeric_limits<double>::max();
}
}  // namespace

int main()
{
  bool ok = true;
  Cloud source;
  source.push_back(point(1, 1, 0));
  source.push_back(point(4, 1, 3));
  source.push_back(point(12, 2, 0));
  source.push_back(point(20, 3, 3));
  Cloud::Ptr target(new Cloud(source));
  for (std::size_t i = 0; i < target->size(); ++i) {
    (*target)[i].x += 0.1f * (i + 1);
  }
  pcl::search::KdTree<pcl::PointXYZI> tree;
  tree.setInputCloud(target);
  const std::vector<int> baseline{1, 1, 2, 2};
  const std::vector<int> weighted{1, 2, 2, 3};
  Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
  for (int nonidentity = 0; nonidentity < 2; ++nonidentity) {
    if (nonidentity) {
      transform.block<3, 3>(0, 0) =
        Eigen::AngleAxisf(0.1f, Eigen::Vector3f::UnitZ()).toRotationMatrix();
      transform(0, 3) = 0.3f;
      transform(1, 3) = -0.2f;
      transform(2, 3) = 0.1f;
    }
    const auto result = evaluateWeightedFitness(source, baseline, weighted, transform, tree);
    ok &= expect(close(result.baseline, duplicateFitness(source, baseline, target, transform)),
      "baseline matches PCL duplicated far-point source under identity and rigid transforms");
    ok &= expect(close(result.weighted, duplicateFitness(source, weighted, target, transform)),
      "height/far 1/2/2/3 weighting matches actual PCL duplicate-source score");
    ok &= expect(result.searches == source.size() && result.searches * 2 <
      static_cast<std::size_t>(std::accumulate(baseline.begin(), baseline.end(), 0) +
      std::accumulate(weighted.begin(), weighted.end(), 0)),
      "both fitness scores require one search per unique point, fewer than duplicate scoring");
    const auto no_height = evaluateWeightedFitness(source, baseline, baseline, transform, tree);
    ok &= expect(close(no_height.baseline, no_height.weighted) &&
      close(no_height.baseline, result.baseline), "disabled height weighting preserves baseline");
  }
  ok &= expect(source.size() == 4 && source[0].x == 1.0f && source[3].intensity == 7.0f,
    "scoring leaves the caller's source unchanged");

  FailingTree failing;
  ok &= expect(rejected(evaluateWeightedFitness(source, baseline, weighted, transform, failing)) &&
    failing.calls == 1, "failed nearest-neighbor query rejects both scores");
  failing.returned = 1;
  for (float invalid : {std::numeric_limits<float>::infinity(),
    std::numeric_limits<float>::quiet_NaN(), -1.0f})
  {
    failing.distance = invalid;
    ok &= expect(rejected(evaluateWeightedFitness(source, baseline, weighted, transform, failing)),
      "nonfinite or negative nearest-neighbor distances reject both scores");
  }
  failing.calls = 0;
  const Cloud empty;
  ok &= expect(rejected(evaluateWeightedFitness(empty, {}, {}, transform, failing)),
    "empty input rejects both scores");
  ok &= expect(rejected(evaluateWeightedFitness(source, {}, weighted, transform, failing)) &&
    rejected(evaluateWeightedFitness(source, baseline, {}, transform, failing)),
    "weight vector size mismatches reject before querying");
  for (const auto & invalid : std::vector<std::vector<int>>{
    {0, 1, 2, 2}, {-1, 1, 2, 2}, {1, 1, 2, 4}})
  {
    ok &= expect(rejected(evaluateWeightedFitness(source, invalid, weighted, transform, failing)),
      "nonpositive baseline weights or weighted weights below baseline reject");
  }
  auto invalid_weighted = weighted;
  invalid_weighted[0] = -1;
  ok &= expect(rejected(evaluateWeightedFitness(
    source, baseline, invalid_weighted, transform, failing)), "negative height weight rejects");
  Cloud invalid_source(source);
  invalid_source[0].x = std::numeric_limits<float>::quiet_NaN();
  ok &= expect(rejected(evaluateWeightedFitness(
    invalid_source, baseline, weighted, transform, failing)), "nonfinite input point rejects");
  Eigen::Matrix4f overflow = Eigen::Matrix4f::Identity();
  overflow(0, 0) = std::numeric_limits<float>::max();
  invalid_source = source;
  invalid_source[0].x = 2.0f;
  ok &= expect(rejected(evaluateWeightedFitness(
    invalid_source, baseline, weighted, overflow, failing)),
    "finite inputs that overflow during transformation reject before querying");
  transform(0, 0) = std::numeric_limits<float>::infinity();
  ok &= expect(rejected(evaluateWeightedFitness(source, baseline, weighted, transform, failing)),
    "nonfinite transform rejects");
  ok &= expect(failing.calls == 0, "invalid inputs never reach the search tree");
  return ok ? 0 : 1;
}
