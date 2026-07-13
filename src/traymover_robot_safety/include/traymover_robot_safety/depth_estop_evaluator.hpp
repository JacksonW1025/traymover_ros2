#ifndef TRAYMOVER_ROBOT_SAFETY__DEPTH_ESTOP_EVALUATOR_HPP_
#define TRAYMOVER_ROBOT_SAFETY__DEPTH_ESTOP_EVALUATOR_HPP_

#include <cstddef>

#include "sensor_msgs/msg/image.hpp"

namespace traymover_robot_safety
{

struct DepthEstopConfig
{
  double depth_scale_m{0.001};
  double stop_distance_m{0.8};
  double release_distance_m{1.0};
  double roi_x_min{0.25};
  double roi_x_max{0.75};
  double roi_y_min{0.20};
  double roi_y_max{0.85};
  double min_valid_fraction{0.50};
  std::size_t grid_columns{16};
  std::size_t grid_rows{12};
  double near_tile_fraction{0.10};
  std::size_t clear_frame_count{8};
};

struct DepthEstopResult
{
  bool estop_requested{true};
  bool frame_valid{false};
  bool near_obstacle{false};
  double valid_fraction{0.0};
  std::size_t consecutive_clear_frames{0};
};

class DepthEstopEvaluator
{
public:
  explicit DepthEstopEvaluator(const DepthEstopConfig & config);

  DepthEstopResult process(const sensor_msgs::msg::Image & image);
  void forceEstop();
  bool estopRequested() const;

private:
  DepthEstopResult invalidFrameResult();

  DepthEstopConfig config_;
  bool estop_requested_{true};
  std::size_t consecutive_clear_frames_{0};
};

}  // namespace traymover_robot_safety

#endif  // TRAYMOVER_ROBOT_SAFETY__DEPTH_ESTOP_EVALUATOR_HPP_
