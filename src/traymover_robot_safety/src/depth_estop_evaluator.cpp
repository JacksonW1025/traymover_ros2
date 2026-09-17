#include "traymover_robot_safety/depth_estop_evaluator.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <stdexcept>
#include <string>
#include <vector>

namespace traymover_robot_safety
{
namespace
{

void validateFraction(double value, const char * name)
{
  if (value <= 0.0 || value > 1.0) {
    throw std::invalid_argument(std::string(name) + " must be in (0, 1]");
  }
}

std::uint16_t readDepth(const sensor_msgs::msg::Image & image, std::size_t offset)
{
  if (image.is_bigendian != 0U) {
    return static_cast<std::uint16_t>(
      (static_cast<std::uint16_t>(image.data[offset]) << 8U) |
      static_cast<std::uint16_t>(image.data[offset + 1U]));
  }
  return static_cast<std::uint16_t>(
    static_cast<std::uint16_t>(image.data[offset]) |
    (static_cast<std::uint16_t>(image.data[offset + 1U]) << 8U));
}

}  // namespace

DepthEstopEvaluator::DepthEstopEvaluator(const DepthEstopConfig & config)
: config_(config)
{
  if (config_.depth_scale_m <= 0.0) {
    throw std::invalid_argument("depth_scale_m must be positive");
  }
  if (config_.stop_distance_m <= 0.0 ||
    config_.release_distance_m < config_.stop_distance_m)
  {
    throw std::invalid_argument(
            "release_distance_m must be greater than or equal to stop_distance_m");
  }
  if (config_.roi_x_min < 0.0 || config_.roi_x_max > 1.0 ||
    config_.roi_x_min >= config_.roi_x_max || config_.roi_y_min < 0.0 ||
    config_.roi_y_max > 1.0 || config_.roi_y_min >= config_.roi_y_max)
  {
    throw std::invalid_argument("ROI bounds must define a non-empty normalized rectangle");
  }
  validateFraction(config_.min_valid_fraction, "min_valid_fraction");
  validateFraction(config_.severe_invalid_fraction, "severe_invalid_fraction");
  if (config_.severe_invalid_fraction > config_.min_valid_fraction) {
    throw std::invalid_argument(
            "severe_invalid_fraction must not exceed min_valid_fraction");
  }
  validateFraction(config_.near_tile_fraction, "near_tile_fraction");
  validateFraction(config_.min_near_pixel_fraction, "min_near_pixel_fraction");
  if (config_.grid_columns == 0U || config_.grid_rows == 0U) {
    throw std::invalid_argument("grid dimensions must be positive");
  }
  if (config_.min_near_tile_count < 2U ||
    config_.min_near_tile_count > config_.grid_columns * config_.grid_rows)
  {
    throw std::invalid_argument("min_near_tile_count must fit the configured grid");
  }
  if (config_.obstacle_frame_count == 0U ||
    config_.low_valid_frame_count == 0U || config_.clear_frame_count == 0U)
  {
    throw std::invalid_argument("frame confirmation counts must be positive");
  }
}

DepthEstopResult DepthEstopEvaluator::invalidFrameResult()
{
  consecutive_obstacle_frames_ = 0U;
  consecutive_low_valid_frames_ = 0U;
  forceEstop();
  return DepthEstopResult{
    true,
    false,
    false,
    false,
    0.0,
    0.0,
    0U,
    consecutive_obstacle_frames_,
    consecutive_low_valid_frames_,
    consecutive_clear_frames_,
    DepthEstopReason::kInvalidFrame,
  };
}

DepthEstopResult DepthEstopEvaluator::process(const sensor_msgs::msg::Image & image)
{
  constexpr std::size_t kBytesPerPixel = 2U;
  if (image.encoding != "16UC1" || image.width == 0U || image.height == 0U) {
    return invalidFrameResult();
  }

  const std::size_t width = image.width;
  const std::size_t height = image.height;
  if (image.step < width * kBytesPerPixel || image.data.size() < image.step * height) {
    return invalidFrameResult();
  }

  const auto normalizedX = [width](double value, bool upper) {
      const double pixels = value * static_cast<double>(width);
      const double rounded = upper ? std::ceil(pixels) : std::floor(pixels);
      return std::min(width, static_cast<std::size_t>(std::max(0.0, rounded)));
    };
  const auto normalizedY = [height](double value, bool upper) {
      const double pixels = value * static_cast<double>(height);
      const double rounded = upper ? std::ceil(pixels) : std::floor(pixels);
      return std::min(height, static_cast<std::size_t>(std::max(0.0, rounded)));
    };

  const std::size_t x_begin = normalizedX(config_.roi_x_min, false);
  const std::size_t x_end = normalizedX(config_.roi_x_max, true);
  const std::size_t y_begin = normalizedY(config_.roi_y_min, false);
  const std::size_t y_end = normalizedY(config_.roi_y_max, true);
  if (x_begin >= x_end || y_begin >= y_end) {
    return invalidFrameResult();
  }

  const std::size_t roi_width = x_end - x_begin;
  const std::size_t roi_height = y_end - y_begin;
  const std::size_t roi_pixels = roi_width * roi_height;
  const std::size_t tile_count = config_.grid_columns * config_.grid_rows;
  std::vector<std::size_t> tile_pixels(tile_count, 0U);
  std::vector<std::size_t> tile_near_pixels(tile_count, 0U);
  std::size_t valid_pixels = 0U;
  std::size_t near_pixels = 0U;
  const double distance_threshold =
    estop_requested_ ? config_.release_distance_m : config_.stop_distance_m;

  for (std::size_t y = y_begin; y < y_end; ++y) {
    const std::size_t tile_y = std::min(
      config_.grid_rows - 1U,
      ((y - y_begin) * config_.grid_rows) / roi_height);
    for (std::size_t x = x_begin; x < x_end; ++x) {
      const std::size_t tile_x = std::min(
        config_.grid_columns - 1U,
        ((x - x_begin) * config_.grid_columns) / roi_width);
      const std::size_t tile_index = tile_y * config_.grid_columns + tile_x;
      ++tile_pixels[tile_index];

      const std::size_t offset = y * image.step + x * kBytesPerPixel;
      const std::uint16_t raw_depth = readDepth(image, offset);
      if (raw_depth == 0U) {
        continue;
      }
      ++valid_pixels;
      if (static_cast<double>(raw_depth) * config_.depth_scale_m <= distance_threshold) {
        ++tile_near_pixels[tile_index];
        ++near_pixels;
      }
    }
  }

  const double valid_fraction = static_cast<double>(valid_pixels) /
    static_cast<double>(roi_pixels);
  const double near_pixel_fraction = static_cast<double>(near_pixels) /
    static_cast<double>(roi_pixels);
  std::vector<bool> near_tiles(tile_count, false);
  std::size_t near_tile_count = 0U;
  for (std::size_t index = 0U; index < tile_count; ++index) {
    if (tile_pixels[index] == 0U) {
      continue;
    }
    const double near_fraction = static_cast<double>(tile_near_pixels[index]) /
      static_cast<double>(tile_pixels[index]);
    if (near_fraction >= config_.near_tile_fraction) {
      near_tiles[index] = true;
      ++near_tile_count;
    }
  }

  bool has_adjacent_near_tiles = false;
  for (std::size_t row = 0U; row < config_.grid_rows && !has_adjacent_near_tiles; ++row) {
    for (std::size_t column = 0U; column < config_.grid_columns; ++column) {
      const std::size_t index = row * config_.grid_columns + column;
      if (!near_tiles[index]) {
        continue;
      }
      const bool adjacent_right =
        column + 1U < config_.grid_columns && near_tiles[index + 1U];
      const bool adjacent_below =
        row + 1U < config_.grid_rows && near_tiles[index + config_.grid_columns];
      if (adjacent_right || adjacent_below) {
        has_adjacent_near_tiles = true;
        break;
      }
    }
  }

  const bool near_obstacle =
    near_tile_count >= config_.min_near_tile_count &&
    has_adjacent_near_tiles &&
    near_pixel_fraction >= config_.min_near_pixel_fraction;
  const bool low_valid_depth = valid_fraction < config_.min_valid_fraction;
  DepthEstopReason reason = DepthEstopReason::kNone;

  if (low_valid_depth) {
    consecutive_obstacle_frames_ = 0U;
    consecutive_clear_frames_ = 0U;
    ++consecutive_low_valid_frames_;
    if (valid_fraction < config_.severe_invalid_fraction ||
      consecutive_low_valid_frames_ >= config_.low_valid_frame_count)
    {
      forceEstop();
      reason = DepthEstopReason::kLowValidDepth;
    }
  } else {
    consecutive_low_valid_frames_ = 0U;
    if (near_obstacle) {
      consecutive_clear_frames_ = 0U;
      ++consecutive_obstacle_frames_;
      if (estop_requested_ ||
        consecutive_obstacle_frames_ >= config_.obstacle_frame_count)
      {
        forceEstop();
        reason = DepthEstopReason::kNearObstacle;
      }
    } else {
      consecutive_obstacle_frames_ = 0U;
      if (estop_requested_) {
        ++consecutive_clear_frames_;
        if (consecutive_clear_frames_ >= config_.clear_frame_count) {
          estop_requested_ = false;
        }
      } else {
        consecutive_clear_frames_ = config_.clear_frame_count;
      }
    }
  }

  return DepthEstopResult{
    estop_requested_,
    true,
    near_obstacle,
    low_valid_depth,
    valid_fraction,
    near_pixel_fraction,
    near_tile_count,
    consecutive_obstacle_frames_,
    consecutive_low_valid_frames_,
    consecutive_clear_frames_,
    reason,
  };
}

void DepthEstopEvaluator::forceEstop()
{
  estop_requested_ = true;
  consecutive_clear_frames_ = 0U;
}

bool DepthEstopEvaluator::estopRequested() const
{
  return estop_requested_;
}

}  // namespace traymover_robot_safety
