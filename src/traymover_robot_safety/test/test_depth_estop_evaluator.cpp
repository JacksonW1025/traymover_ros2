#include <algorithm>
#include <cstddef>
#include <cstdint>

#include "gtest/gtest.h"
#include "sensor_msgs/msg/image.hpp"
#include "traymover_robot_safety/depth_estop_evaluator.hpp"

namespace
{
using traymover_robot_safety::DepthEstopConfig;
using traymover_robot_safety::DepthEstopEvaluator;

sensor_msgs::msg::Image makeDepthImage(
  std::uint32_t width = 640U, std::uint32_t height = 480U,
  std::uint16_t depth_mm = 1240U)
{
  sensor_msgs::msg::Image image;
  image.width = width;
  image.height = height;
  image.encoding = "16UC1";
  image.is_bigendian = 0U;
  image.step = width * 2U;
  image.data.resize(static_cast<std::size_t>(image.step) * height);
  for (std::size_t offset = 0U; offset < image.data.size(); offset += 2U) {
    image.data[offset] = static_cast<std::uint8_t>(depth_mm & 0xFFU);
    image.data[offset + 1U] = static_cast<std::uint8_t>(depth_mm >> 8U);
  }
  return image;
}

void setDepth(sensor_msgs::msg::Image & image, std::size_t x, std::size_t y, std::uint16_t depth)
{
  const std::size_t offset = y * image.step + x * 2U;
  image.data[offset] = static_cast<std::uint8_t>(depth & 0xFFU);
  image.data[offset + 1U] = static_cast<std::uint8_t>(depth >> 8U);
}

void fillRectangle(
  sensor_msgs::msg::Image & image, std::size_t x_begin, std::size_t x_end,
  std::size_t y_begin, std::size_t y_end, std::uint16_t depth)
{
  for (std::size_t y = y_begin; y < y_end; ++y) {
    for (std::size_t x = x_begin; x < x_end; ++x) {
      setDepth(image, x, y, depth);
    }
  }
}

TEST(DepthEstopEvaluatorTest, ReferenceLikeFrameBecomesFreeAfterEightFrames)
{
  DepthEstopEvaluator evaluator(DepthEstopConfig{});
  auto image = makeDepthImage();

  // The sampled reference image had roughly 23.3% invalid pixels in the ROI.
  fillRectangle(image, 160U, 320U, 96U, 241U, 0U);
  for (int frame = 0; frame < 7; ++frame) {
    EXPECT_TRUE(evaluator.process(image).estop_requested);
  }
  const auto result = evaluator.process(image);
  EXPECT_TRUE(result.frame_valid);
  EXPECT_GT(result.valid_fraction, 0.50);
  EXPECT_FALSE(result.estop_requested);
}

TEST(DepthEstopEvaluatorTest, BroadCloseObstacleRequiresTwoFrames)
{
  DepthEstopEvaluator evaluator(DepthEstopConfig{});
  auto image = makeDepthImage();
  for (int frame = 0; frame < 8; ++frame) {
    evaluator.process(image);
  }
  ASSERT_FALSE(evaluator.estopRequested());

  fillRectangle(image, 280U, 340U, 200U, 260U, 600U);
  const auto first_result = evaluator.process(image);
  EXPECT_TRUE(first_result.near_obstacle);
  EXPECT_FALSE(first_result.estop_requested);
  EXPECT_EQ(first_result.consecutive_obstacle_frames, 1U);

  const auto result = evaluator.process(image);
  EXPECT_TRUE(result.near_obstacle);
  EXPECT_TRUE(result.estop_requested);
  EXPECT_EQ(result.reason, traymover_robot_safety::DepthEstopReason::kNearObstacle);
}

TEST(DepthEstopEvaluatorTest, SparseNearNoiseDoesNotTrigger)
{
  DepthEstopEvaluator evaluator(DepthEstopConfig{});
  auto image = makeDepthImage();
  for (int frame = 0; frame < 8; ++frame) {
    evaluator.process(image);
  }

  for (std::size_t index = 0U; index < 20U; ++index) {
    setDepth(image, 300U + index % 10U, 220U + index / 10U, 500U);
  }
  EXPECT_FALSE(evaluator.process(image).estop_requested);
}

TEST(DepthEstopEvaluatorTest, SingleNearTileDoesNotTrigger)
{
  DepthEstopEvaluator evaluator(DepthEstopConfig{});
  auto image = makeDepthImage();
  for (int frame = 0; frame < 8; ++frame) {
    evaluator.process(image);
  }

  fillRectangle(image, 300U, 320U, 200U, 226U, 500U);
  for (int frame = 0; frame < 4; ++frame) {
    const auto result = evaluator.process(image);
    EXPECT_EQ(result.near_tile_count, 1U);
    EXPECT_FALSE(result.near_obstacle);
    EXPECT_FALSE(result.estop_requested);
  }
}

TEST(DepthEstopEvaluatorTest, NonAdjacentNearTilesDoNotTrigger)
{
  DepthEstopEvaluator evaluator(DepthEstopConfig{});
  auto image = makeDepthImage();
  for (int frame = 0; frame < 8; ++frame) {
    evaluator.process(image);
  }

  fillRectangle(image, 200U, 220U, 148U, 174U, 500U);
  fillRectangle(image, 400U, 420U, 330U, 356U, 500U);
  for (int frame = 0; frame < 4; ++frame) {
    const auto result = evaluator.process(image);
    EXPECT_EQ(result.near_tile_count, 2U);
    EXPECT_GT(result.near_pixel_fraction, 0.005);
    EXPECT_FALSE(result.near_obstacle);
    EXPECT_FALSE(result.estop_requested);
  }
}

TEST(DepthEstopEvaluatorTest, SingleBroadObstacleFrameDoesNotTrigger)
{
  DepthEstopEvaluator evaluator(DepthEstopConfig{});
  auto image = makeDepthImage();
  for (int frame = 0; frame < 8; ++frame) {
    evaluator.process(image);
  }

  fillRectangle(image, 280U, 340U, 200U, 260U, 600U);
  EXPECT_FALSE(evaluator.process(image).estop_requested);

  image = makeDepthImage();
  EXPECT_FALSE(evaluator.process(image).estop_requested);
  EXPECT_EQ(evaluator.process(image).consecutive_obstacle_frames, 0U);
}

TEST(DepthEstopEvaluatorTest, ModerateInvalidDepthRequiresTwoFrames)
{
  DepthEstopEvaluator evaluator(DepthEstopConfig{});
  auto image = makeDepthImage();
  for (int frame = 0; frame < 8; ++frame) {
    evaluator.process(image);
  }

  fillRectangle(image, 160U, 480U, 96U, 300U, 0U);
  const auto first_result = evaluator.process(image);
  EXPECT_LT(first_result.valid_fraction, 0.50);
  EXPECT_GT(first_result.valid_fraction, 0.20);
  EXPECT_FALSE(first_result.estop_requested);
  EXPECT_EQ(first_result.consecutive_low_valid_frames, 1U);

  const auto result = evaluator.process(image);
  EXPECT_LT(result.valid_fraction, 0.50);
  EXPECT_TRUE(result.estop_requested);
  EXPECT_EQ(result.reason, traymover_robot_safety::DepthEstopReason::kLowValidDepth);
}

TEST(DepthEstopEvaluatorTest, SevereInvalidDepthTriggersImmediately)
{
  DepthEstopEvaluator evaluator(DepthEstopConfig{});
  auto image = makeDepthImage();
  for (int frame = 0; frame < 8; ++frame) {
    evaluator.process(image);
  }

  fillRectangle(image, 160U, 480U, 96U, 360U, 0U);
  const auto result = evaluator.process(image);
  EXPECT_LT(result.valid_fraction, 0.20);
  EXPECT_TRUE(result.estop_requested);
  EXPECT_EQ(result.reason, traymover_robot_safety::DepthEstopReason::kLowValidDepth);
}

TEST(DepthEstopEvaluatorTest, ReleaseUsesDistanceHysteresisAndEightClearFrames)
{
  DepthEstopEvaluator evaluator(DepthEstopConfig{});
  auto image = makeDepthImage(640U, 480U, 900U);
  EXPECT_TRUE(evaluator.process(image).estop_requested);

  image = makeDepthImage(640U, 480U, 1100U);
  for (int frame = 0; frame < 7; ++frame) {
    EXPECT_TRUE(evaluator.process(image).estop_requested);
  }
  EXPECT_FALSE(evaluator.process(image).estop_requested);
}

TEST(DepthEstopEvaluatorTest, InvalidEncodingForcesEstop)
{
  DepthEstopEvaluator evaluator(DepthEstopConfig{});
  auto image = makeDepthImage();
  for (int frame = 0; frame < 8; ++frame) {
    evaluator.process(image);
  }
  image.encoding = "32FC1";
  const auto result = evaluator.process(image);
  EXPECT_FALSE(result.frame_valid);
  EXPECT_TRUE(result.estop_requested);
}
}  // namespace
