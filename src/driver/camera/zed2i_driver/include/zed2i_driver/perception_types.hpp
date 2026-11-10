#pragma once

#include <array>
#include <cstdint>
#include <limits>

namespace zed2i_driver
{

struct Detection2D
{
  int class_id{};
  float confidence{};
  float x1{};
  float y1{};
  float x2{};
  float y2{};
};

enum class PositionSource : uint8_t
{
  kNone = 0,
  kZedDepth = 1,
  kLidarFused = 2,
};

inline std::array<float, 3> nan_position()
{
  const auto nan = std::numeric_limits<float>::quiet_NaN();
  return {nan, nan, nan};
}

struct PositionedDetection
{
  Detection2D detection;
  std::array<float, 3> position_base{nan_position()};
  PositionSource source{PositionSource::kNone};
};

struct LetterboxTransform
{
  float scale{1.0F};
  float pad_x{};
  float pad_y{};
  int source_width{};
  int source_height{};
};

struct DepthRoi
{
  int x0{};
  int y0{};
  int x1{};
  int y1{};
};

struct Point3f
{
  float x{};
  float y{};
  float z{};
};

struct LidarCluster
{
  Point3f centroid;
  int point_count{};
  float range{};
  float ray_angle_rad{};
};

struct FusionPolicy
{
  float max_range_delta_m{2.0F};
  float max_fallback_angle_rad{0.0F};
};

}  // namespace zed2i_driver
