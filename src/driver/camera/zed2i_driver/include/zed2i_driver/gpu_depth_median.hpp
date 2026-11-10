#pragma once

#include <cstddef>

struct CUstream_st;
using cudaStream_t = CUstream_st *;

namespace zed2i_driver
{

// Computes a validity-filtered, histogram median directly from a ZED GPU
// depth image. Only the resulting scalar is copied back to the host.
float depth_median_gpu(
  const float * depth_gpu, size_t pitch_bytes, int width, int height,
  int x0, int y0, int x1, int y1, int stride, float min_depth, float max_depth,
  int min_samples, cudaStream_t stream);

}  // namespace zed2i_driver
