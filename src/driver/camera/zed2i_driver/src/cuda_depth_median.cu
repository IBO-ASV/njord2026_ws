#include <cuda_runtime.h>

#include "zed2i_driver/gpu_depth_median.hpp"

#include <cmath>

namespace zed2i_driver
{

namespace
{
constexpr int kBins = 1024;

__global__ void clear_histogram(unsigned int * histogram)
{
  const int index = blockIdx.x * blockDim.x + threadIdx.x;
  if (index < kBins) {
    histogram[index] = 0U;
  }
}

__global__ void histogram_depth(
  const float * depth, size_t pitch, int x0, int y0, int x1, int y1,
  int stride, float min_depth, float max_depth, unsigned int * histogram)
{
  const int sample_x = x0 + (blockIdx.x * blockDim.x + threadIdx.x) * stride;
  const int sample_y = y0 + (blockIdx.y * blockDim.y + threadIdx.y) * stride;
  if (sample_x >= x1 || sample_y >= y1) {
    return;
  }
  const auto * row = reinterpret_cast<const float *>(
    reinterpret_cast<const unsigned char *>(depth) + static_cast<size_t>(sample_y) * pitch);
  const float value = row[sample_x];
  if (!isfinite(value) || value < min_depth || value > max_depth) {
    return;
  }
  const float normalized = (value - min_depth) / (max_depth - min_depth);
  const int bin = min(kBins - 1, max(0, static_cast<int>(normalized * kBins)));
  atomicAdd(&histogram[bin], 1U);
}

__global__ void calculate_median(
  const unsigned int * histogram, float min_depth, float max_depth,
  int min_samples, float * result)
{
  if (blockIdx.x != 0 || threadIdx.x != 0) {
    return;
  }
  unsigned int count = 0U;
  for (int index = 0; index < kBins; ++index) {
    count += histogram[index];
  }
  if (count < static_cast<unsigned int>(min_samples)) {
    *result = NAN;
    return;
  }
  const unsigned int target = (count - 1U) / 2U;
  unsigned int cumulative = 0U;
  for (int index = 0; index < kBins; ++index) {
    cumulative += histogram[index];
    if (cumulative > target) {
      *result = min_depth +
        (static_cast<float>(index) + 0.5F) * (max_depth - min_depth) / kBins;
      return;
    }
  }
  *result = NAN;
}
}  // namespace

float depth_median_gpu(
  const float * depth_gpu, size_t pitch_bytes, int width, int height,
  int x0, int y0, int x1, int y1, int stride, float min_depth, float max_depth,
  int min_samples, cudaStream_t stream)
{
  if (!depth_gpu || width <= 0 || height <= 0 || stride <= 0 || min_samples <= 0 ||
    !std::isfinite(min_depth) || !std::isfinite(max_depth) || min_depth >= max_depth) {
    return NAN;
  }
  x0 = max(0, min(width, x0));
  x1 = max(0, min(width, x1));
  y0 = max(0, min(height, y0));
  y1 = max(0, min(height, y1));
  if (x1 <= x0 || y1 <= y0) {
    return NAN;
  }

  // The workspace is process-local and allocated once; frame processing only
  // transfers the final scalar to the host.
  static unsigned int * histogram = nullptr;
  static float * device_result = nullptr;
  if (!histogram && cudaMalloc(&histogram, kBins * sizeof(*histogram)) != cudaSuccess) {
    return NAN;
  }
  if (!device_result && cudaMalloc(&device_result, sizeof(*device_result)) != cudaSuccess) {
    return NAN;
  }
  clear_histogram<<<(kBins + 255) / 256, 256, 0, stream>>>(histogram);
  const dim3 threads(16, 16);
  const dim3 blocks(
    (static_cast<unsigned int>((x1 - x0 + stride - 1) / stride) + threads.x - 1) / threads.x,
    (static_cast<unsigned int>((y1 - y0 + stride - 1) / stride) + threads.y - 1) / threads.y);
  histogram_depth<<<blocks, threads, 0, stream>>>(
    depth_gpu, pitch_bytes, x0, y0, x1, y1, stride, min_depth, max_depth, histogram);
  calculate_median<<<1, 1, 0, stream>>>(histogram, min_depth, max_depth, min_samples, device_result);
  float result = NAN;
  if (cudaMemcpyAsync(&result, device_result, sizeof(result), cudaMemcpyDeviceToHost, stream) != cudaSuccess ||
    cudaStreamSynchronize(stream) != cudaSuccess) {
    return NAN;
  }
  return result;
}

}  // namespace zed2i_driver
