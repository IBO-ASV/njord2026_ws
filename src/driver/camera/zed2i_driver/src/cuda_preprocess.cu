#include <cuda_runtime.h>

#include <cstdint>

namespace zed2i_driver
{
namespace
{
__global__ void bgra_to_nchw_kernel(
  const uint8_t * source, size_t pitch, float * output, int width, int height,
  float scale, float pad_x, float pad_y)
{
  const int x = blockIdx.x * blockDim.x + threadIdx.x;
  const int y = blockIdx.y * blockDim.y + threadIdx.y;
  if (x >= 640 || y >= 640) { return; }
  const float source_x = (static_cast<float>(x) - pad_x) / scale;
  const float source_y = (static_cast<float>(y) - pad_y) / scale;
  float r = 114.0F, g = 114.0F, b = 114.0F;
  if (source_x >= 0.0F && source_y >= 0.0F && source_x < width && source_y < height) {
    const int sx = min(width - 1, max(0, static_cast<int>(source_x)));
    const int sy = min(height - 1, max(0, static_cast<int>(source_y)));
    const auto * pixel = source + static_cast<size_t>(sy) * pitch + static_cast<size_t>(sx) * 4;
    b = static_cast<float>(pixel[0]);
    g = static_cast<float>(pixel[1]);
    r = static_cast<float>(pixel[2]);
  }
  const size_t plane = 640U * 640U;
  const size_t index = static_cast<size_t>(y) * 640U + static_cast<size_t>(x);
  output[index] = r / 255.0F;
  output[plane + index] = g / 255.0F;
  output[2U * plane + index] = b / 255.0F;
}
}  // namespace

void preprocess_bgra_to_nchw(
  const void * bgra_gpu, size_t pitch_bytes, int width, int height,
  float scale, float pad_x, float pad_y, float * output, cudaStream_t stream)
{
  const dim3 blocks((640 + 15) / 16, (640 + 15) / 16);
  const dim3 threads(16, 16);
  bgra_to_nchw_kernel<<<blocks, threads, 0, stream>>>(
    static_cast<const uint8_t *>(bgra_gpu), pitch_bytes, output, width, height, scale, pad_x, pad_y);
}
}  // namespace zed2i_driver
