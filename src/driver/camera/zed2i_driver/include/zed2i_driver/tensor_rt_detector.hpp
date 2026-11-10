#pragma once

#include "zed2i_driver/perception_types.hpp"

#include <memory>
#include <string>
#include <vector>

struct CUstream_st;
using cudaStream_t = CUstream_st *;

namespace zed2i_driver
{

class TensorRtDetector
{
public:
  TensorRtDetector(const std::string & engine_path, float confidence_threshold, int max_detections);
  ~TensorRtDetector();
  TensorRtDetector(TensorRtDetector &&) noexcept;
  TensorRtDetector & operator=(TensorRtDetector &&) noexcept;
  TensorRtDetector(const TensorRtDetector &) = delete;
  TensorRtDetector & operator=(const TensorRtDetector &) = delete;

  std::vector<Detection2D> infer(
    const void * bgra_gpu, size_t pitch_bytes, int width, int height, cudaStream_t stream);

private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace zed2i_driver
