#include "zed2i_driver/tensor_rt_detector.hpp"

#include "zed2i_driver/perception_logic.hpp"

#include <NvInfer.h>
#include <NvInferVersion.h>
#include <cuda_runtime_api.h>

#include <fstream>
#include <stdexcept>

namespace zed2i_driver
{
void preprocess_bgra_to_nchw(
  const void *, size_t, int, int, float, float, float, float *, cudaStream_t);

namespace
{
class Logger final : public nvinfer1::ILogger
{
public:
  void log(Severity severity, const char * message) noexcept override
  {
    if (severity <= Severity::kERROR) { fprintf(stderr, "TensorRT: %s\n", message); }
  }
};
Logger logger;

class TrtDeleter
{
public:
  template<typename T> void operator()(T * pointer) const
  {
    if (!pointer) {
      return;
    }
#if NV_TENSORRT_MAJOR >= 10
    // TensorRT 10 removed the legacy destroy() methods and exposes virtual
    // destructors for these runtime-owned objects.
    delete pointer;
#else
    pointer->destroy();
#endif
  }
};
}  // namespace

struct TensorRtDetector::Impl
{
  std::unique_ptr<nvinfer1::IRuntime, TrtDeleter> runtime;
  std::unique_ptr<nvinfer1::ICudaEngine, TrtDeleter> engine;
  std::unique_ptr<nvinfer1::IExecutionContext, TrtDeleter> context;
  void * input{};
  void * output{};
  size_t input_bytes{};
  size_t output_bytes{};
  int output_count{};
  bool channels_first{};
  bool raw_yolo_world_output{};
  float threshold{};
  int max_detections{};
  std::vector<float> host_output;
  LetterboxTransform letterbox;

  Impl(const std::string & path, float threshold_in, int max_detections_in)
  : threshold(threshold_in), max_detections(max_detections_in)
  {
    std::ifstream stream(path, std::ios::binary);
    if (!stream) { throw std::runtime_error("cannot read TensorRT engine: " + path); }
    stream.seekg(0, std::ios::end);
    const auto size = static_cast<size_t>(stream.tellg());
    stream.seekg(0);
    std::vector<char> bytes(size);
    stream.read(bytes.data(), static_cast<std::streamsize>(size));
    runtime.reset(nvinfer1::createInferRuntime(logger));
    engine.reset(runtime->deserializeCudaEngine(bytes.data(), size));
    if (!engine) { throw std::runtime_error("TensorRT engine deserialization failed"); }
    if (engine->getNbIOTensors() != 2) { throw std::runtime_error("engine must have one input and one output"); }
    const char * input_name = nullptr; const char * output_name = nullptr;
    for (int i = 0; i < engine->getNbIOTensors(); ++i) {
      const char * name = engine->getIOTensorName(i);
      if (engine->getTensorIOMode(name) == nvinfer1::TensorIOMode::kINPUT) input_name = name;
      else output_name = name;
    }
    if (!input_name || !output_name) throw std::runtime_error("invalid TensorRT IO contract");
    const auto input_shape = engine->getTensorShape(input_name);
    if (input_shape.nbDims != 4 || input_shape.d[0] != 1 || input_shape.d[1] != 3 ||
      input_shape.d[2] != 640 || input_shape.d[3] != 640) throw std::runtime_error("input must be [1,3,640,640]");
    if (engine->getTensorDataType(input_name) != nvinfer1::DataType::kFLOAT) {
      throw std::runtime_error("only FP32 input is currently supported by the CUDA preprocessor");
    }
    const auto output_shape = engine->getTensorShape(output_name);
    if (output_shape.nbDims != 3 || output_shape.d[0] != 1 ||
      ((output_shape.d[1] != 6 && output_shape.d[2] != 6) &&
      (output_shape.d[1] != 5 && output_shape.d[2] != 5))) {
      throw std::runtime_error("output must be [1,N,6], [1,6,N], [1,N,5], or [1,5,N]");
    }
    raw_yolo_world_output = output_shape.d[1] == 5 || output_shape.d[2] == 5;
    channels_first = output_shape.d[1] == (raw_yolo_world_output ? 5 : 6);
    output_count = channels_first ? output_shape.d[2] : output_shape.d[1];
    if (output_count <= 0 || engine->getTensorDataType(output_name) != nvinfer1::DataType::kFLOAT) {
      throw std::runtime_error("output must be fixed-size FP32");
    }
    input_bytes = 3U * 640U * 640U * sizeof(float);
    output_bytes = static_cast<size_t>(output_count) * (raw_yolo_world_output ? 5U : 6U) * sizeof(float);
    if (cudaMalloc(&input, input_bytes) != cudaSuccess || cudaMalloc(&output, output_bytes) != cudaSuccess) {
      throw std::runtime_error("CUDA buffer allocation failed");
    }
    context.reset(engine->createExecutionContext());
    if (!context) throw std::runtime_error("TensorRT execution context creation failed");
    context->setTensorAddress(input_name, input); context->setTensorAddress(output_name, output);
    host_output.resize(static_cast<size_t>(output_count) * (raw_yolo_world_output ? 5U : 6U));
  }

  ~Impl() { cudaFree(input); cudaFree(output); }
};

TensorRtDetector::TensorRtDetector(const std::string & path, float threshold, int max_detections)
: impl_(std::make_unique<Impl>(path, threshold, max_detections)) {}
TensorRtDetector::~TensorRtDetector() = default;
TensorRtDetector::TensorRtDetector(TensorRtDetector &&) noexcept = default;
TensorRtDetector & TensorRtDetector::operator=(TensorRtDetector &&) noexcept = default;

std::vector<Detection2D> TensorRtDetector::infer(
  const void * bgra, size_t pitch, int width, int height, cudaStream_t stream)
{
  impl_->letterbox = {std::min(640.0F / width, 640.0F / height), 0.0F, 0.0F, width, height};
  const float scaled_w = width * impl_->letterbox.scale;
  const float scaled_h = height * impl_->letterbox.scale;
  impl_->letterbox.pad_x = (640.0F - scaled_w) * 0.5F;
  impl_->letterbox.pad_y = (640.0F - scaled_h) * 0.5F;
  preprocess_bgra_to_nchw(bgra, pitch, width, height, impl_->letterbox.scale,
    impl_->letterbox.pad_x, impl_->letterbox.pad_y, static_cast<float *>(impl_->input), stream);
  if (!impl_->context->enqueueV3(stream)) throw std::runtime_error("TensorRT enqueue failed");
  if (cudaMemcpyAsync(impl_->host_output.data(), impl_->output, impl_->output_bytes,
    cudaMemcpyDeviceToHost, stream) != cudaSuccess) throw std::runtime_error("TensorRT output copy failed");
  if (cudaStreamSynchronize(stream) != cudaSuccess) throw std::runtime_error("CUDA stream synchronization failed");
  if (impl_->raw_yolo_world_output) {
    return decode_yolo_world_raw_detections(impl_->host_output, impl_->output_count, impl_->channels_first,
      impl_->threshold, impl_->letterbox, impl_->max_detections);
  }
  return decode_detections(impl_->host_output, impl_->output_count, impl_->channels_first,
    impl_->threshold, impl_->letterbox, impl_->max_detections);
}

}  // namespace zed2i_driver
