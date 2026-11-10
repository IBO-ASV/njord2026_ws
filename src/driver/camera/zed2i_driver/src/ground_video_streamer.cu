#include "zed2i_driver/ground_video_streamer.hpp"

#include <cuda_runtime_api.h>
#include <gst/app/gstappsrc.h>
#include <gst/gst.h>

#include <algorithm>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <cstdio>
#include <mutex>
#include <stdexcept>
#include <thread>
#include <utility>
#include <vector>

// This path does not use the libnvjpeg CUDA library. On the target Jetson
// Orin Nano Super image (Driver 595.78, CUDA 13.2, L4T 39.2), nvjpegCreate()/
// nvjpegCreateSimple() return NVJPEG_STATUS_EXECUTION_FAILED (status 6) for
// every backend (0-5), confirmed on-device with a minimal repro
// (/tmp/nvjpeg_repro.cu on the Jetson). The device has no /dev/nvhost-nvjpg*
// node at all (`ls /dev/nvhost-nvjpg*` -> no such file), so the hardware
// NVJPEG engine is not exposed to userspace on this image/kernel
// (6.8.12-1021-tegra); this is a platform/image issue, not an application
// bug, and every further nvjpeg call on the resulting handle just propagates
// failure (observed as NVJPEG_STATUS_INVALID_PARAMETER on a later run; an
// earlier run instead hit SIGILL inside libnvjpeg on the same bad handle --
// both are consistent with "the handle nvjpegCreate returned is not valid").
// Encoding is instead delegated to the GStreamer `nvjpegenc` element
// (gst-plugins-good, package libgstnvjpeg.so, confirmed present via
// `gst-inspect-1.0 nvjpegenc` on-device), which drives the Tegra NVJPG
// hardware block directly and is unaffected by the broken libnvjpeg entry
// points. Its sink pad only accepts I420/NV12 (4:2:0), which happens to be
// exactly the subsampling `rtpjpegpay` (RFC 2435) requires -- payloading a
// 4:4:4 JPEG produces "Invalid component" and drops every frame -- so the
// pipeline below (I420 all the way to nvjpegenc) satisfies that constraint
// by construction. This file therefore only uses CUDA for the cheap GPU-side
// resize + BGRA-to-I420 conversion and the device-to-host copy of that
// downscaled I420 frame; appsrc feeds it directly to nvvidconv/nvjpegenc.

namespace zed2i_driver
{
namespace
{

void check_cuda(cudaError_t result, const char * operation)
{
  if (result != cudaSuccess) {
    throw std::runtime_error(std::string(operation) + ": " + cudaGetErrorString(result));
  }
}

__device__ std::uint8_t clamp_byte(int value)
{
  return static_cast<std::uint8_t>(max(0, min(255, value)));
}

// 3x5 glyphs keep the range readable after the 360px ground-video resize.
__device__ unsigned short glyph(char value)
{
  switch (value) {
    case '0': return 0b111101101101111;
    case '1': return 0b010110010010111;
    case '2': return 0b111001111100111;
    case '3': return 0b111001111001111;
    case '4': return 0b101101111001001;
    case '5': return 0b111100111001111;
    case '6': return 0b111100111101111;
    case '7': return 0b111001010010010;
    case '8': return 0b111101111101111;
    case '9': return 0b111101111001111;
    case 'Z': return 0b111001010100111;
    case 'L': return 0b100100100100111;
    case 'm': return 0b000110101101101;
    case '.': return 0b000000000000010;
    case '?': return 0b111001010000010;
    default: return 0;
  }
}

__device__ bool label_pixel(
  const GroundVideoBox & box, int x, int y,
  int destination_width, int destination_height, int source_width, int source_height)
{
  const int x1 = static_cast<int>(box.x1 * destination_width / source_width);
  const int y1 = static_cast<int>(box.y1 * destination_height / source_height);
  const int label_y = max(0, y1 - 12);
  if (y < label_y || y >= label_y + 10 || x < x1) {
    return false;
  }
  const int character = (x - x1) / 8;
  if (character >= 15 || box.label[character] == '\0') {
    return false;
  }
  const int glyph_x = ((x - x1) % 8) / 2;
  const int glyph_y = (y - label_y) / 2;
  return glyph_x < 3 && glyph_y < 5 &&
         (glyph(box.label[character]) & (1 << (14 - glyph_y * 3 - glyph_x))) != 0;
}

__device__ void load_resized_bgr(
  const std::uint8_t * source, std::size_t source_pitch, int source_width, int source_height,
  int destination_width, int destination_height, int x, int y, int & b, int & g, int & r)
{
  const int source_x = x * source_width / destination_width;
  const int source_y = y * source_height / destination_height;
  const auto * input = source + static_cast<std::size_t>(source_y) * source_pitch + source_x * 4;
  b = input[0];
  g = input[1];
  r = input[2];
}

__global__ void resize_bgra_to_i420(
  const std::uint8_t * source, std::size_t source_pitch, int source_width, int source_height,
  std::uint8_t * destination, int destination_width, int destination_height,
  const GroundVideoBox * boxes, int box_count)
{
  const int x = blockIdx.x * blockDim.x + threadIdx.x;
  const int y = blockIdx.y * blockDim.y + threadIdx.y;
  if (x >= destination_width || y >= destination_height) {
    return;
  }
  int b = 0;
  int g = 0;
  int r = 0;
  load_resized_bgr(
    source, source_pitch, source_width, source_height,
    destination_width, destination_height, x, y, b, g, r);
  for (int i = 0; i < box_count; ++i) {
    const auto & box = boxes[i];
    const int x1 = static_cast<int>(box.x1 * destination_width / source_width);
    const int x2 = static_cast<int>(box.x2 * destination_width / source_width);
    const int y1 = static_cast<int>(box.y1 * destination_height / source_height);
    const int y2 = static_cast<int>(box.y2 * destination_height / source_height);
    if ((x >= x1 && x <= x2 && y >= y1 && y <= y2 &&
      (x - x1 < 2 || x2 - x < 2 || y - y1 < 2 || y2 - y < 2)) ||
      label_pixel(
        box, x, y, destination_width, destination_height, source_width, source_height))
    {
      b = 0;
      g = 255;
      r = 0;
      break;
    }
  }
  destination[static_cast<std::size_t>(y) * destination_width + x] =
    clamp_byte(((66 * r + 129 * g + 25 * b + 128) >> 8) + 16);

  if ((x & 1) != 0 || (y & 1) != 0) {
    return;
  }
  int sum_b = 0;
  int sum_g = 0;
  int sum_r = 0;
  for (int dy = 0; dy < 2; ++dy) {
    for (int dx = 0; dx < 2; ++dx) {
      load_resized_bgr(
        source, source_pitch, source_width, source_height,
        destination_width, destination_height, x + dx, y + dy, b, g, r);
      sum_b += b;
      sum_g += g;
      sum_r += r;
    }
  }
  b = (sum_b + 2) / 4;
  g = (sum_g + 2) / 4;
  r = (sum_r + 2) / 4;
  const std::size_t luma_bytes =
    static_cast<std::size_t>(destination_width) * destination_height;
  const std::size_t chroma_index =
    static_cast<std::size_t>(y / 2) * (destination_width / 2) + x / 2;
  destination[luma_bytes + chroma_index] =
    clamp_byte(((-38 * r - 74 * g + 112 * b + 128) >> 8) + 128);
  destination[luma_bytes + luma_bytes / 4 + chroma_index] =
    clamp_byte(((112 * r - 94 * g - 18 * b + 128) >> 8) + 128);
}

}  // namespace

class GroundVideoStreamer::Impl
{
public:
  explicit Impl(const GroundVideoConfig & config)
  : config_(config), period_(std::chrono::duration<double>(1.0 / config.fps))
  {
    validate();
    gst_init(nullptr, nullptr);
    create_pipeline();
    try {
      check_cuda(cudaStreamCreateWithFlags(&stream_, cudaStreamNonBlocking), "cudaStreamCreateWithFlags");
      slots_.resize(static_cast<std::size_t>(config_.max_pending_frames + 1));
      for (auto & slot : slots_) {
        check_cuda(cudaMalloc(&slot.bgra, max_bgra_bytes()), "cudaMalloc BGRA slot");
        check_cuda(cudaMalloc(&slot.i420, i420_bytes()), "cudaMalloc I420 slot");
        check_cuda(cudaMalloc(&slot.boxes, sizeof(GroundVideoBox) * 32), "cudaMalloc box slot");
        check_cuda(cudaMallocHost(&slot.host, i420_bytes()), "cudaMallocHost I420 slot");
        check_cuda(cudaEventCreateWithFlags(&slot.ready, cudaEventDisableTiming), "cudaEventCreateWithFlags");
      }
      worker_ = std::thread(&Impl::run, this);
    } catch (...) {
      release_cuda();
      release_pipeline();
      throw;
    }
  }

  ~Impl()
  {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      stopping_ = true;
    }
    pending_.notify_all();
    if (worker_.joinable()) {
      worker_.join();
    }
    release_pipeline();
    release_cuda();
  }

  void submit(
    const std::uint8_t * bgra_device, std::size_t bgra_pitch, int width, int height,
    const std::vector<GroundVideoBox> & boxes)
  {
    if (bgra_device == nullptr || width != config_.source_width || height != config_.source_height) {
      return;
    }
    const auto now = std::chrono::steady_clock::now();
    std::lock_guard<std::mutex> lock(mutex_);
    if (stopping_ || failed_ || (last_selected_ != std::chrono::steady_clock::time_point{} &&
      now - last_selected_ < period_))
    {
      return;
    }
    auto slot = std::find_if(slots_.begin(), slots_.end(), [](const Slot & candidate) {
      return candidate.state == State::kFree;
    });
    if (slot == slots_.end()) {
      // A pending slot has not entered nvJPEG yet.  The private CUDA stream
      // serializes the replacement D2D copy after the old one, so replacing it
      // is safe and keeps the queue latest-wins.
      slot = std::find_if(slots_.begin(), slots_.end(), [](const Slot & candidate) {
        return candidate.state == State::kPending;
      });
      if (slot == slots_.end()) {
        ++dropped_no_slot_;
        return;
      }
      ++dropped_pending_;
    }
    try {
      check_cuda(cudaMemcpy2DAsync(
          slot->bgra, static_cast<std::size_t>(width) * 4, bgra_device, bgra_pitch,
          static_cast<std::size_t>(width) * 4, static_cast<std::size_t>(height),
          cudaMemcpyDeviceToDevice, stream_), "cudaMemcpy2DAsync stream slot");
      slot->box_count = std::min(boxes.size(), static_cast<std::size_t>(32));
      if (slot->box_count > 0) {
        check_cuda(cudaMemcpyAsync(
            slot->boxes, boxes.data(), slot->box_count * sizeof(GroundVideoBox),
            cudaMemcpyHostToDevice, stream_), "cudaMemcpyAsync boxes");
      }
      check_cuda(cudaEventRecord(slot->ready, stream_), "cudaEventRecord stream slot");
    } catch (const std::exception &) {
      failed_ = true;
      return;
    }
    slot->source_width = width;
    slot->source_height = height;
    slot->capture_time = now;
    slot->state = State::kPending;
    last_selected_ = now;
    ++selected_;
    pending_.notify_one();
  }

private:
  enum class State {kFree, kPending, kEncoding, kInFlight};
  struct Slot
  {
    std::uint8_t * bgra{nullptr};
    std::uint8_t * i420{nullptr};
    GroundVideoBox * boxes{nullptr};
    std::size_t box_count{};
    std::uint8_t * host{nullptr};
    cudaEvent_t ready{nullptr};
    int source_width{};
    int source_height{};
    std::chrono::steady_clock::time_point capture_time{};
    State state{State::kFree};
  };
  struct BufferLease
  {
    Impl * owner;
    std::size_t slot_index;
  };

  void validate() const
  {
    if (config_.host.empty() || config_.source_width <= 0 || config_.source_height <= 0 ||
      config_.width <= 0 || config_.height <= 0 || (config_.width & 1) != 0 ||
      (config_.height & 1) != 0 || config_.fps <= 0.0 ||
      config_.jpeg_quality < 1 || config_.jpeg_quality > 100 || config_.port < 1 ||
      config_.port > 65535 || config_.max_pending_frames < 1 || config_.max_pending_frames > 3 ||
      config_.mtu < 256 || config_.mtu > 65507)
    {
      throw std::invalid_argument("invalid ground-video configuration");
    }
  }

  std::size_t max_bgra_bytes() const
  {
    return static_cast<std::size_t>(config_.source_width) * config_.source_height * 4;
  }
  std::size_t i420_bytes() const
  {
    return static_cast<std::size_t>(config_.width) * config_.height * 3 / 2;
  }

  void create_pipeline()
  {
    GError * error = nullptr;
    // CUDA produces I420 directly, so no CPU-side videoconvert is needed.
    // nvvidconv copies it into an NVMM buffer and nvjpegenc drives the Tegra
    // hardware JPEG encoder -- see the note above for why libnvjpeg is unused.
    const std::string description = "appsrc name=source is-live=true format=time block=false "
      "do-timestamp=false ! nvvidconv ! "
      "video/x-raw(memory:NVMM),format=I420 ! nvjpegenc quality=" +
      std::to_string(config_.jpeg_quality) + " ! jpegparse ! rtpjpegpay mtu=" +
      std::to_string(config_.mtu) + " ! udpsink host=" + config_.host + " port=" +
      std::to_string(config_.port) + " sync=false async=false";
    pipeline_ = gst_parse_launch(description.c_str(), &error);
    if (error != nullptr || pipeline_ == nullptr) {
      const std::string message = error == nullptr ? "unknown GStreamer error" : error->message;
      if (error != nullptr) {
        g_error_free(error);
      }
      throw std::runtime_error("could not create ground-video pipeline: " + message);
    }
    appsrc_ = GST_APP_SRC(gst_bin_get_by_name(GST_BIN(pipeline_), "source"));
    if (appsrc_ == nullptr) {
      throw std::runtime_error("could not find ground-video appsrc element");
    }
    // Caps MUST be set before the pipeline transitions to PLAYING: appsrc
    // negotiates its source pad caps as part of that state change, and if
    // they are still unset at that point the negotiation can fail silently
    // (no error, but zero frames ever reach the sink) or lock in caps that
    // downstream elements reject. Setting them first guarantees appsrc
    // advertises fixed caps from the very first buffer.
    int fps_numerator = 0;
    int fps_denominator = 1;
    gst_util_double_to_fraction(config_.fps, &fps_numerator, &fps_denominator);
    GstCaps * caps = gst_caps_new_simple(
      "video/x-raw", "format", G_TYPE_STRING, "I420", "width", G_TYPE_INT, config_.width,
      "height", G_TYPE_INT, config_.height, "framerate", GST_TYPE_FRACTION, fps_numerator,
      fps_denominator, nullptr);
    gst_app_src_set_caps(appsrc_, caps);
    gst_caps_unref(caps);
    if (gst_element_set_state(pipeline_, GST_STATE_PLAYING) == GST_STATE_CHANGE_FAILURE) {
      throw std::runtime_error("could not start ground-video pipeline");
    }
  }

  static void release_wrapped_buffer(gpointer data)
  {
    auto * lease = static_cast<BufferLease *>(data);
    auto * owner = lease->owner;
    std::lock_guard<std::mutex> lock(owner->mutex_);
    owner->slots_[lease->slot_index].state = State::kFree;
    delete lease;
    owner->pending_.notify_one();
  }

  void run()
  {
    while (true) {
      std::size_t index = 0;
      {
        std::unique_lock<std::mutex> lock(mutex_);
        pending_.wait(lock, [this] {
          return stopping_ || std::any_of(slots_.begin(), slots_.end(), [](const Slot & slot) {
            return slot.state == State::kPending;
          });
        });
        if (stopping_) {
          return;
        }
        auto slot = std::find_if(slots_.begin(), slots_.end(), [](const Slot & candidate) {
          return candidate.state == State::kPending;
        });
        index = static_cast<std::size_t>(std::distance(slots_.begin(), slot));
        slot->state = State::kEncoding;
      }
      try {
        encode(index);
      } catch (const std::exception &) {
        std::lock_guard<std::mutex> lock(mutex_);
        slots_[index].state = State::kFree;
        failed_ = true;
      }
    }
  }

  void encode(std::size_t index)
  {
    auto & slot = slots_[index];
    check_cuda(cudaEventSynchronize(slot.ready), "cudaEventSynchronize stream slot");
    const dim3 threads(16, 16);
    const dim3 blocks(
      static_cast<unsigned int>((config_.width + threads.x - 1) / threads.x),
      static_cast<unsigned int>((config_.height + threads.y - 1) / threads.y));
    resize_bgra_to_i420<<<blocks, threads, 0, stream_>>>(
      slot.bgra, static_cast<std::size_t>(slot.source_width) * 4, slot.source_width, slot.source_height,
      slot.i420, config_.width, config_.height, slot.boxes, static_cast<int>(slot.box_count));
    check_cuda(cudaGetLastError(), "resize_bgra_to_i420");
    // JPEG encoding itself happens downstream in the GStreamer pipeline
    // (nvvidconv -> nvjpegenc), because libnvjpeg is broken on this device
    // (see note above). This D2H copy of the packed I420
    // frame is the sole device-to-host transfer in the streaming path.
    const std::size_t bytes = i420_bytes();
    check_cuda(cudaMemcpyAsync(slot.host, slot.i420, bytes, cudaMemcpyDeviceToHost, stream_),
      "cudaMemcpyAsync I420 to host");
    check_cuda(cudaStreamSynchronize(stream_), "cudaStreamSynchronize I420 copy");
    auto * lease = new BufferLease{this, index};
    GstBuffer * buffer = gst_buffer_new_wrapped_full(
      GST_MEMORY_FLAG_READONLY, slot.host, bytes, 0, bytes, lease, release_wrapped_buffer);
    GST_BUFFER_PTS(buffer) = static_cast<GstClockTime>(
      std::chrono::duration_cast<std::chrono::nanoseconds>(slot.capture_time - started_).count());
    GST_BUFFER_DTS(buffer) = GST_BUFFER_PTS(buffer);
    GST_BUFFER_DURATION(buffer) = static_cast<GstClockTime>(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period_).count());
    {
      std::lock_guard<std::mutex> lock(mutex_);
      slot.state = State::kInFlight;
      ++encoded_;
    }
    if (gst_app_src_push_buffer(appsrc_, buffer) != GST_FLOW_OK) {
      // push_buffer consumes the GstBuffer, so its release callback frees the slot.
      std::lock_guard<std::mutex> lock(mutex_);
      ++network_errors_;
    }
  }

  void release_pipeline()
  {
    if (pipeline_ != nullptr) {
      gst_element_set_state(pipeline_, GST_STATE_NULL);
    }
    if (appsrc_ != nullptr) {
      gst_object_unref(appsrc_);
      appsrc_ = nullptr;
    }
    if (pipeline_ != nullptr) {
      gst_object_unref(pipeline_);
      pipeline_ = nullptr;
    }
  }

  void release_cuda()
  {
    for (auto & slot : slots_) {
      if (slot.ready != nullptr) {cudaEventDestroy(slot.ready);}
      if (slot.host != nullptr) {cudaFreeHost(slot.host);}
      if (slot.i420 != nullptr) {cudaFree(slot.i420);}
      if (slot.boxes != nullptr) {cudaFree(slot.boxes);}
      if (slot.bgra != nullptr) {cudaFree(slot.bgra);}
    }
    if (stream_ != nullptr) {cudaStreamDestroy(stream_);}
  }

  GroundVideoConfig config_;
  std::chrono::duration<double> period_;
  const std::chrono::steady_clock::time_point started_{std::chrono::steady_clock::now()};
  std::chrono::steady_clock::time_point last_selected_{};
  std::mutex mutex_;
  std::condition_variable pending_;
  bool stopping_{false};
  bool failed_{false};
  std::thread worker_;
  std::vector<Slot> slots_;
  cudaStream_t stream_{nullptr};
  GstElement * pipeline_{nullptr};
  GstAppSrc * appsrc_{nullptr};
  std::uint64_t selected_{0};
  std::uint64_t encoded_{0};
  std::uint64_t dropped_no_slot_{0};
  std::uint64_t dropped_pending_{0};
  std::uint64_t network_errors_{0};
};

GroundVideoStreamer::GroundVideoStreamer(const GroundVideoConfig & config)
: impl_(std::make_unique<Impl>(config))
{
}

GroundVideoStreamer::~GroundVideoStreamer() = default;

void GroundVideoStreamer::submit(
  const std::uint8_t * bgra_device, std::size_t bgra_pitch, int width, int height,
  const std::vector<GroundVideoBox> & boxes)
{
  impl_->submit(bgra_device, bgra_pitch, width, height, boxes);
}

}  // namespace zed2i_driver
