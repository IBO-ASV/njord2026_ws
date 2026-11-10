#ifndef ZED2I_DRIVER__GROUND_VIDEO_STREAMER_HPP_
#define ZED2I_DRIVER__GROUND_VIDEO_STREAMER_HPP_

#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

namespace zed2i_driver
{

struct GroundVideoConfig
{
  int source_width{};
  int source_height{};
  int width{640};
  int height{360};
  double fps{5.0};
  int jpeg_quality{70};
  std::string host;
  int port{5600};
  int max_pending_frames{1};
  int mtu{1200};
  // Human-FOV-like elliptical region of interest. When enabled, pixels
  // outside the ellipse (defined as ratios of the *source* frame) are
  // blacked out on-GPU before JPEG encoding, matching the mask applied to
  // the other RGB-D outputs.
  bool fov_ellipse_enable{false};
  double fov_ellipse_cx_ratio{0.5};
  double fov_ellipse_cy_ratio{0.5};
  double fov_ellipse_a_ratio{0.5};
  double fov_ellipse_b_ratio{0.5};
};

struct GroundVideoBox
{
  float x1;
  float y1;
  float x2;
  float y2;
  char label[16]{};
};

// Owns the stream-only CUDA buffers, nvJPEG state, and RTP/JPEG sender.  submit()
// never waits for encoding or networking: when all slots are busy, it drops the
// new frame so the camera and perception paths remain independent.
class GroundVideoStreamer
{
public:
  explicit GroundVideoStreamer(const GroundVideoConfig & config);
  ~GroundVideoStreamer();

  GroundVideoStreamer(const GroundVideoStreamer &) = delete;
  GroundVideoStreamer & operator=(const GroundVideoStreamer &) = delete;

  void submit(
    const std::uint8_t * bgra_device, std::size_t bgra_pitch, int width, int height,
    const std::vector<GroundVideoBox> & boxes = {});

private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace zed2i_driver

#endif  // ZED2I_DRIVER__GROUND_VIDEO_STREAMER_HPP_
