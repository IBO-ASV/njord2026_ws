# Looping recorded candidate preview

User-requested persistent preview on ground Jetson, 100.115.34.103.
Connect current Foxglove to ws://100.115.34.103:8766 (Tailscale).
Separate ROS domain 76, loopback DDS; existing live ground station is unchanged.
Bridge binds only the Tailscale address. No control publishers or service/client
publish capabilities. This is recorded perception playback, not live detection.

Source: glim_aug10_ordered_20261014/output_bag, original bag seconds 330-355.
Clip: /home/ibo/rosbags/foxglove_candidate_220_clip.
85 point clouds, 78 track frames, 44 observed target-220 frames; 24.55 s loop.
Point clouds and planar object poses are pre-transformed into local odom using
timestamp-interpolated GLIM local odometry (maximum nearest-time gap 0.15 s).
No live TF is needed: /preview/points and /preview/tracks both use odom.
Candidate 220: orange boxes, labels and trail; other candidates: blue; ego: green.
The word candidate is intentional; ship identity is not independently verified.

Display-only message timestamps are refreshed on each publication to avoid
backward-time problems on repeated WebSocket playback. Original source seconds
are displayed in /preview/status. This does not change original bags or estimates.
Markers DELETEALL before each frame, including loop restart, preventing old trails.

Foxglove setup:
1. Open connection -> Foxglove WebSocket -> ws://100.115.34.103:8766.
2. Add a 3D panel; fixed/display frame odom; enable /preview/points and
   /preview/tracks. Zoom out to include the scene if needed (roughly 60 m scale).
3. Optional Raw Messages panel: /preview/status shows original source seconds.
   Source 337-341 s is the compact one-meter candidate's clearest short segment.
4. Point cloud decay should be short/zero for inspecting motion without trails.

Started process group: 715559; log /home/ibo/rosbags/foxglove_candidate_220_preview.log.
Stop only this preview: verify ps -p 715559 -o pid,pgid,args, then kill -INT -- -715559.
The preview is intentionally left running until the user asks to stop it; no boot
service or scheduled automation was installed.

Recreate clip with scripts/prepare_foxglove_candidate_clip.py RESULT_DIR NEW_CLIP.
Launch after sourcing ROS/workspace and loopback DDS:
ROS_DOMAIN_ID=76 ros2 launch /home/ibo/njord2026_ws/scripts/foxglove_candidate_preview.launch.py bag:=CLIP

Validation: Python compilation, clip counts, repeated loop logs, and an actual
WebSocket connection/subscription from the user's Windows PC receiving binary
messages. Installed bridge 3.5.0 uses foxglove.sdk.v1; the initial diagnostic client
using only the legacy foxglove.websocket.v1 protocol was rejected with HTTP 400.
A current Foxglove client supports the SDK protocol. The app UI was not inspected.

Sources:
https://docs.foxglove.dev/docs/visualization/connecting/live
https://docs.foxglove.dev/docs/visualization/panels/3d
https://docs.rs/foxglove/latest/src/foxglove/websocket/handshake.rs.html

## Camera clip (2026-10-14)

The preview on port 8766 now loops `rosbag2_2026_08_10-03_48_56`.
Original source is unchanged. Acquisition-ordered sensors:
`/home/ibo/rosbags/aug10_camera_acquisition_ordered`.
GLIM + local EKF + tracking results:
`/home/ibo/rosbags/glim_aug10_camera_20261014/output_bag`.
Display clip: `/home/ibo/rosbags/foxglove_camera_clip`.
Prepared with `scripts/prepare_foxglove_camera_clip.py RESULT_ROOT CLIP_PATH SOURCE_BAG`.

Contains 116 filtered clouds, 115 track frames, and 819 back-camera JPEG images
(640 x 480; all decoded successfully). Camera starts at source +3.469 s and
ends at +30.917 s. Acquisition headers schedule camera and perception together;
loop publisher refreshes visualization timestamps only. Planar odom transform
and assumed LiDAR mounting remain the same as the previous offline evaluation.
No camera-to-LiDAR projection/calibration or semantic vessel identification is claimed.
A sampled image shows the quay and a moored vessel. This is a different scene
from candidate 220; candidate IDs must not be associated across these bags.

Foxglove Image panel topic: `/preview/camera/back/compressed`.
3D panel: enable `/preview/points` and `/preview/tracks`, frame `odom`.
Object candidates are blue, ownship arrow green. All output is recorded replay.
Current isolated domain 76 preview process group: 729382.
Stop only this preview with `kill -INT -- -729382` after checking its command.
Restart using the existing launch and the camera clip path. Live station stays running.
Log: `/home/ibo/rosbags/foxglove_camera_preview.log`.

## Fourth bag with detection boxes (2026-10-14)

Current source: `rosbag2_2026_08_10-04_01_37`, 732.75 seconds.
Current launcher: `scripts/foxglove_detection_preview.launch.py`, isolated
ROS domain 76, WebSocket `ws://100.115.34.103:8766`.
Process group 943071; log `/home/ibo/rosbags/foxglove_aug10_040137_detection.log`.
This supersedes the previous camera clip preview. No live control nodes launched.

The streaming player reads only LiDAR and compressed camera topics, avoiding
loading the 12-minute recording into RAM. Source files are not modified.
It schedules by recording timestamps and refreshes display headers. This is
approximate camera/LiDAR temporal alignment, not hardware synchronization.
The first sampled camera recording delay was 0.202 s; LiDAR was 0.102 s.
Repeated playback is configured; a whole 12-minute cycle has not been observed.

The existing task2 cloud filter, preprocessing, ground removal and clustering
compute fresh detection boxes during replay. Green boxes are unclassified
LiDAR clusters, not confirmed vessels or persistent tracking IDs. No odometry,
velocity, CPA/TCPA or camera-image bounding boxes are produced in this mode.
Recorded IMU count is zero, preventing the previous GLIM configuration from
running unchanged. Recorded odometry exists but its quality/coverage has not
been validated for this preview.

Foxglove settings:
- Fixed/display frame: `base_link` (not `odom`).
- Enable `/preview/points` for raw cloud and `/preview/detections` for boxes.
- `/preview/filtered_points` optionally shows the cloud after task2 filtering.
- Image panel: `/preview/camera/back/compressed`.
- `/preview/tracks` only clears markers from older previews.

Limitations: mounting TF uses base->LiDAR offset (0.5,0,0.8), roll pi,
matching the source bag static transform; physical calibration is not independently
verified. Camera extrinsics are not calibrated
here. Detection range is 1.5-40 m, rear ROI only 10 m, size cutoff 5 m,
water band -0.3..0.15 m, cluster tolerance 1 m. These can omit small/low or
rear objects and merge nearby objects. A back-camera image can show objects
outside the detection ROI. Camera/LiDAR alignment and semantic labels must be
checked before interpreting boxes as other vessels.

Validation: Python compilation passed. In an 8-second ROS subscription check,
30 detection arrays containing 59 total boxes and 239 camera frames arrived;
all detection frames were `base_link`. Windows WebSocket independently
received three messages subscribed specifically to `/preview/detections`.

### Playback correction
The metadata reports 732.75 s, but a full sequential reader scan currently
returns only 653 LiDAR and 1936 compressed camera messages, in the first two
minute bins. Expected metadata counts were 7229 and 18360. The reason for
this metadata/readable-content mismatch (e.g. incomplete file/index) remains
unresolved; do not claim the full 12-minute recording is available.
The player now repeats on actual sensor EOF instead of leaving an empty wait
until metadata duration expires. After restart both camera-only and detection-only
Windows WebSocket checks each received three messages successfully.

### Current source: third bag
Switched to `rosbag2_2026_08_10-03_52_00` at user request, camera omitted.
Same detection launcher and domain 76, port 8766, base_link frame.
The player now advertises only sensor topics with messages in the source and
labels the status with the actual bag name. No stale camera publisher exists.
This mode computes cluster detection boxes; it does not use the earlier GLIM
odometry or track ID 220. Log: `/home/ibo/rosbags/foxglove_aug10_035200_detection.log`.
Windows subscribed specifically to detection boxes and received three messages
successfully after the switch. Python compilation passed.

### Second half only
Current replay starts at source +202.7143936565 s via
`PREVIEW_START_OFFSET=202.7143936565`, repeating only the latter half.
Japanese recording time: approximately 03:55:22.860 to 03:58:45.574.
The streaming player skips pre-offset records on every loop; no source changes.
Window length approximately 202.7 s. Status retains the full source offset.
Log: `/home/ibo/rosbags/foxglove_aug10_035200_second_half.log`.
Python compilation and Windows detection-only WebSocket reception passed.

### Own-vessel catamaran overlay
`/preview/ownship` now publishes a cyan catamaran outline at base_link origin.
It reuses the GNSS Map Telemetry GUI's silhouette, scaled visually to the
existing 2.4 m x 1.6 m self-crop bounds (not a measured hull model).
Enable this topic in the 3D panel; keep fixed/display frame base_link for the
current own-vessel-relative preview. The existing GUI map panel also receives
recorded `/sensor/vehicle_gnss/fix/raw` and `/tf`; these are historical data,
not live GNSS or newly estimated GLIM odometry. Only visualization timestamps
are refreshed. Other recorded control topics remain excluded.
Bridge whitelist includes those two specific topics. Second-half window stays
unchanged. Python compilation passed; Windows ownship-only subscription received
three marker messages. User Foxglove panel toggles remain to be enabled.
