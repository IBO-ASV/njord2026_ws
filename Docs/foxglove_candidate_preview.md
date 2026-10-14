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
