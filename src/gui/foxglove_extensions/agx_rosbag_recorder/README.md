# AGX Rosbag Recorder

Foxglove panel that starts and safely stops a sensor/navigation rosbag on the AGX. The recorder is launched by `agx_bringup.launch.py`, records active allowlisted sensor, localization, and perception topics as MCAP under `/home/ibo/rosbags/`, and reports status over `/agx/rosbag/recording_status`. Start is refused below 5 GiB free; an active recording is stopped safely below 1 GiB free.

Connect Foxglove to the live Ground Jetson bridge at `ws://100.115.34.103:8765`, install `agx-rosbag-recorder-0.1.0.foxe`, then add the **AGX Rosbag Recorder** panel once to the layout. The panel provides one-click start and stop. It needs the Zenoh service route from Ground Jetson to AGX.

Build the self-contained extension archive with `./scripts/package.sh`.
