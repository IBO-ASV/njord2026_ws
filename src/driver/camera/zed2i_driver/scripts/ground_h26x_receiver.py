#!/usr/bin/env python3
"""Receive RTP/H.264 or RTP/H.265 and publish decoded BGR images locally."""

import sys

import gi

gi.require_version('Gst', '1.0')
from gi.repository import Gst

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image


class GroundH26xReceiver(Node):
    """Low-latency RTP/H.26x receiver for the miniPC back camera."""

    def __init__(self):
        super().__init__('ground_h26x_receiver')
        port = self.declare_parameter('port', 5601).value
        codec = self.declare_parameter('codec', 'h264').value
        jitter_latency_ms = self.declare_parameter('jitter_latency_ms', 50).value
        topic = self.declare_parameter('topic', '/ground_video/back_cam/image_raw').value
        if not 1 <= port <= 65535 or codec not in ('h264', 'h265') or jitter_latency_ms < 0 or not topic:
            raise ValueError('invalid H.26x ground-video receiver configuration')

        depay = 'rtph264depay ! h264parse ! avdec_h264' if codec == 'h264' else \
            'rtph265depay ! h265parse ! avdec_h265'
        encoding_name = 'H264' if codec == 'h264' else 'H265'
        self.publisher = self.create_publisher(Image, topic, qos_profile_sensor_data)
        Gst.init(None)
        pipeline_description = (
            f'udpsrc port={port} '
            f'caps=application/x-rtp,media=video,encoding-name={encoding_name},payload=96 '
            f'! rtpjitterbuffer latency={jitter_latency_ms} drop-on-latency=true '
            f'! {depay} ! videoconvert ! video/x-raw,format=BGR '
            '! appsink name=sink emit-signals=true sync=false max-buffers=1 drop=true'
        )
        self.pipeline = Gst.parse_launch(pipeline_description)
        sink = self.pipeline.get_by_name('sink')
        if sink is None:
            raise RuntimeError('could not find H.26x ground-video appsink element')
        sink.connect('new-sample', self._on_new_sample)
        if self.pipeline.set_state(Gst.State.PLAYING) == Gst.StateChangeReturn.FAILURE:
            raise RuntimeError('could not start H.26x ground-video receiver pipeline')
        self.get_logger().info(f'Receiving RTP/{encoding_name} on UDP port {port} to {topic}')

    def _on_new_sample(self, sink):
        sample = sink.emit('pull-sample')
        if sample is None:
            return Gst.FlowReturn.ERROR
        caps = sample.get_caps().get_structure(0)
        width = caps.get_value('width')
        height = caps.get_value('height')
        buffer = sample.get_buffer()
        success, map_info = buffer.map(Gst.MapFlags.READ)
        if not success:
            return Gst.FlowReturn.ERROR
        try:
            message = Image()
            message.header.stamp = self.get_clock().now().to_msg()
            message.height = height
            message.width = width
            message.encoding = 'bgr8'
            message.is_bigendian = 0
            message.step = width * 3
            message.data = bytes(map_info.data)
            self.publisher.publish(message)
        finally:
            buffer.unmap(map_info)
        return Gst.FlowReturn.OK

    def destroy_node(self):
        if hasattr(self, 'pipeline'):
            self.pipeline.set_state(Gst.State.NULL)
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    receiver = GroundH26xReceiver()
    try:
        rclpy.spin(receiver)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        receiver.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main(sys.argv)
