#!/usr/bin/env python3
"""Receive RTP/JPEG and publish the encoded JPEG on a local ROS topic."""

import sys

import gi

gi.require_version('Gst', '1.0')
from gi.repository import Gst

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage


class GroundReceiver(Node):
    """Bridge the low-bandwidth RTP/JPEG receiver pipeline into local ROS 2."""

    def __init__(self):
        super().__init__('ground_receiver')
        port = self.declare_parameter('port', 5600).value
        jitter_latency_ms = self.declare_parameter('jitter_latency_ms', 50).value
        topic = self.declare_parameter('topic', '/ground_video/image/compressed').value
        if not 1 <= port <= 65535 or jitter_latency_ms < 0 or not topic:
            raise ValueError('invalid ground-video receiver configuration')

        self.publisher = self.create_publisher(
            CompressedImage, topic, qos_profile_sensor_data)
        Gst.init(None)
        pipeline_description = (
            f'udpsrc port={port} '
            'caps=application/x-rtp,media=video,encoding-name=JPEG,payload=26 '
            f'! rtpjitterbuffer latency={jitter_latency_ms} drop-on-latency=true '
            '! rtpjpegdepay '
            '! appsink name=sink emit-signals=true sync=false max-buffers=1 drop=true'
        )
        self.pipeline = Gst.parse_launch(pipeline_description)
        sink = self.pipeline.get_by_name('sink')
        if sink is None:
            raise RuntimeError('could not find ground-video appsink element')
        sink.connect('new-sample', self._on_new_sample)
        if self.pipeline.set_state(Gst.State.PLAYING) == Gst.StateChangeReturn.FAILURE:
            raise RuntimeError('could not start ground-video receiver pipeline')
        self.get_logger().info(
            f'Receiving RTP/JPEG on UDP port {port} and publishing locally to {topic}')

    def _on_new_sample(self, sink):
        sample = sink.emit('pull-sample')
        if sample is None:
            return Gst.FlowReturn.ERROR
        buffer = sample.get_buffer()
        success, map_info = buffer.map(Gst.MapFlags.READ)
        if not success:
            return Gst.FlowReturn.ERROR
        try:
            message = CompressedImage()
            message.header.stamp = self.get_clock().now().to_msg()
            message.format = 'jpeg'
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
    receiver = GroundReceiver()
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
