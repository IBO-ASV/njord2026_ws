"""Publish stable simulated vessel status for the Foxglove overview panels."""

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import Float32, Float32MultiArray, String


class GuiStatusDummy(Node):
    def __init__(self):
        super().__init__('task3_gui_status_dummy')

        self.declare_parameter('publish_rate_hz', 1.0)
        self.declare_parameter('battery_percent', 86.0)
        self.declare_parameter('cell_voltages', [4.05, 4.04, 4.06, 4.05])
        self.declare_parameter('temperature_c', 25.0)
        self.declare_parameter('control_status', 'auto')

        transient_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.battery_percent_pub = self.create_publisher(
            Float32, '/gui/battery_percent', transient_qos)
        self.battery_voltage_pub = self.create_publisher(
            Float32, '/gui/battery_voltage_v', transient_qos)
        self.cell_voltages_pub = self.create_publisher(
            Float32MultiArray, '/bms/cell_voltages', transient_qos)
        self.temperature_pub = self.create_publisher(
            Float32, '/bms/temperature_c', transient_qos)
        self.control_status_pub = self.create_publisher(
            String, '/system/control_status', transient_qos)
        self.operating_mode_pub = self.create_publisher(
            String, '/system/operating_mode', transient_qos)

        rate_hz = max(0.1, float(self.get_parameter('publish_rate_hz').value))
        self.timer = self.create_timer(1.0 / rate_hz, self.publish_status)
        self.publish_status()

    def publish_status(self):
        cells = [float(value) for value in self.get_parameter('cell_voltages').value]
        battery_percent = float(self.get_parameter('battery_percent').value)
        control_status = str(self.get_parameter('control_status').value)
        temperature_c = float(self.get_parameter('temperature_c').value)

        self.battery_percent_pub.publish(Float32(data=battery_percent))
        self.battery_voltage_pub.publish(Float32(data=sum(cells)))
        self.cell_voltages_pub.publish(Float32MultiArray(data=cells))
        self.temperature_pub.publish(Float32(data=temperature_c))
        self.control_status_pub.publish(String(data=control_status))
        self.operating_mode_pub.publish(String(data=control_status))


def main(args=None):
    rclpy.init(args=args)
    node = GuiStatusDummy()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
