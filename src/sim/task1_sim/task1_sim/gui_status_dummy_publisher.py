"""Publish Task1 simulation values used by the Foxglove overview layout."""

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import Float32, Float32MultiArray, String


class GuiStatusDummyPublisher(Node):
    """Provide deterministic battery and autonomous-mode status in simulation."""

    def __init__(self):
        super().__init__("gui_status_dummy_publisher")

        self.declare_parameter("battery_percent", 75.0)
        self.declare_parameter("cell_voltages", [4.00, 4.01, 4.00, 4.01])
        self.declare_parameter("temperature_c", 25.0)
        self.declare_parameter("control_status", "auto")
        self.declare_parameter("publish_rate_hz", 1.0)

        battery_percent = self.get_parameter("battery_percent").value
        control_status = self.get_parameter("control_status").value
        publish_rate_hz = self.get_parameter("publish_rate_hz").value
        if publish_rate_hz <= 0.0:
            raise ValueError("publish_rate_hz must be positive")

        status_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.battery_publisher = self.create_publisher(
            Float32, "/gui/battery_percent", status_qos)
        self.battery_voltage_publisher = self.create_publisher(
            Float32, "/gui/battery_voltage_v", status_qos)
        self.cell_voltages_publisher = self.create_publisher(
            Float32MultiArray, "/bms/cell_voltages", status_qos)
        self.temperature_publisher = self.create_publisher(
            Float32, "/bms/temperature_c", status_qos)
        self.control_status_publisher = self.create_publisher(
            String, "/system/control_status", status_qos)
        self.operating_mode_publisher = self.create_publisher(
            String, "/system/operating_mode", status_qos)
        self.battery_message = Float32(data=float(battery_percent))
        self.cell_voltages_message = Float32MultiArray(
            data=[float(cell) for cell in self.get_parameter("cell_voltages").value])
        self.battery_voltage_message = Float32(data=sum(self.cell_voltages_message.data))
        self.temperature_message = Float32(data=float(self.get_parameter("temperature_c").value))
        self.control_status_message = String(data=str(control_status))

        self.publish()
        self.timer = self.create_timer(1.0 / publish_rate_hz, self.publish)

    def publish(self):
        self.battery_publisher.publish(self.battery_message)
        self.battery_voltage_publisher.publish(self.battery_voltage_message)
        self.cell_voltages_publisher.publish(self.cell_voltages_message)
        self.temperature_publisher.publish(self.temperature_message)
        self.control_status_publisher.publish(self.control_status_message)
        self.operating_mode_publisher.publish(self.control_status_message)


def main(args=None):
    rclpy.init(args=args)
    node = GuiStatusDummyPublisher()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
