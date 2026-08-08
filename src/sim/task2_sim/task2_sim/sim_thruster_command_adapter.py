"""Simulation-only bridge from the real thruster command to the sim dynamics input.

test07089's thruster_driver publishes /thruster_command as
std_msgs/Float32MultiArray in Newtons (FR, FL, RR, RL), while
dutyed_tf_pub_with_disturbance still consumes std_msgs/Int16MultiArray duty
counts. This node converts Newtons back to duty counts so the simulator can
run against the unchanged real-hardware command chain. Never launch this on
the real vessel.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, Int16MultiArray


class SimThrusterCommandAdapter(Node):
    def __init__(self):
        super().__init__("sim_thruster_command_adapter")

        self.declare_parameter("input_topic", "/thruster_command")
        self.declare_parameter("output_topic", "/sim/thruster_duty")
        # Must match thruster_driver config: thrusters.force_per_duty
        self.declare_parameter("force_per_duty", [40.0, 40.0, 40.0, 40.0])
        # Must match dutyed_tf_pub_with_disturbance config: duty_resolution
        self.declare_parameter("duty_resolution", 1000)

        self._force_per_duty = [
            float(v) for v in self.get_parameter("force_per_duty").value
        ]
        self._duty_resolution = int(self.get_parameter("duty_resolution").value)

        self._pub = self.create_publisher(
            Int16MultiArray, self.get_parameter("output_topic").value, 10
        )
        self._sub = self.create_subscription(
            Float32MultiArray,
            self.get_parameter("input_topic").value,
            self._on_command,
            10,
        )

    def _on_command(self, msg: Float32MultiArray):
        duties = Int16MultiArray()
        res = self._duty_resolution
        for i, newtons in enumerate(msg.data):
            fpd = self._force_per_duty[min(i, len(self._force_per_duty) - 1)]
            if fpd <= 0.0:
                duty = 0
            else:
                duty = int(round(newtons / fpd * res))
            duties.data.append(max(-res, min(res, duty)))
        self._pub.publish(duties)


def main(args=None):
    rclpy.init(args=args)
    node = SimThrusterCommandAdapter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
