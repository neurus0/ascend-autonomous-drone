import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from px4_msgs.msg import VehicleLocalPosition


class FlowNode(Node):

    def __init__(self):
        super().__init__('flow_node')

        # PX4 uses BEST_EFFORT QoS
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        self.subscription = self.create_subscription(
            VehicleLocalPosition,
            '/fmu/out/vehicle_local_position',
            self.callback,
            qos_profile
        )

    def callback(self, msg):
        altitude = -msg.z
        self.get_logger().info(f"Altitude: {altitude:.2f} m")


def main(args=None):
    rclpy.init(args=args)

    node = FlowNode()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
