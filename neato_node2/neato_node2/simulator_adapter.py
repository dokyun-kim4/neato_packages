import rclpy
from rclpy.node import Node
from neato2_interfaces.msg import Accel, Bump
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
# TODO: it would be nice if raw_vel actually changed single wheel velocities

from gazebo_msgs.msg import ContactsState
from sensor_msgs.msg import Imu, LaserScan


class RawVelRelayNode(Node):
    def __init__(self):
        super().__init__("simulator_relay")
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )
        # it is less important that this wheel base is accurate for the simulated robot but that it is true to the real robot
        self.real_robot_wheel_base = 0.235
        # Note: from neato_description/neato_gazebo.urdf.xacro <wheelDiameter>0.07</wheelDiameter> (this might not be physically accurate, but it is what is there for the differential drive controller
        self.wheel_radius = 0.07 / 2
        self.scan_sub = self.create_subscription(
            LaserScan, "scan", self.scan_received, qos_profile
        )
        self.imu_sub = self.create_subscription(
            Imu, "imu", self.imu_received, qos_profile
        )
        self.bump_sub = self.create_subscription(
            ContactsState, "bumper", self.contacts_received, qos_profile
        )
        self.accel_pub = self.create_publisher(Accel, "accel", 10)
        self.scan_pub = self.create_publisher(LaserScan, "stable_scan", 10)
        self.bumper_pub = self.create_publisher(Bump, "bump", 10)

    def scan_received(self, msg):
        self.scan_pub.publish(msg)

    def imu_received(self, msg):
        self.accel_pub.publish(
            Accel(
                accel_x=msg.linear_acceleration.x / 9.8,
                accel_y=msg.linear_acceleration.y / 9.8,
                accel_z=msg.linear_acceleration.z / 9.8,
            )
        )

    def contacts_received(self, msg):
        # NOTE: this is oversimplified (just uses front bumper in a binary fashion)
        if len(msg.states):
            self.bumper_pub.publish(
                Bump(left_front=1, left_side=1, right_front=1, right_side=1)
            )
        else:
            self.bumper_pub.publish(
                Bump(left_front=0, left_side=0, right_front=0, right_side=0)
            )


def main(args=None):
    rclpy.init(args=args)
    node = RawVelRelayNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
