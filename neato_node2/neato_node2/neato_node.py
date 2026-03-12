import rclpy
from rclpy.node import Node
import threading
from .neato_driver.neato_hybrid_driver import (
    xv11,
    BASE_WIDTH,
    MAX_SPEED,
    WHEEL_DIAMETER,
)
from geometry_msgs.msg import Twist, Quaternion, TransformStamped
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
from neato2_interfaces.msg import Accel, Bump
import time
import numpy as np
from copy import copy
from math import pi, cos, sin


class NeatoNode(Node):
    def __init__(self):
        super().__init__("neato_node")

        (use_udp, udp_port, host, robot_name) = self.declare_parameters(
            namespace="",
            parameters=[
                ("use_udp", True),
                ("udp_port", 7777),
                ("host", ""),
                ("robot_name", ""),
            ],
        )
        self.get_logger().info("Connecting to host: %s" % (host.value))
        self.get_logger().info("Namespace is set to: %s" % (robot_name.value))
        self.get_logger().info("UDP port is set to: %s" % (udp_port.value))
        self.robot = xv11(host.value, use_udp.value, udp_port.value)
        self.subscription = self.create_subscription(
            Twist, "cmd_vel", self.cmdVelCb, 10
        )
        self.tf_prefix = robot_name.value
        self.scanPub = self.create_publisher(LaserScan, "scan", 10)
        self.odomPub = self.create_publisher(Odometry, "odom", 10)
        self.bumpPub = self.create_publisher(Bump, "bump", 10)
        self.accelPub = self.create_publisher(Accel, "accel", 10)
        self.odomBroadcaster = TransformBroadcaster(self)
        self.robot_setup()
        self.timer = self.create_timer(0.01, self.main_run_loop)

    def robot_setup(self):
        self.last_loop_time = time.time()
        self.cmd_vel = None
        self.cmd_vel_lock = threading.Lock()
        self.old_ranges = None
        self.encoders = [0, 0]

        self.x = 0  # position in xy plane
        self.y = 0
        self.th = 0

        # things that don't ever change
        # TODO: get this using rosparam
        self.scan_link = "base_laser_link"
        scan = LaserScan()
        scan.header.frame_id = self.tf_prefix + self.scan_link
        scan.angle_min = -pi
        scan.angle_max = pi
        scan.angle_increment = pi / 180.0
        scan.range_min = 0.020
        scan.range_max = 5.0
        scan.scan_time = 0.2
        scan.time_increment = 1.0 / (5 * 360)
        self.scan = scan
        self.odom = Odometry()
        self.odom.header.frame_id = self.tf_prefix + "odom"
        self.odom.child_frame_id = self.tf_prefix + "base_footprint"
        time.sleep(4.0)
        # do UDP hole punching to make sure the sensor data from the robot makes it through
        self.robot.do_udp_hole_punch()
        self.robot.send_keep_alive()
        self.last_keep_alive = time.time()

        self.robot.send_keep_alive()
        self.last_keep_alive = time.time()

        scan.header.stamp = self.get_clock().now().to_msg()
        self.last_motor_time = self.get_clock().now()
        self.last_set_motor_time = self.get_clock().now()

        self.total_dth = 0.0

    def main_run_loop(self):
        if time.time() - self.last_keep_alive > 10.0:
            self.robot.send_keep_alive()
            self.last_keep_alive = time.time()
        self.robot.requestScan()
        new_loop_time = time.time()
        delta_t = new_loop_time - self.last_loop_time
        self.last_loop_time = new_loop_time
        self.scan.header.stamp = self.get_clock().now().to_msg()
        (self.scan.ranges, self.scan.intensities) = self.robot.getScanRanges()
        # repeat last measurement to simulate -pi to pi (instead of -pi to pi - pi/180)
        # This is important in order to adhere to ROS conventions regarding laser scanners
        if len(self.scan.ranges):
            self.scan.ranges.append(self.scan.ranges[0])
            self.scan.intensities.append(self.scan.intensities[0])
            if self.old_ranges == self.scan.ranges:
                self.scan.ranges, self.scan.intensities = [], []
            else:
                self.old_ranges = copy(self.scan.ranges)

        # TODO: do this better
        for i in range(len(self.scan.ranges)):
            if self.scan.ranges[i] == 0:
                self.scan.ranges[i] = float("inf")
        if delta_t - 0.2 > 0.1:
            self.get_logger().warn(
                "Iteration took longer than expected (should be 0.2) %f" % (delta_t)
            )

        # get motor encoder values
        curr_motor_time = self.get_clock().now()
        try:
            motors = self.robot.getMotors()
            if motors:
                # unpack the motor values since we got them.
                left, right = motors
                # now update position information
                # might consider moving curr_motor_time down
                dt = (curr_motor_time - self.last_motor_time).nanoseconds / 10**9
                self.last_motor_time = curr_motor_time

                # TODO: implement
                # self.encodersPub.publish(Float32MultiArray(data=[left/1000.0, right/1000.0]))

                d_left = (left - self.encoders[0]) / 1000.0
                d_right = (right - self.encoders[1]) / 1000.0

                self.encoders = [left, right]
                dx = (d_left + d_right) / 2
                dth = (d_right - d_left) / (BASE_WIDTH / 1000.0)
                self.total_dth += dth

                x = cos(dth) * dx
                y = -sin(dth) * dx

                self.x += cos(self.th) * x - sin(self.th) * y
                self.y += sin(self.th) * x + cos(self.th) * y
                self.th += dth

                quaternion = Quaternion()
                quaternion.z = sin(self.th / 2.0)
                quaternion.w = cos(self.th / 2.0)

                # prepare odometry
                self.odom.header.stamp = curr_motor_time.to_msg()
                self.odom.pose.pose.position.x = self.x
                self.odom.pose.pose.position.y = self.y
                self.odom.pose.pose.position.z = 0.0
                self.odom.pose.pose.orientation = quaternion
                # copying these from the iRobot create
                self.odom.pose.covariance = np.array(
                    [
                        10**-3,
                        0.0,
                        0.0,
                        0.0,
                        0.0,
                        0.0,
                        0.0,
                        10**-3,
                        0.0,
                        0.0,
                        0.0,
                        0.0,
                        0.0,
                        0.0,
                        10**-3,
                        0.0,
                        0.0,
                        0.0,
                        0.0,
                        0.0,
                        0.0,
                        10**-3,
                        0.0,
                        0.0,
                        0.0,
                        0.0,
                        0.0,
                        0,
                        10**-3,
                        0.0,
                        0.0,
                        0.0,
                        0.0,
                        0.0,
                        0.0,
                        10**-2,
                    ],
                    dtype=np.float64,
                )
                self.odom.twist.twist.linear.x = dx / dt
                self.odom.twist.twist.angular.z = dth / dt
                self.odom.twist.covariance = np.array(
                    [
                        10**-3,
                        0.0,
                        0.0,
                        0.0,
                        0.0,
                        0.0,
                        0.0,
                        10**-3,
                        0.0,
                        0.0,
                        0.0,
                        0.0,
                        0.0,
                        0.0,
                        10**-3,
                        0.0,
                        0.0,
                        0.0,
                        0.0,
                        0.0,
                        0.0,
                        10**-3,
                        0.0,
                        0.0,
                        0.0,
                        0.0,
                        0.0,
                        0,
                        10**-3,
                        0.0,
                        0.0,
                        0.0,
                        0.0,
                        0.0,
                        0.0,
                        10**-2,
                    ],
                    dtype=np.float64,
                )
                transform = TransformStamped()
                transform.header.stamp = curr_motor_time.to_msg()
                transform.header.frame_id = self.tf_prefix + "odom"
                transform.child_frame_id = self.tf_prefix + "base_footprint"
                transform.transform.translation.x = self.x
                transform.transform.translation.y = self.y
                transform.transform.rotation.x = quaternion.x
                transform.transform.rotation.y = quaternion.y
                transform.transform.rotation.z = quaternion.z
                transform.transform.rotation.w = quaternion.w

                self.odomBroadcaster.sendTransform(transform)
                self.odomPub.publish(self.odom)

                transform = TransformStamped()
                transform.header.stamp = curr_motor_time.to_msg()
                transform.header.frame_id = self.tf_prefix + "base_footprint"
                transform.child_frame_id = self.tf_prefix + "wheel_right_link"
                transform.transform.translation.y = -BASE_WIDTH / 1000.0 / 2
                transform.transform.translation.z = 0.025
                right_th = (right / 1000.00) / (WHEEL_DIAMETER / 2000.0)
                transform.transform.rotation.y = sin(right_th / 2.0)
                transform.transform.rotation.w = cos(right_th / 2.0)
                self.odomBroadcaster.sendTransform(transform)

                transform = TransformStamped()
                transform.header.stamp = curr_motor_time.to_msg()
                transform.header.frame_id = self.tf_prefix + "base_footprint"
                transform.child_frame_id = self.tf_prefix + "wheel_left_link"
                transform.transform.translation.y = BASE_WIDTH / 1000.0 / 2
                transform.transform.translation.z = 0.025
                left_th = (left / 1000.00) / (WHEEL_DIAMETER / 2000.0)
                transform.transform.rotation.y = sin(left_th / 2.0)
                transform.transform.rotation.w = cos(left_th / 2.0)
                self.odomBroadcaster.sendTransform(transform)

        except Exception as err:
            print("my error is " + str(err))
        with self.cmd_vel_lock:
            if self.cmd_vel:
                self.robot.setMotors(
                    self.cmd_vel[0],
                    self.cmd_vel[1],
                    max(abs(self.cmd_vel[0]), abs(self.cmd_vel[1])),
                )
                self.cmd_vel = None
            elif (
                self.get_clock().now() - self.last_set_motor_time
                > rclpy.time.Duration(seconds=0.2)
            ):
                self.robot.resend_last_motor_command()
                self.last_set_motor_time = self.get_clock().now()

        try:
            bump_sensors = self.robot.getDigitalSensors()
            if bump_sensors:
                """ Indices of bump_sensors map as follows
                        0: front left
                        1: side left
                        2: front right
                        3: side right
                """
                msg = Bump(
                    left_front=int(bump_sensors[0]),
                    left_side=int(bump_sensors[1]),
                    right_front=int(bump_sensors[2]),
                    right_side=int(bump_sensors[3]),
                )
                self.bumpPub.publish(msg)
        except Exception as ex:
            print("failed to get bump sensors!", ex)

        try:
            accelerometer = self.robot.getAccel()
            if accelerometer:
                # Indices 2, 3, 4 of accelerometer correspond to x, y, and z direction respectively
                msg = Accel(
                    accel_x=accelerometer[2],
                    accel_y=accelerometer[3],
                    accel_z=accelerometer[4],
                )
                self.accelPub.publish(msg)
        except Exception as err:
            print("failed to get accelerometer!", err)

        if len(self.scan.ranges):
            self.scanPub.publish(self.scan)

    def cmdVelCb(self, req):
        # Simple odometry model
        x = req.linear.x * 1000
        th = req.angular.z * (BASE_WIDTH / 2)
        k = max(abs(x - th), abs(x + th))
        # sending commands higher than max speed will fail
        if k > MAX_SPEED:
            x = x * MAX_SPEED / k
            th = th * MAX_SPEED / k
        with self.cmd_vel_lock:
            self.cmd_vel = [int(x - th), int(x + th)]
        # print self.cmd_vel, "SENDING THIS VEL"


def main(args=None):
    rclpy.init(args=args)

    node = NeatoNode()

    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
