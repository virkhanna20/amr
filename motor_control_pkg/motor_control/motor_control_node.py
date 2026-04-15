#!/usr/bin/env python3
"""Motor control node for AMR differential drive robot.

Communicates with Arduino via serial to:
- Send motor commands (from /cmd_vel)
- Read encoder data and compute odometry
- Publish /odom, /joint_states, and odom->base_link TF
"""

import math
import serial
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped, Quaternion
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from tf2_ros import TransformBroadcaster


def quaternion_from_yaw(yaw):
    """Create quaternion from yaw angle."""
    return Quaternion(
        x=0.0, y=0.0,
        z=math.sin(yaw / 2.0),
        w=math.cos(yaw / 2.0),
    )


class MotorControlNode(Node):
    def __init__(self):
        super().__init__('motor_control_node')

        # Parameters
        self.declare_parameter('wheel_separation', 0.35)
        self.declare_parameter('wheel_radius', 0.05)
        self.declare_parameter('encoder_ticks_per_rev', 330)
        self.declare_parameter('serial_port', '/dev/ttyACM0')
        self.declare_parameter('baud_rate', 115200)
        self.declare_parameter('cmd_vel_timeout', 0.5)
        self.declare_parameter('publish_rate', 50.0)

        self.wheel_sep = self.get_parameter('wheel_separation').value
        self.wheel_rad = self.get_parameter('wheel_radius').value
        self.ticks_per_rev = self.get_parameter('encoder_ticks_per_rev').value
        self.serial_port = self.get_parameter('serial_port').value
        self.baud_rate = self.get_parameter('baud_rate').value
        self.cmd_vel_timeout = self.get_parameter('cmd_vel_timeout').value
        publish_rate = self.get_parameter('publish_rate').value

        # Odometry state
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.prev_left_ticks = None
        self.prev_right_ticks = None
        self.left_wheel_pos = 0.0
        self.right_wheel_pos = 0.0

        # Motor command state
        self.target_left_rpm = 0.0
        self.target_right_rpm = 0.0
        self.last_cmd_vel_time = self.get_clock().now()

        # Serial connection
        self.ser = None
        self._connect_serial()

        # Publishers
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.joint_pub = self.create_publisher(JointState, '/joint_states', 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        # Subscriber
        self.create_subscription(Twist, '/cmd_vel', self._cmd_vel_callback, 10)

        # Main timer
        period = 1.0 / publish_rate
        self.create_timer(period, self._timer_callback)

        self.get_logger().info(
            f'Motor control started: wheel_sep={self.wheel_sep}, '
            f'wheel_rad={self.wheel_rad}, ticks/rev={self.ticks_per_rev}'
        )

    def _connect_serial(self):
        """Attempt to connect to Arduino serial port."""
        try:
            self.ser = serial.Serial(
                self.serial_port, self.baud_rate, timeout=0.01
            )
            self.get_logger().info(f'Connected to Arduino on {self.serial_port}')
        except serial.SerialException as e:
            self.get_logger().warn(
                f'Could not open {self.serial_port}: {e}. '
                'Will retry on next cycle.'
            )
            self.ser = None

    def _cmd_vel_callback(self, msg):
        """Convert Twist to wheel RPMs and send to Arduino."""
        linear_x = msg.linear.x
        angular_z = msg.angular.z

        # Differential drive inverse kinematics
        v_left = (linear_x - angular_z * self.wheel_sep / 2.0) / self.wheel_rad
        v_right = (linear_x + angular_z * self.wheel_sep / 2.0) / self.wheel_rad

        # Convert rad/s to RPM
        self.target_left_rpm = v_left * 60.0 / (2.0 * math.pi)
        self.target_right_rpm = v_right * 60.0 / (2.0 * math.pi)
        self.last_cmd_vel_time = self.get_clock().now()

        self._send_motor_command()

    def _send_motor_command(self):
        """Send motor command to Arduino."""
        if self.ser is None:
            return
        try:
            cmd = f'M:{self.target_left_rpm:.1f},{self.target_right_rpm:.1f}\n'
            self.ser.write(cmd.encode())
        except serial.SerialException:
            self.get_logger().warn('Serial write failed, reconnecting...')
            self.ser = None

    def _timer_callback(self):
        """Main loop: read encoders, compute odom, publish."""
        now = self.get_clock().now()

        # Safety: zero motors if no cmd_vel received recently
        dt_cmd = (now - self.last_cmd_vel_time).nanoseconds / 1e9
        if dt_cmd > self.cmd_vel_timeout:
            if self.target_left_rpm != 0.0 or self.target_right_rpm != 0.0:
                self.target_left_rpm = 0.0
                self.target_right_rpm = 0.0
                self._send_motor_command()

        # Reconnect serial if needed
        if self.ser is None:
            self._connect_serial()
            return

        # Read all available encoder data (use latest)
        left_ticks = None
        right_ticks = None
        dt_ms = None

        try:
            while self.ser.in_waiting > 0:
                line = self.ser.readline().decode('utf-8', errors='ignore').strip()
                if line.startswith('E:'):
                    parts = line[2:].split(',')
                    if len(parts) >= 3:
                        left_ticks = int(parts[0])
                        right_ticks = int(parts[1])
                        dt_ms = int(parts[2])
        except (serial.SerialException, ValueError) as e:
            self.get_logger().warn(f'Serial read error: {e}')
            self.ser = None
            return

        if left_ticks is None:
            return

        # Compute odometry from encoder deltas
        if self.prev_left_ticks is None:
            self.prev_left_ticks = left_ticks
            self.prev_right_ticks = right_ticks
            return

        dl_ticks = left_ticks - self.prev_left_ticks
        dr_ticks = right_ticks - self.prev_right_ticks
        self.prev_left_ticks = left_ticks
        self.prev_right_ticks = right_ticks

        # Distance traveled by each wheel
        meters_per_tick = (2.0 * math.pi * self.wheel_rad) / self.ticks_per_rev
        delta_left = dl_ticks * meters_per_tick
        delta_right = dr_ticks * meters_per_tick

        # Differential drive forward kinematics
        delta_s = (delta_left + delta_right) / 2.0
        delta_theta = (delta_right - delta_left) / self.wheel_sep

        # Update pose
        self.theta += delta_theta
        self.x += delta_s * math.cos(self.theta)
        self.y += delta_s * math.sin(self.theta)

        # Update wheel positions (radians)
        rads_per_tick = (2.0 * math.pi) / self.ticks_per_rev
        self.left_wheel_pos += dl_ticks * rads_per_tick
        self.right_wheel_pos += dr_ticks * rads_per_tick

        # Compute velocities
        dt = dt_ms / 1000.0 if dt_ms > 0 else 0.02
        v_linear = delta_s / dt
        v_angular = delta_theta / dt

        # Publish odometry
        odom_msg = Odometry()
        odom_msg.header.stamp = now.to_msg()
        odom_msg.header.frame_id = 'odom'
        odom_msg.child_frame_id = 'base_link'
        odom_msg.pose.pose.position.x = self.x
        odom_msg.pose.pose.position.y = self.y
        odom_msg.pose.pose.orientation = quaternion_from_yaw(self.theta)
        odom_msg.twist.twist.linear.x = v_linear
        odom_msg.twist.twist.angular.z = v_angular
        # Covariance (diagonal)
        odom_msg.pose.covariance[0] = 0.01   # x
        odom_msg.pose.covariance[7] = 0.01   # y
        odom_msg.pose.covariance[35] = 0.05  # yaw
        odom_msg.twist.covariance[0] = 0.01
        odom_msg.twist.covariance[35] = 0.05
        self.odom_pub.publish(odom_msg)

        # Broadcast odom -> base_link TF
        t = TransformStamped()
        t.header.stamp = now.to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.rotation = quaternion_from_yaw(self.theta)
        self.tf_broadcaster.sendTransform(t)

        # Publish joint states
        js = JointState()
        js.header.stamp = now.to_msg()
        js.name = ['left_wheel_joint', 'right_wheel_joint']
        js.position = [self.left_wheel_pos, self.right_wheel_pos]
        js.velocity = [
            (dl_ticks * rads_per_tick) / dt,
            (dr_ticks * rads_per_tick) / dt,
        ]
        self.joint_pub.publish(js)


def main(args=None):
    rclpy.init(args=args)
    node = MotorControlNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Stop motors on shutdown
        if node.ser:
            try:
                node.ser.write(b'M:0.0,0.0\n')
            except serial.SerialException:
                pass
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
