#!/usr/bin/env python3

import math
import threading
from typing import Optional, Tuple

import rclpy
from geometry_msgs.msg import Vector3
from rclpy.node import Node
from sensor_msgs.msg import Imu, MagneticField

from rsl_comm_py import UM7Serial
from rsl_comm_py.um7_broadcast_packets import UM7AllProcPacket, UM7EulerPacket, UM7QuaternionPacket


SQRT1_2 = math.sqrt(0.5)


def quat_multiply(
    q1: Tuple[float, float, float, float], q2: Tuple[float, float, float, float]
) -> Tuple[float, float, float, float]:
    w1, x1, y1, z1 = q1
    w2, x2, y2, z2 = q2
    return (
        w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
        w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
        w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
        w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
    )


def quat_normalize(q: Tuple[float, float, float, float]) -> Tuple[float, float, float, float]:
    w, x, y, z = q
    n = math.sqrt(w * w + x * x + y * y + z * z)
    if n <= 0.0:
        return (1.0, 0.0, 0.0, 0.0)
    return (w / n, x / n, y / n, z / n)


def euler_to_quat(roll: float, pitch: float, yaw: float) -> Tuple[float, float, float, float]:
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)

    w = cr * cp * cy + sr * sp * sy
    x = sr * cp * cy - cr * sp * sy
    y = cr * sp * cy + sr * cp * sy
    z = cr * cp * sy - sr * sp * cy
    return quat_normalize((w, x, y, z))


def ned_to_enu_vector(x: float, y: float, z: float) -> Tuple[float, float, float]:
    return (y, x, -z)


def ned_to_enu_quaternion(w: float, x: float, y: float, z: float) -> Tuple[float, float, float, float]:
    # Rotation mapping NED frame basis into ENU frame basis.
    q_ned_to_enu = (0.0, SQRT1_2, SQRT1_2, 0.0)
    return quat_normalize(quat_multiply(q_ned_to_enu, (w, x, y, z)))


class UmxDriverNode(Node):
    def __init__(self) -> None:
        super().__init__("um7_driver")

        self.declare_parameter("port", "/dev/ttyUSB0")
        self.declare_parameter("baud", 115200)
        self.declare_parameter("frame_id", "imu_link")
        self.declare_parameter("tf_ned_to_enu", True)
        self.declare_parameter("mag_updates", True)
        self.declare_parameter("zero_gyros", True)

        self.port = str(self.get_parameter("port").value)
        self.baud = int(self.get_parameter("baud").value)
        self.frame_id = str(self.get_parameter("frame_id").value)
        self.tf_ned_to_enu = bool(self.get_parameter("tf_ned_to_enu").value)
        self.mag_updates = bool(self.get_parameter("mag_updates").value)
        self.zero_gyros = bool(self.get_parameter("zero_gyros").value)

        self.imu_pub = self.create_publisher(Imu, "/imu/data", 20)
        self.mag_pub = self.create_publisher(MagneticField, "/imu/mag", 20)

        self._latest_quat: Optional[Tuple[float, float, float, float]] = None
        self._running = True
        self._sensor = None

        try:
            self._sensor = UM7Serial(port_name=self.port)
            self._sensor.port.baudrate = self.baud

            # Enable all processed data and quaternion broadcasts at a practical rate.
            all_proc_rate_hz = 40
            quat_rate_hz = 40
            euler_rate_hz = 40
            self._sensor.creg_com_rates4 = all_proc_rate_hz
            self._sensor.creg_com_rates5 = (quat_rate_hz << 24) | (euler_rate_hz << 16)

            if self.zero_gyros:
                self._sensor.zero_gyros = 1

            self.get_logger().info(
                f"Connected to UM7 on {self.port} @ {self.baud}. "
                f"frame_id={self.frame_id}, tf_ned_to_enu={self.tf_ned_to_enu}, "
                f"mag_updates={self.mag_updates}, zero_gyros={self.zero_gyros}"
            )
        except Exception as exc:
            self.get_logger().error(f"Failed to initialize UM7Serial: {exc}")
            raise

        self._reader_thread = threading.Thread(target=self._reader_loop, daemon=True)
        self._reader_thread.start()

    def destroy_node(self) -> bool:
        self._running = False
        try:
            if self._sensor is not None and self._sensor.port.is_open:
                self._sensor.port.close()
        except Exception:
            pass
        return super().destroy_node()

    def _reader_loop(self) -> None:
        assert self._sensor is not None

        try:
            for packet in self._sensor.recv_broadcast(flush_buffer_on_start=True):
                if not self._running:
                    break

                if isinstance(packet, UM7QuaternionPacket):
                    qw, qx, qy, qz = packet.q_w, packet.q_x, packet.q_y, packet.q_z
                    if self.tf_ned_to_enu:
                        qw, qx, qy, qz = ned_to_enu_quaternion(qw, qx, qy, qz)
                    else:
                        qw, qx, qy, qz = quat_normalize((qw, qx, qy, qz))
                    self._latest_quat = (qw, qx, qy, qz)

                elif isinstance(packet, UM7EulerPacket):
                    # Fallback orientation path if quaternion packets are not present.
                    qw, qx, qy, qz = euler_to_quat(
                        math.radians(packet.roll),
                        math.radians(packet.pitch),
                        math.radians(packet.yaw),
                    )
                    if self.tf_ned_to_enu:
                        qw, qx, qy, qz = ned_to_enu_quaternion(qw, qx, qy, qz)
                    self._latest_quat = (qw, qx, qy, qz)

                elif isinstance(packet, UM7AllProcPacket):
                    self._publish_from_all_proc(packet)
        except Exception as exc:
            self.get_logger().error(f"UM7 reader loop stopped due to error: {exc}")

    def _publish_from_all_proc(self, packet: UM7AllProcPacket) -> None:
        imu_msg = Imu()
        imu_msg.header.stamp = self.get_clock().now().to_msg()
        imu_msg.header.frame_id = self.frame_id

        if self._latest_quat is not None:
            imu_msg.orientation.w = self._latest_quat[0]
            imu_msg.orientation.x = self._latest_quat[1]
            imu_msg.orientation.y = self._latest_quat[2]
            imu_msg.orientation.z = self._latest_quat[3]
            imu_msg.orientation_covariance = [0.02, 0.0, 0.0, 0.0, 0.02, 0.0, 0.0, 0.0, 0.02]
        else:
            imu_msg.orientation_covariance[0] = -1.0

        gx, gy, gz = packet.gyro_proc_x, packet.gyro_proc_y, packet.gyro_proc_z
        ax, ay, az = packet.accel_proc_x, packet.accel_proc_y, packet.accel_proc_z
        mx, my, mz = packet.mag_proc_x, packet.mag_proc_y, packet.mag_proc_z

        if self.tf_ned_to_enu:
            gx, gy, gz = ned_to_enu_vector(gx, gy, gz)
            ax, ay, az = ned_to_enu_vector(ax, ay, az)
            mx, my, mz = ned_to_enu_vector(mx, my, mz)

        # UM7 processed gyro is deg/s. Convert to rad/s for sensor_msgs/Imu.
        imu_msg.angular_velocity = Vector3(
            x=math.radians(gx), y=math.radians(gy), z=math.radians(gz)
        )
        imu_msg.angular_velocity_covariance = [0.03, 0.0, 0.0, 0.0, 0.03, 0.0, 0.0, 0.0, 0.03]

        # UM7 processed accel is in g. Convert to m/s^2.
        g_to_mps2 = 9.80665
        imu_msg.linear_acceleration = Vector3(
            x=ax * g_to_mps2, y=ay * g_to_mps2, z=az * g_to_mps2
        )
        imu_msg.linear_acceleration_covariance = [0.05, 0.0, 0.0, 0.0, 0.05, 0.0, 0.0, 0.0, 0.05]

        self.imu_pub.publish(imu_msg)

        if self.mag_updates:
            mag_msg = MagneticField()
            mag_msg.header = imu_msg.header
            # UM7 processed mag is in Gauss. Convert to Tesla.
            gauss_to_tesla = 1e-4
            mag_msg.magnetic_field = Vector3(
                x=mx * gauss_to_tesla, y=my * gauss_to_tesla, z=mz * gauss_to_tesla
            )
            mag_msg.magnetic_field_covariance = [0.0] * 9
            self.mag_pub.publish(mag_msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = UmxDriverNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
