#!/usr/bin/env python3

import math
import threading
from typing import Callable, Dict, Optional, Tuple

import rclpy
from geometry_msgs.msg import Vector3
from rclpy.node import Node
from sensor_msgs.msg import Imu, MagneticField

from rsl_comm_py import UM7Serial
from rsl_comm_py.um7_broadcast_packets import (
    UM7AllProcPacket,
    UM7EulerPacket,
    UM7QuaternionPacket,
)


Quaternion = Tuple[float, float, float, float]
Vector3Tuple = Tuple[float, float, float]

SQRT1_2 = math.sqrt(0.5)
GRAVITY_MPS2 = 9.80665
GAUSS_TO_TESLA = 1e-4
ORIENTATION_COVARIANCE = [0.02, 0.0, 0.0, 0.0, 0.02, 0.0, 0.0, 0.0, 0.02]
ANGULAR_VELOCITY_COVARIANCE = [0.03, 0.0, 0.0, 0.0, 0.03, 0.0, 0.0, 0.0, 0.03]
LINEAR_ACCELERATION_COVARIANCE = [0.05, 0.0, 0.0, 0.0, 0.05, 0.0, 0.0, 0.0, 0.05]
ZERO_COVARIANCE = [0.0] * 9
SERIAL_TIMEOUT_SECONDS = 0.1
SERIAL_BUFFER_SIZE = 64
QUATERNION_RATE_HZ = 20
EULER_RATE_HZ = 0


def quat_multiply(q1: Quaternion, q2: Quaternion) -> Quaternion:
    w1, x1, y1, z1 = q1
    w2, x2, y2, z2 = q2
    return (
        w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
        w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
        w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
        w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
    )


def quat_normalize(quaternion: Quaternion) -> Quaternion:
    w, x, y, z = quaternion
    norm = math.sqrt(w * w + x * x + y * y + z * z)
    if norm <= 0.0:
        return (1.0, 0.0, 0.0, 0.0)
    return (w / norm, x / norm, y / norm, z / norm)


def euler_to_quat(roll: float, pitch: float, yaw: float) -> Quaternion:
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


def ned_to_enu_vector(x: float, y: float, z: float) -> Vector3Tuple:
    return (y, x, -z)


def ned_to_enu_quaternion(w: float, x: float, y: float, z: float) -> Quaternion:
    # Rotation mapping NED frame basis into ENU frame basis.
    q_ned_to_enu = (0.0, SQRT1_2, SQRT1_2, 0.0)
    return quat_normalize(quat_multiply(q_ned_to_enu, (w, x, y, z)))


class PatchedUM7Serial(UM7Serial):
    def find_packet(self, sensor_response: bytes) -> Tuple[bytes, bytes]:
        preamble = self.get_preamble()
        packet_start_idx = sensor_response.find(preamble)

        if packet_start_idx == -1:
            return bytes(), bytes()

        next_packet_rel_idx = sensor_response[packet_start_idx + 3 :].find(preamble)
        if next_packet_rel_idx == -1:
            # Hold partial packet data until the next serial read completes it.
            return bytes(), sensor_response[packet_start_idx:]

        next_packet_start_idx = packet_start_idx + 3 + next_packet_rel_idx
        packet = sensor_response[packet_start_idx:next_packet_start_idx]
        remainder = sensor_response[next_packet_start_idx:]
        return packet, remainder

    def recv_broadcast(self, num_packets: int = -1, flush_buffer_on_start: bool = False):
        received_packets = 0
        if flush_buffer_on_start:
            self.port.reset_input_buffer()
            self.buffer = bytes()

        packet_decoders = self._build_broadcast_decoder_map()

        while num_packets == -1 or received_packets < num_packets:
            self.recv()
            while self.buffer:
                previous_buffer = self.buffer
                packet, self.buffer = self.find_packet(self.buffer)
                if not packet:
                    self.buffer = previous_buffer
                    break

                if not self.verify_checksum(packet):
                    continue
                if not self.check_packet(packet):
                    continue

                decoder = packet_decoders.get((packet[4], len(packet)))
                if decoder is None:
                    continue

                yield decoder(packet)
                received_packets += 1

    def _build_broadcast_decoder_map(self) -> Dict[Tuple[int, int], Callable[[bytes], object]]:
        register_address = self._register_address
        return {
            (register_address("DREG_HEALTH"), 11): self.decode_health_broadcast,
            (register_address("DREG_EULER_PHI_THETA"), 27): self.decode_euler_broadcast,
            (register_address("DREG_GYRO_PROC_X"), 55): self.decode_all_proc_broadcast,
            (register_address("DREG_GYRO_PROC_X"), 23): self.decode_proc_gyro_broadcast,
            (register_address("DREG_ACCEL_PROC_X"), 23): self.decode_proc_accel_broadcast,
            (register_address("DREG_MAG_PROC_X"), 23): self.decode_proc_mag_broadcast,
            (register_address("DREG_GYRO_RAW_XY"), 51): self.decode_all_raw_broadcast,
            (register_address("DREG_GYRO_RAW_XY"), 19): self.decode_raw_gyro_broadcast,
            (register_address("DREG_ACCEL_RAW_XY"), 19): self.decode_raw_accel_broadcast,
            (register_address("DREG_MAG_RAW_XY"), 23): self.decode_raw_mag_broadcast,
            (register_address("DREG_QUAT_AB"), 19): self.decode_quaternion_broadcast,
            (register_address("DREG_GYRO_BIAS_X"), 19): self.decode_gyro_bias_broadcast,
        }

    def _register_address(self, register_name: str) -> int:
        return self.svd_parser.find_register_by(name=register_name).address


class UmxDriverNode(Node):
    def __init__(self) -> None:
        super().__init__("um7_driver")

        self._declare_parameters()
        self._load_parameters()

        self.imu_pub = self.create_publisher(Imu, "/imu/data", 20)
        self.mag_pub = self.create_publisher(MagneticField, "/imu/mag", 20)

        self._latest_quat: Optional[Quaternion] = None
        self._running = True
        self._sensor: Optional[PatchedUM7Serial] = None

        self._initialize_sensor()

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

    def _declare_parameters(self) -> None:
        self.declare_parameter("port", "/dev/ttyUSB0")
        self.declare_parameter("baud", 115200)
        self.declare_parameter("update_rate", 100)
        self.declare_parameter("frame_id", "imu_link")
        self.declare_parameter("tf_ned_to_enu", True)
        self.declare_parameter("mag_updates", False)
        self.declare_parameter("zero_gyros", True)

    def _load_parameters(self) -> None:
        self.port = str(self.get_parameter("port").value)
        self.baud = int(self.get_parameter("baud").value)
        self.update_rate = int(self.get_parameter("update_rate").value)
        self.frame_id = str(self.get_parameter("frame_id").value)
        self.tf_ned_to_enu = bool(self.get_parameter("tf_ned_to_enu").value)
        self.mag_updates = bool(self.get_parameter("mag_updates").value)
        self.zero_gyros = bool(self.get_parameter("zero_gyros").value)

    def _initialize_sensor(self) -> None:
        try:
            sensor = PatchedUM7Serial(port_name=self.port)
            self._configure_sensor(sensor)
            self._sensor = sensor
            self._log_connection()
        except Exception as exc:
            self.get_logger().error(f"Failed to initialize UM7Serial: {exc}")
            raise

    def _configure_sensor(self, sensor: PatchedUM7Serial) -> None:
        sensor.port.baudrate = self.baud
        # A finite timeout lets recv() return with a partial read so the
        # register-write ACK is not buried behind queued broadcast packets.
        # At 100 Hz the 64-byte chunk arrives in ~11 ms, well under 100 ms.
        sensor.port.timeout = SERIAL_TIMEOUT_SECONDS
        sensor.buffer_size = SERIAL_BUFFER_SIZE

        # Discard broadcast packets that accumulated since the port opened
        # so the register-write ACK is the first thing recv() sees.
        sensor.port.reset_input_buffer()
        sensor.buffer = bytes()

        self._configure_broadcast_rates(sensor)

        if self.zero_gyros:
            sensor.zero_gyros = 1

    def _configure_broadcast_rates(self, sensor: PatchedUM7Serial) -> None:
        all_proc_rate_hz = max(1, min(self.update_rate, 255))
        sensor.creg_com_rates4 = all_proc_rate_hz
        sensor.creg_com_rates5 = (QUATERNION_RATE_HZ << 24) | (EULER_RATE_HZ << 16)

    def _log_connection(self) -> None:
        self.get_logger().info(
            f"Connected to UM7 on {self.port} @ {self.baud}. "
            f"update_rate={self.update_rate}, frame_id={self.frame_id}, "
            f"tf_ned_to_enu={self.tf_ned_to_enu}, mag_updates={self.mag_updates}, "
            f"zero_gyros={self.zero_gyros}"
        )

    def _reader_loop(self) -> None:
        assert self._sensor is not None

        try:
            for packet in self._sensor.recv_broadcast(flush_buffer_on_start=True):
                if not self._running:
                    break
                self._handle_packet(packet)
        except Exception as exc:
            self.get_logger().error(f"UM7 reader loop stopped due to error: {exc}")

    def _handle_packet(self, packet: object) -> None:
        if isinstance(packet, UM7QuaternionPacket):
            self._latest_quat = self._normalized_packet_quaternion(packet)
            return

        if isinstance(packet, UM7EulerPacket):
            self._latest_quat = self._quat_from_euler_packet(packet)
            return

        if isinstance(packet, UM7AllProcPacket):
            self._publish_from_all_proc(packet)

    def _normalized_packet_quaternion(self, packet: UM7QuaternionPacket) -> Quaternion:
        quaternion = (packet.q_w, packet.q_x, packet.q_y, packet.q_z)
        if self.tf_ned_to_enu:
            return ned_to_enu_quaternion(*quaternion)
        return quat_normalize(quaternion)

    def _quat_from_euler_packet(self, packet: UM7EulerPacket) -> Quaternion:
        # Fallback orientation path if quaternion packets are not present.
        quaternion = euler_to_quat(
            math.radians(packet.roll),
            math.radians(packet.pitch),
            math.radians(packet.yaw),
        )
        if self.tf_ned_to_enu:
            return ned_to_enu_quaternion(*quaternion)
        return quaternion

    def _publish_from_all_proc(self, packet: UM7AllProcPacket) -> None:
        imu_msg = self._create_imu_message(packet)
        self.imu_pub.publish(imu_msg)

        if self.mag_updates:
            self.mag_pub.publish(self._create_magnetic_field_message(packet, imu_msg.header))

    def _create_imu_message(self, packet: UM7AllProcPacket) -> Imu:
        imu_msg = Imu()
        imu_msg.header.stamp = self.get_clock().now().to_msg()
        imu_msg.header.frame_id = self.frame_id

        self._populate_orientation(imu_msg)

        gyro, accel, _ = self._processed_vectors(packet)
        imu_msg.angular_velocity = Vector3(
            x=math.radians(gyro[0]),
            y=math.radians(gyro[1]),
            z=math.radians(gyro[2]),
        )
        imu_msg.angular_velocity_covariance = list(ANGULAR_VELOCITY_COVARIANCE)

        imu_msg.linear_acceleration = Vector3(
            x=accel[0] * GRAVITY_MPS2,
            y=accel[1] * GRAVITY_MPS2,
            z=accel[2] * GRAVITY_MPS2,
        )
        imu_msg.linear_acceleration_covariance = list(LINEAR_ACCELERATION_COVARIANCE)
        return imu_msg

    def _populate_orientation(self, imu_msg: Imu) -> None:
        if self._latest_quat is None:
            imu_msg.orientation_covariance[0] = -1.0
            return

        imu_msg.orientation.w = self._latest_quat[0]
        imu_msg.orientation.x = self._latest_quat[1]
        imu_msg.orientation.y = self._latest_quat[2]
        imu_msg.orientation.z = self._latest_quat[3]
        imu_msg.orientation_covariance = list(ORIENTATION_COVARIANCE)

    def _processed_vectors(
        self, packet: UM7AllProcPacket
    ) -> Tuple[Vector3Tuple, Vector3Tuple, Vector3Tuple]:
        gyro = (packet.gyro_proc_x, packet.gyro_proc_y, packet.gyro_proc_z)
        accel = (packet.accel_proc_x, packet.accel_proc_y, packet.accel_proc_z)
        mag = (packet.mag_proc_x, packet.mag_proc_y, packet.mag_proc_z)

        if not self.tf_ned_to_enu:
            return gyro, accel, mag

        return (
            ned_to_enu_vector(*gyro),
            ned_to_enu_vector(*accel),
            ned_to_enu_vector(*mag),
        )

    def _create_magnetic_field_message(self, packet: UM7AllProcPacket, header) -> MagneticField:
        _, _, mag = self._processed_vectors(packet)
        mag_msg = MagneticField()
        mag_msg.header = header
        # UM7 processed mag is in Gauss. Convert to Tesla.
        mag_msg.magnetic_field = Vector3(
            x=mag[0] * GAUSS_TO_TESLA,
            y=mag[1] * GAUSS_TO_TESLA,
            z=mag[2] * GAUSS_TO_TESLA,
        )
        mag_msg.magnetic_field_covariance = list(ZERO_COVARIANCE)
        return mag_msg


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
