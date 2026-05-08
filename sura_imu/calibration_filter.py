import math
import os

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, MagneticField

from sura_imu.paths import package_config_path


def default_calibration_file():
    return package_config_path("sura_imu_calibration.yaml")


def rotate_xy(x_value, y_value, yaw_deg):
    yaw_rad = math.radians(yaw_deg)
    cos_yaw = math.cos(yaw_rad)
    sin_yaw = math.sin(yaw_rad)
    return (
        cos_yaw * x_value - sin_yaw * y_value,
        sin_yaw * x_value + cos_yaw * y_value,
    )


class CalibrationData:
    def __init__(self):
        self.gyro_bias = (0.0, 0.0, 0.0)
        self.mag_offset = (0.0, 0.0, 0.0)
        self.mag_scale = (1.0, 1.0, 1.0)
        self.yaw_offset_deg = 0.0


def _parse_scalar(lines, section, key, default_value):
    current_section = None
    for raw_line in lines:
        stripped = raw_line.strip()
        if not stripped or stripped.startswith("#"):
            continue
        if stripped.endswith(":"):
            current_section = stripped[:-1]
            continue
        if current_section != section or ":" not in stripped:
            continue
        parsed_key, parsed_value = [part.strip() for part in stripped.split(":", 1)]
        if parsed_key == key:
            return float(parsed_value.strip("'\""))
    return default_value


def load_calibration_file(path):
    data = CalibrationData()
    if not path or not os.path.exists(path):
        return data

    with open(path, "r", encoding="utf-8") as input_file:
        lines = input_file.readlines()

    data.gyro_bias = (
        _parse_scalar(lines, "gyro", "bias_x", 0.0),
        _parse_scalar(lines, "gyro", "bias_y", 0.0),
        _parse_scalar(lines, "gyro", "bias_z", 0.0),
    )
    data.mag_offset = (
        _parse_scalar(lines, "magnetometer", "offset_x", 0.0),
        _parse_scalar(lines, "magnetometer", "offset_y", 0.0),
        _parse_scalar(lines, "magnetometer", "offset_z", 0.0),
    )
    data.mag_scale = (
        _parse_scalar(lines, "magnetometer", "scale_x", 1.0),
        _parse_scalar(lines, "magnetometer", "scale_y", 1.0),
        _parse_scalar(lines, "magnetometer", "scale_z", 1.0),
    )
    data.yaw_offset_deg = _parse_scalar(lines, "compass", "yaw_offset_deg", 0.0)
    return data


class CalibrationFilterNode(Node):
    def __init__(self):
        super().__init__("sura_imu_calibration_filter")

        self.declare_parameter("input_imu_topic", "/imu_broadcaster/imu")
        self.declare_parameter("input_mag_topic", "/magnetometer_broadcaster/mag")
        self.declare_parameter("output_imu_topic", "/sura/imu/data_raw_calibrated")
        self.declare_parameter("output_mag_topic", "/sura/imu/mag_calibrated")
        self.declare_parameter("calibration_file", default_calibration_file())
        self.declare_parameter("enable_calibration", True)

        input_imu_topic = self.get_parameter("input_imu_topic").value
        input_mag_topic = self.get_parameter("input_mag_topic").value
        output_imu_topic = self.get_parameter("output_imu_topic").value
        output_mag_topic = self.get_parameter("output_mag_topic").value
        calibration_file = self.get_parameter("calibration_file").value
        self._enable_calibration = self.get_parameter("enable_calibration").value

        self._calibration = load_calibration_file(calibration_file)

        self._imu_pub = self.create_publisher(Imu, output_imu_topic, 20)
        self._mag_pub = self.create_publisher(MagneticField, output_mag_topic, 20)
        self.create_subscription(Imu, input_imu_topic, self._on_imu, 50)
        self.create_subscription(MagneticField, input_mag_topic, self._on_mag, 50)

        self.get_logger().info(
            "Calibration filter enabled=%s file=%s | gyro_bias=%s mag_offset=%s mag_scale=%s yaw_offset_deg=%.3f"
            % (
                self._enable_calibration,
                calibration_file,
                self._calibration.gyro_bias,
                self._calibration.mag_offset,
                self._calibration.mag_scale,
                self._calibration.yaw_offset_deg,
            )
        )

    def _on_imu(self, msg):
        calibrated = Imu()
        calibrated.header = msg.header
        calibrated.orientation = msg.orientation
        calibrated.orientation_covariance = list(msg.orientation_covariance)
        calibrated.linear_acceleration = msg.linear_acceleration
        calibrated.linear_acceleration_covariance = list(msg.linear_acceleration_covariance)

        if self._enable_calibration:
            calibrated.angular_velocity.x = (
                msg.angular_velocity.x - self._calibration.gyro_bias[0]
            )
            calibrated.angular_velocity.y = (
                msg.angular_velocity.y - self._calibration.gyro_bias[1]
            )
            calibrated.angular_velocity.z = (
                msg.angular_velocity.z - self._calibration.gyro_bias[2]
            )
        else:
            calibrated.angular_velocity = msg.angular_velocity
        calibrated.angular_velocity_covariance = list(msg.angular_velocity_covariance)

        self._imu_pub.publish(calibrated)

    def _on_mag(self, msg):
        if self._enable_calibration:
            x_value = (msg.magnetic_field.x - self._calibration.mag_offset[0]) * self._calibration.mag_scale[0]
            y_value = (msg.magnetic_field.y - self._calibration.mag_offset[1]) * self._calibration.mag_scale[1]
            z_value = (msg.magnetic_field.z - self._calibration.mag_offset[2]) * self._calibration.mag_scale[2]
            x_value, y_value = rotate_xy(x_value, y_value, self._calibration.yaw_offset_deg)
        else:
            x_value = msg.magnetic_field.x
            y_value = msg.magnetic_field.y
            z_value = msg.magnetic_field.z

        calibrated = MagneticField()
        calibrated.header = msg.header
        calibrated.magnetic_field.x = x_value
        calibrated.magnetic_field.y = y_value
        calibrated.magnetic_field.z = z_value
        calibrated.magnetic_field_covariance = list(msg.magnetic_field_covariance)

        self._mag_pub.publish(calibrated)


def main():
    rclpy.init()
    node = CalibrationFilterNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
