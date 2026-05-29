import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from sura_imu.paths import package_config_path


def generate_launch_description():
    package_share = get_package_share_directory("sura_imu")
    config_file = os.path.join(package_share, "config", "imu_filter.yaml")
    calibration_file = package_config_path("sura_imu_calibration.yaml")

    raw_imu_topic = LaunchConfiguration("raw_imu_topic")
    mag_topic = LaunchConfiguration("mag_topic")
    calibrated_imu_topic = LaunchConfiguration("calibrated_imu_topic")
    calibrated_mag_topic = LaunchConfiguration("calibrated_mag_topic")
    filtered_imu_topic = LaunchConfiguration("filtered_imu_topic")
    fixed_frame = LaunchConfiguration("fixed_frame")
    world_frame = LaunchConfiguration("world_frame")
    use_mag = LaunchConfiguration("use_mag")
    use_calibration = LaunchConfiguration("use_calibration")
    calibration_yaml = LaunchConfiguration("calibration_yaml")
    publish_filter_tf = LaunchConfiguration("publish_filter_tf")
    gain = LaunchConfiguration("gain")
    zeta = LaunchConfiguration("zeta")
    orientation_stddev = LaunchConfiguration("orientation_stddev")
    constant_dt = LaunchConfiguration("constant_dt")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "raw_imu_topic",
                default_value="controller/imu_broadcaster/imu",
            ),
            DeclareLaunchArgument(
                "mag_topic",
                default_value="magnetometer_broadcaster/mag",
            ),
            DeclareLaunchArgument(
                "calibrated_imu_topic",
                default_value="imu/data_raw",
            ),
            DeclareLaunchArgument(
                "calibrated_mag_topic",
                default_value="imu/mag",
            ),
            DeclareLaunchArgument(
                "filtered_imu_topic",
                default_value="sensors/imu",
            ),
            DeclareLaunchArgument("fixed_frame", default_value="world_ned"),
            DeclareLaunchArgument("world_frame", default_value="ned"),
            DeclareLaunchArgument("use_mag", default_value="true"),
            DeclareLaunchArgument("use_calibration", default_value="true"),
            DeclareLaunchArgument("calibration_yaml", default_value=calibration_file),
            DeclareLaunchArgument("publish_filter_tf", default_value="false"),
            DeclareLaunchArgument("gain", default_value="0.2"),
            DeclareLaunchArgument("zeta", default_value="0.0"),
            DeclareLaunchArgument("orientation_stddev", default_value="0.02"),
            DeclareLaunchArgument("constant_dt", default_value="0.0"),
            Node(
                package="sura_imu",
                executable="calibration_filter",
                name="sura_imu_calibration_filter",
                output="screen",
                parameters=[
                    {
                        "input_imu_topic": raw_imu_topic,
                        "input_mag_topic": mag_topic,
                        "output_imu_topic": calibrated_imu_topic,
                        "output_mag_topic": calibrated_mag_topic,
                        "calibration_file": calibration_yaml,
                        "enable_calibration": use_calibration,
                    }
                ],
            ),
            Node(
                package="imu_filter_madgwick",
                executable="imu_filter_madgwick_node",
                name="sura_imu_filter",
                output="screen",
                parameters=[
                    config_file,
                    {
                        "use_mag": use_mag,
                        "publish_tf": publish_filter_tf,
                        "fixed_frame": fixed_frame,
                        "world_frame": world_frame,
                        "gain": gain,
                        "zeta": zeta,
                        "orientation_stddev": orientation_stddev,
                        "constant_dt": constant_dt,
                    },
                ],
                remappings=[
                    ("imu/data_raw", calibrated_imu_topic),
                    ("imu/mag", calibrated_mag_topic),
                    ("imu/data", filtered_imu_topic),
                ],
            ),
        ]
    )
