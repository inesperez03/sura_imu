from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config_file = PathJoinSubstitution(
        [FindPackageShare("sura_imu"), "config", "imu_filter.yaml"]
    )
    calibration_file = PathJoinSubstitution(
        [FindPackageShare("sura_imu"), "config", "sura_imu_calibration.yaml"]
    )

    raw_imu_topic = LaunchConfiguration("raw_imu_topic")
    mag_topic = LaunchConfiguration("mag_topic")
    calibrated_imu_topic = LaunchConfiguration("calibrated_imu_topic")
    calibrated_mag_topic = LaunchConfiguration("calibrated_mag_topic")
    filtered_imu_topic = LaunchConfiguration("filtered_imu_topic")
    use_calibration = LaunchConfiguration("use_calibration")
    calibration_yaml = LaunchConfiguration("calibration_yaml")
    environment = LaunchConfiguration("environment")
    is_sim = PythonExpression(["'", environment, "' == 'sim'"])

    return LaunchDescription(
        [
            DeclareLaunchArgument("environment", default_value="real"),
            DeclareLaunchArgument(
                "raw_imu_topic",
                default_value="controller/imu_broadcaster/imu",
            ),
            DeclareLaunchArgument(
                "mag_topic",
                default_value="controller/magnetometer_broadcaster/mag",
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
            DeclareLaunchArgument("use_calibration", default_value="true"),
            DeclareLaunchArgument("calibration_yaml", default_value=calibration_file),
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
                condition=UnlessCondition(is_sim),
            ),
            Node(
                package="imu_filter_madgwick",
                executable="imu_filter_madgwick_node",
                name="sura_imu_filter",
                output="screen",
                parameters=[config_file],
                remappings=[
                    ("imu/data_raw", calibrated_imu_topic),
                    ("imu/mag", calibrated_mag_topic),
                    ("imu/data", filtered_imu_topic),
                ],
                condition=UnlessCondition(is_sim),
            ),
            Node(
                package="sura_imu",
                executable="calibration_filter",
                name="sura_imu_sim_relay",
                output="screen",
                parameters=[
                    {
                        "input_imu_topic": raw_imu_topic,
                        "input_mag_topic": mag_topic,
                        "output_imu_topic": filtered_imu_topic,
                        "output_mag_topic": calibrated_mag_topic,
                        "calibration_file": calibration_yaml,
                        "enable_calibration": False,
                    }
                ],
                condition=IfCondition(is_sim),
            ),
        ]
    )
