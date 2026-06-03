#include <ament_index_cpp/get_package_share_directory.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/magnetic_field.hpp>

#include <cmath>
#include <filesystem>
#include <fstream>
#include <memory>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

namespace
{

std::string trim(const std::string & input)
{
  const auto begin = input.find_first_not_of(" \t\r\n");
  if (begin == std::string::npos) {
    return "";
  }
  const auto end = input.find_last_not_of(" \t\r\n");
  return input.substr(begin, end - begin + 1);
}

std::string strip_quotes(std::string value)
{
  value = trim(value);
  if (value.size() >= 2 &&
    ((value.front() == '\'' && value.back() == '\'') ||
    (value.front() == '"' && value.back() == '"')))
  {
    return value.substr(1, value.size() - 2);
  }
  return value;
}

double parse_scalar(
  const std::vector<std::string> & lines,
  const std::string & section,
  const std::string & key,
  const double default_value)
{
  std::string current_section;
  for (const auto & raw_line : lines) {
    const auto line = trim(raw_line);
    if (line.empty() || line[0] == '#') {
      continue;
    }
    if (line.back() == ':') {
      current_section = trim(line.substr(0, line.size() - 1));
      continue;
    }
    if (current_section != section) {
      continue;
    }
    const auto separator = line.find(':');
    if (separator == std::string::npos) {
      continue;
    }
    if (trim(line.substr(0, separator)) == key) {
      return std::stod(strip_quotes(line.substr(separator + 1)));
    }
  }
  return default_value;
}

std::string package_config_path(const std::string & filename)
{
  const auto share = std::filesystem::path(
    ament_index_cpp::get_package_share_directory("sura_imu"));
  const auto source_candidate = share / ".." / ".." / ".." / "src" / "sura_imu" / "config" / filename;
  if (std::filesystem::exists(source_candidate)) {
    return std::filesystem::weakly_canonical(source_candidate).string();
  }
  return (share / "config" / filename).string();
}

struct CalibrationData
{
  double gyro_bias_x{0.0};
  double gyro_bias_y{0.0};
  double gyro_bias_z{0.0};
  double mag_offset_x{0.0};
  double mag_offset_y{0.0};
  double mag_offset_z{0.0};
  double mag_scale_x{1.0};
  double mag_scale_y{1.0};
  double mag_scale_z{1.0};
  double yaw_offset_deg{0.0};
};

CalibrationData load_calibration_file(const std::string & path)
{
  CalibrationData data;
  std::ifstream input(path);
  if (!input.is_open()) {
    return data;
  }

  std::vector<std::string> lines;
  std::string line;
  while (std::getline(input, line)) {
    lines.push_back(line);
  }

  data.gyro_bias_x = parse_scalar(lines, "gyro", "bias_x", 0.0);
  data.gyro_bias_y = parse_scalar(lines, "gyro", "bias_y", 0.0);
  data.gyro_bias_z = parse_scalar(lines, "gyro", "bias_z", 0.0);
  data.mag_offset_x = parse_scalar(lines, "magnetometer", "offset_x", 0.0);
  data.mag_offset_y = parse_scalar(lines, "magnetometer", "offset_y", 0.0);
  data.mag_offset_z = parse_scalar(lines, "magnetometer", "offset_z", 0.0);
  data.mag_scale_x = parse_scalar(lines, "magnetometer", "scale_x", 1.0);
  data.mag_scale_y = parse_scalar(lines, "magnetometer", "scale_y", 1.0);
  data.mag_scale_z = parse_scalar(lines, "magnetometer", "scale_z", 1.0);
  data.yaw_offset_deg = parse_scalar(lines, "compass", "yaw_offset_deg", 0.0);
  return data;
}

std::pair<double, double> rotate_xy(
  const double x,
  const double y,
  const double yaw_deg)
{
  const double yaw = yaw_deg * M_PI / 180.0;
  const double c = std::cos(yaw);
  const double s = std::sin(yaw);
  return {c * x - s * y, s * x + c * y};
}

class CalibrationFilterNode : public rclcpp::Node
{
public:
  CalibrationFilterNode()
  : Node("sura_imu_calibration_filter")
  {
    declare_parameter<std::string>("input_imu_topic", "/imu_broadcaster/imu");
    declare_parameter<std::string>("input_mag_topic", "/magnetometer_broadcaster/mag");
    declare_parameter<std::string>("output_imu_topic", "/sura/imu/data_raw_calibrated");
    declare_parameter<std::string>("output_mag_topic", "/sura/imu/mag_calibrated");
    declare_parameter<std::string>("calibration_file", package_config_path("sura_imu_calibration.yaml"));
    declare_parameter<bool>("enable_calibration", true);

    const auto input_imu_topic = get_parameter("input_imu_topic").as_string();
    const auto input_mag_topic = get_parameter("input_mag_topic").as_string();
    const auto output_imu_topic = get_parameter("output_imu_topic").as_string();
    const auto output_mag_topic = get_parameter("output_mag_topic").as_string();
    const auto calibration_file = get_parameter("calibration_file").as_string();
    enable_calibration_ = get_parameter("enable_calibration").as_bool();
    calibration_ = load_calibration_file(calibration_file);

    imu_pub_ = create_publisher<sensor_msgs::msg::Imu>(output_imu_topic, 20);
    mag_pub_ = create_publisher<sensor_msgs::msg::MagneticField>(output_mag_topic, 20);
    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
      input_imu_topic, 50,
      [this](const sensor_msgs::msg::Imu::SharedPtr msg) { on_imu(*msg); });
    mag_sub_ = create_subscription<sensor_msgs::msg::MagneticField>(
      input_mag_topic, 50,
      [this](const sensor_msgs::msg::MagneticField::SharedPtr msg) { on_mag(*msg); });

    RCLCPP_INFO(
      get_logger(),
      "Calibration filter enabled=%s file=%s | gyro_bias=(%.6f, %.6f, %.6f)",
      enable_calibration_ ? "true" : "false",
      calibration_file.c_str(),
      calibration_.gyro_bias_x,
      calibration_.gyro_bias_y,
      calibration_.gyro_bias_z);
  }

private:
  void on_imu(const sensor_msgs::msg::Imu & msg)
  {
    auto calibrated = msg;
    if (enable_calibration_) {
      calibrated.angular_velocity.x = msg.angular_velocity.x - calibration_.gyro_bias_x;
      calibrated.angular_velocity.y = msg.angular_velocity.y - calibration_.gyro_bias_y;
      calibrated.angular_velocity.z = msg.angular_velocity.z - calibration_.gyro_bias_z;
    }
    imu_pub_->publish(calibrated);
  }

  void on_mag(const sensor_msgs::msg::MagneticField & msg)
  {
    auto calibrated = msg;
    if (enable_calibration_) {
      double x = (msg.magnetic_field.x - calibration_.mag_offset_x) * calibration_.mag_scale_x;
      double y = (msg.magnetic_field.y - calibration_.mag_offset_y) * calibration_.mag_scale_y;
      const double z =
        (msg.magnetic_field.z - calibration_.mag_offset_z) * calibration_.mag_scale_z;
      const auto rotated = rotate_xy(x, y, calibration_.yaw_offset_deg);
      x = rotated.first;
      y = rotated.second;
      calibrated.magnetic_field.x = x;
      calibrated.magnetic_field.y = y;
      calibrated.magnetic_field.z = z;
    }
    mag_pub_->publish(calibrated);
  }

  CalibrationData calibration_;
  bool enable_calibration_{true};
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  rclcpp::Publisher<sensor_msgs::msg::MagneticField>::SharedPtr mag_pub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<sensor_msgs::msg::MagneticField>::SharedPtr mag_sub_;
};

}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CalibrationFilterNode>());
  rclcpp::shutdown();
  return 0;
}
