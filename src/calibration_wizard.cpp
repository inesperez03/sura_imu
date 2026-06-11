#include <ament_index_cpp/get_package_share_directory.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/magnetic_field.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <deque>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <limits>
#include <map>
#include <memory>
#include <mutex>
#include <numeric>
#include <set>
#include <sstream>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

namespace
{

using Vector3 = std::tuple<double, double, double>;

template<typename T>
T clamp(T value, T low, T high)
{
  return std::max(low, std::min(high, value));
}

double mean(const std::vector<double> & values)
{
  if (values.empty()) {
    return 0.0;
  }
  return std::accumulate(values.begin(), values.end(), 0.0) / static_cast<double>(values.size());
}

double stddev(const std::vector<double> & values)
{
  if (values.size() < 2) {
    return 0.0;
  }
  const double mu = mean(values);
  double sum = 0.0;
  for (const auto value : values) {
    sum += (value - mu) * (value - mu);
  }
  return std::sqrt(sum / static_cast<double>(values.size()));
}

double norm3(const Vector3 & vector)
{
  const auto [x, y, z] = vector;
  return std::sqrt(x * x + y * y + z * z);
}

double wrap_angle_deg(double angle)
{
  double wrapped = std::fmod(angle + 180.0, 360.0);
  if (wrapped < 0.0) {
    wrapped += 360.0;
  }
  wrapped -= 180.0;
  return wrapped == -180.0 ? 180.0 : wrapped;
}

double circular_mean_deg(const std::vector<double> & angles)
{
  if (angles.empty()) {
    return 0.0;
  }
  double sin_sum = 0.0;
  double cos_sum = 0.0;
  for (const auto angle : angles) {
    sin_sum += std::sin(angle * M_PI / 180.0);
    cos_sum += std::cos(angle * M_PI / 180.0);
  }
  return std::atan2(sin_sum, cos_sum) * 180.0 / M_PI;
}

double circular_std_deg(const std::vector<double> & angles)
{
  if (angles.size() < 2) {
    return 0.0;
  }
  std::vector<double> sins;
  std::vector<double> coses;
  sins.reserve(angles.size());
  coses.reserve(angles.size());
  for (const auto angle : angles) {
    sins.push_back(std::sin(angle * M_PI / 180.0));
    coses.push_back(std::cos(angle * M_PI / 180.0));
  }
  const double r = std::hypot(mean(sins), mean(coses));
  if (r <= 1.0e-9) {
    return 180.0;
  }
  return std::sqrt(std::max(0.0, -2.0 * std::log(r))) * 180.0 / M_PI;
}

std::string quality_label(double score)
{
  if (score >= 90.0) {
    return "excellent";
  }
  if (score >= 75.0) {
    return "good";
  }
  if (score >= 55.0) {
    return "fair";
  }
  return "poor";
}

std::string format_vector(const Vector3 & vector, int precision)
{
  const auto [x, y, z] = vector;
  std::ostringstream out;
  out.setf(std::ios::fixed);
  out.precision(precision);
  out << "(" << x << ", " << y << ", " << z << ")";
  return out.str();
}

void prompt(const std::string & message)
{
  std::cout << message << "\n> press Enter to continue " << std::flush;
  std::string ignored;
  std::getline(std::cin, ignored);
}

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

std::string package_config_path(const std::string & filename)
{
  auto current = std::filesystem::current_path();
  while (true) {
    const auto source_candidate = current / "src" / "sura_imu" / "config" / filename;
    if (std::filesystem::exists(source_candidate.parent_path())) {
      return source_candidate.string();
    }
    if (!current.has_parent_path() || current == current.parent_path()) {
      break;
    }
    current = current.parent_path();
  }

  try {
    const auto share = std::filesystem::path(
      ament_index_cpp::get_package_share_directory("sura_imu"));
    const auto source_candidate =
      share / ".." / ".." / ".." / "src" / "sura_imu" / "config" / filename;
    if (std::filesystem::exists(source_candidate.parent_path())) {
      return std::filesystem::weakly_canonical(source_candidate).string();
    }
  } catch (const std::exception &) {
  }

  return (std::filesystem::path("src") / "sura_imu" / "config" / filename).string();
}

std::string package_file_path(
  const std::string & package,
  const std::vector<std::string> & relative_parts)
{
  auto current = std::filesystem::current_path();
  while (true) {
    auto source_path = current / "src" / package;
    for (const auto & part : relative_parts) {
      source_path /= part;
    }
    if (std::filesystem::exists(source_path)) {
      return source_path.string();
    }
    if (!current.has_parent_path() || current == current.parent_path()) {
      break;
    }
    current = current.parent_path();
  }

  try {
    auto path = std::filesystem::path(ament_index_cpp::get_package_share_directory(package));
    auto source_path = path / ".." / ".." / ".." / "src" / package;
    for (const auto & part : relative_parts) {
      source_path /= part;
    }
    if (std::filesystem::exists(source_path)) {
      return std::filesystem::weakly_canonical(source_path).string();
    }

    for (const auto & part : relative_parts) {
      path /= part;
    }
    return path.string();
  } catch (const std::exception &) {
    return "";
  }
}

std::map<std::string, std::string> read_simple_yaml_section(
  const std::string & path,
  const std::string & section)
{
  std::ifstream input(path);
  std::map<std::string, std::string> values;
  std::string current;
  std::string line;
  while (std::getline(input, line)) {
    line = trim(line);
    if (line.empty() || line[0] == '#') {
      continue;
    }
    if (line.back() == ':') {
      current = trim(line.substr(0, line.size() - 1));
      continue;
    }
    const auto separator = line.find(':');
    if (separator == std::string::npos || current != section) {
      continue;
    }
    values[trim(line.substr(0, separator))] = strip_quotes(line.substr(separator + 1));
  }
  return values;
}

std::string strip_slashes(std::string value)
{
  value = trim(value);
  while (!value.empty() && value.front() == '/') {
    value.erase(value.begin());
  }
  while (!value.empty() && value.back() == '/') {
    value.pop_back();
  }
  return value;
}

std::string resolve_robot_topic(const std::string & topic, const std::string & robot_namespace)
{
  const auto trimmed_topic = trim(topic);
  if (trimmed_topic.empty() || robot_namespace.empty() || trimmed_topic.front() == '/') {
    return trimmed_topic;
  }
  return "/" + strip_slashes(robot_namespace) + "/" + strip_slashes(trimmed_topic);
}

struct Args
{
  std::string mode{"wizard"};
  std::string bringup_description{
    package_file_path("sura_bringup", {"config", "bringup_description.yaml"})};
  std::string robot_namespace;
  std::string imu_topic{"controller/imu_broadcaster/imu"};
  std::string mag_topic{"controller/magnetometer_broadcaster/mag"};
  double gyro_duration{8.0};
  double mag_duration{35.0};
  double compass_duration{4.0};
  std::string input_calibration;
  std::string output{package_config_path("sura_imu_calibration.yaml")};
  std::string report;
  bool skip_save{false};
  bool skip_report{false};
  bool robot_namespace_cli{false};
  bool imu_topic_cli{false};
  bool mag_topic_cli{false};
};

struct GyroResult
{
  Vector3 bias;
  Vector3 noise;
  double quality_score{0.0};
  std::string quality_label;
  int sample_count{0};
  double duration_sec{0.0};
  std::vector<Vector3> samples;
};

struct MagResult
{
  Vector3 offset;
  Vector3 scale;
  double quality_score{0.0};
  std::string quality_label;
  int sample_count{0};
  double duration_sec{0.0};
  int octant_coverage{0};
  double residual_percent{0.0};
  Vector3 axis_ranges;
  std::vector<Vector3> samples;
};

struct CompassDirectionResult
{
  std::string label;
  double reference_deg{0.0};
  double measured_deg{0.0};
  double offset_deg{0.0};
  double stddev_deg{0.0};
  std::vector<double> headings;
};

struct CompassResult
{
  double yaw_offset_deg{0.0};
  double heading_stddev_deg{0.0};
  double consistency_deg{0.0};
  double quality_score{0.0};
  std::string quality_label;
  int heading_samples{0};
  std::vector<CompassDirectionResult> directions;
};

class CalibrationWizardNode : public rclcpp::Node
{
public:
  CalibrationWizardNode(const std::string & imu_topic, const std::string & mag_topic)
  : Node("sura_imu_calibration_wizard")
  {
    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
      imu_topic, 50,
      [this](const sensor_msgs::msg::Imu::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(mutex_);
        latest_imu_ = *msg;
        has_imu_ = true;
      });
    mag_sub_ = create_subscription<sensor_msgs::msg::MagneticField>(
      mag_topic, 50,
      [this](const sensor_msgs::msg::MagneticField::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(mutex_);
        latest_mag_ = *msg;
        has_mag_ = true;
      });
  }

  bool wait_for_topics(bool require_mag, double timeout_sec)
  {
    const auto end = std::chrono::steady_clock::now() +
      std::chrono::duration<double>(timeout_sec);
    while (std::chrono::steady_clock::now() < end) {
      rclcpp::spin_some(shared_from_this());
      {
        std::lock_guard<std::mutex> lock(mutex_);
        if (has_imu_ && (has_mag_ || !require_mag)) {
          return true;
        }
      }
      rclcpp::sleep_for(std::chrono::milliseconds(100));
    }
    return false;
  }

  std::vector<Vector3> capture_gyro(double duration_sec)
  {
    std::vector<Vector3> samples;
    const auto end = std::chrono::steady_clock::now() +
      std::chrono::duration<double>(duration_sec);
    while (std::chrono::steady_clock::now() < end) {
      rclcpp::spin_some(shared_from_this());
      {
        std::lock_guard<std::mutex> lock(mutex_);
        if (has_imu_) {
          samples.emplace_back(
            latest_imu_.angular_velocity.x,
            latest_imu_.angular_velocity.y,
            latest_imu_.angular_velocity.z);
        }
      }
      rclcpp::sleep_for(std::chrono::milliseconds(50));
    }
    return samples;
  }

  std::vector<Vector3> capture_mag(double duration_sec)
  {
    std::vector<Vector3> samples;
    const auto start = std::chrono::steady_clock::now();
    const auto end = start + std::chrono::duration<double>(duration_sec);
    double last_progress = 0.0;
    while (std::chrono::steady_clock::now() < end) {
      rclcpp::spin_some(shared_from_this());
      {
        std::lock_guard<std::mutex> lock(mutex_);
        if (has_mag_) {
          samples.emplace_back(
            latest_mag_.magnetic_field.x,
            latest_mag_.magnetic_field.y,
            latest_mag_.magnetic_field.z);
        }
      }
      const auto elapsed =
        std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count();
      if (elapsed - last_progress >= 1.0 && samples.size() > 10) {
        last_progress = elapsed;
        std::cout << "[mag] samples=" << samples.size()
                  << " octants=" << compute_octant_coverage(samples) << "/8\n";
      }
      rclcpp::sleep_for(std::chrono::milliseconds(50));
    }
    return samples;
  }

  std::vector<double> capture_heading_window(
    double duration_sec,
    const Vector3 & mag_offset,
    const Vector3 & mag_scale)
  {
    std::vector<double> headings;
    const auto end = std::chrono::steady_clock::now() +
      std::chrono::duration<double>(duration_sec);
    while (std::chrono::steady_clock::now() < end) {
      rclcpp::spin_some(shared_from_this());
      sensor_msgs::msg::Imu imu;
      sensor_msgs::msg::MagneticField mag;
      bool ready = false;
      {
        std::lock_guard<std::mutex> lock(mutex_);
        ready = has_imu_ && has_mag_;
        imu = latest_imu_;
        mag = latest_mag_;
      }
      if (ready) {
        const Vector3 accel{
          imu.linear_acceleration.x,
          imu.linear_acceleration.y,
          imu.linear_acceleration.z};
        const Vector3 mag_vector{
          mag.magnetic_field.x,
          mag.magnetic_field.y,
          mag.magnetic_field.z};
        const auto heading = tilt_compensated_heading_deg(accel, mag_vector, mag_offset, mag_scale);
        if (std::isfinite(heading)) {
          headings.push_back(heading);
        }
      }
      rclcpp::sleep_for(std::chrono::milliseconds(50));
    }
    return headings;
  }

private:
  static int compute_octant_coverage(const std::vector<Vector3> & samples)
  {
    std::set<std::tuple<int, int, int>> octants;
    for (const auto & sample : samples) {
      const auto [x, y, z] = sample;
      octants.emplace(x >= 0.0, y >= 0.0, z >= 0.0);
    }
    return static_cast<int>(octants.size());
  }

  static double tilt_compensated_heading_deg(
    const Vector3 & accel,
    const Vector3 & mag,
    const Vector3 & mag_offset,
    const Vector3 & mag_scale)
  {
    auto [ax, ay, az] = accel;
    auto [mx, my, mz] = mag;
    const auto [ox, oy, oz] = mag_offset;
    const auto [sx, sy, sz] = mag_scale;
    mx = (mx - ox) * sx;
    my = (my - oy) * sy;
    mz = (mz - oz) * sz;
    const double accel_norm = norm3(accel);
    const double mag_norm = norm3({mx, my, mz});
    if (accel_norm < 1.0e-6 || mag_norm < 1.0e-9) {
      return std::numeric_limits<double>::quiet_NaN();
    }
    ax /= accel_norm;
    ay /= accel_norm;
    az /= accel_norm;
    const double roll = std::atan2(ay, az);
    const double pitch = std::atan2(-ax, std::sqrt(ay * ay + az * az));
    const double xh = mx * std::cos(pitch) + mz * std::sin(pitch);
    const double yh =
      mx * std::sin(roll) * std::sin(pitch) +
      my * std::cos(roll) -
      mz * std::sin(roll) * std::cos(pitch);
    if (std::abs(xh) < 1.0e-12 && std::abs(yh) < 1.0e-12) {
      return std::numeric_limits<double>::quiet_NaN();
    }
    double heading = std::atan2(yh, xh) * 180.0 / M_PI;
    if (heading < 0.0) {
      heading += 360.0;
    }
    return heading;
  }

  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<sensor_msgs::msg::MagneticField>::SharedPtr mag_sub_;
  std::mutex mutex_;
  sensor_msgs::msg::Imu latest_imu_;
  sensor_msgs::msg::MagneticField latest_mag_;
  bool has_imu_{false};
  bool has_mag_{false};
};

int compute_octant_coverage(const std::vector<Vector3> & samples)
{
  std::set<std::tuple<int, int, int>> octants;
  for (const auto & sample : samples) {
    const auto [x, y, z] = sample;
    octants.emplace(x >= 0.0, y >= 0.0, z >= 0.0);
  }
  return static_cast<int>(octants.size());
}

MagResult compute_mag_calibration(const std::vector<Vector3> & samples)
{
  std::vector<double> xs;
  std::vector<double> ys;
  std::vector<double> zs;
  for (const auto & sample : samples) {
    const auto [x, y, z] = sample;
    xs.push_back(x);
    ys.push_back(y);
    zs.push_back(z);
  }
  const auto minmax_x = std::minmax_element(xs.begin(), xs.end());
  const auto minmax_y = std::minmax_element(ys.begin(), ys.end());
  const auto minmax_z = std::minmax_element(zs.begin(), zs.end());
  const Vector3 offset{
    0.5 * (*minmax_x.first + *minmax_x.second),
    0.5 * (*minmax_y.first + *minmax_y.second),
    0.5 * (*minmax_z.first + *minmax_z.second)};
  const Vector3 half_ranges{
    0.5 * (*minmax_x.second - *minmax_x.first),
    0.5 * (*minmax_y.second - *minmax_y.first),
    0.5 * (*minmax_z.second - *minmax_z.first)};
  const auto [hx0, hy0, hz0] = half_ranges;
  const double hx = std::max(hx0, 1.0e-9);
  const double hy = std::max(hy0, 1.0e-9);
  const double hz = std::max(hz0, 1.0e-9);
  const double radius = (hx + hy + hz) / 3.0;
  const Vector3 scale{radius / hx, radius / hy, radius / hz};
  const auto [ox, oy, oz] = offset;
  const auto [sx, sy, sz] = scale;
  std::vector<double> radii;
  for (const auto & sample : samples) {
    const auto [x, y, z] = sample;
    radii.push_back(norm3({(x - ox) * sx, (y - oy) * sy, (z - oz) * sz}));
  }
  const double residual = 100.0 * stddev(radii) / std::max(mean(radii), 1.0e-9);
  const int octants = compute_octant_coverage(samples);
  const double sample_score = clamp(static_cast<double>(samples.size()) / 1800.0, 0.0, 1.0);
  const double octant_score = static_cast<double>(octants) / 8.0;
  const double balance_score = clamp(std::min({hx, hy, hz}) / std::max({hx, hy, hz}), 0.0, 1.0);
  const double residual_score = clamp(1.0 - residual / 35.0, 0.0, 1.0);
  const double quality = 100.0 * (
    0.30 * sample_score + 0.25 * octant_score + 0.20 * balance_score + 0.25 * residual_score);
  return {
    offset,
    scale,
    quality,
    quality_label(quality),
    static_cast<int>(samples.size()),
    0.0,
    octants,
    residual,
    {2.0 * hx0, 2.0 * hy0, 2.0 * hz0},
    samples};
}

GyroResult run_gyro(CalibrationWizardNode & node, double duration)
{
  std::cout << "\n=== Gyroscope calibration ===\n";
  std::cout << "Place the robot completely still and avoid vibrations.\n";
  prompt("When it is motionless, capture will start");
  const auto start = std::chrono::steady_clock::now();
  const auto samples = node.capture_gyro(duration);
  const double elapsed =
    std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count();
  if (samples.size() < 20) {
    throw std::runtime_error("Not enough gyroscope samples were received for calibration.");
  }
  std::vector<double> xs;
  std::vector<double> ys;
  std::vector<double> zs;
  for (const auto & sample : samples) {
    const auto [x, y, z] = sample;
    xs.push_back(x);
    ys.push_back(y);
    zs.push_back(z);
  }
  const Vector3 bias{mean(xs), mean(ys), mean(zs)};
  const Vector3 noise{stddev(xs), stddev(ys), stddev(zs)};
  const auto [nx, ny, nz] = noise;
  const double score =
    100.0 * clamp(1.0 - mean(std::vector<double>{nx, ny, nz}) / 0.03, 0.0, 1.0);
  std::cout << "Samples used: " << samples.size() << "\n";
  std::cout << "Estimated bias [rad/s]: " << format_vector(bias, 6) << "\n";
  std::cout << "Noise stddev [rad/s]:   " << format_vector(noise, 6) << "\n";
  std::cout << "Quality: " << quality_label(score) << " (" << score << "/100)\n";
  return {
    bias,
    noise,
    score,
    quality_label(score),
    static_cast<int>(samples.size()),
    elapsed,
    samples};
}

MagResult run_mag(CalibrationWizardNode & node, double duration)
{
  std::cout << "\n=== Magnetometer calibration ===\n";
  std::cout << "Move the robot slowly through many orientations, away from metal.\n";
  prompt("When ready, magnetometer capture will start");
  const auto start = std::chrono::steady_clock::now();
  const auto samples = node.capture_mag(duration);
  const double elapsed =
    std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count();
  if (samples.size() < 100) {
    throw std::runtime_error("Not enough magnetometer samples were received.");
  }
  auto result = compute_mag_calibration(samples);
  result.duration_sec = elapsed;
  std::cout << "Samples used: " << result.sample_count << "\n";
  std::cout << "Offset hard-iron [T]: " << format_vector(result.offset, 7) << "\n";
  std::cout << "Soft-iron scale:      " << format_vector(result.scale, 5) << "\n";
  std::cout << "3D coverage: " << result.octant_coverage << "/8 octants\n";
  std::cout << "Residual radial: " << result.residual_percent << "%\n";
  std::cout << "Quality: " << result.quality_label << " (" << result.quality_score << "/100)\n";
  return result;
}

CompassResult run_compass(CalibrationWizardNode & node, const MagResult * mag, double duration)
{
  const std::vector<std::pair<std::string, double>> directions{
    {"North", 0.0}, {"East", 90.0}, {"South", 180.0}, {"West", 270.0}};
  const Vector3 offset = mag ? mag->offset : Vector3{0.0, 0.0, 0.0};
  const Vector3 scale = mag ? mag->scale : Vector3{1.0, 1.0, 1.0};
  std::vector<double> offsets;
  std::vector<double> deviations;
  std::vector<CompassDirectionResult> direction_results;
  int total_samples = 0;
  for (const auto & [label, reference] : directions) {
    std::cout << "\nPoint the robot toward " << label << " (" << reference << " deg).\n";
    prompt("Keep it still and capture will start");
    const auto headings = node.capture_heading_window(duration, offset, scale);
    if (headings.size() < 15) {
      throw std::runtime_error("Could not estimate heading reliably for " + label);
    }
    const double measured = circular_mean_deg(headings);
    const double heading_std = circular_std_deg(headings);
    const double partial_offset = wrap_angle_deg(reference - measured);
    offsets.push_back(partial_offset);
    deviations.push_back(heading_std);
    direction_results.push_back(
      {label, reference, measured, partial_offset, heading_std, headings});
    total_samples += static_cast<int>(headings.size());
    std::cout << label << ": measured heading=" << measured
              << " deg, partial offset=" << partial_offset
              << " deg, spread=" << heading_std << " deg\n";
  }
  const double yaw_offset = circular_mean_deg(offsets);
  const double consistency = circular_std_deg(offsets);
  const double heading_std = mean(deviations);
  const double score = 100.0 * (
    0.55 * clamp(1.0 - heading_std / 12.0, 0.0, 1.0) +
    0.45 * clamp(1.0 - consistency / 20.0, 0.0, 1.0));
  std::cout << "\nFinal yaw offset: " << yaw_offset << " deg\n";
  std::cout << "Quality: " << quality_label(score) << " (" << score << "/100)\n";
  return {
    yaw_offset,
    heading_std,
    consistency,
    score,
    quality_label(score),
    total_samples,
    direction_results};
}

std::map<std::string, std::string> read_section(const std::string & path, const std::string & section)
{
  std::ifstream input(path);
  std::map<std::string, std::string> values;
  std::string current;
  std::string line;
  while (std::getline(input, line)) {
    line = trim(line);
    if (line.empty() || line[0] == '#') {
      continue;
    }
    if (line.back() == ':') {
      current = line.substr(0, line.size() - 1);
      continue;
    }
    const auto separator = line.find(':');
    if (separator == std::string::npos || current != section) {
      continue;
    }
    values[trim(line.substr(0, separator))] = strip_quotes(line.substr(separator + 1));
  }
  return values;
}

std::unique_ptr<MagResult> read_saved_mag(const std::string & path)
{
  const auto values = read_section(path, "magnetometer");
  const auto available = values.find("available");
  if (values.empty() || available == values.end() || available->second != "true") {
    return nullptr;
  }
  auto result = std::make_unique<MagResult>();
  result->offset = {
    std::stod(values.at("offset_x")),
    std::stod(values.at("offset_y")),
    std::stod(values.at("offset_z"))};
  result->scale = {
    std::stod(values.at("scale_x")),
    std::stod(values.at("scale_y")),
    std::stod(values.at("scale_z"))};
  result->quality_score = values.count("quality_score") ? std::stod(values.at("quality_score")) : 0.0;
  result->quality_label = values.count("quality_label") ? values.at("quality_label") : "unknown";
  return result;
}

std::string default_report_path(const std::string & output)
{
  auto path = std::filesystem::path(output);
  path.replace_extension(".html");
  return path.string();
}

std::string html_escape(const std::string & text)
{
  std::string escaped;
  for (const auto c : text) {
    switch (c) {
      case '&':
        escaped += "&amp;";
        break;
      case '<':
        escaped += "&lt;";
        break;
      case '>':
        escaped += "&gt;";
        break;
      case '"':
        escaped += "&quot;";
        break;
      default:
        escaped += c;
        break;
    }
  }
  return escaped;
}

double vector_component(const Vector3 & vector, int axis)
{
  const auto [x, y, z] = vector;
  if (axis == 0) {
    return x;
  }
  if (axis == 1) {
    return y;
  }
  return z;
}

Vector3 corrected_mag_sample(const Vector3 & sample, const MagResult & mag)
{
  const auto [x, y, z] = sample;
  const auto [ox, oy, oz] = mag.offset;
  const auto [sx, sy, sz] = mag.scale;
  return {(x - ox) * sx, (y - oy) * sy, (z - oz) * sz};
}

std::string svg_axes_label(int axis)
{
  if (axis == 0) {
    return "X";
  }
  if (axis == 1) {
    return "Y";
  }
  return "Z";
}

std::string svg_scatter(
  const std::vector<Vector3> & samples,
  int axis_x,
  int axis_y,
  const std::string & title,
  const std::string & color)
{
  constexpr double width = 360.0;
  constexpr double height = 300.0;
  constexpr double margin = 34.0;
  std::ostringstream svg;
  svg.setf(std::ios::fixed);
  svg.precision(3);

  svg << "<svg viewBox=\"0 0 " << width << " " << height << "\" role=\"img\" "
      << "aria-label=\"" << html_escape(title) << "\">";
  svg << "<rect width=\"100%\" height=\"100%\" fill=\"#ffffff\"/>";
  svg << "<text x=\"14\" y=\"22\" font-size=\"14\" font-weight=\"700\">" << html_escape(title)
      << "</text>";
  svg << "<line x1=\"" << margin << "\" y1=\"" << height - margin << "\" x2=\""
      << width - 12.0 << "\" y2=\"" << height - margin
      << "\" stroke=\"#9aa0a6\" stroke-width=\"1\"/>";
  svg << "<line x1=\"" << margin << "\" y1=\"32\" x2=\"" << margin << "\" y2=\""
      << height - margin << "\" stroke=\"#9aa0a6\" stroke-width=\"1\"/>";
  svg << "<text x=\"" << width - 36.0 << "\" y=\"" << height - 10.0
      << "\" font-size=\"11\">" << svg_axes_label(axis_x) << "</text>";
  svg << "<text x=\"8\" y=\"42\" font-size=\"11\">" << svg_axes_label(axis_y) << "</text>";

  if (samples.empty()) {
    svg << "<text x=\"80\" y=\"155\" font-size=\"13\" fill=\"#5f6368\">no samples</text></svg>";
    return svg.str();
  }

  double min_x = std::numeric_limits<double>::max();
  double max_x = std::numeric_limits<double>::lowest();
  double min_y = std::numeric_limits<double>::max();
  double max_y = std::numeric_limits<double>::lowest();
  for (const auto & sample : samples) {
    const double x = vector_component(sample, axis_x);
    const double y = vector_component(sample, axis_y);
    min_x = std::min(min_x, x);
    max_x = std::max(max_x, x);
    min_y = std::min(min_y, y);
    max_y = std::max(max_y, y);
  }
  const double pad_x = std::max((max_x - min_x) * 0.08, 1.0e-9);
  const double pad_y = std::max((max_y - min_y) * 0.08, 1.0e-9);
  min_x -= pad_x;
  max_x += pad_x;
  min_y -= pad_y;
  max_y += pad_y;
  const double plot_w = width - margin - 12.0;
  const double plot_h = height - margin - 32.0;
  for (const auto & sample : samples) {
    const double x = vector_component(sample, axis_x);
    const double y = vector_component(sample, axis_y);
    const double px = margin + (x - min_x) / (max_x - min_x) * plot_w;
    const double py = height - margin - (y - min_y) / (max_y - min_y) * plot_h;
    svg << "<circle cx=\"" << px << "\" cy=\"" << py << "\" r=\"1.6\" fill=\"" << color
        << "\" fill-opacity=\"0.55\"/>";
  }
  svg << "</svg>";
  return svg.str();
}

std::string svg_series(
  const std::vector<Vector3> & samples,
  const std::string & title,
  const std::string & y_label)
{
  constexpr double width = 760.0;
  constexpr double height = 280.0;
  constexpr double margin = 38.0;
  std::ostringstream svg;
  svg.setf(std::ios::fixed);
  svg.precision(3);
  svg << "<svg viewBox=\"0 0 " << width << " " << height << "\" role=\"img\" "
      << "aria-label=\"" << html_escape(title) << "\">";
  svg << "<rect width=\"100%\" height=\"100%\" fill=\"#ffffff\"/>";
  svg << "<text x=\"14\" y=\"22\" font-size=\"14\" font-weight=\"700\">" << html_escape(title)
      << "</text>";
  svg << "<line x1=\"" << margin << "\" y1=\"" << height - margin << "\" x2=\""
      << width - 16.0 << "\" y2=\"" << height - margin
      << "\" stroke=\"#9aa0a6\" stroke-width=\"1\"/>";
  svg << "<line x1=\"" << margin << "\" y1=\"32\" x2=\"" << margin << "\" y2=\""
      << height - margin << "\" stroke=\"#9aa0a6\" stroke-width=\"1\"/>";
  svg << "<text x=\"8\" y=\"42\" font-size=\"11\">" << html_escape(y_label) << "</text>";
  if (samples.size() < 2) {
    svg << "<text x=\"100\" y=\"145\" font-size=\"13\" fill=\"#5f6368\">no samples</text></svg>";
    return svg.str();
  }

  double min_y = std::numeric_limits<double>::max();
  double max_y = std::numeric_limits<double>::lowest();
  for (const auto & sample : samples) {
    for (int axis = 0; axis < 3; ++axis) {
      const double value = vector_component(sample, axis);
      min_y = std::min(min_y, value);
      max_y = std::max(max_y, value);
    }
  }
  const double pad_y = std::max((max_y - min_y) * 0.12, 1.0e-9);
  min_y -= pad_y;
  max_y += pad_y;
  const double plot_w = width - margin - 16.0;
  const double plot_h = height - margin - 32.0;
  const std::vector<std::string> colors{"#2563eb", "#16a34a", "#dc2626"};
  for (int axis = 0; axis < 3; ++axis) {
    svg << "<polyline fill=\"none\" stroke=\"" << colors[axis]
        << "\" stroke-width=\"1.8\" points=\"";
    for (std::size_t index = 0; index < samples.size(); ++index) {
      const double x = margin + static_cast<double>(index) /
        static_cast<double>(samples.size() - 1) * plot_w;
      const double y = height - margin -
        (vector_component(samples[index], axis) - min_y) / (max_y - min_y) * plot_h;
      svg << x << "," << y << " ";
    }
    svg << "\"/>";
    svg << "<text x=\"" << width - 74.0 << "\" y=\"" << 50.0 + axis * 18.0
        << "\" font-size=\"12\" fill=\"" << colors[axis] << "\">"
        << svg_axes_label(axis) << "</text>";
  }
  svg << "</svg>";
  return svg.str();
}

std::string svg_heading_bars(const CompassResult & compass)
{
  constexpr double width = 760.0;
  constexpr double height = 260.0;
  std::ostringstream svg;
  svg.setf(std::ios::fixed);
  svg.precision(2);
  svg << "<svg viewBox=\"0 0 " << width << " " << height << "\" role=\"img\" "
      << "aria-label=\"compass by orientation\">";
  svg << "<rect width=\"100%\" height=\"100%\" fill=\"#ffffff\"/>";
  svg << "<text x=\"14\" y=\"22\" font-size=\"14\" font-weight=\"700\">Compass: offset by orientation</text>";
  if (compass.directions.empty()) {
    svg << "<text x=\"110\" y=\"135\" font-size=\"13\" fill=\"#5f6368\">no samples</text></svg>";
    return svg.str();
  }
  const double zero_y = 130.0;
  const double scale = 3.2;
  svg << "<line x1=\"42\" y1=\"" << zero_y << "\" x2=\"" << width - 24.0 << "\" y2=\""
      << zero_y << "\" stroke=\"#9aa0a6\"/>";
  const double slot = (width - 100.0) / static_cast<double>(compass.directions.size());
  for (std::size_t index = 0; index < compass.directions.size(); ++index) {
    const auto & direction = compass.directions[index];
    const double center = 70.0 + slot * static_cast<double>(index) + slot * 0.5;
    const double bar_h = clamp(direction.offset_deg * scale, -86.0, 86.0);
    const double y = bar_h >= 0.0 ? zero_y - bar_h : zero_y;
    svg << "<rect x=\"" << center - 24.0 << "\" y=\"" << y << "\" width=\"48\" height=\""
        << std::abs(bar_h) << "\" fill=\"#0f766e\" fill-opacity=\"0.82\"/>";
    svg << "<text x=\"" << center - 18.0 << "\" y=\"224\" font-size=\"12\">"
        << html_escape(direction.label) << "</text>";
    svg << "<text x=\"" << center - 28.0 << "\" y=\"" << (bar_h >= 0.0 ? y - 8.0 : y + std::abs(bar_h) + 18.0)
        << "\" font-size=\"11\">" << direction.offset_deg << " deg</text>";
  }
  svg << "</svg>";
  return svg.str();
}

void write_report(
  const std::string & output,
  const Args & args,
  const GyroResult * gyro,
  const MagResult * mag,
  const CompassResult * compass)
{
  const auto directory = std::filesystem::path(output).parent_path();
  if (!directory.empty()) {
    std::filesystem::create_directories(directory);
  }
  std::ofstream file(output);
  file << "<!doctype html><html lang=\"en\"><head><meta charset=\"utf-8\">";
  file << "<title>SURA IMU calibration report</title>";
  file << "<style>";
  file << "body{font-family:Inter,Arial,sans-serif;margin:24px;color:#1f2937;background:#f6f7f9}";
  file << "main{max-width:1120px;margin:auto}h1{font-size:28px;margin:0 0 6px}";
  file << "h2{font-size:20px;margin:26px 0 12px}.muted{color:#667085}";
  file << ".grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(320px,1fr));gap:14px}";
  file << ".panel{background:white;border:1px solid #d9dee7;border-radius:8px;padding:14px}";
  file << ".metric{display:inline-block;margin:6px 14px 6px 0}.metric b{display:block;font-size:18px}";
  file << "svg{width:100%;height:auto;border:1px solid #e5e7eb;border-radius:6px;background:white}";
  file << "</style></head><body><main>";
  file << "<h1>SURA IMU calibration report</h1>";
  file << "<p class=\"muted\">Mode " << html_escape(args.mode) << " | IMU "
       << html_escape(args.imu_topic) << " | Mag " << html_escape(args.mag_topic) << "</p>";

  if (gyro) {
    file << "<h2>Gyroscope</h2><div class=\"panel\">";
    file << "<span class=\"metric\"><b>" << gyro->quality_score << "/100</b>quality "
         << html_escape(gyro->quality_label) << "</span>";
    file << "<span class=\"metric\"><b>" << gyro->sample_count << "</b>samples</span>";
    file << "<span class=\"metric\"><b>" << format_vector(gyro->bias, 6)
         << "</b>bias rad/s</span>";
    file << svg_series(gyro->samples, "Still gyroscope", "rad/s");
    file << "</div>";
  }

  if (mag) {
    std::vector<Vector3> corrected;
    corrected.reserve(mag->samples.size());
    for (const auto & sample : mag->samples) {
      corrected.push_back(corrected_mag_sample(sample, *mag));
    }
    file << "<h2>Magnetometer</h2><div class=\"panel\">";
    file << "<span class=\"metric\"><b>" << mag->quality_score << "/100</b>quality "
         << html_escape(mag->quality_label) << "</span>";
    file << "<span class=\"metric\"><b>" << mag->octant_coverage << "/8</b>3D coverage</span>";
    file << "<span class=\"metric\"><b>" << mag->residual_percent << "%</b>radial residual</span>";
    file << "<span class=\"metric\"><b>" << format_vector(mag->offset, 7)
         << "</b>offset T</span>";
    file << "<div class=\"grid\">";
    file << svg_scatter(mag->samples, 0, 1, "Raw X/Y", "#64748b");
    file << svg_scatter(mag->samples, 0, 2, "Raw X/Z", "#64748b");
    file << svg_scatter(mag->samples, 1, 2, "Raw Y/Z", "#64748b");
    file << svg_scatter(corrected, 0, 1, "Corrected X/Y", "#2563eb");
    file << svg_scatter(corrected, 0, 2, "Corrected X/Z", "#2563eb");
    file << svg_scatter(corrected, 1, 2, "Corrected Y/Z", "#2563eb");
    file << "</div></div>";
  }

  if (compass) {
    file << "<h2>Compass</h2><div class=\"panel\">";
    file << "<span class=\"metric\"><b>" << compass->quality_score << "/100</b>quality "
         << html_escape(compass->quality_label) << "</span>";
    file << "<span class=\"metric\"><b>" << compass->yaw_offset_deg << " deg</b>offset yaw</span>";
    file << "<span class=\"metric\"><b>" << compass->consistency_deg
         << " deg</b>consistency</span>";
    file << svg_heading_bars(*compass);
    file << "</div>";
  }

  file << "</main></body></html>";
}

void write_yaml(
  const std::string & output,
  const Args & args,
  const GyroResult * gyro,
  const MagResult * mag,
  const CompassResult * compass)
{
  const auto directory = std::filesystem::path(output).parent_path();
  if (!directory.empty()) {
    std::filesystem::create_directories(directory);
  }
  std::ofstream file(output);
  file << "sura_imu_calibration:\n";
  file << "  metadata:\n";
  file << "    generated_at_unix: "
       << std::chrono::duration<double>(std::chrono::system_clock::now().time_since_epoch()).count()
       << "\n";
  file << "    imu_topic: '" << args.imu_topic << "'\n";
  file << "    mag_topic: '" << args.mag_topic << "'\n";
  file << "    mode: '" << args.mode << "'\n";
  file << "  gyro:\n";
  if (!gyro) {
    file << "    available: false\n";
  } else {
    const auto [bx, by, bz] = gyro->bias;
    const auto [nx, ny, nz] = gyro->noise;
    file << "    available: true\n";
    file << "    quality_score: " << gyro->quality_score << "\n";
    file << "    quality_label: '" << gyro->quality_label << "'\n";
    file << "    sample_count: " << gyro->sample_count << "\n";
    file << "    duration_sec: " << gyro->duration_sec << "\n";
    file << "    bias_x: " << bx << "\n";
    file << "    bias_y: " << by << "\n";
    file << "    bias_z: " << bz << "\n";
    file << "    noise_stddev_x: " << nx << "\n";
    file << "    noise_stddev_y: " << ny << "\n";
    file << "    noise_stddev_z: " << nz << "\n";
  }
  file << "  magnetometer:\n";
  if (!mag) {
    file << "    available: false\n";
  } else {
    const auto [ox, oy, oz] = mag->offset;
    const auto [sx, sy, sz] = mag->scale;
    file << "    available: true\n";
    file << "    quality_score: " << mag->quality_score << "\n";
    file << "    quality_label: '" << mag->quality_label << "'\n";
    file << "    sample_count: " << mag->sample_count << "\n";
    file << "    duration_sec: " << mag->duration_sec << "\n";
    file << "    offset_x: " << ox << "\n";
    file << "    offset_y: " << oy << "\n";
    file << "    offset_z: " << oz << "\n";
    file << "    scale_x: " << sx << "\n";
    file << "    scale_y: " << sy << "\n";
    file << "    scale_z: " << sz << "\n";
    file << "    octant_coverage: " << mag->octant_coverage << "\n";
    file << "    residual_percent: " << mag->residual_percent << "\n";
  }
  file << "  compass:\n";
  if (!compass) {
    file << "    available: false\n";
  } else {
    file << "    available: true\n";
    file << "    quality_score: " << compass->quality_score << "\n";
    file << "    quality_label: '" << compass->quality_label << "'\n";
    file << "    yaw_offset_deg: " << compass->yaw_offset_deg << "\n";
    file << "    heading_stddev_deg: " << compass->heading_stddev_deg << "\n";
    file << "    consistency_deg: " << compass->consistency_deg << "\n";
    file << "    heading_samples: " << compass->heading_samples << "\n";
  }
}

Args parse_args(int argc, char ** argv)
{
  Args args;
#ifdef SURA_IMU_DEFAULT_MODE
  args.mode = SURA_IMU_DEFAULT_MODE;
#endif
  for (int i = 1; i < argc; ++i) {
    const std::string key = argv[i];
    auto take_value = [&](const std::string & name) {
        if (i + 1 >= argc) {
          throw std::runtime_error("Missing value for " + name);
        }
        return std::string(argv[++i]);
      };
    if (key == "--mode") {
      args.mode = take_value(key);
    } else if (key == "--bringup-description") {
      args.bringup_description = take_value(key);
    } else if (key == "--robot-namespace" || key == "--namespace") {
      args.robot_namespace = strip_slashes(take_value(key));
      args.robot_namespace_cli = true;
    } else if (key == "--imu-topic") {
      args.imu_topic = take_value(key);
      args.imu_topic_cli = true;
    } else if (key == "--mag-topic") {
      args.mag_topic = take_value(key);
      args.mag_topic_cli = true;
    } else if (key == "--gyro-duration") {
      args.gyro_duration = std::stod(take_value(key));
    } else if (key == "--mag-duration") {
      args.mag_duration = std::stod(take_value(key));
    } else if (key == "--compass-duration") {
      args.compass_duration = std::stod(take_value(key));
    } else if (key == "--input-calibration") {
      args.input_calibration = take_value(key);
    } else if (key == "--output") {
      args.output = take_value(key);
    } else if (key == "--report") {
      args.report = take_value(key);
    } else if (key == "--skip-save") {
      args.skip_save = true;
    } else if (key == "--skip-report") {
      args.skip_report = true;
    } else if (key == "--help" || key == "-h") {
      std::cout << "Usage: calibration_wizard [--mode wizard|gyro|mag|compass] [options]\n"
                << "  --bringup-description PATH bringup YAML used for defaults\n"
                << "  --robot-namespace NS       override bringup robot.name\n"
                << "  --imu-topic TOPIC          override bringup imu.raw_imu_topic\n"
                << "  --mag-topic TOPIC          override bringup imu.mag_topic\n"
                << "  --report PATH              write an HTML report with plots\n"
                << "  --skip-report              do not generate an HTML report\n";
      std::exit(0);
    }
  }
  if (args.mode != "wizard" && args.mode != "gyro" && args.mode != "mag" && args.mode != "compass") {
    throw std::runtime_error("Unsupported mode: " + args.mode);
  }
  return args;
}

void apply_bringup_defaults(Args & args)
{
  if (args.bringup_description.empty() || !std::filesystem::exists(args.bringup_description)) {
    return;
  }

  const auto robot = read_simple_yaml_section(args.bringup_description, "robot");
  const auto imu = read_simple_yaml_section(args.bringup_description, "imu");

  if (!args.robot_namespace_cli && robot.count("name") > 0) {
    args.robot_namespace = strip_slashes(robot.at("name"));
  }
  if (!args.imu_topic_cli && imu.count("raw_imu_topic") > 0) {
    args.imu_topic = imu.at("raw_imu_topic");
  }
  if (!args.mag_topic_cli && imu.count("mag_topic") > 0) {
    args.mag_topic = imu.at("mag_topic");
  }
}

}  // namespace

int main(int argc, char ** argv)
{
  Args args;
  try {
    args = parse_args(argc, argv);
  } catch (const std::exception & error) {
    std::cerr << "Error: " << error.what() << "\n";
    return 1;
  }

  apply_bringup_defaults(args);

  rclcpp::init(argc, argv);
  args.imu_topic = resolve_robot_topic(args.imu_topic, args.robot_namespace);
  args.mag_topic = resolve_robot_topic(args.mag_topic, args.robot_namespace);
  auto node = std::make_shared<CalibrationWizardNode>(args.imu_topic, args.mag_topic);

  std::unique_ptr<GyroResult> gyro;
  std::unique_ptr<MagResult> mag;
  std::unique_ptr<CompassResult> compass;

  try {
    const bool require_mag = args.mode == "wizard" || args.mode == "mag" || args.mode == "compass";
    std::cout << "\nWaiting for ROS 2 calibration data...\n";
    std::cout << "IMU: " << args.imu_topic << "\n";
    std::cout << "Mag: " << args.mag_topic << "\n";
    if (!node->wait_for_topics(require_mag, 12.0)) {
      throw std::runtime_error(
              "No IMU or magnetometer data received. Check topics and broadcasters.");
    }
    if (args.mode == "wizard" || args.mode == "gyro") {
      gyro = std::make_unique<GyroResult>(run_gyro(*node, args.gyro_duration));
    }
    if (args.mode == "wizard" || args.mode == "mag") {
      mag = std::make_unique<MagResult>(run_mag(*node, args.mag_duration));
    }
    if (args.mode == "wizard" || args.mode == "compass") {
      if (!mag && !args.input_calibration.empty()) {
        mag = read_saved_mag(args.input_calibration);
      }
      compass = std::make_unique<CompassResult>(
        run_compass(*node, mag.get(), args.compass_duration));
    }
    if (!args.skip_save) {
      write_yaml(args.output, args, gyro.get(), mag.get(), compass.get());
      std::cout << "\nResults saved to: " << args.output << "\n";
    }
    if (!args.skip_report) {
      const auto report_path = args.report.empty() ? default_report_path(args.output) : args.report;
      write_report(report_path, args, gyro.get(), mag.get(), compass.get());
      std::cout << "Plot report saved to: " << report_path << "\n";
    }
    std::cout << "\nCalibration complete.\n";
  } catch (const std::exception & error) {
    std::cerr << "\nError: " << error.what() << "\n";
    rclcpp::shutdown();
    return 1;
  }

  rclcpp::shutdown();
  return 0;
}
