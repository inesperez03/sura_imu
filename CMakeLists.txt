cmake_minimum_required(VERSION 3.8)
project(sura_imu)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

add_executable(calibration_filter src/calibration_filter.cpp)
ament_target_dependencies(calibration_filter
  ament_index_cpp
  rclcpp
  sensor_msgs
)
target_compile_features(calibration_filter PUBLIC cxx_std_17)

foreach(target calibration_wizard calibrate_gyro calibrate_magnetometer calibrate_compass)
  add_executable(${target} src/calibration_wizard.cpp)
  ament_target_dependencies(${target}
    ament_index_cpp
    rclcpp
    sensor_msgs
  )
  target_compile_features(${target} PUBLIC cxx_std_17)
endforeach()

target_compile_definitions(calibrate_gyro PRIVATE SURA_IMU_DEFAULT_MODE=\"gyro\")
target_compile_definitions(calibrate_magnetometer PRIVATE SURA_IMU_DEFAULT_MODE=\"mag\")
target_compile_definitions(calibrate_compass PRIVATE SURA_IMU_DEFAULT_MODE=\"compass\")

install(TARGETS
  calibration_filter
  calibration_wizard
  calibrate_gyro
  calibrate_magnetometer
  calibrate_compass
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
