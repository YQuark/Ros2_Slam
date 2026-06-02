/*
 *  YDLIDAR SYSTEM
 *  YDLIDAR ROS 2 Node Client
 *
 *  Copyright 2017 - 2020 EAI TEAM
 *  http://www.eaibot.com
 *
 */

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include <math.h>

#define RAD2DEG(x) ((x)*180./M_PI)

static void scanCb(sensor_msgs::msg::LaserScan::SharedPtr scan) {
  // Guard against division-by-zero on time_increment (H1)
  if (scan->time_increment <= 0.0f) {
    printf("[YDLIDAR WARN]: Skipping scan: time_increment is zero or negative\n");
    return;
  }

  int count = scan->scan_time / scan->time_increment;
  // Clamp count to actual ranges size to prevent buffer over-read (H1)
  if (count > static_cast<int>(scan->ranges.size())) {
    count = static_cast<int>(scan->ranges.size());
  }

  printf("[YDLIDAR INFO]: I heard a laser scan %s[%d]:\n", scan->header.frame_id.c_str(), count);
  printf("[YDLIDAR INFO]: angle_range : [%f, %f]\n", RAD2DEG(scan->angle_min),
         RAD2DEG(scan->angle_max));

  for (int i = 0; i < count; i++) {
    float degree = RAD2DEG(scan->angle_min + scan->angle_increment * i);
    printf("[YDLIDAR INFO]: angle-distance : [%f, %f]\n", degree, scan->ranges[i]);
  }
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  auto node = rclcpp::Node::make_shared("ydlidar_ros2_driver_client");

  auto lidar_info_sub = node->create_subscription<sensor_msgs::msg::LaserScan>(
                        "scan", rclcpp::SensorDataQoS(), scanCb);

  rclcpp::spin(node);

  rclcpp::shutdown();


  return 0;
}
