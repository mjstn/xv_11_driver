// Copyright 2016 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
// This xv-11 lidar driver started from this link:
// https://github.com/ros2/demos/blob/master/dummy_robot/dummy_sensors/src/dummy_laser.cpp
// And xv11 code:  https://github.com/rohbotics/xv_11_laser_driver
// Also refer to conversion from ROS to ROS2  at https://index.ros.org/doc/ros2/Contributing/Migration-Guide/
//
// Launch:  ros2 run xv_11_driver xv_11_driver
//

#include <chrono>
#include <iostream>
#include <memory>
#include <string>

#include "rclcpp/clock.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/time_source.hpp"

// Make files need to change to support project include in a clean way
#include "../include/xv_11_driver/xv11_laser.h"

#include <std_msgs/msg/string.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

#define DEG2RAD (M_PI/180.0)

// default xv11 dev name.  Suggest you use symbolic link for an easier to ricognize name on your system
// Here I assume you have setup symbolic link to your actual serial port tty driver to the lidar
#define XV11_PORT_DEFAULT "/dev/tty_xv11_driver"
#define XV11_BAUD_RATE_DEFAULT 115200
#define XV11_FRAME_ID_DEFAULT "neato_laser"


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto node = rclcpp::Node::make_shared("xv11_laser");

  int firmware_number = 2;

  node->declare_parameter("port");
  auto port_param      = rclcpp::Parameter("port", XV11_PORT_DEFAULT);
  node->declare_parameter("baud_rate");
  auto baud_rate_param = rclcpp::Parameter("baud_rate", XV11_BAUD_RATE_DEFAULT);
  node->declare_parameter("frame_id");
  auto frame_id_param  = rclcpp::Parameter("frame_id", XV11_FRAME_ID_DEFAULT);
    
  node->get_parameter_or("port", port_param, port_param);
  node->get_parameter_or("baud_rate", baud_rate_param, baud_rate_param);
  node->get_parameter_or("frame_id", frame_id_param, frame_id_param);

  std::string port     = port_param.value_to_string();
  int baud_rate        = baud_rate_param.as_int();
  std::string frame_id = frame_id_param.value_to_string();

  auto laser_pub = node->create_publisher<sensor_msgs::msg::LaserScan>("scan", 10);

  // std_msgs::msg::UInt16 rpms;
  boost::asio::io_service io;

  try {
    xv_11_driver::XV11Laser laser(port, baud_rate, firmware_number, io);

    // auto motor_pub = node->create_publisher<std_msgs::UInt16>("rpms",1000);

    while (rclcpp::ok()) {
      sensor_msgs::msg::LaserScan *scan;
      scan = new sensor_msgs::msg::LaserScan;
      // sensor_msgs::LaserScan::Ptr scan(new sensor_msgs::LaserScan);
      scan->header.frame_id = frame_id;
      scan->header.stamp = rclcpp::Clock().now();   //  ROS was  Time::now();
      laser.poll(scan);
      laser_pub->publish(*scan);

      //rpms.data=laser.rpms;
      // motor_pub->publish(rpms);

    }
    laser.close();
    return 0;
  } catch (...) {
    RCLCPP_ERROR(node->get_logger(), "Error instantiating laser object. Check correct port and baud rate!");
    return -1;
  }


  rclcpp::shutdown();
  return 0;
}
