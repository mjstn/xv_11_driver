/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2011, Eric Perko, Chad Rockey
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Case Western Reserve University nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/


/*********************************************************************
 *
 * This xv-11 lidar driver started from a popular XV-11 or 'Neato' Lidar driver.
 *
 * https://github.com/ros2/demos/blob/master/dummy_robot/dummy_sensors/src/dummy_laser.cpp
 * And xv11 code:  https://github.com/rohbotics/xv_11_laser_driver
 * Also refer to conversion from ROS to ROS2  at https://index.ros.org/doc/ros2/Contributing/Migration-Guide/
 *
 * Launch:  ros2 run xv_11_driver xv_11_driver
 *********************************************************************/

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
#include <std_msgs/std_msgs/msg/u_int16.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

#define DEG2RAD (M_PI/180.0)

// default xv11 dev name.  Suggest you use symbolic link for an easier to ricognize name on your system
// example:  sudo ln -s /dev/yourTtyDevName /dev/ttyXV11
// Here I assume you have setup symbolic link to your actual serial port tty driver to the lidar
#define XV11_PORT_DEFAULT "/dev/ttyXV11"           // Serial device driver name (sym link to real dev)
#define XV11_BAUD_RATE_DEFAULT 115200              // Serial baud rate
#define XV11_FRAME_ID_DEFAULT "neato_laser"        // frame_id in LaserScan messages
#define XV11_FIRMWARE_VERSION_DEFAULT 2            // XV-11 firmware rev. 1 is oldest


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto node = rclcpp::Node::make_shared("xv11_laser");

  node->declare_parameter("port", rclcpp::PARAMETER_STRING);
  auto port_param      = rclcpp::Parameter("port", XV11_PORT_DEFAULT);

  node->declare_parameter("baud_rate", rclcpp::PARAMETER_STRING);
  auto baud_rate_param = rclcpp::Parameter("baud_rate", XV11_BAUD_RATE_DEFAULT);

  node->declare_parameter("frame_id", rclcpp::PARAMETER_STRING);
  auto frame_id_param  = rclcpp::Parameter("frame_id", XV11_FRAME_ID_DEFAULT);

  node->declare_parameter("firmware_version", rclcpp::PARAMETER_STRING);
  auto firmware_param  = rclcpp::Parameter("firmware_version", XV11_FIRMWARE_VERSION_DEFAULT);
    
  node->get_parameter_or("port", port_param, port_param);
  node->get_parameter_or("baud_rate", baud_rate_param, baud_rate_param);
  node->get_parameter_or("frame_id", frame_id_param, frame_id_param);
  node->get_parameter_or("firmware_version", firmware_param, firmware_param);

  std::string port     = port_param.value_to_string();
  int baud_rate        = baud_rate_param.as_int();
  std::string frame_id = frame_id_param.value_to_string();
  int firmware_number  = firmware_param.as_int();

  auto laser_pub = node->create_publisher<sensor_msgs::msg::LaserScan>("scan", 10);

  std_msgs::uint16 rpms;
  boost::asio::io_service io;

  try {
    xv_11_driver::XV11Laser laser(port, baud_rate, firmware_number, io);

  auto motor_pub = node->create_publisher<std_msgs::uint16>("rpms",1000);

    while (rclcpp::ok()) {
      sensor_msgs::msg::LaserScan *scan;
      scan = new sensor_msgs::msg::LaserScan;
      // sensor_msgs::LaserScan::Ptr scan(new sensor_msgs::LaserScan);
      scan->header.frame_id = frame_id;
      scan->header.stamp = rclcpp::Clock().now();   //  ROS was  Time::now();
      laser.poll(scan);
      laser_pub->publish(*scan);

      rpms.data=laser.rpms;
      motor_pub->publish(rpms);

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
