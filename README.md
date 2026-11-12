# XV-11 Driver For Lidar From Neato

This is a port of the popular ROS driver called  xv_11_laser_driver

This is a port so most credit goes to original developers.
Please see ROS version here:  https://github.com/rohbotics/xv_11_laser_driver

## Changes in this fork
Fork of https://github.com/mjstn/xv_11_driver, modified by Michael Moser:
- Parameters are declared with their defaults, the overload without a default no longer exists on ROS 2 kilted.
- `scan_time` and `time_increment` are computed from the motor speed (firmware 2); before, `scan_time` was 0 and `time_increment` about 2.7x too small.
- `header.stamp` is derived from the time the last ray arrived instead of the time before reading started. New parameter `stamp_mid_scan` (default `false`): `true` stamps the middle of the rotation instead of the first ray, for consumers that place a whole scan at its stamp (e.g. slam_toolbox). With a scan taking ~0.2 s, the first-ray stamp makes such consumers misplace scans while the robot turns.

Place this repository in your colcon_ws workspace

    cd ~/colcon_ws/src
    git clone <this-repository>
    cd ..
    colcon build

Once it has been made and registered by sourcing it to your environment 
you may run it as follows

The following ROS2 parameters are defaulted as below and can be set using parameters

    port              /dev/tty_xv11_driver
    baud_rate         115200
    frame_id          neato_laser
    firmware_version  2
    stamp_mid_scan    false

Run the driver and it should generate scan data on topic /scan

    ros2 run xv_11_driver xv_11_driver

Run the driver using a different frame_id

    ros2 run xv_11_driver xv_11_driver --ros-args -p frame_id:=xv11_scan

If all goes well this will produce messages of type LaserScan on topic /scan

    

