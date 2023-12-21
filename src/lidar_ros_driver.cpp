//
// The MIT License (MIT)
//
// Copyright (c) 2020 EAIBOT. All rights reserved.
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
//

#include <ros/ros.h>
#include "sensor_msgs/LaserScan.h"
#include "std_srvs/Empty.h"
#include "src/CLidar.h"


#define SDKROSVerision "1.0.2"
CLidar cLidar;

bool stop_scan(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res){
  ROS_DEBUG("Stop scan");
  return cLidar.turnOff();
}

bool start_scan(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res){
  ROS_DEBUG("Start scan");
  return cLidar.turnOn();
}

int main(int argc, char **argv) {

  ros::init(argc, argv, "lidar_ros_driver");
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");
  ros::Publisher scan_pub = nh.advertise<sensor_msgs::LaserScan>("scan", 1);
  ros::ServiceServer stop_scan_service = nh.advertiseService("stop_scan", stop_scan);
  ros::ServiceServer start_scan_service = nh.advertiseService("start_scan", start_scan);
  ROS_INFO("LIDAR ROS Driver Version: %s", SDKROSVerision);	

  
  //////////////////////string property/////////////////  
  std::string frame_id;
  nh_private.param<std::string>("frame_id", frame_id, "laser_frame");

  std::string str_optvalue;
  nh_private.param<std::string>("port", str_optvalue, "192.168.0.100");
  cLidar.setlidaropt(LidarPropSerialPort, str_optvalue.c_str(), str_optvalue.size());

  //////////////////////int property/////////////////
  int optval;

  nh_private.param<int>("baudrate", optval, 8090);
  cLidar.setlidaropt(LidarPropSerialBaudrate, &optval, sizeof(int));

  nh_private.param<int>("lidar_type", optval, TYPE_LIDAR);
  cLidar.setlidaropt(LidarPropLidarType, &optval, sizeof(int));

  //////////////////////bool property/////////////////
  bool b_optvalue;

  nh_private.param<bool>("auto_reconnect", b_optvalue, true);
  cLidar.setlidaropt(LidarPropAutoReconnect, &b_optvalue, sizeof(bool));

  //////////////////////float property/////////////////
  /// unit: °
  float f_optvalue, angle_max, angle_min, range_max, range_min;
 
  nh_private.param<float>("angle_max", f_optvalue, 330.f);
  cLidar.setlidaropt(LidarPropMaxAngle, &f_optvalue, sizeof(float));
  angle_max = f_optvalue;

  nh_private.param<float>("angle_min", f_optvalue, 30.f);
  cLidar.setlidaropt(LidarPropMinAngle, &f_optvalue, sizeof(float));
  angle_min = f_optvalue;
  /// unit: m
  nh_private.param<float>("range_max", f_optvalue, 64.f);
  cLidar.setlidaropt(LidarPropMaxRange, &f_optvalue, sizeof(float));
  range_max = f_optvalue;

  nh_private.param<float>("range_min", f_optvalue, 0.01);
  cLidar.setlidaropt(LidarPropMinRange, &f_optvalue, sizeof(float));
  range_min = f_optvalue;

  /// unit: Hz
  nh_private.param<float>("frequency", f_optvalue, 30.f);
  cLidar.setlidaropt(LidarPropScanFrequency, &f_optvalue, sizeof(float));
 
  nh_private.param<int>("sample_rate", optval, 20);
  cLidar.setlidaropt(LidarPropSampleRate, &optval, sizeof(int));
 
  // initialize SDK and LiDAR
  bool ret = cLidar.initialize();
  if (ret) {
    ret = cLidar.turnOn();
  } else {
    ROS_ERROR("%s\n", cLidar.DescribeError());
  }
  ros::Rate r(30);	
  while (ros::ok() && ret) {
    LaserScan scan;
    if(cLidar.doProcessSimple(scan))
    {
      sensor_msgs::LaserScan scan_msg;
      ros::Time start_scan_time = ros::Time::now();

      scan_msg.header.stamp = start_scan_time;
      scan_msg.header.frame_id = frame_id;
      scan_msg.angle_min = scan.config.min_angle;
      scan_msg.angle_max = scan.config.max_angle;
      scan.config.angle_increment = 300.0 / scan.points.size() / 180 * M_PI;
      scan_msg.angle_increment = scan.config.angle_increment;//点与点之间的角度间隔
      scan_msg.scan_time = scan.config.scan_time;//扫描一圈所用的时间(秒)
      scan_msg.time_increment = scan.config.time_increment;//采样一个点所用的时间(秒)
      scan_msg.range_min = scan.config.min_range;
      scan_msg.range_max = scan.config.max_range;
      int size = (scan.config.max_angle - scan.config.min_angle) /
                 scan.config.angle_increment + 1;

      scan_msg.ranges.resize(size);
      scan_msg.intensities.resize(size);
      int trueSize = 0;
      for(size_t i = 0; i < scan.points.size(); i++) {
        int index = std::ceil(((scan.points[i].angle - 180) / 180.f * M_PI - scan.config.min_angle) /
                              scan.config.angle_increment);
        if (index >= 0 && index < size) {
          if (scan.points[i].range >= scan.config.min_range && scan.points[i].range <= scan.config.max_range) {
            trueSize++;
            scan_msg.ranges[index] = scan.points[i].range;
            scan_msg.intensities[index] = scan.points[i].intensity;
          }
        }
      }
      scan_pub.publish(scan_msg);
    }
    else {
      ROS_ERROR("Failed to get Lidar Data");
    }
    r.sleep();
    ros::spinOnce();
  }
  cLidar.turnOff();
  ROS_INFO("[LIDAR INFO] Now LIDAR is stopping .......");
  cLidar.disconnecting();
  return 0;
}

