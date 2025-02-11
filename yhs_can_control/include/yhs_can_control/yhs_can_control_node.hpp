#ifndef __YHS_CANCONTROL_NODE_H__
#define __YHS_CANCONTROL_NODE_H__

#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>

#include <chrono>
#include <cmath>
#include <iomanip>
#include <sstream>

#include <linux/can.h>
#include <linux/can/raw.h>

#include "geometry_msgs/msg/pose_with_covariance.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "yhs_can_interfaces/msg/chassis_info_fb.hpp"
#include "yhs_can_interfaces/msg/ctrl_cmd.hpp"
#include "yhs_can_interfaces/msg/io_cmd.hpp"

#define READ_PARAM(TYPE, NAME, VAR, VALUE)  \
  VAR = VALUE;                              \
  node->declare_parameter<TYPE>(NAME, VAR); \
  node->get_parameter(NAME, VAR);

namespace yhs {

class CanControl {
 public:
  explicit CanControl(rclcpp::Node::SharedPtr);
  ~CanControl();

  bool run();
  void stop();

 private:
  rclcpp::Node::SharedPtr node_;

  std::string if_name_;
  int can_socket_;
  double wheel_base_;
  std::thread thread_;

  std::vector<int64_t> ultrasonic_number_;

  rclcpp::Time last_cmd_vel_time_;

  rclcpp::Subscription<yhs_can_interfaces::msg::IoCmd>::SharedPtr
      io_cmd_subscriber_;
  rclcpp::Subscription<yhs_can_interfaces::msg::CtrlCmd>::SharedPtr
      ctrl_cmd_subscriber_;

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr
      cmd_vel_subscription_;

  rclcpp::Publisher<yhs_can_interfaces::msg::ChassisInfoFb>::SharedPtr
      chassis_info_fb_publisher_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;

  void io_cmd_callback(
      const yhs_can_interfaces::msg::IoCmd::SharedPtr io_cmd_msg);

  void ctrl_cmd_callback(
      const yhs_can_interfaces::msg::CtrlCmd::SharedPtr ctrl_cmd_msg);

  void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr cmd_vel_msg);

  bool wait_for_can_frame();

  void can_data_recv_callback();

  void publish_odom(const double velocity, const double steering);
};

}  // namespace yhs

#endif
