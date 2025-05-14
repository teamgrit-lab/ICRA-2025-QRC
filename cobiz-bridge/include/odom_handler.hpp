#ifndef ODOM_HANDLER_HPP
#define ODOM_HANDLER_HPP

#include "message_handler_base.hpp"
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/serialization.hpp>

class OdomHandler : public MessageHandlerBase {
public:
  OdomHandler() = default;
  ~OdomHandler() override = default;

  void handle(std::shared_ptr<rclcpp::SerializedMessage> serialized_msg) override;
  std::string getMessageType() const override;

private:
  rclcpp::Serialization<nav_msgs::msg::Odometry> serialization_;
};

#endif // ODOM_HANDLER_HPP
