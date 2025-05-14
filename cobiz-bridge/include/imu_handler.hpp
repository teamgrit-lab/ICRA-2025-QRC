#ifndef IMU_HANDLER_HPP
#define IMU_HANDLER_HPP

#include "message_handler_base.hpp"
#include <sensor_msgs/msg/imu.hpp>
#include <rclcpp/serialization.hpp>

class ImuHandler : public MessageHandlerBase {
public:
  ImuHandler() = default;
  ~ImuHandler() override = default;

  void handle(std::shared_ptr<rclcpp::SerializedMessage> serialized_msg) override;
  std::string getMessageType() const override;

private:
  rclcpp::Serialization<sensor_msgs::msg::Imu> serialization_;
};

#endif // IMU_HANDLER_HPP
