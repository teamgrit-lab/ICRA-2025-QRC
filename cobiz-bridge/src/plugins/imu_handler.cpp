#include "message_handler_base.hpp"
#include <sensor_msgs/msg/imu.hpp>
#include <nlohmann/json.hpp>

using boost::asio::ip::tcp;
namespace beast = boost::beast;
namespace ssl = boost::asio::ssl;
namespace websocket = beast::websocket;
using json = nlohmann::json;

class ImuHandler : public cobiz_bridge::MessageHandlerBase {
public:
  ImuHandler() = default;
  ~ImuHandler() override = default;

  void plugin_init() override {
    mime_ = "text/json";
    send_mime();
    ws_ptr->binary(true);
  }

  void handle(std::shared_ptr<rclcpp::SerializedMessage> serialized_msg) override {
    // 로거 생성
    // auto logger = rclcpp::get_logger("ImuHandler");
    // RCLCPP_INFO(logger, "IMU 메시지 처리 중: %zu 바이트", serialized_msg->size());
    
    try {
      // 메시지 Deserialization
      auto msg = std::make_shared<sensor_msgs::msg::Imu>();
      rclcpp::Serialization<sensor_msgs::msg::Imu> serialization;
      serialization.deserialize_message(serialized_msg.get(), msg.get());

      json j;
      j["topic"] = topic_;
      j["header"]["stamp"]["sec"] = msg->header.stamp.sec;
      j["header"]["stamp"]["nanosec"] = msg->header.stamp.nanosec;
      j["header"]["frame_id"] = msg->header.frame_id;
      j["orientation"]["x"] = msg->orientation.x;
      j["orientation"]["y"] = msg->orientation.y;
      j["orientation"]["z"] = msg->orientation.z;
      j["orientation"]["w"] = msg->orientation.w;
      j["angular_velocity"]["x"] = msg->angular_velocity.x;
      j["angular_velocity"]["y"] = msg->angular_velocity.y;
      j["angular_velocity"]["z"] = msg->angular_velocity.z;
      j["linear_acceleration"]["x"] = msg->linear_acceleration.x;
      j["linear_acceleration"]["y"] = msg->linear_acceleration.y;
      j["linear_acceleration"]["z"] = msg->linear_acceleration.z;
      j["orientation_covariance"] = msg->orientation_covariance;
      j["angular_velocity_covariance"] = msg->angular_velocity_covariance;
      j["linear_acceleration_covariance"] = msg->linear_acceleration_covariance;
      std::string json_str = j.dump();
      send_message(json_str);
      // RCLCPP_INFO(logger, "IMU 메시지 JSON 형식으로 전송: %s", json_str.c_str());
      
    } catch (const std::exception& e) {
      // RCLCPP_ERROR(logger, "IMU 메시지 deserialize 실패: %s", e.what());
    }
  }

  std::string getMessageType() const override {
    return "sensor_msgs/msg/Imu";
  }
};

// 플러그인 클래스 등록
PLUGINLIB_EXPORT_CLASS(ImuHandler, cobiz_bridge::MessageHandlerBase)
