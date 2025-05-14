#include "message_handler_base.hpp"
#include <nav_msgs/msg/odometry.hpp>  // Odometry 메시지 타입 헤더

#include <nlohmann/json.hpp>

using boost::asio::ip::tcp;
namespace beast = boost::beast;
namespace ssl = boost::asio::ssl;
namespace websocket = beast::websocket;
using json = nlohmann::json;

class OdomHandler : public cobiz_bridge::MessageHandlerBase {
public:
  OdomHandler() = default;
  ~OdomHandler() override = default;

  void plugin_init() override {
    mime_ = "text/json";
    send_mime();
    ws_ptr->binary(true);
  }

  void handle(std::shared_ptr<rclcpp::SerializedMessage> serialized_msg) override {
    // 로거 생성
    auto logger = rclcpp::get_logger("OdomHandler");
    // RCLCPP_INFO(logger, "Odometry 메시지 처리 중: %zu 바이트", serialized_msg->size());
    
    try {
      // 메시지 Deserialization
      auto msg = std::make_shared<nav_msgs::msg::Odometry>();
      rclcpp::Serialization<nav_msgs::msg::Odometry> serialization;
      serialization.deserialize_message(serialized_msg.get(), msg.get());
      
      // YAML 형식으로 출력
      json j;
      j["topic"] = topic_;
      j["type"] = "odom";
      j["header"]["stamp"]["sec"] = msg->header.stamp.sec;
      j["header"]["stamp"]["nanosec"] = msg->header.stamp.nanosec;
      j["header"]["frame_id"] = msg->header.frame_id;
      j["child_frame_id"] = msg->child_frame_id;
      j["pose"]["pose"]["position"]["x"] = msg->pose.pose.position.x;
      j["pose"]["pose"]["position"]["y"] = msg->pose.pose.position.y;
      j["pose"]["pose"]["position"]["z"] = msg->pose.pose.position.z;
      j["pose"]["pose"]["orientation"]["x"] = msg->pose.pose.orientation.x;
      j["pose"]["pose"]["orientation"]["y"] = msg->pose.pose.orientation.y;
      j["pose"]["pose"]["orientation"]["z"] = msg->pose.pose.orientation.z;
      j["pose"]["pose"]["orientation"]["w"] = msg->pose.pose.orientation.w;
      j["pose"]["covariance"] = msg->pose.covariance;
      j["twist"]["twist"]["linear"]["x"] = msg->twist.twist.linear.x;
      j["twist"]["twist"]["linear"]["y"] = msg->twist.twist.linear.y;
      j["twist"]["twist"]["linear"]["z"] = msg->twist.twist.linear.z;
      j["twist"]["twist"]["angular"]["x"] = msg->twist.twist.angular.x;
      j["twist"]["twist"]["angular"]["y"] = msg->twist.twist.angular.y;
      j["twist"]["twist"]["angular"]["z"] = msg->twist.twist.angular.z;
      j["twist"]["covariance"] = msg->twist.covariance;
      std::string json_str = j.dump();
      send_message(json_str);
      
    } catch (const std::exception& e) {
      RCLCPP_ERROR(logger, "Odometry 메시지 deserialize 실패: %s", e.what());
    }
  }

  std::string getMessageType() const override {
    return "nav_msgs/msg/Odometry";
  }
};

// 플러그인 클래스 등록
PLUGINLIB_EXPORT_CLASS(OdomHandler, cobiz_bridge::MessageHandlerBase)
