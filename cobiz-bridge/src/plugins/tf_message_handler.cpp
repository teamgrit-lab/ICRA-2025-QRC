#include "message_handler_base.hpp"
#include <tf2_msgs/msg/tf_message.hpp>
#include <nlohmann/json.hpp>

using boost::asio::ip::tcp;
namespace beast = boost::beast;
namespace ssl = boost::asio::ssl;
namespace websocket = beast::websocket;
using json = nlohmann::json;

class TFMessageHandler : public cobiz_bridge::MessageHandlerBase {
public:
  TFMessageHandler() = default;
  ~TFMessageHandler() override = default;

  void plugin_init() override {
    mime_ = "text/json";
    send_mime();
    ws_ptr->binary(true);
    message_count_ = 10;  // 메시지 수 설정
  }

  void handle(std::shared_ptr<rclcpp::SerializedMessage> serialized_msg) override {
    // 로거 생성
    auto logger = rclcpp::get_logger("TFMessageHandler");
    // RCLCPP_INFO(logger, "TFMessage 메시지 처리 중: %zu 바이트", serialized_msg->size());
    
    try {
      // 메시지 Deserialization
      auto msg = std::make_shared<tf2_msgs::msg::TFMessage>();
      rclcpp::Serialization<tf2_msgs::msg::TFMessage> serialization;
      serialization.deserialize_message(serialized_msg.get(), msg.get());
      
      // YAML 형식으로 출력
      for(const auto& transform : msg->transforms) {
        json j;
        j["topic"] = topic_;
        j["header"]["stamp"]["sec"] = transform.header.stamp.sec;
        j["header"]["stamp"]["nanosec"] = transform.header.stamp.nanosec;
        j["header"]["frame_id"] = transform.header.frame_id;
        j["child_frame_id"] = transform.child_frame_id;
        j["transform"]["translation"]["x"] = transform.transform.translation.x;
        j["transform"]["translation"]["y"] = transform.transform.translation.y;
        j["transform"]["translation"]["z"] = transform.transform.translation.z;
        j["transform"]["rotation"]["x"] = transform.transform.rotation.x;
        j["transform"]["rotation"]["y"] = transform.transform.rotation.y;
        j["transform"]["rotation"]["z"] = transform.transform.rotation.z;
        j["transform"]["rotation"]["w"] = transform.transform.rotation.w;
        accumulated_json_.push_back(j);
      }

      if (accumulated_json_.size() >= message_count_) {
        std::string json_str = accumulated_json_.dump();
        send_message(json_str);
        accumulated_json_ = json::array(); // 초기화
      }

    } catch (const std::exception& e) {
      RCLCPP_ERROR(logger, "TFMessage 메시지 deserialize 실패: %s", e.what());
    }
  }

  std::string getMessageType() const override {
    return "tf2_msgs/msg/TFMessage";
  }

private:
  json accumulated_json_ = json::array();
  uint8_t message_count_;
};

// 플러그인 클래스 등록
PLUGINLIB_EXPORT_CLASS(TFMessageHandler, cobiz_bridge::MessageHandlerBase)
