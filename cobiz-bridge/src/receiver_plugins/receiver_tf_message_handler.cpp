#include "message_handler_base.hpp"
#include <tf2_msgs/msg/tf_message.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/serialization.hpp>
#include <nlohmann/json.hpp>

using boost::asio::ip::tcp;
namespace beast = boost::beast;
namespace ssl = boost::asio::ssl;
namespace websocket = beast::websocket;
using json = nlohmann::json;

class ReceiverTFMessageHandler : public cobiz_bridge::MessageHandlerBase,
                                public std::enable_shared_from_this<ReceiverTFMessageHandler> {
public:
  ReceiverTFMessageHandler() = default;
  ~ReceiverTFMessageHandler() override = default;

  void plugin_init() override {
    mime_ = "text/json";
    send_mime();
    ws_ptr->binary(true);
    
    read_timer_ = std::make_shared<boost::asio::steady_timer>(get_io_context(), std::chrono::milliseconds(100));
    write_timer_ = std::make_shared<boost::asio::steady_timer>(get_io_context(), std::chrono::seconds(10));
    
    async_receive_message();
    async_send_message();
    
    io_thread_ = std::thread([this]() {
        ioc_.run();
    });
  }

  std::string getMessageType() const override {
    return "tf2_msgs/msg/TFMessage";
  }
  
  void handle(std::shared_ptr<rclcpp::SerializedMessage> serialized_msg) override {
    // 수신기에서는 이 메서드를 사용하지 않음
  }

private:
  void async_receive_message(){
    read_timer_->expires_after(std::chrono::milliseconds(100));
    read_timer_->async_wait([this](beast::error_code ec) {
      on_read_timeout(ec);
    });
  }
  
  void on_read_timeout(beast::error_code ec) {
    if (ec) {
      RCLCPP_ERROR(rclcpp::get_logger("ReceiverTFMessageHandler"), "읽기 타임아웃 오류: %s", ec.message().c_str());
      return;
    }
    
    ws_ptr->async_read(buffer_,
      [this](beast::error_code ec, std::size_t bytes_transferred) {
        on_read(ec, bytes_transferred);
      });
  }
  
  void on_read(beast::error_code ec, std::size_t bytes_transferred) {
    if (ec) {
      if (ec != beast::websocket::error::closed)
        RCLCPP_ERROR(rclcpp::get_logger("ReceiverTFMessageHandler"), "읽기 오류: %s", ec.message().c_str());
      return;
    }
    
    std::string message = beast::buffers_to_string(buffer_.data());
    buffer_.consume(bytes_transferred);
    
    try {
      json j = json::parse(message);
      
      // TFMessage 메시지 생성
      auto msg = std::make_shared<tf2_msgs::msg::TFMessage>();
      
      // transforms 배열 처리
      if (j.is_array()) {
        for (const auto& transform_json : j) {
          geometry_msgs::msg::TransformStamped transform;

          // 헤더 설정
          transform.header.stamp.sec = transform_json["header"]["stamp"]["sec"];
          transform.header.stamp.nanosec = transform_json["header"]["stamp"]["nanosec"];
          transform.header.frame_id = transform_json["header"]["frame_id"];
          transform.child_frame_id = transform_json["child_frame_id"];

          // 변환 설정
          transform.transform.translation.x = transform_json["transform"]["translation"]["x"];
          transform.transform.translation.y = transform_json["transform"]["translation"]["y"];
          transform.transform.translation.z = transform_json["transform"]["translation"]["z"];
          transform.transform.rotation.x = transform_json["transform"]["rotation"]["x"];
          transform.transform.rotation.y = transform_json["transform"]["rotation"]["y"];
          transform.transform.rotation.z = transform_json["transform"]["rotation"]["z"];
          transform.transform.rotation.w = transform_json["transform"]["rotation"]["w"];

          msg->transforms.push_back(transform);
        }
      }
      // if (j["transforms"].is_array()) {
      //   for (const auto& transform_json : j["transforms"]) {
      //     geometry_msgs::msg::TransformStamped transform;
      //     // 헤더 설정
      //     transform.header.stamp.sec = transform_json["header"]["stamp"]["sec"];
      //     transform.header.stamp.nanosec = transform_json["header"]["stamp"]["nanosec"];
      //     transform.header.frame_id = transform_json["header"]["frame_id"];
      //     transform.child_frame_id = transform_json["child_frame_id"];
          
      //     // 변환 설정
      //     transform.transform.translation.x = transform_json["transform"]["translation"]["x"];
      //     transform.transform.translation.y = transform_json["transform"]["translation"]["y"];
      //     transform.transform.translation.z = transform_json["transform"]["translation"]["z"];
      //     transform.transform.rotation.x = transform_json["transform"]["rotation"]["x"];
      //     transform.transform.rotation.y = transform_json["transform"]["rotation"]["y"];
      //     transform.transform.rotation.z = transform_json["transform"]["rotation"]["z"];
      //     transform.transform.rotation.w = transform_json["transform"]["rotation"]["w"];
          
      //     msg->transforms.push_back(transform);
      //   }
      // }
      
      // 메시지 직렬화 및 발행
      rclcpp::SerializedMessage serialized_msg;
      rclcpp::Serialization<tf2_msgs::msg::TFMessage> serializer;
      serializer.serialize_message(msg.get(), &serialized_msg);
      
      if (publisher_) {
        publisher_->publish(serialized_msg);
      } else {
        RCLCPP_WARN(rclcpp::get_logger("ReceiverTFMessageHandler"), "퍼블리셔가 없습니다");
      }
    }
    catch (const std::exception& e) {
      RCLCPP_ERROR(rclcpp::get_logger("ReceiverTFMessageHandler"), "TFMessage 메시지 처리 실패: %s", e.what());
    }
    
    async_receive_message();
  }
  
  void async_send_message(){
    write_timer_->expires_after(std::chrono::seconds(10));
    write_timer_->async_wait(
      [this](beast::error_code ec) {
        on_write_timeout(ec);
      });
  }
  
  void on_write_timeout(beast::error_code ec) {
    if (ec) return;
    
    std::string message = "ping";
    ws_ptr->async_write(boost::asio::buffer(message),
      [this](beast::error_code ec, std::size_t) {
        if (ec) {
          RCLCPP_ERROR(rclcpp::get_logger("ReceiverTFMessageHandler"), "핑 메시지 전송 실패: %s", ec.message().c_str());
        }
        async_send_message();
      });
  }
};

// 플러그인 클래스 등록
PLUGINLIB_EXPORT_CLASS(ReceiverTFMessageHandler, cobiz_bridge::MessageHandlerBase)
