#include "message_handler_base.hpp"
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nlohmann/json.hpp>

using boost::asio::ip::tcp;
namespace beast = boost::beast;
namespace ssl = boost::asio::ssl;
namespace websocket = beast::websocket;
using json = nlohmann::json;

class PoseStampedHandler : public cobiz_bridge::MessageHandlerBase, 
                            public std::enable_shared_from_this<PoseStampedHandler> {
public:
  PoseStampedHandler() = default;
  ~PoseStampedHandler() override = default;

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
    return "geometry_msgs/msg/PoseStamped";
  }

  void handle(std::shared_ptr<rclcpp::SerializedMessage> serialized_msg) override {
    
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
      RCLCPP_ERROR(rclcpp::get_logger("PoseStampedHandler"), "읽기 타임아웃 오류: %s", ec.message().c_str());
      return;
    }
    ws_ptr->async_read(buffer_,
      [this](beast::error_code ec, std::size_t bytes_transferred) {
        on_read(ec, bytes_transferred);
      });
  }
  void on_read(beast::error_code ec, std::size_t bytes_transferred) {
    if (ec) {
      RCLCPP_ERROR(rclcpp::get_logger("PoseStampedHandler"), "읽기 오류: %s", ec.message().c_str());
      return;
    }
    // 메시지 처리
    std::string message = beast::buffers_to_string(buffer_.data());
    buffer_.consume(bytes_transferred);
    
    // JSON 파싱z
    try {
      json j = json::parse(message);
      
      // 메시지 생성
      auto msg = std::make_shared<geometry_msgs::msg::PoseStamped>();
      msg->header.stamp.sec = j["header"]["stamp"]["sec"];
      msg->header.stamp.nanosec = j["header"]["stamp"]["nanosec"];
      msg->header.frame_id = j["header"]["frame_id"];
      msg->pose.position.x = j["pose"]["position"]["x"];
      msg->pose.position.y = j["pose"]["position"]["y"];
      msg->pose.position.z = j["pose"]["position"]["z"];
      msg->pose.orientation.x = j["pose"]["orientation"]["x"];
      msg->pose.orientation.y = j["pose"]["orientation"]["y"];
      msg->pose.orientation.z = j["pose"]["orientation"]["z"];
      msg->pose.orientation.w = j["pose"]["orientation"]["w"];
      rclcpp::SerializedMessage serialized_msg;
      rclcpp::Serialization<geometry_msgs::msg::PoseStamped> serializer;
      serializer.serialize_message(msg.get(), &serialized_msg);
      if (publisher_) {
        publisher_->publish(serialized_msg);
        // RCLCPP_INFO(rclcpp::get_logger("PoseStampedHandler"), "PoseStamped 메시지 퍼블리시 완료");
      } else {
        RCLCPP_WARN(rclcpp::get_logger("PoseStampedHandler"), "퍼블리셔가 없습니다");
      }

    }
    catch (const std::exception& e) {
      RCLCPP_ERROR(rclcpp::get_logger("PoseStampedHandler"), "PoseStamped 메시지 deserialize 실패: %s", e.what());
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
    if (ec) {
      // RCLCPP_ERROR(rclcpp::get_logger("PoseStampedHandler"), "쓰기 타임아웃 오류: %s", ec.message().c_str());
      return;
    }
    std::string message = "ping";
    // 메시지 전송
    ws_ptr->async_write(boost::asio::buffer(message),
      [this](beast::error_code ec, std::size_t bytes_transferred) {
        on_write(ec, bytes_transferred);
      });
  }
  void on_write(beast::error_code ec, std::size_t bytes_transferred) {
    if (ec) {
      RCLCPP_ERROR(rclcpp::get_logger("PoseStampedHandler"), "쓰기 오류: %s", ec.message().c_str());
      return;
    }
    // 메시지 전송 완료
    async_send_message();
  }
};

// 플러그인 클래스 등록
PLUGINLIB_EXPORT_CLASS(PoseStampedHandler, cobiz_bridge::MessageHandlerBase)
