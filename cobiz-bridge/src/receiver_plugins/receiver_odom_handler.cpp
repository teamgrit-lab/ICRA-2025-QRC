#include "message_handler_base.hpp"
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/serialization.hpp>
#include <nlohmann/json.hpp>

using boost::asio::ip::tcp;
namespace beast = boost::beast;
namespace ssl = boost::asio::ssl;
namespace websocket = beast::websocket;
using json = nlohmann::json;

class ReceiverOdomHandler : public cobiz_bridge::MessageHandlerBase,
                           public std::enable_shared_from_this<ReceiverOdomHandler> {
public:
  ReceiverOdomHandler() = default;
  ~ReceiverOdomHandler() override = default;

  void plugin_init() override {
    mime_ = "text/json";
    send_mime();
    ws_ptr->binary(true);
    
    read_timer_ = std::make_shared<boost::asio::steady_timer>(get_io_context(), std::chrono::milliseconds(10));
    write_timer_ = std::make_shared<boost::asio::steady_timer>(get_io_context(), std::chrono::seconds(10));
    
    async_receive_message();
    async_send_message();
    
    io_thread_ = std::thread([this]() {
        ioc_.run();
    });
  }

  std::string getMessageType() const override {
    return "nav_msgs/msg/Odometry";
  }
  
  void handle(std::shared_ptr<rclcpp::SerializedMessage> serialized_msg) override {
    // 수신기에서는 이 메서드를 사용하지 않음
  }

private:
  void async_receive_message(){
    read_timer_->expires_after(std::chrono::milliseconds(10));
    read_timer_->async_wait([this](beast::error_code ec) {
      on_read_timeout(ec);
    });
  }
  
  void on_read_timeout(beast::error_code ec) {
    if (ec) {
      RCLCPP_ERROR(rclcpp::get_logger("ReceiverOdomHandler"), "읽기 타임아웃 오류: %s", ec.message().c_str());
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
        RCLCPP_ERROR(rclcpp::get_logger("ReceiverOdomHandler"), "읽기 오류: %s", ec.message().c_str());
      return;
    }
    
    // 웹소켓에서 받은 메시지 처리
    std::string message = beast::buffers_to_string(buffer_.data());
    buffer_.consume(bytes_transferred);
    
    try {
      json j = json::parse(message);
      
      // Odometry 메시지 생성
      auto msg = std::make_shared<nav_msgs::msg::Odometry>();
      
      // 헤더 설정
      msg->header.stamp.sec = j["header"]["stamp"]["sec"];
      msg->header.stamp.nanosec = j["header"]["stamp"]["nanosec"];
      msg->header.frame_id = j["header"]["frame_id"];
      msg->child_frame_id = j["child_frame_id"];
      
      // Pose 설정
      msg->pose.pose.position.x = j["pose"]["pose"]["position"]["x"];
      msg->pose.pose.position.y = j["pose"]["pose"]["position"]["y"];
      msg->pose.pose.position.z = j["pose"]["pose"]["position"]["z"];
      msg->pose.pose.orientation.x = j["pose"]["pose"]["orientation"]["x"];
      msg->pose.pose.orientation.y = j["pose"]["pose"]["orientation"]["y"];
      msg->pose.pose.orientation.z = j["pose"]["pose"]["orientation"]["z"];
      msg->pose.pose.orientation.w = j["pose"]["pose"]["orientation"]["w"];
      
      // Twist 설정
      msg->twist.twist.linear.x = j["twist"]["twist"]["linear"]["x"];
      msg->twist.twist.linear.y = j["twist"]["twist"]["linear"]["y"];
      msg->twist.twist.linear.z = j["twist"]["twist"]["linear"]["z"];
      msg->twist.twist.angular.x = j["twist"]["twist"]["angular"]["x"];
      msg->twist.twist.angular.y = j["twist"]["twist"]["angular"]["y"];
      msg->twist.twist.angular.z = j["twist"]["twist"]["angular"]["z"];
      
      // 공분산 설정 (필요한 경우)
      if (j["pose"]["covariance"].is_array() && j["pose"]["covariance"].size() == 36) {
        for (size_t i = 0; i < 36; ++i) {
          msg->pose.covariance[i] = j["pose"]["covariance"][i];
        }
      }
      
      if (j["twist"]["covariance"].is_array() && j["twist"]["covariance"].size() == 36) {
        for (size_t i = 0; i < 36; ++i) {
          msg->twist.covariance[i] = j["twist"]["covariance"][i];
        }
      }
      
      // 메시지 직렬화 및 게시
      rclcpp::SerializedMessage serialized_msg;
      rclcpp::Serialization<nav_msgs::msg::Odometry> serializer;
      serializer.serialize_message(msg.get(), &serialized_msg);
      
      // ROS 토픽으로 발행
      if (publisher_) {
        publisher_->publish(serialized_msg);
      } else {
        RCLCPP_WARN(rclcpp::get_logger("ReceiverOdomHandler"), "퍼블리셔가 없습니다");
      }
    }
    catch (const std::exception& e) {
      RCLCPP_ERROR(rclcpp::get_logger("ReceiverOdomHandler"), "Odometry 메시지 처리 실패: %s", e.what());
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
          RCLCPP_ERROR(rclcpp::get_logger("ReceiverOdomHandler"), "핑 메시지 전송 실패: %s", ec.message().c_str());
        }
        async_send_message();
      });
  }
};

// 플러그인 클래스 등록
PLUGINLIB_EXPORT_CLASS(ReceiverOdomHandler, cobiz_bridge::MessageHandlerBase)