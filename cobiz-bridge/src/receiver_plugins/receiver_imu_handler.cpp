#include "message_handler_base.hpp"
#include <sensor_msgs/msg/imu.hpp>
#include <rclcpp/serialization.hpp>
#include <nlohmann/json.hpp>

using boost::asio::ip::tcp;
namespace beast = boost::beast;
namespace ssl = boost::asio::ssl;
namespace websocket = beast::websocket;
using json = nlohmann::json;

class ReceiverImuHandler : public cobiz_bridge::MessageHandlerBase,
                          public std::enable_shared_from_this<ReceiverImuHandler> {
public:
  ReceiverImuHandler() = default;
  ~ReceiverImuHandler() override = default;

  void plugin_init() override {
    mime_ = "text/json";
    send_mime();
    ws_ptr->binary(true);
    
    read_timer_ = std::make_shared<boost::asio::steady_timer>(get_io_context(), std::chrono::milliseconds(100));
    write_timer_ = std::make_shared<boost::asio::steady_timer>(get_io_context(), std::chrono::seconds(10));
    
    // WebSocket에서 데이터를 읽어와 ROS 토픽으로 발행
    async_receive_message();
    async_send_message();
    
    io_thread_ = std::thread([this]() {
        ioc_.run();
    });
  }

  std::string getMessageType() const override {
    return "sensor_msgs/msg/Imu";
  }
  
  // ROS 메시지 처리 (수신기에서는 사용 안함)
  void handle(std::shared_ptr<rclcpp::SerializedMessage> serialized_msg) override {
    // 이 메서드는 ReceiverImuHandler에서는 사용하지 않음 (WebSocket → ROS만 처리)
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
      RCLCPP_ERROR(rclcpp::get_logger("ReceiverImuHandler"), "읽기 타임아웃 오류: %s", ec.message().c_str());
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
        RCLCPP_ERROR(rclcpp::get_logger("ReceiverImuHandler"), "읽기 오류: %s", ec.message().c_str());
      return;
    }
    
    // 웹소켓에서 받은 메시지 처리
    std::string message = beast::buffers_to_string(buffer_.data());
    buffer_.consume(bytes_transferred);
    
    try {
      json j = json::parse(message);
      
      // IMU 메시지 생성
      auto msg = std::make_shared<sensor_msgs::msg::Imu>();
      
      // 헤더 설정
      msg->header.stamp.sec = j["header"]["stamp"]["sec"];
      msg->header.stamp.nanosec = j["header"]["stamp"]["nanosec"];
      msg->header.frame_id = j["header"]["frame_id"];
      
      // 방향 설정
      msg->orientation.x = j["orientation"]["x"];
      msg->orientation.y = j["orientation"]["y"];
      msg->orientation.z = j["orientation"]["z"];
      msg->orientation.w = j["orientation"]["w"];
      
      // 각속도 설정
      msg->angular_velocity.x = j["angular_velocity"]["x"];
      msg->angular_velocity.y = j["angular_velocity"]["y"];
      msg->angular_velocity.z = j["angular_velocity"]["z"];
      
      // 선형 가속도 설정
      msg->linear_acceleration.x = j["linear_acceleration"]["x"];
      msg->linear_acceleration.y = j["linear_acceleration"]["y"];
      msg->linear_acceleration.z = j["linear_acceleration"]["z"];
      
      // 공분산 설정 (필요시)
      for (int i = 0; i < 9; i++) {
        if (j["orientation_covariance"].is_array() && j["orientation_covariance"].size() >= 9) {
          msg->orientation_covariance[i] = j["orientation_covariance"][i];
        }
        if (j["angular_velocity_covariance"].is_array() && j["angular_velocity_covariance"].size() >= 9) {
          msg->angular_velocity_covariance[i] = j["angular_velocity_covariance"][i];
        }
        if (j["linear_acceleration_covariance"].is_array() && j["linear_acceleration_covariance"].size() >= 9) {
          msg->linear_acceleration_covariance[i] = j["linear_acceleration_covariance"][i];
        }
      }
      
      // 메시지 직렬화 및 게시
      rclcpp::SerializedMessage serialized_msg;
      rclcpp::Serialization<sensor_msgs::msg::Imu> serializer;
      serializer.serialize_message(msg.get(), &serialized_msg);
      
      // ROS 토픽으로 발행
      if (publisher_) {
        publisher_->publish(serialized_msg);
      } else {
        RCLCPP_WARN(rclcpp::get_logger("ReceiverImuHandler"), "퍼블리셔가 없습니다");
      }
    }
    catch (const std::exception& e) {
      RCLCPP_ERROR(rclcpp::get_logger("ReceiverImuHandler"), "IMU 메시지 처리 실패: %s", e.what());
    }
    
    // 다음 메시지 읽기 예약
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
    
    // 핑 메시지 전송
    std::string message = "ping";
    ws_ptr->async_write(boost::asio::buffer(message),
      [this](beast::error_code ec, std::size_t) {
        if (ec) {
          RCLCPP_ERROR(rclcpp::get_logger("ReceiverImuHandler"), "핑 메시지 전송 실패: %s", ec.message().c_str());
        }
        async_send_message();
      });
  }
};

// 플러그인 클래스 등록
PLUGINLIB_EXPORT_CLASS(ReceiverImuHandler, cobiz_bridge::MessageHandlerBase)
