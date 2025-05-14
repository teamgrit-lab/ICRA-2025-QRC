#include "message_handler_base.hpp"
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nlohmann/json.hpp>

using boost::asio::ip::tcp;
namespace beast = boost::beast;
namespace ssl = boost::asio::ssl;
namespace websocket = beast::websocket;
using json = nlohmann::json;

class ReceiverPoseStampedHandler : public cobiz_bridge::MessageHandlerBase,
                           public std::enable_shared_from_this<ReceiverPoseStampedHandler> {
public:
  ReceiverPoseStampedHandler() = default;
  ~ReceiverPoseStampedHandler() override = default;

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
    // CMD_VEL 메시지를 JSON으로 변환하여 전송
    try {
      auto msg = std::make_shared<geometry_msgs::msg::PoseStamped>();
      rclcpp::Serialization<geometry_msgs::msg::PoseStamped> serialization;
      serialization.deserialize_message(serialized_msg.get(), msg.get());
      std::cout << "PoseStamped 메시지 수신: " << msg->header.frame_id << std::endl;
      std::cout << "PoseStamped 메시지 수신: " << msg->pose.position.x << ", " 
                << msg->pose.position.y << ", " << msg->pose.position.z << std::endl; 
      
      json j;
      j["header"]["stamp"]["sec"] = msg->header.stamp.sec;
      j["header"]["stamp"]["nanosec"] = msg->header.stamp.nanosec;
      j["header"]["frame_id"] = msg->header.frame_id;
      j["pose"]["position"]["x"] = msg->pose.position.x;
      j["pose"]["position"]["y"] = msg->pose.position.y;
      j["pose"]["position"]["z"] = msg->pose.position.z;
      j["pose"]["orientation"]["x"] = msg->pose.orientation.x;
      j["pose"]["orientation"]["y"] = msg->pose.orientation.y;
      j["pose"]["orientation"]["z"] = msg->pose.orientation.z;
      j["pose"]["orientation"]["w"] = msg->pose.orientation.w;
      
      std::string message = j.dump();
      
      ws_ptr->async_write(boost::asio::buffer(message),
        [this](beast::error_code ec, std::size_t bytes_transferred) {
          if (ec) {
            RCLCPP_ERROR(rclcpp::get_logger("ReceiverPoseStampedHandler"), "쓰기 오류: %s", ec.message().c_str());
          }
        });
    }
    catch (const std::exception& e) {
      RCLCPP_ERROR(rclcpp::get_logger("ReceiverPoseStampedHandler"), "PoseStamped 메시지 처리 실패: %s", e.what());
    }
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
      RCLCPP_ERROR(rclcpp::get_logger("ReceiverPoseStampedHandler"), "읽기 타임아웃 오류: %s", ec.message().c_str());
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
        RCLCPP_ERROR(rclcpp::get_logger("ReceiverPoseStampedHandler"), "읽기 오류: %s", ec.message().c_str());
      return;
    }
    
    std::string message = beast::buffers_to_string(buffer_.data());
    buffer_.consume(bytes_transferred);
    
    // try {
    //   json j = json::parse(message);
      
    //   auto msg = std::make_shared<geometry_msgs::msg::PoseStamped>();
      
    //   rclcpp::SerializedMessage serialized_msg;
    //   rclcpp::Serialization<geometry_msgs::msg::PoseStamped> serializer;
    //   serializer.serialize_message(msg.get(), &serialized_msg);
      
    //   if (publisher_) {
    //     publisher_->publish(serialized_msg);
    //   } else {
    //     RCLCPP_WARN(rclcpp::get_logger("ReceiverPoseStampedHandler"), "퍼블리셔가 없습니다");
    //   }
    // }
    // catch (const std::exception& e) {
    //   RCLCPP_ERROR(rclcpp::get_logger("ReceiverPoseStampedHandler"), "JSON 파싱 실패: %s", e.what());
    // }
    
    async_receive_message();
  }
  
  void async_send_message(){
    write_timer_->expires_after(std::chrono::seconds(10));
    write_timer_->async_wait([this](beast::error_code ec) {
      on_write_timeout(ec);
    });
  }
  
  void on_write_timeout(beast::error_code ec) {
    if (ec) return;
    
    std::string message = "ping";
    ws_ptr->async_write(boost::asio::buffer(message),
      [this](beast::error_code ec, std::size_t) {
        if (ec) {
          RCLCPP_ERROR(rclcpp::get_logger("ReceiverPoseStampedHandler"), "핑 메시지 전송 실패: %s", ec.message().c_str());
        }
        async_send_message();
      });
  }
};

// 플러그인 클래스 등록
PLUGINLIB_EXPORT_CLASS(ReceiverPoseStampedHandler, cobiz_bridge::MessageHandlerBase)
