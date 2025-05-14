#include "message_handler_base.hpp"
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <rclcpp/serialization.hpp>
#include <nlohmann/json.hpp>

using boost::asio::ip::tcp;
namespace beast = boost::beast;
namespace ssl = boost::asio::ssl;
namespace websocket = beast::websocket;
using json = nlohmann::json;

class ReceiverPointCloud2Handler : public cobiz_bridge::MessageHandlerBase,
                                  public std::enable_shared_from_this<ReceiverPointCloud2Handler> {
public:
  ReceiverPointCloud2Handler() = default;
  ~ReceiverPointCloud2Handler() override = default;

  void plugin_init() override {
    mime_ = "application/octet-stream";
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
    return "sensor_msgs/msg/PointCloud2";
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
      RCLCPP_ERROR(rclcpp::get_logger("ReceiverPointCloud2Handler"), "읽기 타임아웃 오류: %s", ec.message().c_str());
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
        RCLCPP_ERROR(rclcpp::get_logger("ReceiverPointCloud2Handler"), "읽기 오류: %s", ec.message().c_str());
      return;
    }
    
    try {
      // 데이터 처리 (헤더와 데이터 분리)
      std::string data_str(static_cast<const char*>(buffer_.data().data()), bytes_transferred);
      
      // 헤더와 데이터 분리 (JSON 헤더 + 바이너리 데이터 형식 가정)
      size_t header_end = data_str.find("\n\n");
      if (header_end == std::string::npos) {
        throw std::runtime_error("헤더와 데이터를 분리할 수 없습니다");
      }
      
      std::string header_str = data_str.substr(0, header_end);
      json header = json::parse(header_str);
      
      // PointCloud2 메시지 생성
      auto msg = std::make_shared<sensor_msgs::msg::PointCloud2>();
      
      // 헤더 설정
      msg->header.stamp.sec = header["header"]["stamp"]["sec"];
      msg->header.stamp.nanosec = header["header"]["stamp"]["nanosec"];
      msg->header.frame_id = header["header"]["frame_id"];
      
      // 기타 필드 설정
      msg->height = header["height"];
      msg->width = header["width"];
      msg->is_dense = header["is_dense"];
      msg->is_bigendian = header["is_bigendian"];
      msg->point_step = header["point_step"];
      msg->row_step = header["row_step"];
      
      // 필드 정보 설정
      if (header["fields"].is_array()) {
        for (const auto& field : header["fields"]) {
          sensor_msgs::msg::PointField point_field;
          point_field.name = field["name"];
          point_field.offset = field["offset"];
          point_field.datatype = field["datatype"];
          point_field.count = field["count"];
          msg->fields.push_back(point_field);
        }
      }
      
      // 데이터 부분 처리
      const char* data_start = data_str.data() + header_end + 2;
      size_t data_size = data_str.size() - header_end - 2;
      msg->data.resize(data_size);
      std::memcpy(msg->data.data(), data_start, data_size);
      
      // 메시지 직렬화 및 발행
      rclcpp::SerializedMessage serialized_msg;
      rclcpp::Serialization<sensor_msgs::msg::PointCloud2> serializer;
      serializer.serialize_message(msg.get(), &serialized_msg);
      
      if (publisher_) {
        publisher_->publish(serialized_msg);
      } else {
        RCLCPP_WARN(rclcpp::get_logger("ReceiverPointCloud2Handler"), "퍼블리셔가 없습니다");
      }
    }
    catch (const std::exception& e) {
      RCLCPP_ERROR(rclcpp::get_logger("ReceiverPointCloud2Handler"), "PointCloud2 메시지 처리 실패: %s", e.what());
    }
    
    buffer_.consume(bytes_transferred);
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
          RCLCPP_ERROR(rclcpp::get_logger("ReceiverPointCloud2Handler"), "핑 메시지 전송 실패: %s", ec.message().c_str());
        }
        async_send_message();
      });
  }
};

// 플러그인 클래스 등록
PLUGINLIB_EXPORT_CLASS(ReceiverPointCloud2Handler, cobiz_bridge::MessageHandlerBase)
