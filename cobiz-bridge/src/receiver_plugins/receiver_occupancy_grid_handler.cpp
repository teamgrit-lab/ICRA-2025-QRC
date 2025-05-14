#include "message_handler_base.hpp"
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <rclcpp/serialization.hpp>
#include <opencv2/opencv.hpp>
#include <openssl/bio.h>
#include <openssl/evp.h>
#include <vector>
#include <nlohmann/json.hpp>

using boost::asio::ip::tcp;
namespace beast = boost::beast;
namespace ssl = boost::asio::ssl;
namespace websocket = beast::websocket;
using json = nlohmann::json;

class ReceiverOccupancyGridHandler : public cobiz_bridge::MessageHandlerBase,
                               public std::enable_shared_from_this<ReceiverOccupancyGridHandler> {
public:
  ReceiverOccupancyGridHandler() = default;
  ~ReceiverOccupancyGridHandler() override = default;

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
    return "nav_msgs/msg/OccupancyGrid";
  }
  
  void handle(std::shared_ptr<rclcpp::SerializedMessage> /*serialized_msg*/) override {
    // Publisher mode - this function is not used in receiver
    RCLCPP_WARN(rclcpp::get_logger("ReceiverOccupancyGridHandler"), "handle 함수는 수신기에서 사용되지 않습니다");
  }

private:
  void async_receive_message() {
    read_timer_->expires_after(std::chrono::milliseconds(100));
    read_timer_->async_wait([this](beast::error_code ec) {
      on_read_timeout(ec);
    });
  }
  
  void on_read_timeout(beast::error_code ec) {
    if (ec) {
      RCLCPP_ERROR(rclcpp::get_logger("ReceiverOccupancyGridHandler"), "읽기 타임아웃 오류: %s", ec.message().c_str());
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
        RCLCPP_ERROR(rclcpp::get_logger("ReceiverOccupancyGridHandler"), "읽기 오류: %s", ec.message().c_str());
      return;
    }
    
    // JSON 데이터 처리
    std::string message = beast::buffers_to_string(buffer_.data());
    buffer_.consume(bytes_transferred);
    
    try {
      json j = json::parse(message);
      
      // OccupancyGrid 메시지 생성
      auto msg = std::make_shared<nav_msgs::msg::OccupancyGrid>();
      
      // 헤더 설정
      if (j.contains("header")) {
        msg->header.stamp.sec = j["header"]["stamp"]["sec"];
        msg->header.stamp.nanosec = j["header"]["stamp"]["nanosec"];
        msg->header.frame_id = j["header"]["frame_id"];
      } else {
        msg->header.stamp = rclcpp::Clock().now();
        msg->header.frame_id = "map";
      }
      
      // Info 설정
      if (j.contains("info")) {
        msg->info.map_load_time.sec = j["info"]["map_load_time"]["sec"];
        msg->info.map_load_time.nanosec = j["info"]["map_load_time"]["nanosec"];
        msg->info.width = j["info"]["width"];
        msg->info.height = j["info"]["height"];
        msg->info.resolution = j["info"]["resolution"];
        msg->info.origin.position.x = j["info"]["origin"]["position"]["x"];
        msg->info.origin.position.y = j["info"]["origin"]["position"]["y"];
        msg->info.origin.position.z = j["info"]["origin"]["position"]["z"];
        msg->info.origin.orientation.x = j["info"]["origin"]["orientation"]["x"];
        msg->info.origin.orientation.y = j["info"]["origin"]["orientation"]["y"];
        msg->info.origin.orientation.z = j["info"]["origin"]["orientation"]["z"];
        msg->info.origin.orientation.w = j["info"]["origin"]["orientation"]["w"];
      }
      
      // Base64로 인코딩된 PNG 데이터 디코딩
      if (j.contains("data")) {
        std::string base64_data = j["data"];
        std::vector<uint8_t> png_data = base64Decode(base64_data);
        
        // PNG를 OccupancyGrid로 변환
        convertPNGToOccupancyGrid(png_data, msg);
      } else {
        RCLCPP_ERROR(rclcpp::get_logger("ReceiverOccupancyGridHandler"), "데이터 필드가 없습니다");
        async_receive_message();
        return;
      }
      
      // 메시지 시리얼라이즈 및 발행
      rclcpp::SerializedMessage serialized_msg;
      rclcpp::Serialization<nav_msgs::msg::OccupancyGrid> serializer;
      serializer.serialize_message(msg.get(), &serialized_msg);
      
      if (publisher_) {
        publisher_->publish(serialized_msg);
        publisher_->publish(serialized_msg);
        RCLCPP_DEBUG(rclcpp::get_logger("ReceiverOccupancyGridHandler"), 
                    "OccupancyGrid 메시지 발행 완료 (%dx%d)",
                    msg->info.width, msg->info.height);
      } else {
        RCLCPP_WARN(rclcpp::get_logger("ReceiverOccupancyGridHandler"), "퍼블리셔가 없습니다");
      }
    } catch (const std::exception& e) {
      RCLCPP_ERROR(rclcpp::get_logger("ReceiverOccupancyGridHandler"), "JSON 파싱 실패: %s", e.what());
    }
    
    async_receive_message();
  }
  
  std::vector<uint8_t> base64Decode(const std::string& input) {
    BIO* bio, *b64;
    std::vector<uint8_t> result(input.size()); // 디코딩 결과는 인코딩보다 작으므로 충분한 크기

    b64 = BIO_new(BIO_f_base64());
    bio = BIO_new_mem_buf(input.data(), static_cast<int>(input.size()));
    bio = BIO_push(b64, bio);
    BIO_set_flags(bio, BIO_FLAGS_BASE64_NO_NL);
    
    int decoded_size = BIO_read(bio, result.data(), static_cast<int>(input.size()));
    result.resize(decoded_size > 0 ? decoded_size : 0); // 실제 디코딩 크기로 조정
    
    BIO_free_all(bio);
    return result;
  }
  
  void convertPNGToOccupancyGrid(const std::vector<uint8_t>& png_data, nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
    // PNG 데이터를 OpenCV 이미지로 디코딩
    cv::Mat image = cv::imdecode(png_data, cv::IMREAD_GRAYSCALE);
    
    if (image.empty()) {
      RCLCPP_ERROR(rclcpp::get_logger("ReceiverOccupancyGridHandler"), "PNG 데이터를 이미지로 변환할 수 없습니다");
      return;
    }
    
    // 이미지 크기가 맞는지 확인
    if (image.rows != static_cast<int>(msg->info.height) || image.cols != static_cast<int>(msg->info.width)) {
      RCLCPP_WARN(rclcpp::get_logger("ReceiverOccupancyGridHandler"), 
                 "이미지 크기 불일치: %dx%d (이미지), %dx%d (메시지)", 
                 image.cols, image.rows, msg->info.width, msg->info.height);
      
      // 이미지 크기로 업데이트
      msg->info.width = image.cols;
      msg->info.height = image.rows;
    }
    
    // 이미지를 OccupancyGrid 데이터로 변환
    msg->data.resize(msg->info.width * msg->info.height);
    
    for (int i = 0; i < image.rows; ++i) {
      for (int j = 0; j < image.cols; ++j) {
        int index = i * image.cols + j;
        uchar pixel = image.at<uchar>(i, j);
        
        // 픽셀 값 변환: 255(흰색) = 자유 공간, 0(검은색) = 점유 공간, 기타 = 미지 영역
        if (pixel == 255) {
          msg->data[index] = -1; // 자유 공간
        } else if (pixel == 0) {
          msg->data[index] = 100; // 점유 공간
        } else {
          msg->data[index] = 0; // 미지 영역
        }
      }
    }
  }
  
  void async_send_message() {
    write_timer_->expires_after(std::chrono::seconds(10));
    write_timer_->async_wait([this](beast::error_code ec) {
      on_write_timeout(ec);
    });
  }
  
  void on_write_timeout(beast::error_code ec) {
    if (ec) return;
    
    std::string message = "ping";
    ws_ptr->async_write(boost::asio::buffer(message),
      [this](beast::error_code ec, std::size_t /*bytes_transferred*/) {
        if (ec) {
          RCLCPP_ERROR(rclcpp::get_logger("ReceiverOccupancyGridHandler"), "핑 메시지 전송 실패: %s", ec.message().c_str());
        }
        async_send_message();
      });
  }
};

// 플러그인 클래스 등록
#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(ReceiverOccupancyGridHandler, cobiz_bridge::MessageHandlerBase)
