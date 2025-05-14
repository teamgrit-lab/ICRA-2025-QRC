#include "message_handler_base.hpp"
#include <nav_msgs/msg/occupancy_grid.hpp>  // OccupancyGrid 메시지 타입 헤더
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

class OccupancyGridHandler : public cobiz_bridge::MessageHandlerBase {
public:
  OccupancyGridHandler() = default;
  ~OccupancyGridHandler() override = default;

  void plugin_init() override {
    mime_ = "text/json";
    send_mime();
    ws_ptr->binary(true);
  }

  void handle(std::shared_ptr<rclcpp::SerializedMessage> serialized_msg) override {
    // 로거 생성
    auto logger = rclcpp::get_logger("OccupancyGridHandler");
    // RCLCPP_INFO(logger, "OccupancyGrid 메시지 처리 중: %zu 바이트", serialized_msg->size());
    
    try {
      // 메시지 Deserialization
      auto msg = std::make_shared<nav_msgs::msg::OccupancyGrid>();
      rclcpp::Serialization<nav_msgs::msg::OccupancyGrid> serialization;
      serialization.deserialize_message(serialized_msg.get(), msg.get());
      
      // OccupancyGrid 메시지 처리
      std::vector<uint8_t> compressed_data = compressToPNG(msg);
      std::string base64_data = base64Encode(compressed_data);
      json j;
      j["topic"] = topic_;
      j["type"] = "map";
      j["header"]["stamp"]["sec"] = msg->header.stamp.sec;
      j["header"]["stamp"]["nanosec"] = msg->header.stamp.nanosec;
      j["header"]["frame_id"] = msg->header.frame_id;
      j["info"]["map_load_time"]["sec"] = msg->info.map_load_time.sec;
      j["info"]["map_load_time"]["nanosec"] = msg->info.map_load_time.nanosec;
      j["info"]["width"] = msg->info.width;
      j["info"]["height"] = msg->info.height;
      j["info"]["resolution"] = msg->info.resolution;
      j["info"]["origin"]["position"]["x"] = msg->info.origin.position.x;
      j["info"]["origin"]["position"]["y"] = msg->info.origin.position.y;
      j["info"]["origin"]["position"]["z"] = msg->info.origin.position.z;
      j["info"]["origin"]["orientation"]["x"] = msg->info.origin.orientation.x;
      j["info"]["origin"]["orientation"]["y"] = msg->info.origin.orientation.y;
      j["info"]["origin"]["orientation"]["z"] = msg->info.origin.orientation.z;
      j["info"]["origin"]["orientation"]["w"] = msg->info.origin.orientation.w;
      j["data"] = base64_data;
      std::string json_str = j.dump();
      send_message(json_str);
      
    } catch (const std::exception& e) {
      RCLCPP_ERROR(logger, "OccupancyGrid 메시지 deserialize 실패: %s", e.what());
    }
  }

  std::string getMessageType() const override {
    return "nav_msgs/msg/OccupancyGrid";
  }
private:
  std::vector<uint8_t> compressToPNG(const nav_msgs::msg::OccupancyGrid::SharedPtr msg){
    int width = msg->info.width;
    int height = msg->info.height;
    cv::Mat image(height, width, CV_8UC1);

    for (int i = 0; i < height; ++i) {
      for (int j = 0; j < width; ++j) {
        int index = i * width + j;
        if (msg->data[index] == -1) {
          image.at<uchar>(i, j) = 255; // Unknown
        } else {
          image.at<uchar>(i, j) = msg->data[index] == 0 ? 100 : 0; // Free or Occupied
        }
      }
    }
    std::vector<uint8_t> buffer;
    cv::imencode(".png", image, buffer);
    return buffer;
  }

  
  std::string base64Encode(const std::vector<uint8_t>& input) {
    BIO* bio, * b64;
    BUF_MEM* bufferPtr;
    b64 = BIO_new(BIO_f_base64());
    bio = BIO_new(BIO_s_mem());
    bio = BIO_push(b64, bio);
    BIO_set_flags(bio, BIO_FLAGS_BASE64_NO_NL);
    BIO_write(bio, input.data(), input.size());
    BIO_flush(bio);
    BIO_get_mem_ptr(bio, &bufferPtr);
    std::string encodedData(bufferPtr->data, bufferPtr->length);
    // std::cout << "base64 size: " << bufferPtr->length << std::endl;
    BIO_free_all(bio);
    return encodedData;
  }
};

// 플러그인 클래스 등록
PLUGINLIB_EXPORT_CLASS(OccupancyGridHandler, cobiz_bridge::MessageHandlerBase)
