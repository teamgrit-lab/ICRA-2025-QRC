#include "message_handler_base.hpp"
#include <sensor_msgs/msg/laser_scan.hpp>

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <cmath>

#include <draco/compression/encode.h>
#include <draco/point_cloud/point_cloud.h>
#include <draco/point_cloud/point_cloud_builder.h>
#include <limits>

#include <nlohmann/json.hpp>

using boost::asio::ip::tcp;
namespace beast = boost::beast;
namespace ssl = boost::asio::ssl;
namespace websocket = beast::websocket;
using json = nlohmann::json;

class LaserScanHandler : public cobiz_bridge::MessageHandlerBase {
public:
  LaserScanHandler() = default;
  ~LaserScanHandler() override = default;

  void plugin_init() override {
    mime_ = "lidar/draco";
    send_mime();
    ws_ptr->binary(true);
  }

  void handle(std::shared_ptr<rclcpp::SerializedMessage> serialized_msg) override {
    // 로거 생성
    auto logger = rclcpp::get_logger("LaserScanHandler");
    // RCLCPP_INFO(logger, "LaserScan 메시지 처리 중: %zu 바이트", serialized_msg->size());
    
    try {
      // 메시지 Deserialization
      auto msg = std::make_shared<sensor_msgs::msg::LaserScan>();
      rclcpp::Serialization<sensor_msgs::msg::LaserScan> serialization;
      serialization.deserialize_message(serialized_msg.get(), msg.get());
      
      auto cloud_msg = std::make_shared<sensor_msgs::msg::PointCloud2>();
      convertLaserScanToPointCloud2(*msg, *cloud_msg);
      std::string compressed_data;
      compressPointCloud2(*cloud_msg, compressed_data);
      
      send_message(compressed_data);
      // RCLCPP_INFO(logger, "LaserScan 메시지 JSON 형식으로 %ld 바이트 전송", compressed_data.size());
      
    } catch (const std::exception& e) {
      RCLCPP_ERROR(logger, "LaserScan 메시지 deserialize 실패: %s", e.what());
    }
  }


  std::string getMessageType() const override {
    return "sensor_msgs/msg/LaserScan";
  }
private:
  void convertLaserScanToPointCloud2(const sensor_msgs::msg::LaserScan &scan, sensor_msgs::msg::PointCloud2 & cloud) {
    // PointCloud2 메시지 초기화
    cloud.header = scan.header;
    cloud.height = 1;
    cloud.width = scan.ranges.size();
    cloud.is_dense = false;
    cloud.is_bigendian = false;

    // 필드 정의
    sensor_msgs::PointCloud2Modifier modifier(cloud);
    modifier.setPointCloud2FieldsByString(1, "xyz");
    modifier.resize(scan.ranges.size());

    // 포인트 클라우드 데이터 변환
    sensor_msgs::PointCloud2Iterator<float> iter_x(cloud, "x");
    sensor_msgs::PointCloud2Iterator<float> iter_y(cloud, "y");
    sensor_msgs::PointCloud2Iterator<float> iter_z(cloud, "z");

    for (size_t i = 0; i < scan.ranges.size(); ++i) {
      float range = scan.ranges[i];
      if (range >= scan.range_min && range <= scan.range_max) {
        float angle = scan.angle_min + i * scan.angle_increment;
        *iter_x = range * cos(angle);
        *iter_y = range * sin(angle);
        *iter_z = 0.0f; // Z축은 0으로 설정
      } else {
        *iter_x = std::numeric_limits<float>::quiet_NaN();
        *iter_y = std::numeric_limits<float>::quiet_NaN();
        *iter_z = std::numeric_limits<float>::quiet_NaN();
      }
      ++iter_x;
      ++iter_y;
      ++iter_z;
    }
  }

  void compressPointCloud2(const sensor_msgs::msg::PointCloud2 &cloud, std::string &compressed_data) {
    draco::PointCloudBuilder builder;
    const int32_t num_points = cloud.width * cloud.height;
    builder.Start(num_points);

    int pos_attribute_id = builder.AddAttribute(draco::GeometryAttribute::POSITION, 3, draco::DT_FLOAT32);
    
    std::vector<float> point_data(num_points * 3);
    for (int i = 0; i < num_points; ++i) {
      float x, y, z;
      memcpy(&x, &cloud.data[i * cloud.point_step + cloud.fields[0].offset], sizeof(float));
      memcpy(&y, &cloud.data[i * cloud.point_step + cloud.fields[1].offset], sizeof(float));
      memcpy(&z, &cloud.data[i * cloud.point_step + cloud.fields[2].offset], sizeof(float));

      if (std::isnan(x) || std::isnan(y) || std::isnan(z)) {
        continue; // 유효하지 않은 포인트는 건너뜀
    }
      point_data[i * 3] = x;
      point_data[i * 3 + 1] = y;
      point_data[i * 3 + 2] = z;
    }
    builder.SetAttributeValuesForAllPoints(pos_attribute_id, point_data.data(), sizeof(float) * 3);
    std::unique_ptr<draco::PointCloud> buffer = builder.Finalize(false);

    draco::Encoder encoder;
    encoder.SetSpeedOptions(10, 10);
    encoder.SetAttributeQuantization(draco::GeometryAttribute::POSITION, 0);
    draco::EncoderBuffer compressed_buffer;
    encoder.EncodePointCloudToBuffer(*buffer, &compressed_buffer);
    compressed_data.assign(compressed_buffer.data(), compressed_buffer.size());

  }
};


// 플러그인 클래스 등록
PLUGINLIB_EXPORT_CLASS(LaserScanHandler, cobiz_bridge::MessageHandlerBase)
