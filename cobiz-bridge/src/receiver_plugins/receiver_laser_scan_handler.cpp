#include "message_handler_base.hpp"
#include <sensor_msgs/msg/laser_scan.hpp>
#include <rclcpp/serialization.hpp>
#include <nlohmann/json.hpp>
#include <vector>
#include <draco/compression/decode.h>
#include <draco/point_cloud/point_cloud.h>
#include <cmath>
#include <limits>
#include <algorithm>

using boost::asio::ip::tcp;
namespace beast = boost::beast;
namespace ssl = boost::asio::ssl;
namespace websocket = beast::websocket;
using json = nlohmann::json;

class ReceiverLaserScanHandler : public cobiz_bridge::MessageHandlerBase,
                               public std::enable_shared_from_this<ReceiverLaserScanHandler> {
public:
  ReceiverLaserScanHandler() = default;
  ~ReceiverLaserScanHandler() override = default;

  void plugin_init() override {
    mime_ = "application/octet-stream"; // Changed for binary Draco data
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
    return "sensor_msgs/msg/LaserScan";
  }
  
  void handle(std::shared_ptr<rclcpp::SerializedMessage> serialized_msg) override {
    try {
      auto msg = std::make_shared<sensor_msgs::msg::LaserScan>();
      rclcpp::Serialization<sensor_msgs::msg::LaserScan> serialization;
      serialization.deserialize_message(serialized_msg.get(), msg.get());
      
      json j;
      
      // 기본 필드 설정
      j["header"]["stamp"]["sec"] = msg->header.stamp.sec;
      j["header"]["stamp"]["nanosec"] = msg->header.stamp.nanosec;
      j["header"]["frame_id"] = msg->header.frame_id;
      j["angle_min"] = msg->angle_min;
      j["angle_max"] = msg->angle_max;
      j["angle_increment"] = msg->angle_increment;
      j["time_increment"] = msg->time_increment;
      j["scan_time"] = msg->scan_time;
      j["range_min"] = msg->range_min;
      j["range_max"] = msg->range_max;
      
      // 배열 필드 설정
      j["ranges"] = json::array();
      for (const auto& range : msg->ranges) {
        j["ranges"].push_back(range);
      }
      
      j["intensities"] = json::array();
      for (const auto& intensity : msg->intensities) {
        j["intensities"].push_back(intensity);
      }
      
      std::string message = j.dump();
      ws_ptr->async_write(boost::asio::buffer(message),
        [this](beast::error_code ec, std::size_t /*bytes_transferred*/) {
          if (ec) {
            RCLCPP_ERROR(rclcpp::get_logger("ReceiverLaserScanHandler"), "쓰기 오류: %s", ec.message().c_str());
          }
        });
    }
    catch (const std::exception& e) {
      RCLCPP_ERROR(rclcpp::get_logger("ReceiverLaserScanHandler"), "LaserScan 메시지 처리 실패: %s", e.what());
    }
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
      RCLCPP_ERROR(rclcpp::get_logger("ReceiverLaserScanHandler"), "읽기 타임아웃 오류: %s", ec.message().c_str());
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
        RCLCPP_ERROR(rclcpp::get_logger("ReceiverLaserScanHandler"), "읽기 오류: %s", ec.message().c_str());
      return;
    }
    
    // Process binary Draco data
    std::string data = beast::buffers_to_string(buffer_.data());
    buffer_.consume(bytes_transferred);
    
    RCLCPP_DEBUG(rclcpp::get_logger("ReceiverLaserScanHandler"), 
                "Draco 압축 데이터 수신: %zu 바이트", data.size());
    
    try {
      // Draco 압축 해제
      draco::DecoderBuffer decoder_buffer;
      decoder_buffer.Init(data.data(), data.size());
      
      // Draco 디코더 설정
      draco::Decoder decoder;
      auto statusor = decoder.DecodePointCloudFromBuffer(&decoder_buffer);
      if (!statusor.ok()) {
        RCLCPP_ERROR(rclcpp::get_logger("ReceiverLaserScanHandler"), 
                    "Draco 압축 해제 실패: %s", statusor.status().error_msg());
        async_receive_message();
        return;
      }

      // 압축 해제된 포인트 클라우드 추출
      std::unique_ptr<draco::PointCloud> pc = std::move(statusor).value();
      
      // 사용 가능한 모든 속성 로깅 - 디버깅용
      RCLCPP_DEBUG(rclcpp::get_logger("ReceiverLaserScanHandler"), 
                 "Draco 포인트 클라우드 속성 수: %d", pc->num_attributes());
      
      for (int att_id = 0; att_id < pc->num_attributes(); ++att_id) {
        const auto* attr = pc->attribute(att_id);
        RCLCPP_DEBUG(rclcpp::get_logger("ReceiverLaserScanHandler"), 
                   "속성 #%d: 타입=%d, 데이터 타입=%d, 요소 수=%d",
                   att_id, attr->attribute_type(), attr->data_type(),
                   attr->num_components());
      }
      
      // 포지션 속성 찾기 (LaserScanHandler에서 사용한 것과 일치)
      const draco::PointAttribute* position_att = pc->GetNamedAttribute(draco::GeometryAttribute::POSITION, 0);
      if (!position_att) {
        // 대체 속성 찾기 시도
        position_att = pc->GetNamedAttribute(draco::GeometryAttribute::GENERIC, 0);
        if (!position_att && pc->num_attributes() > 0) {
          position_att = pc->attribute(0);
        }
        
        if (!position_att) {
          RCLCPP_ERROR(rclcpp::get_logger("ReceiverLaserScanHandler"), 
                      "포인트 클라우드에서 위치 속성을 찾을 수 없습니다");
          async_receive_message();
          return;
        }
      }
      
      // LaserScan 메시지 생성
      auto msg = std::make_shared<sensor_msgs::msg::LaserScan>();
      
      // 헤더 설정 - 현재 시간 사용
      msg->header.stamp = rclcpp::Clock().now();
      msg->header.frame_id = "laser"; // 필요에 따라 수정
      
      // 포인트 수 가져오기
      int num_points = pc->num_points();
      RCLCPP_DEBUG(rclcpp::get_logger("ReceiverLaserScanHandler"), 
                 "포인트 클라우드 포인트 수: %d", num_points);
      
      if (num_points == 0) {
        RCLCPP_WARN(rclcpp::get_logger("ReceiverLaserScanHandler"), 
                  "빈 포인트 클라우드를 받았습니다");
        async_receive_message();
        return;
      }
      
      // 각도 범위 계산을 위한 변수들
      float min_angle = std::numeric_limits<float>::max();
      float max_angle = -std::numeric_limits<float>::max();
      
      // 포인트 클라우드에서 거리 값 추출
      std::vector<float> ranges;
      std::vector<float> angles;
      ranges.reserve(num_points);
      angles.reserve(num_points);
      
      for (int i = 0; i < num_points; ++i) {
        // 각 점의 XYZ 좌표 얻기
        std::array<float, 3> point_xyz;
        const draco::AttributeValueIndex point_index(i);
        
        if (position_att->num_components() == 3) {
          position_att->GetValue(point_index, &point_xyz);
        } else if (position_att->num_components() == 2) {
          std::array<float, 2> point_xy;
          position_att->GetValue(point_index, &point_xy);
          point_xyz[0] = point_xy[0];
          point_xyz[1] = point_xy[1];
          point_xyz[2] = 0.0f;
        } else {
          continue; // 지원하지 않는 컴포넌트 수
        }
        
        float x = point_xyz[0];
        float y = point_xyz[1];
        
        // NaN 체크 - 유효한 포인트만 처리
        if (std::isnan(x) || std::isnan(y)) {
          continue;
        }
        
        // 직교 좌표를 극좌표로 변환
        float range = std::sqrt(x * x + y * y);
        float angle = std::atan2(y, x);
        
        // 유효한 범위 내의 값만 저장
        if (range > 0.0f && range < 100.0f) {  // 적절한 최대 거리 설정
          ranges.push_back(range);
          angles.push_back(angle);
          
          // 최소/최대 각도 업데이트
          min_angle = std::min(min_angle, angle);
          max_angle = std::max(max_angle, angle);
        }
      }
      
      // 데이터가 충분한지 확인
      if (ranges.empty()) {
        RCLCPP_WARN(rclcpp::get_logger("ReceiverLaserScanHandler"), 
                  "유효한 레이저 스캔 포인트가 없습니다");
        async_receive_message();
        return;
      }
      
      // 각도 간격 계산 - 균일하게 간격이 나뉜 스캔을 가정
      float angle_range = max_angle - min_angle;
      float angle_increment = angle_range / (ranges.size() - 1);
      if (ranges.size() == 1) {
        angle_increment = 0.01f; // 포인트가 하나뿐인 경우 기본값 설정
      }
      
      // LaserScan 필드 설정
      msg->angle_min = min_angle;
      msg->angle_max = max_angle;
      msg->angle_increment = angle_increment;
      msg->time_increment = 0.0;
      msg->scan_time = 0.1;
      msg->range_min = 0.1;
      msg->range_max = 100.0;
      
      // 각도에 따라 포인트를 정렬하여 스캔 순서에 맞게 배열
      std::vector<std::pair<float, float>> angle_range_pairs;
      for (size_t i = 0; i < angles.size(); ++i) {
        angle_range_pairs.emplace_back(angles[i], ranges[i]);
      }
      
      std::sort(angle_range_pairs.begin(), angle_range_pairs.end(),
                [](const auto& a, const auto& b) { return a.first < b.first; });
      
      // 최종 레인지 배열 설정
      msg->ranges.resize(angle_range_pairs.size());
      for (size_t i = 0; i < angle_range_pairs.size(); ++i) {
        msg->ranges[i] = angle_range_pairs[i].second;
      }
      
      // 메시지 시리얼라이즈 및 발행
      rclcpp::SerializedMessage serialized_msg;
      rclcpp::Serialization<sensor_msgs::msg::LaserScan> serializer;
      serializer.serialize_message(msg.get(), &serialized_msg);
      
      if (publisher_) {
        publisher_->publish(serialized_msg);
        RCLCPP_DEBUG(rclcpp::get_logger("ReceiverLaserScanHandler"), 
                    "LaserScan 메시지 발행 완료 (%zu 범위)",
                    msg->ranges.size());
      } else {
        RCLCPP_WARN(rclcpp::get_logger("ReceiverLaserScanHandler"), "퍼블리셔가 없습니다");
      }
    }
    catch (const std::exception& e) {
      RCLCPP_ERROR(rclcpp::get_logger("ReceiverLaserScanHandler"), "Draco 데이터 처리 실패: %s", e.what());
    }
    
    async_receive_message();
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
      [this](beast::error_code ec, std::size_t) {
        if (ec) {
          RCLCPP_ERROR(rclcpp::get_logger("ReceiverLaserScanHandler"), "핑 메시지 전송 실패: %s", ec.message().c_str());
        }
        async_send_message();
      });
  }
};

// 플러그인 클래스 등록
#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(ReceiverLaserScanHandler, cobiz_bridge::MessageHandlerBase)