#include "message_handler_base.hpp"
#include <sensor_msgs/msg/image.hpp>
#include <rclcpp/serialization.hpp>
#include <nlohmann/json.hpp>
#include <gst/gst.h>
#include <gst/app/gstappsrc.h>
#include <gst/app/gstappsink.h>
#include <opencv2/opencv.hpp>

using boost::asio::ip::tcp;
namespace beast = boost::beast;
namespace ssl = boost::asio::ssl;
namespace websocket = beast::websocket;
using json = nlohmann::json;

class ReceiverImageHandler : public cobiz_bridge::MessageHandlerBase,
                           public std::enable_shared_from_this<ReceiverImageHandler> {
public:
  ReceiverImageHandler() {
    // GStreamer 초기화 (필요 시)
    static bool gst_initialized = false;
    if (!gst_initialized) {
      gst_initialized = true;
      gst_init(nullptr, nullptr);
    }
  }
  
  ~ReceiverImageHandler() override {
    // GStreamer 파이프라인 정리
    if (pipeline_) {
      gst_element_set_state(pipeline_, GST_STATE_NULL);
      gst_object_unref(pipeline_);
      pipeline_ = nullptr;
    }
  }

  void plugin_init() override {
    mime_ = "application/h264";  // H.264 스트림을 기대함
    send_mime();
    ws_ptr->binary(true);
    
    // GStreamer 파이프라인 설정
    setup_gstreamer();
    
    read_timer_ = std::make_shared<boost::asio::steady_timer>(get_io_context(), std::chrono::milliseconds(100));
    write_timer_ = std::make_shared<boost::asio::steady_timer>(get_io_context(), std::chrono::seconds(10));
    
    async_receive_message();
    async_send_message();
    
    io_thread_ = std::thread([this]() {
        ioc_.run();
    });
  }

  std::string getMessageType() const override {
    return "sensor_msgs/msg/Image";
  }
  
  void handle(std::shared_ptr<rclcpp::SerializedMessage> serialized_msg) override {
    // 수신기에서는 이 메서드가 사용되지 않음
  }

private:
  GstElement* pipeline_ = nullptr;
  GstElement* appsrc_ = nullptr;
  GstElement* appsink_ = nullptr;
  bool first_frame_ = true;
  
  void setup_gstreamer() {
    std::string pipeline_str = 
        "appsrc name=source is-live=true format=time ! "
        "h264parse ! "
        "decodebin ! "
        "videoconvert ! "
        "video/x-raw,format=BGR ! "
        "appsink name=sink sync=false";
    
    GError* error = nullptr;
    pipeline_ = gst_parse_launch(pipeline_str.c_str(), &error);
    
    if (error) {
      RCLCPP_ERROR(rclcpp::get_logger("ReceiverImageHandler"), 
                  "GStreamer 파이프라인 생성 실패: %s", error->message);
      g_error_free(error);
      return;
    }
    
    // AppSrc 설정
    appsrc_ = gst_bin_get_by_name(GST_BIN(pipeline_), "source");
    g_object_set(G_OBJECT(appsrc_), 
                "stream-type", GST_APP_STREAM_TYPE_STREAM,
                "format", GST_FORMAT_TIME,
                NULL);
    
    // AppSink 설정
    appsink_ = gst_bin_get_by_name(GST_BIN(pipeline_), "sink");
    g_object_set(G_OBJECT(appsink_), "emit-signals", TRUE, NULL);
    g_signal_connect(appsink_, "new-sample", G_CALLBACK(on_new_sample_static), this);
    
    // 파이프라인 시작
    gst_element_set_state(pipeline_, GST_STATE_PLAYING);
  }
  
  static GstFlowReturn on_new_sample_static(GstElement* sink, gpointer user_data) {
    return static_cast<ReceiverImageHandler*>(user_data)->on_new_sample(sink);
  }
  
  GstFlowReturn on_new_sample(GstElement* sink) {
    GstSample* sample = gst_app_sink_pull_sample(GST_APP_SINK(sink));
    if (!sample) return GST_FLOW_ERROR;
    
    GstBuffer* buffer = gst_sample_get_buffer(sample);
    GstCaps* caps = gst_sample_get_caps(sample);
    GstStructure* structure = gst_caps_get_structure(caps, 0);
    
    // 이미지 크기 가져오기
    int width, height;
    gst_structure_get_int(structure, "width", &width);
    gst_structure_get_int(structure, "height", &height);
    
    // 버퍼 맵핑
    GstMapInfo map;
    if (gst_buffer_map(buffer, &map, GST_MAP_READ)) {
      try {
        // OpenCV Mat으로 변환
        cv::Mat frame(height, width, CV_8UC3, (void*)map.data);
        
        // ROS 메시지 생성 및 발행
        auto img_msg = std::make_shared<sensor_msgs::msg::Image>();
        img_msg->header.stamp = rclcpp::Clock().now();
        img_msg->header.frame_id = "camera";
        img_msg->height = height;
        img_msg->width = width;
        img_msg->encoding = "bgr8";
        img_msg->is_bigendian = false;
        img_msg->step = width * 3;  // 3 bytes per pixel for BGR
        
        const size_t size = frame.total() * frame.elemSize();
        img_msg->data.resize(size);
        std::memcpy(img_msg->data.data(), frame.data, size);
        
        // 메시지 직렬화 및 발행
        rclcpp::SerializedMessage serialized_msg;
        rclcpp::Serialization<sensor_msgs::msg::Image> serializer;
        serializer.serialize_message(img_msg.get(), &serialized_msg);
        
        if (publisher_) {
          publisher_->publish(serialized_msg);
        } else {
          RCLCPP_WARN(rclcpp::get_logger("ReceiverImageHandler"), "퍼블리셔가 없습니다");
        }
      } catch (const std::exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("ReceiverImageHandler"), "이미지 처리 실패: %s", e.what());
      }
      
      gst_buffer_unmap(buffer, &map);
    }
    
    gst_sample_unref(sample);
    return GST_FLOW_OK;
  }
  
  void feed_gstreamer(const void* data, size_t size) {
    if (!pipeline_ || !appsrc_) return;
    
    GstBuffer* buffer = gst_buffer_new_allocate(nullptr, size, nullptr);
    GstMapInfo map;
    if (gst_buffer_map(buffer, &map, GST_MAP_WRITE)) {
      memcpy(map.data, data, size);
      gst_buffer_unmap(buffer, &map);
      
      // 첫 프레임이면 키프레임으로 표시
      if (first_frame_) {
        GST_BUFFER_FLAG_UNSET(buffer, GST_BUFFER_FLAG_DELTA_UNIT);
        first_frame_ = false;
      } else {
        GST_BUFFER_FLAG_SET(buffer, GST_BUFFER_FLAG_DELTA_UNIT);
      }
      
      GstFlowReturn ret;
      g_signal_emit_by_name(appsrc_, "push-buffer", buffer, &ret);
      gst_buffer_unref(buffer);
      
      if (ret != GST_FLOW_OK) {
        RCLCPP_ERROR(rclcpp::get_logger("ReceiverImageHandler"), "버퍼 푸시 실패");
      }
    }
  }

  void async_receive_message() {
    read_timer_->expires_after(std::chrono::milliseconds(10));
    read_timer_->async_wait([this](beast::error_code ec) {
      on_read_timeout(ec);
    });
  }
  
  void on_read_timeout(beast::error_code ec) {
    if (ec) {
      RCLCPP_ERROR(rclcpp::get_logger("ReceiverImageHandler"), "읽기 타임아웃 오류: %s", ec.message().c_str());
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
        RCLCPP_ERROR(rclcpp::get_logger("ReceiverImageHandler"), "읽기 오류: %s", ec.message().c_str());
      return;
    }
    
    try {
      // H.264 인코딩 데이터를 GStreamer로 전달
      const void* data = buffer_.data().data();
      feed_gstreamer(data, bytes_transferred);
    }
    catch (const std::exception& e) {
      RCLCPP_ERROR(rclcpp::get_logger("ReceiverImageHandler"), "데이터 처리 실패: %s", e.what());
    }
    
    buffer_.consume(bytes_transferred);
    async_receive_message();
  }
  
  void async_send_message() {
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
          RCLCPP_ERROR(rclcpp::get_logger("ReceiverImageHandler"), "핑 메시지 전송 실패: %s", ec.message().c_str());
        }
        async_send_message();
      });
  }
};

// 플러그인 클래스 등록
PLUGINLIB_EXPORT_CLASS(ReceiverImageHandler, cobiz_bridge::MessageHandlerBase)
