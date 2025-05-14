#include "message_handler_base.hpp"
#include <sensor_msgs/msg/image.hpp>
#include <nlohmann/json.hpp>

#include <gst/gst.h>
#include <gst/app/gstappsink.h>
#include <gst/app/gstappsrc.h>

using boost::asio::ip::tcp;
namespace beast = boost::beast;
namespace ssl = boost::asio::ssl;
namespace websocket = beast::websocket;

class ImageHandler : public cobiz_bridge::MessageHandlerBase {
public:
  ImageHandler() = default;
  ~ImageHandler() override = default;

  void plugin_init() override {
    if (!gst_is_initialized()) {
      gst_init(nullptr, nullptr);
      // RCLCPP_INFO(rclcpp::get_logger("ImageHandler"), "GStreamer 초기화 완료");
    }
  }


  void handle(std::shared_ptr<rclcpp::SerializedMessage> serialized_msg) override {
    // 로거 생성
    // auto logger = rclcpp::get_logger("ImageHandler");
    // RCLCPP_INFO(logger, "Image 메시지 처리 중: %zu 바이트", serialized_msg->size());
    
    try {
      // 메시지 Deserialization
      auto msg = std::make_shared<sensor_msgs::msg::Image>();
      rclcpp::Serialization<sensor_msgs::msg::Image> serialization;
      serialization.deserialize_message(serialized_msg.get(), msg.get());
      if (!setup_) {
        setup_ = true;
        setup_pipeline(std::to_string(msg->width), std::to_string(msg->height), msg->encoding);
      }
      else if (pipeline_setup_) {
        // GStreamer 파이프라인에 이미지 데이터 전송
        GstBuffer* buffer = gst_buffer_new_allocate(nullptr, msg->data.size(), nullptr);
        gst_buffer_fill(buffer, 0, msg->data.data(), msg->data.size());
        GST_BUFFER_PTS(buffer) = gst_util_uint64_scale(msg->header.stamp.sec * 1e9 + msg->header.stamp.nanosec, GST_SECOND, 1);
        GST_BUFFER_DURATION(buffer) = gst_util_uint64_scale(1e9 / 30, GST_SECOND, 1); // 30 FPS
        GstFlowReturn ret;
        g_signal_emit_by_name(appsrc_, "push-buffer", buffer, &ret);
        gst_buffer_unref(buffer);
        if (ret != GST_FLOW_OK) {
          // RCLCPP_ERROR(logger, "GStreamer 파이프라인에 이미지 데이터 전송 실패");
        }
        // RCLCPP_INFO(logger, "Image 메시지 GStreamer 파이프라인에 전송 완료");
      }

      
    } catch (const std::exception& e) {
      // RCLCPP_ERROR(logger, "Image 메시지 deserialize 실패: %s", e.what());
    }
  }

  std::string getMessageType() const override {
    return "sensor_msgs/msg/Image";
  }

private:
  bool setup_ = false;
  bool send_mime_ = false;
  bool pipeline_setup_ = false;

  GstElement* pipeline_ = nullptr;
  GstElement* appsrc_ = nullptr;
  GstElement* appsink_ = nullptr;


  void setup_pipeline(const std::string& width, const std::string& height, const std::string& encoding) {
    // GStreamer 파이프라인 설정
    std::string bitrate;
    if (width >= "1920" && height >= "1080") {
      bitrate = "8192"; // 5Mbps
    } else if (width >= "1280" && height >= "720") {
      bitrate = "4096"; // 2Mbps
    } else {
      bitrate = "1024"; // 1Mbps
    }
    std::string encoding_;
    if (encoding == "rgb8") {
      encoding_ = "RGB";
    } else if (encoding == "bgr8") {
      encoding_ = "BGR";
    } else if (encoding == "bgra8") {
      encoding_ = "BGRA";
    } else if (encoding == "rgba8") {
      encoding_ = "RGBA";
    } else if (encoding == "yuv422") {
      encoding_ = "YUV";
    } else if (encoding == "yuv420") {
      encoding_ = "YUV";
    } else if (encoding == "yuv444") {
      encoding_ = "YUV";
    } else if (encoding == "yuv400") {
      encoding_ = "YUV";
    } else if (encoding == "yuv420p") {
      encoding_ = "YUV";
    } else if (encoding == "yuv422p") {
      encoding_ = "YUV";
    } else if (encoding == "yuv444p") {
      encoding_ = "YUV";
    } else {
      encoding_ = "RGB";
    }

    std::string pipeline_desc = "appsrc name=src is-live=true do-timestamp=true caps=video/x-raw,format=" + encoding_ + ",width=" + width + ",height=" + height + " ! videoconvert ! x264enc tune=zerolatency bitrate=" + bitrate + " speed-preset=ultrafast key-int-max=30 ! video/x-h264, profile=high, stream-format=byte-stream, alignment=au ! h264parse config-interval=1 ! video/x-h264, stream-format=byte-stream, alignment=au ! queue leaky=2 ! appsink name=sink sync=false max-buffers=1 drop=true emit-signals=true";
    // RCLCPP_INFO(rclcpp::get_logger("ImageHandler"), "GStreamer 파이프라인 설정: %s", pipeline_desc.c_str());
    GError *error = nullptr;
    pipeline_ = gst_parse_launch(pipeline_desc.c_str(), &error);

    if (error) {
      RCLCPP_ERROR(rclcpp::get_logger("ImageHandler"), "GStreamer 파이프라인 설정 실패: %s", error->message);
      g_error_free(error);
      return;
    }
    appsrc_ = gst_bin_get_by_name(GST_BIN(pipeline_), "src");
    appsink_ = gst_bin_get_by_name(GST_BIN(pipeline_), "sink");
    g_signal_connect(appsink_, "new-sample", G_CALLBACK(on_new_sample), this);
    gst_element_set_state(pipeline_, GST_STATE_PLAYING);
    // RCLCPP_INFO(rclcpp::get_logger("ImageHandler"), "GStreamer 파이프라인 설정 완료");
    pipeline_setup_ = true;
    mime_ = "video/h264;width=" + width + ";height=" + height + ";framerate=30;codecs=avc1.42002A";
    send_mime();
    ws_ptr->binary(true);
  }

  static GstFlowReturn on_new_sample(GstElement* sink, ImageHandler* handler) {
    // 새로운 샘플 수신 시 호출되는 콜백
    GstSample* sample = nullptr;
    g_signal_emit_by_name(sink, "pull-sample", &sample);
    if (sample) {
      // 샘플 처리 코드
      GstBuffer* buffer = gst_sample_get_buffer(sample);
      GstMapInfo map;
      if (gst_buffer_map(buffer, &map, GST_MAP_READ)) {
        // 버퍼 데이터 처리
        std::string data(reinterpret_cast<const char*>(map.data), map.size);
        handler->send_message(data);
        gst_buffer_unmap(buffer, &map);
      }
      gst_sample_unref(sample);
    }
    return GST_FLOW_OK;
  }

};

// 플러그인 클래스 등록
PLUGINLIB_EXPORT_CLASS(ImageHandler, cobiz_bridge::MessageHandlerBase)
