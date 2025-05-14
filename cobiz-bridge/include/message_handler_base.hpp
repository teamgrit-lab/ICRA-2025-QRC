#ifndef MESSAGE_HANDLER_BASE_HPP
#define MESSAGE_HANDLER_BASE_HPP

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/serialized_message.hpp>
#include <string>
#include <memory>
#include <pluginlib/class_list_macros.hpp>
#include <rclcpp/serialization.hpp>

#include <boost/beast/core.hpp>
#include <boost/beast/websocket.hpp>
#include <boost/beast/websocket/ssl.hpp>
#include <boost/asio.hpp>
#include <boost/asio/ssl.hpp>
#include <boost/asio/strand.hpp>
#include <boost/asio/ip/tcp.hpp>

using boost::asio::ip::tcp;
namespace beast = boost::beast;
namespace ssl = boost::asio::ssl;
namespace websocket = beast::websocket;

namespace cobiz_bridge {

class MessageHandlerBase {
public:
  // virtual MessageHandlerBase() = default;
  virtual ~MessageHandlerBase() = default;
  
  // 메시지를 처리하는 메서드
  virtual void handle(std::shared_ptr<rclcpp::SerializedMessage> msg) = 0;
  
  // 핸들러가 처리할 수 있는 메시지 타입을 반환
  virtual std::string getMessageType() const = 0;

  virtual void init(const std::string& host, const std::string& port, const std::string& endpoint, const std::string& topic) {
    // 초기화 코드 (예: WebSocket 연결 설정)
    RCLCPP_INFO(rclcpp::get_logger("MessageHandlerBase"), "Initializing WebSocket connection...");

    host_ = host;
    port_ = port;
    endpoint_ = endpoint;
    topic_ = topic;
    std::string wss = "wss://";
    if (host_.rfind(wss, 0) == 0) {
      host_ = host_.substr(wss.length());
    }
    tcp::resolver resolver(boost::asio::make_strand(ioc_));
    auto result = resolver.resolve(host_, port_);

    ssl::context ctx(ssl::context::tls_client);
    ws_ptr = std::make_shared<websocket::stream<ssl::stream<tcp::socket>>>(boost::asio::make_strand(ioc_), ctx);

    boost::asio::connect(ws_ptr->next_layer().next_layer(), result.begin(), result.end());
    ws_ptr->next_layer().handshake(ssl::stream_base::client);
    ws_ptr->handshake(host_, endpoint_);
    // RCLCPP_INFO(rclcpp::get_logger("ImuHandler"), "WebSocket 연결 완료");
    plugin_init();
  }

  virtual void init(const std::string& host, const std::string& port, const std::string& endpoint, const std::string& topic, std::shared_ptr<rclcpp::GenericPublisher> publisher) {
    // 초기화 코드 (예: WebSocket 연결 설정)
    RCLCPP_INFO(rclcpp::get_logger("MessageHandlerBase"), "Initializing WebSocket connection...");

    host_ = host;
    port_ = port;
    endpoint_ = endpoint;
    topic_ = topic;
    publisher_ = publisher;
    std::string wss = "wss://";
    if (host_.rfind(wss, 0) == 0) {
      host_ = host_.substr(wss.length());
    }
    tcp::resolver resolver(boost::asio::make_strand(ioc_));
    auto result = resolver.resolve(host_, port_);

    ssl::context ctx(ssl::context::tls_client);
    ws_ptr = std::make_shared<websocket::stream<ssl::stream<tcp::socket>>>(boost::asio::make_strand(ioc_), ctx);

    boost::asio::connect(ws_ptr->next_layer().next_layer(), result.begin(), result.end());
    ws_ptr->next_layer().handshake(ssl::stream_base::client);
    ws_ptr->handshake(host_, endpoint_);
    // RCLCPP_INFO(rclcpp::get_logger("ImuHandler"), "WebSocket 연결 완료");
    plugin_init();
  }

  boost::asio::io_context& get_io_context() {
    return ioc_;
  }

  // Connection info setter
  void setConnectionInfo(const std::string& host, const std::string& port, const std::string& endpoint) {
    host_ = host;
    port_ = port;
    endpoint_ = endpoint;
  }

  // 접근자 추가
  std::string getHost() const { return host_; }
  std::string getPort() const { return port_; }
  std::string getEndpoint() const { return endpoint_; }

protected:
  std::shared_ptr<websocket::stream<ssl::stream<tcp::socket>>> ws_ptr;
  std::string host_;
  std::string port_;
  std::string endpoint_;
  std::string mime_;
  std::string topic_;
  boost::asio::io_context ioc_;
  std::thread io_thread_;
  std::shared_ptr<rclcpp::GenericPublisher> publisher_;
  beast::flat_buffer buffer_;
  std::shared_ptr<boost::asio::steady_timer> read_timer_;
  std::shared_ptr<boost::asio::steady_timer> write_timer_;

  
  virtual void plugin_init() = 0;

  virtual void send_message(const std::string& message) {
    try{
      ws_ptr->write(boost::asio::buffer(message));
      // RCLCPP_INFO(rclcpp::get_logger("MessageHandlerBase"), "WebSocket 메시지 전송 완료: %ld", message.size());
    } catch (const std::exception& e) {
      // RCLCPP_ERROR(logger_, "WebSocket 메시지 전송 실패: %s", e.what());
    }
  }

  void send_mime() {
    try {
      ws_ptr->binary(false);
      ws_ptr->write(boost::asio::buffer(mime_));
      ws_ptr->binary(true);
    } catch (const std::exception& e) {
      RCLCPP_ERROR(rclcpp::get_logger("MessageHandlerBase"), "WebSocket MIME 전송 실패: %s", e.what());
    }
  }
};

} // namespace cobiz_bridge

#endif // MESSAGE_HANDLER_BASE_HPP
