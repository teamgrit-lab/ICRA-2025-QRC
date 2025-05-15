#include <rclcpp/rclcpp.hpp>
#include <rclcpp/serialized_message.hpp>
#include <rclcpp/create_generic_subscription.hpp>
#include <rclcpp/generic_subscription.hpp>
#include <rclcpp/create_generic_publisher.hpp>
#include <rclcpp/generic_publisher.hpp>
#include <rclcpp/executors/multi_threaded_executor.hpp>
#include <yaml-cpp/yaml.h>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <pluginlib/class_loader.hpp>
#include <iostream>
#include <vector>
#include <string>
#include <fstream>
#include <thread>
#include "message_handler_base.hpp"
#include <nlohmann/json.hpp>
#include <curl/curl.h>

using json = nlohmann::json;

// Helper function to replace all '/' with '_'
std::string replace_slashes(const std::string &input) {
  std::string result = input;
  std::replace(result.begin(), result.end(), '/', '_');
  return result;
}

// config.yaml에서 토픽 이름 리스트 로드 - subscribers와 publishers 구분
struct TopicLists {
  std::vector<std::string> subscribers;
  std::vector<std::string> publishers;
};

TopicLists load_topics(const std::string & filename) {
  TopicLists topics;
  try {
    YAML::Node config = YAML::LoadFile(filename);
    
    // 구독자 토픽 로드
    if (config["subscribers"]) {
      for (const auto & node : config["subscribers"]) {
        topics.subscribers.push_back(node.as<std::string>());
      }
    }
    
    // 발행자 토픽 로드
    if (config["publishers"]) {
      for (const auto & node : config["publishers"]) {
        topics.publishers.push_back(node.as<std::string>());
      }
    }
    
    if (topics.subscribers.empty() && topics.publishers.empty()) {
      // 이전 버전 호환성 유지 (단일 'topics' 섹션)
      if (config["topics"]) {
        for (const auto & node : config["topics"]) {
          topics.subscribers.push_back(node.as<std::string>());
        }
      } else {
        throw std::runtime_error("subscribers/publishers 섹션이 없습니다");
      }
    }
  } catch (const std::exception& e) {
    std::cerr << "YAML 파일 로드 에러: " << e.what() << std::endl;
  }
  return topics;
}

// 토픽 구독용 노드 클래스 정의
class TopicSubscriberNode : public rclcpp::Node {
public:
  explicit TopicSubscriberNode(
    const std::string &topic_name, 
    const std::string &type_str,
    std::shared_ptr<cobiz_bridge::MessageHandlerBase> handler)
  : Node("topic_subscriber" + replace_slashes(topic_name)), // '/'를 제거하여 노드 이름 생성
    handler_(handler)
  {
    RCLCPP_INFO(this->get_logger(), "토픽 %s (타입: %s) 구독 시작", 
               topic_name.c_str(), type_str.c_str());
    
    // 구독 생성 - 핸들러로 전달
    subscription_ = this->create_generic_subscription(
      topic_name,
      type_str,
      10,
      [this, topic_name](std::shared_ptr<rclcpp::SerializedMessage> msg) {
        // 핸들러가 메시지를 처리
        if (handler_) {
          handler_->handle(msg);
        } else {
          RCLCPP_WARN(this->get_logger(), "토픽 %s: 핸들러가 없습니다", topic_name.c_str());
        }
      }
    );
  }

private:
  std::shared_ptr<rclcpp::GenericSubscription> subscription_;
  std::shared_ptr<cobiz_bridge::MessageHandlerBase> handler_;
};

// 토픽 발행용 노드 클래스 정의 - 수정
class TopicPublisherNode : public rclcpp::Node {
public:
  explicit TopicPublisherNode(
    const std::string &topic_name, 
    const std::string &type_str,
    std::shared_ptr<cobiz_bridge::MessageHandlerBase> handler)
  : Node("topic_publisher" + replace_slashes(topic_name)), // '/'를 제거하여 노드 이름 생성
    handler_(handler)
  {
    RCLCPP_INFO(this->get_logger(), "토픽 %s (타입: %s) 발행 시작", 
               topic_name.c_str(), type_str.c_str());
    
    // ROS 발행자 생성
    publisher_ = this->create_generic_publisher(
      topic_name,
      type_str,
      10
    );
    
    // 핸들러에 발행자 전달 - 웹소켓에서 데이터를 받아서 ROS로 발행
    if (handler_) {
      handler_->init(handler_->getHost(), handler_->getPort(), handler_->getEndpoint(), topic_name, publisher_);
    }
  }
private:
  std::shared_ptr<rclcpp::GenericPublisher> publisher_;
  std::shared_ptr<cobiz_bridge::MessageHandlerBase> handler_;
};

class WebscoketHealthCheck {
public:
  WebscoketHealthCheck(const std::string& host, const std::string& port, const std::string& endpoint, const std::string& token)
    : host_(host), port_(port), endpoint_(endpoint), token_(token){
      host_ = host;
      port_ = port;
      endpoint_ = endpoint;
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
      send_connect();
      ws_ptr->binary(true);
      read_timer_ = std::make_shared<boost::asio::steady_timer>(ioc_, std::chrono::milliseconds(10));

      io_thread_ = std::thread([this]() {
        try {
          ioc_.run();
        } catch (const std::exception& e) {
          std::cerr << "Error in WebSocket thread: " << e.what() << std::endl;
        }
      });
    }

  void send_connect() {
    try {
      json j;
      j["type"] = "CONNECT";
      j["token"] = token_;
      std::string message = j.dump();
      ws_ptr->async_write(boost::asio::buffer(message), 
        [this](beast::error_code ec, std::size_t bytes_transferred) {
          if (ec) {
            std::cerr << "Error sending message: " << ec.message() << std::endl;
          } else {
            std::cout << "sent connect message: " << bytes_transferred << " bytes" << std::endl;
          }
        });
    } catch (const std::exception& e) {
    }
  }

  void async_read_message() {
    read_timer_->expires_after(std::chrono::milliseconds(10));
    read_timer_->async_wait([this](beast::error_code ec) {
      on_read_timeout(ec);
    });
  }

  void on_read_timeout(beast::error_code ec) {
    if (ec) {
      std::cerr << "Read timeout error: " << ec.message() << std::endl;
      return;
    }
    ws_ptr->async_read(buffer_,
      [this](beast::error_code ec, std::size_t bytes_transferred) {
        on_read(ec, bytes_transferred);
      });
  }

  void on_read(beast::error_code ec, std::size_t bytes_transferred) {
    if (ec) {
      std::cerr << "Read error: " << ec.message() << std::endl;
      return;
    }
    std::string message = beast::buffers_to_string(buffer_.data());
    buffer_.consume(buffer_.size());
    // 메시지 처리
    try {
      json j = json::parse(message);
      if (j["type"] == "HEALTHCHECK") {
        json response;
        response["type"] = "HEARTBEAT";
        response["token"] = token_;
        std::string response_message = response.dump();
        ws_ptr->async_write(boost::asio::buffer(response_message), 
          [this](beast::error_code ec, std::size_t bytes_transferred) {
            if (ec) {
              std::cerr << "Error sending heartbeat: " << ec.message() << std::endl;
            } else {
              std::cout << "sent heartbeat message: " << bytes_transferred << " bytes" << std::endl;
            }
          });
      } else if (j["type"] == "TASK") {
        if (j["token"] == token_) {
          id_ = j["task"]["id"];
          std::string name = j["task"]["name"];
          // std::vector<float> payload = j["task"]["payload"];
        /////////////////////////////////////////////////////// 테스크 이름에 따른 작업 연동 ////////////////////////////////////////////////////////////
        }
      }
    } catch (const std::exception& e) {
      std::cerr << "Error parsing JSON: " << e.what() << std::endl;
    }
    async_read_message();
  }
private:
  int id_;
  std::string host_;
  std::string port_;
  std::string endpoint_;
  std::string token_;
  boost::asio::io_context ioc_;
  std::shared_ptr<websocket::stream<ssl::stream<tcp::socket>>> ws_ptr;
  std::thread io_thread_;
  beast::flat_buffer buffer_;
  std::shared_ptr<boost::asio::steady_timer> read_timer_;
};

static size_t write_callback(void *contents, size_t size, size_t nmemb, std::string *output) {
	size_t total_size = size * nmemb;
	output->append((char *)contents, total_size);
	return total_size;
}

void load_url(std::vector<std::string> topic_names, std::vector<std::tuple<std::string, std::string, std::string, std::string>> *topics, json requested_data_) {
  std::string package_share_dir = ament_index_cpp::get_package_share_directory("cobiz-bridge");
  for (const auto& topic_name : topic_names){
    std::string host_;
    std::string port_;
    std::string endpoint_;

    for (const auto& module : requested_data_["modules"]){
      if (module["name"].get<std::string>() == topic_name){
        host_ = module["connection"]["host"];
        port_ = module["connection"]["port"];
        endpoint_ = module["connection"]["endpoint"];
        break;
      }
    }

    topics->emplace_back(topic_name, host_, port_, endpoint_);
    
    std::cout << "-----------------" << std::endl;
    std::cout << "topic_name: " << topic_name << std::endl;
    std::cout << "host: " << host_ << std::endl;
    std::cout << "port: " << port_ << std::endl;
    std::cout << "endpoint: " << endpoint_ << std::endl;
    std::cout << "-----------------" << std::endl;

  }
}

void request_url_to_server(std::vector<std::string> topic_names, std::vector<std::tuple<std::string, std::string, std::string, std::string>> *topics) {
  try {
    std::string package_share_dir = ament_index_cpp::get_package_share_directory("cobiz-bridge");
    YAML::Node request_yaml = YAML::LoadFile(package_share_dir + "/config/request.yaml");
    if (request_yaml["state"].as<std::string>() == "NotRegistered"){
      CURL *curl;
      CURLcode res;
      std::string response;

      json json_data;
      json_data["preset"] = request_yaml["preset"].as<std::string>();
			json_data["secret_key"] = request_yaml["secret_key"].as<std::string>();

			std::string jsonData = json_data.dump(2);
//			std::cout << "jsonData: " << jsonData << std::endl;
			std::string server_address = request_yaml["server_address"].as<std::string>();

			curl_global_init(CURL_GLOBAL_ALL);
			curl = curl_easy_init();

//      std::cout << "server_address: " << server_address << std::endl;

			if (curl) {
				struct curl_slist *headers = NULL;
				headers = curl_slist_append(headers, "Content-Type: application/json"); // JSON 헤더 추가
				curl_easy_setopt(curl, CURLOPT_URL, server_address.c_str());
				curl_easy_setopt(curl, CURLOPT_HTTPHEADER, headers);
				curl_easy_setopt(curl, CURLOPT_POST, 1L);
				curl_easy_setopt(curl, CURLOPT_POSTFIELDS, jsonData.c_str());

				curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, write_callback);
				curl_easy_setopt(curl, CURLOPT_WRITEDATA, &response);

				res = curl_easy_perform(curl);
				if (res != CURLE_OK) {
					std::cerr << "curl_easy_perform() failed: " << curl_easy_strerror(res) << std::endl;
				}

				curl_slist_free_all(headers);
				curl_easy_cleanup(curl);

			}


			curl_global_cleanup();
//			std::cout << "response: \n" << response << std::endl;
			json jsonResponse = json::parse(response);
			std::string id = std::to_string(jsonResponse["id"].get<int>());
			std::string token = jsonResponse["token"];
//			std::cout << "id: " << id << std::endl;

			request_yaml["id"] = id;
			request_yaml["state"] = "Registered";
			request_yaml["token"] = token;
			YAML::Emitter out;
			out << request_yaml;
			std::ofstream fout(package_share_dir + "/config/request.yaml");
			if (!fout) {
				std::cout << "Failed to open file" << std::endl;;
				return;
			}
			fout << out.c_str();
			fout.close();
			request_url_to_server(topic_names, topics);
      return;

    }
    else if (request_yaml["state"].as<std::string>() == "Registered"){
      CURL *curl;
      CURLcode res;
      std::string response;
      std::string server_address = request_yaml["server_address"].as<std::string>();
      std::string address_id = server_address + "/" + request_yaml["id"].as<std::string>();
      std::string authorization_header = "Authorization: Device " + request_yaml["token"].as<std::string>();

			curl_global_init(CURL_GLOBAL_ALL);
			curl = curl_easy_init();

			if (curl) {
				struct curl_slist *headers = NULL;
				headers = curl_slist_append(headers, authorization_header.c_str());
				curl_easy_setopt(curl, CURLOPT_URL, address_id.c_str());
				curl_easy_setopt(curl, CURLOPT_HTTPHEADER, headers);
				curl_easy_setopt(curl, CURLOPT_HTTPGET, 1L);

				curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, write_callback);
				curl_easy_setopt(curl, CURLOPT_WRITEDATA, &response);

				res = curl_easy_perform(curl);
				if (res != CURLE_OK) {
					std::cerr << "curl_easy_perform() failed: " << curl_easy_strerror(res) << std::endl;

					std::cout << "reset yaml" << std::endl;
					request_url_to_server(topic_names, topics);
					return;
				}

				curl_slist_free_all(headers);
				curl_easy_cleanup(curl);

			}

			curl_global_cleanup();
			json requested_data_ = json::parse(response);
//			std::cout << "requested_data: " << requested_data_ << std::endl;
			if (requested_data_.contains("message")) {
				if (requested_data_["message"] == "Unregistered token.") {
					request_yaml["id"] = "null";
					request_yaml["state"] = "NotRegistered";
					request_yaml["token"] = "null";
					YAML::Emitter out;
					out << request_yaml;
					std::ofstream fout(package_share_dir + "/config/request.yaml");
					if (!fout) {
						std::cout << "Failed to open file" << std::endl;;
						return;
					}
					fout << out.c_str();
					fout.close();
          request_url_to_server(topic_names, topics);
					return;
				}
			}
      load_url(topic_names, topics, requested_data_);

    }

  }
  catch (const std::exception& e) {
    std::cerr << "YAML 파일 로드 에러: " << e.what() << std::endl;
  }
}

int main(int argc, char ** argv) {

  
  rclcpp::init(argc, argv);
  
  // 메인 노드 생성 (설정 파일 로드 및 토픽 검색용)
  auto main_node = rclcpp::Node::make_shared("topic_size_printer_main");

  try {
    // 플러그인 로더 생성 - 네임스페이스 수정
    pluginlib::ClassLoader<cobiz_bridge::MessageHandlerBase> loader(
      "cobiz-bridge",
      "cobiz_bridge::MessageHandlerBase"
    );
//    RCLCPP_INFO(main_node->get_logger(), "플러그인 로더 생성됨");
    
    // 패키지 경로에서 config 파일 찾기
    std::string pkg_share_dir;
    try {
      pkg_share_dir = ament_index_cpp::get_package_share_directory("cobiz-bridge");
    } catch (const std::exception& e) {
      RCLCPP_ERROR(main_node->get_logger(), "패키지 경로를 찾을 수 없습니다: %s", e.what());
      return 1;
    }
    
    // config 파일 경로 설정
    std::string config_path = pkg_share_dir + "/config/config.yaml";
//    RCLCPP_INFO(main_node->get_logger(), "설정 파일 경로: %s", config_path.c_str());
    
    // 토픽 이름 로드 - 구독자와 발행자 모두 로드
    auto topics = load_topics(config_path);
    
    if (topics.subscribers.empty() && topics.publishers.empty()) {
      RCLCPP_ERROR(main_node->get_logger(), "토픽 이름을 불러오지 못했습니다.");
      return 1;
    }
    
    RCLCPP_INFO(main_node->get_logger(), "총 %zu개의 구독 토픽과 %zu개의 발행 토픽을 찾았습니다.", 
               topics.subscribers.size(), topics.publishers.size());
    
    // 토픽 디스커버리를 위한 준비 시간
    rclcpp::sleep_for(std::chrono::seconds(1));
    rclcpp::spin_some(main_node);

    // 토픽 메타데이터 조회
    auto topic_map = main_node->get_topic_names_and_types();
    
    // 멀티스레드 실행자 생성
    // CPU 코어 수나 토픽 수를 기반으로 스레드 수 결정
    unsigned int num_threads = std::min(
      static_cast<unsigned int>(topics.subscribers.size() + topics.publishers.size()),
      std::max(std::thread::hardware_concurrency(), 2u)
    );
    
//    RCLCPP_INFO(main_node->get_logger(), "멀티스레드 실행자 생성 (스레드 %u개)", num_threads);
    rclcpp::executors::MultiThreadedExecutor executor(
      rclcpp::ExecutorOptions(), num_threads
    );
    
    // 모든 토픽에 대한 URL 정보 요청
    std::vector<std::string> all_topics;
    all_topics.insert(all_topics.end(), topics.subscribers.begin(), topics.subscribers.end());
    all_topics.insert(all_topics.end(), topics.publishers.begin(), topics.publishers.end());
    std::vector<std::tuple<std::string, std::string, std::string, std::string>> topic_urls;
    request_url_to_server(all_topics, &topic_urls);

    // 플러그인 타입 맵핑을 위한 사전 처리
    std::unordered_map<std::string, std::string> plugin_type_map;
    for (const auto& plugin_name : loader.getDeclaredClasses()) {
      try {
        auto plugin = loader.createSharedInstance(plugin_name);
        std::string message_type = plugin->getMessageType();
        plugin_type_map[message_type] = plugin_name;
      } catch (const pluginlib::PluginlibException& ex) {
        RCLCPP_ERROR(main_node->get_logger(), "플러그인 '%s' 로드 실패: %s", plugin_name.c_str(), ex.what());
      }
    }

    // 각 구독 토픽에 대한 구독 노드 생성
    std::vector<std::shared_ptr<TopicSubscriberNode>> subscriber_nodes;
    for (const auto & topic : topics.subscribers) {
      // 토픽 타입 확인
      auto it = topic_map.find(topic);
      if (it == topic_map.end() || it->second.empty()) {
        RCLCPP_WARN(main_node->get_logger(), "토픽 %s 정보를 찾을 수 없습니다.", topic.c_str());
        continue;
      }
      
      std::string type_str = it->second.front();
      
      // 토픽 타입에 맞는 핸들러 찾기
      std::shared_ptr<cobiz_bridge::MessageHandlerBase> handler;      
      auto plugin_it = plugin_type_map.find(type_str);
      if (plugin_it != plugin_type_map.end()) {
        std::string plugin_name = plugin_it->second;
//        RCLCPP_INFO(main_node->get_logger(), "플러그인 이름: %s", plugin_name.c_str());
        
        try {
          handler = loader.createSharedInstance(plugin_name);
          
          // 연결 정보 설정
          for (const auto& topic_info : topic_urls) {
            if (std::get<0>(topic_info) == topic) {
              handler->init(std::get<1>(topic_info), std::get<2>(topic_info), std::get<3>(topic_info), topic);
              break;
            }
          }
          
          RCLCPP_INFO(main_node->get_logger(), "토픽 %s에 '%s' 핸들러 사용 (구독자)", 
                     topic.c_str(), plugin_name.c_str());
        } catch (const pluginlib::PluginlibException& ex) {
          RCLCPP_ERROR(main_node->get_logger(), "플러그인 '%s' 생성 실패: %s", plugin_name.c_str(), ex.what());
        }
      }
      
      if (!handler) {
        RCLCPP_WARN(main_node->get_logger(), "토픽 %s(타입: %s)에 대한 핸들러 없음", 
                   topic.c_str(), type_str.c_str());
        continue;
      }
      
      // 구독 노드 생성 및 실행자에 추가
      auto subscriber_node = std::make_shared<TopicSubscriberNode>(topic, type_str, handler);
      subscriber_nodes.push_back(subscriber_node);
      executor.add_node(subscriber_node);
    }

    // 각 발행 토픽에 대한 발행 노드 생성
    std::vector<std::shared_ptr<TopicPublisherNode>> publisher_nodes;
    for (const auto & topic : topics.publishers) {
      // 토픽 타입 확인
      auto it = topic_map.find(topic);
      if (it == topic_map.end() || it->second.empty()) {
        RCLCPP_WARN(main_node->get_logger(), "토픽 %s 정보를 찾을 수 없습니다.", topic.c_str());
        continue;
      }
      
      std::string type_str = it->second.front();
      
      // 토픽 타입에 맞는 핸들러 찾기
      std::shared_ptr<cobiz_bridge::MessageHandlerBase> handler;      
      auto plugin_it = plugin_type_map.find(type_str);
      if (plugin_it != plugin_type_map.end()) {
        std::string plugin_name = plugin_it->second;
//        RCLCPP_INFO(main_node->get_logger(), "플러그인 이름: %s", plugin_name.c_str());
        
        try {
          handler = loader.createSharedInstance(plugin_name);
          
          // 연결 정보만 설정 (init은 TopicPublisherNode에서 완료)
          for (const auto& topic_info : topic_urls) {
            if (std::get<0>(topic_info) == topic) {
              handler->setConnectionInfo(std::get<1>(topic_info), std::get<2>(topic_info), std::get<3>(topic_info));
              break;
            }
          }
          
          RCLCPP_INFO(main_node->get_logger(), "토픽 %s에 '%s' 핸들러 사용 (발행자)", 
                     topic.c_str(), plugin_name.c_str());
        } catch (const pluginlib::PluginlibException& ex) {
          RCLCPP_ERROR(main_node->get_logger(), "플러그인 '%s' 생성 실패: %s", plugin_name.c_str(), ex.what());
        }
      }
      
      if (!handler) {
        RCLCPP_WARN(main_node->get_logger(), "토픽 %s(타입: %s)에 대한 핸들러 없음", 
                   topic.c_str(), type_str.c_str());
        continue;
      }
      
      // 발행 노드 생성 및 실행자에 추가
      auto publisher_node = std::make_shared<TopicPublisherNode>(topic, type_str, handler);
      publisher_nodes.push_back(publisher_node);
      executor.add_node(publisher_node);
    }
    
    // 메인 노드도 실행자에 추가
    executor.add_node(main_node);
    
    RCLCPP_INFO(main_node->get_logger(), "모든 토픽 구독 및 발행 설정 완료. 멀티스레드 실행 시작 (%zu 구독 토픽, %zu 발행 토픽)", 
               subscriber_nodes.size(), publisher_nodes.size());
    
    // 멀티스레드 실행자 시작
    executor.spin();
    
  } catch (const std::exception& e) {
    RCLCPP_ERROR(main_node->get_logger(), "예외 발생: %s", e.what());
    return 1;
  }
  
  rclcpp::shutdown();
  return 0;
}
