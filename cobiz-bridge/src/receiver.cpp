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

// 연결 정보 구조체
struct WebSocketEndpoint {
  std::string host;
  std::string port;
  std::string endpoint;
};

// YAML 파일에서 엔드포인트 정보 로드
std::unordered_map<std::string, WebSocketEndpoint> loadEndpointsFromYaml(const std::string& yaml_path) {
  std::unordered_map<std::string, WebSocketEndpoint> endpoints;
  
  try {
    YAML::Node config = YAML::LoadFile(yaml_path);
    if (!config["endpoints"]) {
      throw std::runtime_error("YAML 파일에 endpoints 섹션이 없습니다");
    }
    
    auto endpoints_section = config["endpoints"];
    for (const auto& endpoint : endpoints_section) {
      std::string topic = endpoint.first.as<std::string>();
      WebSocketEndpoint ws_endpoint;
      ws_endpoint.host = endpoint.second["host"].as<std::string>();
      ws_endpoint.port = endpoint.second["port"].as<std::string>();
      ws_endpoint.endpoint = endpoint.second["endpoint"].as<std::string>();
      
      endpoints[topic] = ws_endpoint;
    }
    
    RCLCPP_INFO(rclcpp::get_logger("receiver"), "YAML 파일에서 %zu개의 엔드포인트 설정을 로드했습니다", endpoints.size());
  } catch (const std::exception& e) {
    RCLCPP_ERROR(rclcpp::get_logger("receiver"), "엔드포인트 설정 로드 실패: %s", e.what());
  }
  
  return endpoints;
}

// config.yaml에서 토픽 이름 리스트 로드 - 역할 반전
struct TopicLists {
  std::vector<std::string> subscribers;  // main.cpp의 publishers
  std::vector<std::string> publishers;   // main.cpp의 subscribers
};

TopicLists load_reversed_topics(const std::string & filename) {
  TopicLists topics;
  try {
    YAML::Node config = YAML::LoadFile(filename);
    
    // 원래의 구독자 토픽을 발행자로 설정
    if (config["subscribers"]) {
      for (const auto & node : config["subscribers"]) {
        topics.publishers.push_back(node.as<std::string>());
      }
    }
    
    // 원래의 발행자 토픽을 구독자로 설정
    if (config["publishers"]) {
      for (const auto & node : config["publishers"]) {
        topics.subscribers.push_back(node.as<std::string>());
      }
    }
    
    // 단일 'topics' 섹션 지원
    if (topics.subscribers.empty() && topics.publishers.empty()) {
      if (config["topics"]) {
        for (const auto & node : config["topics"]) {
          topics.publishers.push_back(node.as<std::string>());
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

// 토픽 타입 로더 함수 추가
std::unordered_map<std::string, std::string> loadTopicTypes(const std::string& yaml_path) {
  std::unordered_map<std::string, std::string> topic_types;
  
  try {
    YAML::Node config = YAML::LoadFile(yaml_path);
    if (!config["topic_types"]) {
      throw std::runtime_error("YAML 파일에 topic_types 섹션이 없습니다");
    }
    
    auto types_section = config["topic_types"];
    for (const auto& type_entry : types_section) {
      std::string topic = type_entry.first.as<std::string>();
      std::string type = type_entry.second.as<std::string>();
      topic_types[topic] = type;
    }
    
    RCLCPP_INFO(rclcpp::get_logger("receiver"), "YAML 파일에서 %zu개의 토픽 타입을 로드했습니다", topic_types.size());
  } catch (const std::exception& e) {
    RCLCPP_ERROR(rclcpp::get_logger("receiver"), "토픽 타입 로드 실패: %s", e.what());
    
    RCLCPP_WARN(rclcpp::get_logger("receiver"), "기본 토픽 타입을 사용합니다");
  }
  
  return topic_types;
}

// 토픽 구독용 노드 클래스 정의
class ReceiverSubscriberNode : public rclcpp::Node {
public:
  explicit ReceiverSubscriberNode(
    const std::string &topic_name, 
    const std::string &type_str,
    std::shared_ptr<cobiz_bridge::MessageHandlerBase> handler)
  : Node("receiver_subscriber_" + topic_name.substr(1)), // '/'를 제거하여 노드 이름 생성
    handler_(handler)
  {
    RCLCPP_INFO(this->get_logger(), "토픽 %s (타입: %s) 구독 시작 (Receiver)", 
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

// 토픽 발행용 노드 클래스 정의
class ReceiverPublisherNode : public rclcpp::Node {
public:
  explicit ReceiverPublisherNode(
    const std::string &topic_name, 
    const std::string &type_str,
    std::shared_ptr<cobiz_bridge::MessageHandlerBase> handler)
  : Node("receiver_publisher_" + topic_name.substr(1)), 
    handler_(handler)
  {
    RCLCPP_INFO(this->get_logger(), "토픽 %s (타입: %s) 발행 시작 (Receiver)", 
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

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  
  auto main_node = rclcpp::Node::make_shared("receiver_main");

  try {
    // 수신 측 플러그인 로더 생성 
    pluginlib::ClassLoader<cobiz_bridge::MessageHandlerBase> loader(
      "cobiz-bridge",
      "cobiz_bridge::ReceiverMessageHandlerBase"
    );
    
    RCLCPP_INFO(main_node->get_logger(), "수신 측 플러그인 로더 생성됨");
    
    // 패키지 경로 찾기
    std::string pkg_share_dir = ament_index_cpp::get_package_share_directory("cobiz-bridge");
    
    // config 파일 경로 설정
    std::string config_path = pkg_share_dir + "/config/config.yaml";
    std::string receiver_config_path = pkg_share_dir + "/config/receiver.yaml";
    std::string topic_types_path = pkg_share_dir + "/config/topic_types.yaml";
    RCLCPP_INFO(main_node->get_logger(), "설정 파일 경로: %s", config_path.c_str());
    RCLCPP_INFO(main_node->get_logger(), "수신기 설정 파일 경로: %s", receiver_config_path.c_str());
    RCLCPP_INFO(main_node->get_logger(), "토픽 타입 설정 파일 경로: %s", topic_types_path.c_str());
    
    // 토픽 이름 로드 - 역할 반전
    auto topics = load_reversed_topics(config_path);
    
    // 토픽 타입 로드
    auto topic_type_map = loadTopicTypes(topic_types_path);
    
    if (topics.subscribers.empty() && topics.publishers.empty()) {
      RCLCPP_ERROR(main_node->get_logger(), "토픽 이름을 불러오지 못했습니다.");
      return 1;
    }
    
    RCLCPP_INFO(main_node->get_logger(), "수신 측: %zu개의 구독 토픽과 %zu개의 발행 토픽을 변환했습니다.", 
               topics.subscribers.size(), topics.publishers.size());
    
    // 엔드포인트 설정 로드
    auto endpoints = loadEndpointsFromYaml(receiver_config_path);
    
    // 토픽 디스커버리를 위한 준비 시간
    rclcpp::sleep_for(std::chrono::seconds(1));
    
    // 토픽 메타데이터 조회
    auto topic_map = main_node->get_topic_names_and_types();
    
    // 멀티스레드 실행자 생성
    unsigned int num_threads = std::max(std::thread::hardware_concurrency(), 2u);
    rclcpp::executors::MultiThreadedExecutor executor(
      rclcpp::ExecutorOptions(), num_threads
    );
    
    // 플러그인 타입 맵핑
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

    // 각 구독 토픽에 대한 구독 노드 생성 (/cmd_vel)
    std::vector<std::shared_ptr<ReceiverSubscriberNode>> subscriber_nodes;
    for (const auto & topic : topics.subscribers) {
      // 토픽 타입 확인 - YAML에서 불러온 토픽 타입을 먼저 사용
      std::string type_str;
      auto yaml_type_it = topic_type_map.find(topic);
      
      if (yaml_type_it != topic_type_map.end()) {
        // YAML에 정의된 타입 사용
        type_str = yaml_type_it->second;
        RCLCPP_INFO(main_node->get_logger(), "토픽 %s: YAML에서 타입 로드 - %s", 
                   topic.c_str(), type_str.c_str());
      } else {
        // YAML에 없는 경우 토픽 맵에서 검색
        auto it = topic_map.find(topic);
        if (it != topic_map.end() && !it->second.empty()) {
          type_str = it->second.front();
          RCLCPP_INFO(main_node->get_logger(), "토픽 %s: ROS에서 타입 찾음 - %s", 
                     topic.c_str(), type_str.c_str());
        } else {
          RCLCPP_WARN(main_node->get_logger(), "토픽 %s 정보를 찾을 수 없습니다.", topic.c_str());
          continue;
        }
      }
      
      // 토픽 타입에 맞는 핸들러 찾기
      std::shared_ptr<cobiz_bridge::MessageHandlerBase> handler;      
      auto plugin_it = plugin_type_map.find(type_str);
      if (plugin_it != plugin_type_map.end()) {
        std::string plugin_name = plugin_it->second;
        
        try {
          handler = loader.createSharedInstance(plugin_name);
          
          // YAML에서 로드한 연결 정보 설정
          auto endpoint_it = endpoints.find(topic);
          if (endpoint_it != endpoints.end()) {
            handler->setConnectionInfo(
              endpoint_it->second.host,
              endpoint_it->second.port,
              endpoint_it->second.endpoint
            );
            
            handler->init(
              endpoint_it->second.host,
              endpoint_it->second.port,
              endpoint_it->second.endpoint,
              topic
            );
            
            RCLCPP_INFO(main_node->get_logger(), "토픽 %s: 수신 연결 설정 (host=%s, port=%s, endpoint=%s)",
                       topic.c_str(), endpoint_it->second.host.c_str(), 
                       endpoint_it->second.port.c_str(), endpoint_it->second.endpoint.c_str());
          } else {
            RCLCPP_ERROR(main_node->get_logger(), "토픽 %s의 연결 정보가 없습니다.", topic.c_str());
            continue;
          }
          
          RCLCPP_INFO(main_node->get_logger(), "토픽 %s에 '%s' 핸들러 사용 (수신 구독자)", 
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
      auto subscriber_node = std::make_shared<ReceiverSubscriberNode>(topic, type_str, handler);
      subscriber_nodes.push_back(subscriber_node);
      executor.add_node(subscriber_node);
    }

    // 각 발행 토픽에 대한 발행 노드 생성 (/imu, /scan 등)
    std::vector<std::shared_ptr<ReceiverPublisherNode>> publisher_nodes;
    for (const auto & topic : topics.publishers) {
      // 토픽 타입 확인 - YAML에서 불러온 토픽 타입을 먼저 사용
      std::string type_str;
      auto yaml_type_it = topic_type_map.find(topic);
      
      if (yaml_type_it != topic_type_map.end()) {
        // YAML에 정의된 타입 사용
        type_str = yaml_type_it->second;
        RCLCPP_INFO(main_node->get_logger(), "토픽 %s: YAML에서 타입 로드 - %s", 
                   topic.c_str(), type_str.c_str());
      } else {
        // YAML에 없는 경우 토픽 맵에서 검색
        auto it = topic_map.find(topic);
        if (it != topic_map.end() && !it->second.empty()) {
          type_str = it->second.front();
          RCLCPP_INFO(main_node->get_logger(), "토픽 %s: ROS에서 타입 찾음 - %s", 
                     topic.c_str(), type_str.c_str());
        } else {
          RCLCPP_WARN(main_node->get_logger(), "토픽 %s 정보를 찾을 수 없습니다.", topic.c_str());
          continue;
        }
      }
      
      // 토픽 타입에 맞는 핸들러 찾기
      std::shared_ptr<cobiz_bridge::MessageHandlerBase> handler;      
      auto plugin_it = plugin_type_map.find(type_str);
      if (plugin_it != plugin_type_map.end()) {
        std::string plugin_name = plugin_it->second;
        
        try {
          handler = loader.createSharedInstance(plugin_name);
          
          // YAML에서 로드한 연결 정보 설정
          auto endpoint_it = endpoints.find(topic);
          if (endpoint_it != endpoints.end()) {
            handler->setConnectionInfo(
              endpoint_it->second.host,
              endpoint_it->second.port,
              endpoint_it->second.endpoint
            );
            
            RCLCPP_INFO(main_node->get_logger(), "토픽 %s: 수신 연결 설정 (host=%s, port=%s, endpoint=%s)",
                       topic.c_str(), endpoint_it->second.host.c_str(), 
                       endpoint_it->second.port.c_str(), endpoint_it->second.endpoint.c_str());
          } else {
            RCLCPP_ERROR(main_node->get_logger(), "토픽 %s의 연결 정보가 없습니다.", topic.c_str());
            continue;
          }
          
          RCLCPP_INFO(main_node->get_logger(), "토픽 %s에 '%s' 핸들러 사용 (수신 발행자)", 
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
      auto publisher_node = std::make_shared<ReceiverPublisherNode>(topic, type_str, handler);
      publisher_nodes.push_back(publisher_node);
      executor.add_node(publisher_node);
    }
    
    executor.add_node(main_node);
    
    RCLCPP_INFO(main_node->get_logger(), "수신 측 설정 완료. 멀티스레드 실행 시작 (%zu 구독 토픽, %zu 발행 토픽)",
               subscriber_nodes.size(), publisher_nodes.size());
    
    executor.spin();
    
  } catch (const std::exception& e) {
    RCLCPP_ERROR(main_node->get_logger(), "예외 발생: %s", e.what());
    return 1;
  }
  
  rclcpp::shutdown();
  return 0;
}
