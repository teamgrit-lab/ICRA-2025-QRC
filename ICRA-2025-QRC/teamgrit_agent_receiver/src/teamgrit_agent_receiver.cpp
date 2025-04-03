#include <vector>
#include <rclcpp/rclcpp.hpp>
#include <yaml-cpp/yaml.h>
#include <boost/beast/core.hpp>
#include <boost/beast/websocket.hpp>
#include <boost/asio/connect.hpp>
#include <boost/asio/strand.hpp>
#include <boost/asio/ip/tcp.hpp>
#include <boost/asio.hpp>
#include <boost/beast/ssl.hpp>
#include <thread>
#include <teamgrit_agent_msgs/msg/agent_msg.hpp>
#include <teamgrit_agent_msgs/msg/agent_control.hpp>
#include <nlohmann/json.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <iostream>

using boost::asio::ip::tcp;
namespace beast = boost::beast;
namespace ssl = boost::asio::ssl;
namespace websocket = beast::websocket;
using json = nlohmann::json;


class TeamgritAgentReceiver {
public:
	TeamgritAgentReceiver(const std::string& host, const std::string& port, const std::string& endpoint, const std::string& type, const std::string& name)
		: host_(host), port_(port), endpoint_(endpoint), type_(type), name_(name), ioc_(), read_timer_(ioc_), write_timer_(ioc_) {
		try {
			tcp::resolver resolver(boost::asio::make_strand(ioc_));
			auto results = resolver.resolve(host_, port_);

			ssl::context ctx(ssl::context::tls_client);
			ws_ptr = std::make_shared<websocket::stream<ssl::stream<tcp::socket>>>(boost::asio::make_strand(ioc_), ctx);

			boost::asio::connect(ws_ptr->next_layer().next_layer(), results.begin(), results.end());
			ws_ptr->next_layer().handshake(ssl::stream_base::client);
			ws_ptr->handshake(host_, endpoint_);

			ws_ptr->binary(false);
			ws_ptr->write(boost::asio::buffer(mime_));
			ws_ptr->binary(true);

			start_read_timer();

			std::cout << "Connected to WebSocket:" << host_.c_str() << port_.c_str() << endpoint_.c_str() << std::endl;

			thread_ = std::thread([this]() { ioc_.run(); });
		} catch (const std::exception& e) {
			std::cout <<  "WebSocket connection failed" << e.what() << std::endl;
		}
	}

	~TeamgritAgentReceiver() {
		if (thread_.joinable()) {
			ioc_.stop();
			thread_.join();
		}
	}

	void reconnect() {
		try {
			std::this_thread::sleep_for(std::chrono::seconds(1));
			tcp::resolver resolver(boost::asio::make_strand(ioc_));
			auto results = resolver.resolve(host_, port_);
			ssl::context ctx(ssl::context::tls_client);
			ws_ptr = std::make_shared<websocket::stream<ssl::stream<tcp::socket>>>(boost::asio::make_strand(ioc_), ctx);
			boost::asio::connect(ws_ptr->next_layer().next_layer(), results.begin(), results.end());
			ws_ptr->next_layer().handshake(ssl::stream_base::client);
			ws_ptr->handshake(host_, endpoint_);
			ws_ptr->binary(false);
			ws_ptr->write(boost::asio::buffer(mime_));
			ws_ptr->binary(true);
			std::cout << "Reconnected to WebSocket: " << type_ << std::endl;
		} catch (const std::exception& e) {
			std::cout << "Reconnected failed" << std::endl;
		}
	}

	void send_message(const std::vector<uint8_t>& msg) {
		try {
			ws_ptr->write(boost::asio::buffer(msg.data(), msg.size()));
		} catch (const std::exception& e) {
			RCLCPP_ERROR(rclcpp::get_logger("TeamgritAgentSDK"), "Failed to send message: %s", e.what());
			reconnect();
		}
	}

	bool async_read_flag_return() {
		return async_read_flag;
	}

	std::vector<uint8_t> get_received_data() {
		async_flag_change();
		return received_data;
	}

private:
	std::string host_;
	std::string port_;
	std::string endpoint_;
	std::string mime_;
	std::string type_;
	std::string name_;
	boost::asio::io_context ioc_;
	std::shared_ptr<websocket::stream<ssl::stream<tcp::socket>>> ws_ptr;
	std::thread thread_;
	boost::asio::steady_timer read_timer_;
	boost::asio::steady_timer write_timer_;
	boost::beast::flat_buffer buffer_;
	std::vector<uint8_t> received_data;
	bool async_read_flag = false;

	void async_flag_change() {
		async_read_flag = false;
	}

	void start_read_timer() {
		read_timer_.expires_after(std::chrono::milliseconds(10));
		read_timer_.async_wait([this](boost::system::error_code ec) {
			if (!ec) {
				read_data();
			}
		});
	}

	void read_data() {
		ws_ptr->async_read(buffer_,
			[this](boost::system::error_code ec, std::size_t bytes_transferred) {
				if (ec) {
					std::cerr << "Error reading: " << ec.message() << std::endl;
					reconnect();
					start_read_timer();
					return;
				}

				std::string data = beast::buffers_to_string(buffer_.data());
				buffer_.consume(buffer_.size());
				try {
					received_data.assign(data.begin(), data.end());
				} catch (...) {
					std::cout << "read_data error" << std::endl;
				}
				async_read_flag = true;
				start_read_timer();
			});
	}

//	void start_write_timer() {
//		write_timer_.expires_after(std::chrono::seconds(10));
//		write_timer_.async_wait([this](boost::system::error_code ec) {
//			if (!ec) {
//				write_data();
//			}
//		});
//	}
//
//	void write_data() {
//		std::string message = "ping";
//		ws_ptr->async_write(boost::asio::buffer(message),
//		[this](boost::system::error_code ec, std::size_t bytes_transferred) {
//			if (ec) {
//				std::cerr << "Error writing: " << ec.message() << std::endl;
//				reconnect();
//				start_write_timer();
//				return;
//			}
//			std::cout << "Sent: ping" << std::endl;
//			start_write_timer();
//		});
//	}
//
//	void start_write_timer() {
//		write_timer_.expires_after(std::chrono::seconds(10));
//		write_timer_.async_wait([this](boost::system::error_code ec) {
//			if (!ec) {
//				write_data();
//			}
//		});
//	}
//
//	void write_data() {
//		std::string message = "ping";
//		ws_ptr->async_write(boost::asio::buffer(message),
//		[this](boost::system::error_code ec, std::size_t bytes_transferred) {
//			if (ec) {
//				std::cerr << "Error writing: " << ec.message() << std::endl;
//				reconnect();
//				start_write_timer();
//				return;
//			}
//			std::cout << "Sent: ping" << std::endl;
//			start_write_timer();
//		});
//	}

};

class WebSocketBridge : public rclcpp::Node {
public:
	WebSocketBridge(const std::vector<std::tuple<std::string, std::string, std::string, std::string, std::string, std::string>>& topics)
		: Node("teamgrit_agent_receiver") {
		for (const auto& [topic, name, host, port, endpoint, type] : topics) {
			auto client = std::make_shared<TeamgritAgentReceiver>(host, port, endpoint, type, name);
			ws_clients_[topic] = client;

			if (type == "CONTROL") {
				auto sub = this->create_subscription<teamgrit_agent_msgs::msg::AgentControl>(
					topic, 10, [this, topic](const teamgrit_agent_msgs::msg::AgentControl::SharedPtr msg) {
						control_callback(topic, msg);
					}
				);
				control_subscriptions_.push_back(sub);
			} else if (type == "SPEAKER"){
				auto sub = this->create_subscription<teamgrit_agent_msgs::msg::AgentMsg>(
					topic, 10, [this, topic](const teamgrit_agent_msgs::msg::AgentMsg::SharedPtr msg) {
						speaker_callback(topic, msg);
					}
				);
				subscriptions_.push_back(sub);
			} else {
				auto pub = this->create_publisher<teamgrit_agent_msgs::msg::AgentMsg>(topic, 10);
				message_publishers_[topic] = pub;

				auto timer = this->create_wall_timer(
					std::chrono::milliseconds(10),
					[this, topic]() { publish_message(topic); }
				);

				timers_.push_back(timer);
			}
		}
	}

private:
	std::unordered_map<std::string, std::shared_ptr<TeamgritAgentReceiver>> ws_clients_;
	std::vector<rclcpp::Subscription<teamgrit_agent_msgs::msg::AgentMsg>::SharedPtr> subscriptions_;
	std::vector<rclcpp::Subscription<teamgrit_agent_msgs::msg::AgentControl>::SharedPtr> control_subscriptions_;
	std::unordered_map<std::string, rclcpp::Publisher<teamgrit_agent_msgs::msg::AgentMsg>::SharedPtr> message_publishers_;
	std::vector<rclcpp::TimerBase::SharedPtr> timers_;
	std::vector<rclcpp::TimerBase::SharedPtr> pub_timers_;

	void publish_message(const std::string& topic) {
		auto it = ws_clients_.find(topic);
		if (it == ws_clients_.end()) {
			RCLCPP_ERROR(this->get_logger(), "No WebSocket client found for topic: %s", topic.c_str());
			return;
		}
		auto ws_client = it->second;

		try {
			std::vector<uint8_t> received_data;
			if (ws_client->async_read_flag_return()){
				received_data = ws_client->get_received_data();
				auto pub_it = message_publishers_.find(topic);
				if (pub_it != message_publishers_.end()) {
					teamgrit_agent_msgs::msg::AgentMsg msg;
					msg.header.stamp = this->now();
					msg.data = received_data;
					pub_it->second->publish(msg);
				}
			}
		} catch (...) {
			std::cout << "publish_message error" << std::endl;
		}
	}

	void speaker_callback(const std::string& topic, const teamgrit_agent_msgs::msg::AgentMsg::SharedPtr msg) {
		auto it = ws_clients_.find(topic);
		if (it != ws_clients_.end()) {
			it->second->send_message(msg->data);
		} else {
			RCLCPP_ERROR(this->get_logger(), "No WebSocket client found for topic: %s", topic.c_str());
		}
	}

	void control_callback(const std::string& topic, const teamgrit_agent_msgs::msg::AgentControl::SharedPtr msg) {
		auto it = ws_clients_.find(topic);
		if (it != ws_clients_.end()) {
			json j;
			j["channel"][0] = msg->lx;
			j["channel"][1] = msg->ly;
			j["channel"][2] = msg->rx;
			j["channel"][3] = msg->ry;
			for (size_t i=0; i<16; ++i) {
				j["channel"][i+4] = msg->button[i];
			}
			std::string json_str = j.dump();
			std::vector<uint8_t> message(json_str.begin(), json_str.end());
			it->second->send_message(message);
		} else {
			RCLCPP_ERROR(this->get_logger(), "No WebSocket client found for topic: %s", topic.c_str());
		}
	}

};

void get_yaml(rclcpp::executors::SingleThreadedExecutor& executor) {
	try {
		std::string package_share_directory = ament_index_cpp::get_package_share_directory("teamgrit_agent_receiver");
		std::string yaml_path = package_share_directory + "/config/config.yaml";

		YAML::Node config_yaml = YAML::LoadFile(yaml_path);
		if (!config_yaml["topics"] || !config_yaml["topics"].IsSequence()) {
			std::cerr << "Error: 'topics' key is missing or not a sequence!" << std::endl;
			return;
		}

		std::vector<std::tuple<std::string, std::string, std::string, std::string, std::string, std::string>> topics;

		for (const auto& topic_node : config_yaml["topics"]) {
			std::string topic = topic_node["topic"].as<std::string>();
			std::string name = topic_node["name"].as<std::string>();
			std::string host = topic_node["host"].as<std::string>();
			std::string port = topic_node["port"].as<std::string>();
			std::string endpoint = topic_node["endpoint"].as<std::string>();
			std::string type = topic_node["type"].as<std::string>();

			topics.emplace_back(topic, name, host, port, endpoint, type);
		}

		auto node = std::make_shared<WebSocketBridge>(topics);
		executor.add_node(node);
		executor.spin();
	} catch (const YAML::BadConversion &e) {
		std::cerr << "YAML conversion error: " << e.what() << std::endl;
	} catch (const YAML::Exception &e) {
		std::cerr << "YAML parsing error: " << e.what() << std::endl;
	}
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::executors::SingleThreadedExecutor executor;
    get_yaml(executor);
    rclcpp::shutdown();
    return 0;
}
