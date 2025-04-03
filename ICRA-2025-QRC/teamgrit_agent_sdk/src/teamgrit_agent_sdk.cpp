#include <rclcpp/rclcpp.hpp>
#include <yaml-cpp/yaml.h>
#include <boost/beast/core.hpp>
#include <boost/beast/websocket.hpp>
#include <boost/asio/connect.hpp>
#include <boost/asio/strand.hpp>
#include <boost/asio/ip/tcp.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <thread>
#include <unordered_map>
#include <sstream>
#include <vector>
#include <teamgrit_agent_msgs/msg/agent_msg.hpp>
#include <teamgrit_agent_msgs/msg/agent_control.hpp>
#include <nlohmann/json.hpp>
#include <fstream>
#include <iostream>
#include <curl/curl.h>
#include <boost/asio.hpp>
#include <boost/beast/ssl.hpp>

using namespace std::chrono_literals;
using boost::asio::ip::tcp;
namespace beast = boost::beast;
namespace ssl = boost::asio::ssl;
namespace websocket = beast::websocket;
using json = nlohmann::json;

class TeamgritAgentSDK {
public:
    TeamgritAgentSDK(const std::string& host, const std::string& port, const std::string& endpoint, const std::string& mime, const std::string& type)
        : host_(host), port_(port), endpoint_(endpoint), mime_(mime), type_(type), ioc_(), read_timer_(ioc_), write_timer_(ioc_) {
        try {
            std::string wss = "wss://";
            if (host_.rfind(wss, 0) == 0) { // rfind(prefix, 0) == 0 : 맨 앞에서 찾았을 때만 제거
                host_ = host_.substr(wss.length());
            }

            tcp::resolver resolver(boost::asio::make_strand(ioc_));
            auto results = resolver.resolve(host_, port_);


            ssl::context ctx(ssl::context::tls_client);
            ws_ptr = std::make_shared<websocket::stream<ssl::stream<tcp::socket>>>(boost::asio::make_strand(ioc_), ctx);

//            ws_ptr = std::make_shared<websocket::stream<tcp::socket>>(boost::asio::make_strand(ioc_));
            boost::asio::connect(ws_ptr->next_layer().next_layer(), results.begin(), results.end());
            ws_ptr->next_layer().handshake(ssl::stream_base::client);
            ws_ptr->handshake(host_, endpoint_);

            ws_ptr->binary(false);
            ws_ptr->write(boost::asio::buffer(mime_));
            ws_ptr->binary(true);

            if (type_ == "CONTROL") {
                start_read_timer();
                start_write_timer();
            }
            else if (type_ == "SPEAKER") {
                start_audio_read_timer();
            }

            std::cout << "Connected to WebSocket:" << host_.c_str() << port_.c_str() << endpoint_.c_str() << std::endl;

            thread_ = std::thread([this]() { ioc_.run(); });
        } catch (const std::exception& e) {
            std::cout <<  "WebSocket connection failed" << e.what() << std::endl;
        }
    }

    ~TeamgritAgentSDK() {
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

    bool receive_message(std::vector<uint8_t>& received_data) {
        try {
            beast::flat_buffer buffer;
            ws_ptr->read(buffer);


            auto data = buffer.data();
            received_data.assign(
                boost::asio::buffer_cast<const uint8_t*>(data),
                boost::asio::buffer_cast<const uint8_t*>(data) + data.size()
            );
            return true;
        } catch (const std::exception& e) {
            RCLCPP_ERROR(rclcpp::get_logger("TeamgritAgentSDK"), "Failed to receive message: %s", e.what());
            reconnect();
        }
        return false;
    }

    void send_message(const std::vector<uint8_t>& msg) {
        try {
            ws_ptr->write(boost::asio::buffer(msg.data(), msg.size()));
        } catch (const std::exception& e) {
            RCLCPP_ERROR(rclcpp::get_logger("TeamgritAgentSDK"), "Failed to send message: %s", e.what());
            reconnect();
        }
    }

    bool async_read_flag_return(){
        return async_read_flag;
    }

    json get_received_data() {
        async_flag_change();
        return received_data;
    }

    std::vector<uint8_t> get_audio_received_data() {
        async_flag_change();
        return audio_received_data;
    }

private:
    std::string host_;
    std::string port_;
    std::string endpoint_;
    std::string mime_;
    std::string type_;
    boost::asio::io_context ioc_;
    std::shared_ptr<websocket::stream<ssl::stream<tcp::socket>>> ws_ptr;
//    std::shared_ptr<websocket::stream<tcp::socket>> ws_ptr;
    std::thread thread_;
    boost::asio::steady_timer read_timer_;
    boost::asio::steady_timer write_timer_;
    boost::beast::flat_buffer buffer_;
    json received_data;
    std::vector<uint8_t> audio_received_data;
    bool async_read_flag = false;

    void async_flag_change(){
        async_read_flag = false;
    }

    void start_read_timer() {
        read_timer_.expires_after(std::chrono::milliseconds(50)); // 0.1초 후
        read_timer_.async_wait([this](boost::system::error_code ec) {
            if (!ec) {
                read_data(); // read 데이터 수신
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
                    received_data = json::parse(data);
                } catch (const json::parse_error& e) {
                    std::cerr << "JSON parse error: " << e.what() << std::endl;
                } catch (...) {}
                async_read_flag = true;
                start_read_timer(); // 타이머 재설정
            });
    }

    void start_audio_read_timer() {
        read_timer_.expires_after(std::chrono::milliseconds(10)); // 0.1초 후
        read_timer_.async_wait([this](boost::system::error_code ec) {
            if (!ec) {
                audio_read_data(); // read 데이터 수신
            }
        });
    }

    void audio_read_data() {
        ws_ptr->async_read(buffer_,
            [this](boost::system::error_code ec, std::size_t bytes_transferred) {
                if (ec) {
                    std::cerr << "Error reading: " << ec.message() << std::endl;
                    reconnect();
                    start_audio_read_timer();
                    return;
                }

                std::string data = beast::buffers_to_string(buffer_.data());
                buffer_.consume(buffer_.size());
                try {
                    audio_received_data.assign(data.begin(), data.end());
                } catch (const json::parse_error& e) {
                    std::cerr << "JSON parse error: " << e.what() << std::endl;
                } catch (...) {}
                async_read_flag = true;
                start_audio_read_timer(); // 타이머 재설정
            });
    }

    void start_write_timer() {
        write_timer_.expires_after(std::chrono::seconds(10)); // 1초 후
        write_timer_.async_wait([this](boost::system::error_code ec) {
            if (!ec) {
                write_data(); // "ping" 메시지 전송
            }
        });
    }

    void write_data() {
        std::string message = "ping";
        ws_ptr->async_write(boost::asio::buffer(message),
            [this](boost::system::error_code ec, std::size_t bytes_transferred) {
                if (ec) {
                    std::cerr << "Error writing: " << ec.message() << std::endl;
                    reconnect();
                    start_write_timer();
                    return;
                }
                std::cout << "Sent: ping" << std::endl;
                start_write_timer(); // 타이머 재설정
            });
    }

};

class WebSocketBridge : public rclcpp::Node {
public:
    WebSocketBridge(const std::vector<std::tuple<std::string, std::string, std::string, std::string, std::string, std::string>>& topics)
        : Node("teamgrit_agent_sdk") {
        for (const auto& [topic, host, port, endpoint, mime, type] : topics) {
            auto client = std::make_shared<TeamgritAgentSDK>(host, port, endpoint, mime, type);
            ws_clients_[topic] = client;

            if (type == "CONTROL") {
                auto pub = this->create_publisher<teamgrit_agent_msgs::msg::AgentControl>(topic, 10);
                std::cout << "topic: " << topic << std::endl;

                control_publishers_[topic] = pub;
                RCLCPP_INFO(this->get_logger(), "Published From: %s -> %s:%s%s",
                            topic.c_str(), host.c_str(), port.c_str(), endpoint.c_str());

                auto timer = this->create_wall_timer(
                    std::chrono::milliseconds(50),
                    [this, topic]() { publish_message(topic); }
                );

                auto pub_timer = this->create_wall_timer(
                    std::chrono::seconds(1),
                    [this, topic]() { publish_ping(topic); }
                );

                timers_.push_back(timer);
                pub_timers_.push_back(pub_timer);
            }
            else if (type == "SPEAKER") {
                auto pub = this->create_publisher<teamgrit_agent_msgs::msg::AgentMsg>(topic, 10);
                std::cout << "topic: " << topic << std::endl;

                audio_publishers_[topic] = pub;
                RCLCPP_INFO(this->get_logger(), "Published From: %s -> %s:%s%s",
                            topic.c_str(), host.c_str(), port.c_str(), endpoint.c_str());

                auto timer = this->create_wall_timer(
                    std::chrono::milliseconds(10),
                    [this, topic]() { audio_publish_message(topic); }
                );

                auto pub_timer = this->create_wall_timer(
                    std::chrono::seconds(1),
                    [this, topic]() { publish_ping(topic); }
                );

                timers_.push_back(timer);
                pub_timers_.push_back(pub_timer);

            }
            else {
                auto sub = this->create_subscription<teamgrit_agent_msgs::msg::AgentMsg>(
                    topic, 10, [this, topic](const teamgrit_agent_msgs::msg::AgentMsg::SharedPtr msg) {
                        message_callback(topic, msg);
                    });
                std::cout << "topic: " << topic << std::endl;

                subscriptions_.push_back(sub);
                RCLCPP_INFO(this->get_logger(), "Subscribed to: %s -> %s:%s%s",
                            topic.c_str(), host.c_str(), port.c_str(), endpoint.c_str());
            }
        }
    }

private:
    std::unordered_map<std::string, std::shared_ptr<TeamgritAgentSDK>> ws_clients_;
    std::vector<rclcpp::Subscription<teamgrit_agent_msgs::msg::AgentMsg>::SharedPtr> subscriptions_;
    std::unordered_map<std::string, rclcpp::Publisher<teamgrit_agent_msgs::msg::AgentControl>::SharedPtr> control_publishers_;
    std::unordered_map<std::string, rclcpp::Publisher<teamgrit_agent_msgs::msg::AgentMsg>::SharedPtr> audio_publishers_;
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
            json received_data;
            if (ws_client->async_read_flag_return()){
                received_data = ws_client->get_received_data();
                auto pub_it = control_publishers_.find(topic);
                if (pub_it != control_publishers_.end()) {
                    teamgrit_agent_msgs::msg::AgentControl msg;
                    msg.lx = received_data["channel"][0];
                    msg.ly = received_data["channel"][1];
                    msg.rx = received_data["channel"][2];
                    msg.ry = received_data["channel"][3];
                    for (size_t i=0; i<16; ++i) {
                        msg.button[i] = received_data["channel"][i+4];
                    }
                    pub_it->second->publish(msg);
                }
            }
        } catch (...) {
            std::cout << "123" << std::endl;
        }

    }

    void audio_publish_message(const std::string& topic) {
        auto it = ws_clients_.find(topic);
        if (it == ws_clients_.end()) {
            RCLCPP_ERROR(this->get_logger(), "No WebSocket client found for topic: %s", topic.c_str());
            return;
        }
        auto ws_client = it->second;

        try {
            std::vector<uint8_t> received_data;
            if (ws_client->async_read_flag_return()){
                received_data = ws_client->get_audio_received_data();
                auto pub_it = audio_publishers_.find(topic);
                if (pub_it != audio_publishers_.end()) {
                    teamgrit_agent_msgs::msg::AgentMsg msg;
                    msg.data = received_data;
                    pub_it->second->publish(msg);
                }
            }
        } catch (...) {
            std::cout << "123" << std::endl;
        }

    }

    void publish_ping(const std::string& topic) {
        auto it = ws_clients_.find(topic);
        if (it != ws_clients_.end()) {
            std::vector<uint8_t> ping_msg = {'p', 'i', 'n', 'g'};
            it->second->send_message(ping_msg);
        } else {
            RCLCPP_ERROR(this->get_logger(), "No WebSocket client found for topic: %s", topic.c_str());
        }
    }

    void message_callback(const std::string& topic, const teamgrit_agent_msgs::msg::AgentMsg::SharedPtr msg) {
        auto it = ws_clients_.find(topic);
        if (it != ws_clients_.end()) {
            it->second->send_message(msg->data);
        } else {
            RCLCPP_ERROR(this->get_logger(), "No WebSocket client found for topic: %s", topic.c_str());
        }
    }
};

static size_t write_callback(void *contents, size_t size, size_t nmemb, std::string *output) {
	size_t total_size = size * nmemb;
	output->append((char *)contents, total_size);
	return total_size;
}

void load_config_and_run(rclcpp::executors::SingleThreadedExecutor& executor, json requested_data) {
    std::string package_share_directory = ament_index_cpp::get_package_share_directory("teamgrit_agent_sdk");
    YAML::Node config = YAML::LoadFile(package_share_directory + "/config/config.yaml");
    std::vector<std::tuple<std::string, std::string, std::string, std::string, std::string, std::string>> topics;

    for (const auto& item : config["topics"]) {
        std::string topic = item["topic"].as<std::string>();
        std::string mime = item["mime"].as<std::string>();
        std::string host;
        std::string port;
        std::string endpoint;
        std::string name = item["name"].as<std::string>();
        std::string type;
        for (const auto& module : requested_data["modules"]) {
            if (module["name"] == name) {
                host = module["connection"]["host"];
                port = module["connection"]["port"];
                endpoint = module["connection"]["endpoint"];
                type = module["type"];
                break;
            }
        }
        topics.emplace_back(topic, host, port, endpoint, mime, type);
    }
    auto node = std::make_shared<WebSocketBridge>(topics);
    executor.add_node(node);
    executor.spin();
}

void request_to_server(rclcpp::executors::SingleThreadedExecutor& executor) {
	try {
		std::string package_share_directory = ament_index_cpp::get_package_share_directory("teamgrit_agent_sdk");
		YAML::Node request_yaml = YAML::LoadFile(package_share_directory + "/config/request.yaml");
		std::cout << "request_yaml state is: " << request_yaml["state"].as<std::string>() << std::endl;
		if (request_yaml["state"].as<std::string>() == "NotRegistered"){
			CURL *curl;
			CURLcode res;
			std::string response;

			json json_data;
			json_data["preset"] = request_yaml["preset"].as<std::string>();
			json_data["secret_key"] = request_yaml["secret_key"].as<std::string>();

			std::string jsonData = json_data.dump(2);
			std::cout << "jsonData: " << jsonData << std::endl;
			std::string server_address = request_yaml["server_address"].as<std::string>();

			curl_global_init(CURL_GLOBAL_ALL);
			curl = curl_easy_init();

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
			std::cout << "response: \n" << response << std::endl;
			json jsonResponse = json::parse(response);
			std::string id = std::to_string(jsonResponse["id"].get<int>());
			std::string token = jsonResponse["token"];
			std::cout << "id: " << id << std::endl;

			request_yaml["id"] = id;
			request_yaml["state"] = "Registered";
			request_yaml["token"] = token;
			YAML::Emitter out;
			out << request_yaml;
			std::ofstream fout(package_share_directory + "/config/request.yaml");
			if (!fout) {
				std::cout << "Failed to open file" << std::endl;;
				return;
			}
			fout << out.c_str();
			fout.close();
			request_to_server(executor);
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
					request_to_server(executor);
					return;
				}

				curl_slist_free_all(headers);
				curl_easy_cleanup(curl);

			}

			curl_global_cleanup();
			json requested_data_ = json::parse(response);
			std::cout << "requested_data: " << requested_data_ << std::endl;
			if (requested_data_.contains("message")) {
				if (requested_data_["message"] == "Unregistered token.") {
					request_yaml["id"] = "null";
					request_yaml["state"] = "NotRegistered";
					request_yaml["token"] = "null";
					YAML::Emitter out;
					out << request_yaml;
					std::ofstream fout(package_share_directory + "/config/request.yaml");
					if (!fout) {
						std::cout << "Failed to open file" << std::endl;;
						return;
					}
					fout << out.c_str();
					fout.close();
					request_to_server(executor);
					return;
				}
			}
			load_config_and_run(executor, requested_data_);
		}
	} catch(const std::exception &e){
		std::cout << "err: " << e.what() << std::endl;
	}
}

int main(int argc, char** argv) {
    sleep(5);
    rclcpp::init(argc, argv);
    rclcpp::executors::SingleThreadedExecutor executor;
    request_to_server(executor);
    rclcpp::shutdown();
    return 0;
}
