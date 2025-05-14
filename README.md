# cobiz-ros2-bridge

`CoBiz ROS2 Bridge`는 ROS2 humble 이상의 버전에서 사용 가능한 패키지로, 간편한 세팅과정을 통해 다양한 토픽의 데이터를 실시간으로 원격 송수신 할 수 있도록 설계된 SDK입니다.

> **문의 메일**
> 
> yslim@teamgrit.kr
>
> jhs10429@teamgrit.kr

# 목차

- [dependencies](#dependencies)
- [주요 기능](#주요-기능)
- [빠른 시작 가이드](#빠른-시작-가이드)
- [플러그인 개발 및 등록](#플러그인-개발-및-등록)
- [데이터 압축](#데이터-압축)

# dependencies

- [gstreamer](https://gstreamer.freedesktop.org/documentation/installing/on-linux.html?gi-language=c) // 예제의 RGB 데이터를 압축하기 위해 사용 불필요시 제외 가능
- [google/draco](https://github.com/google/draco) // 예제의 PointCloud2 데이터를 압축하기 위해 사용 불필요시 제외 가능
- [yaml-cpp](https://github.com/jbeder/yaml-cpp)
- OpenCV // 예제의 map 데이터를 png로 변환하기 위해 사용 불필요시 제외 가능
- OpenSSL
- CURL
- nlohmann_json

# 주요 기능

`CoBiz ROS2 Bridge`는 ROS2 메시지의 송수신을 위한 SDK입니다.

`main.cpp`파일은 YAML파일에 작성된 사용자의 토픽 정보들을 가져오고, 가져온 토픽 정보를 바탕으로 Subscriber 및 Publisher를 생성하여 각각의 플러그인에 전달합니다.

플러그인에서는 웹소켓을 활용하여 ROS2 토픽과 Moth 서버를 이어주는 브릿지 역할을 수행합니다.

## 1. Config 파일

### 1-1. config.yaml

`main.cpp`에서 사용될 토픽들의 이름이 정의되어있습니다.

subscribers에 해당하는 토픽들은 각각의 메시지 타입에 따라 `main.cpp`에서 subscription으로 생성됩니다.

**예를들어 `/odom`이라는 데이터를 로봇에서 원격 컴퓨터로 송신을 하려고 할 때 subscribers의 위치에 다음과 같이 작성합니다.**

```
subscribers:
  - /odom
publishers:
```

**반대로 `/cmd_vel`이라는 데이터를 원격 컴퓨터로부터 로봇이 수신하려 할 때 publishers의 위치에 다음과 같이 작성합니다.**

```
subscribers:
  - /odom
publishers:
  - /cmd_vel
```

이 YAML파일을 통해 생성된 subscription / publisher 들은 각각의 메시지 타입에 해당하는 plugin에 전달됩니다.

### 1-2. request.yaml

6가지의 변수가 정의되어있습니다.

|변수|기능|
|------|---|
|state|로봇의 토큰 발행 상태|
|id|로봇의 프리셋 아이디|
|token|로봇의 프리셋 토큰|
|preset|CoBiz 웹사이트에 등록한 프리셋|
|secret_key|프리셋을 사용하기 위한 키값|
|server_address|CoBiz 서버의 주소|

사용자는 CoBiz 웹사이트에 본인이 등록한 프리셋을 preset 변수에 적어주어야 사용 할 수 있습니다. (하단에 기재)

server_address 및 secret_key는 TeamGRIT으로 문의 주시면 발급이 가능합니다.

### 1-3. receiver.yaml

Linux to Linux 시스템을 사용하기 위해 `receiver.cpp`에서 사용하는 YAML파일입니다.

로봇의 `main.cpp`을 실행했을 때 나오는 로그를 확인하여 각각의 토픽에 따른 `host`, `port`, `endpoint`를 확인 할 수 있습니다. (로봇에 모니터를 사용 할 수 없는 상황이라면 팀그릿에 문의하세요)

**각각의 토픽에 따른 정보들을 아래와 같이 작성해주세요.**

```
endpoints:
  /odom:
    host: "localhost"
    port: "8080"
    endpoint: "/your/odom/endpoint"
  /imu:
    ...
```

### 1-4. topic_types.yaml

`receiver.cpp`에서 사용하는 YAML파일입니다.

이 파일에 정의되어있는 토픽 및 토픽 타입에 따라 수신측에서 각각의 토픽이 생성됩니다.

`config.yaml`에 작성된 토픽의 pub/sub 방식을 확인하여 `receiver.cpp`에서는 publisher와 subscription을 생성합니다.

생성된 publisher 및 subscription의 타입은 `topic_types.yaml`에 적혀있는 타입을 사용하므로,

**사용자는 웹소켓으로 수신되는 각각의 토픽에 따른 타입을 명시해주어야 합니다.**

```
topic_types:
  /odom: "nav_msgs/msg/Odometry"
  /imu: "sensor_msgs/msg/Imu"
  ...
```

## 2. 실행 파일

### 2-1. main.cpp

Moth 서버와 ROS2 메시지 간의 브릿지 역할을 수행합니다.

`request.yaml`파일에 설정된 상태를 바탕으로 각각의 작업을 수행합니다.

로봇에 할당된 id가 없다면 CoBiz 서버 주소로 preset 및 secret_key를 보냅니다.

CoBiz 서버에서는 preset에 해당하는 Moth 서버 url을 생성하여 로봇으로 전달하게 됩니다.

`main.cpp`에서는 서버로부터 받은 url 정보를 plugin에 전달하게됩니다.

`config.yaml`파일을 로드하여 브릿지로 연결할 토픽들에 대해 subscription 및 publisher를 생성합니다.

각각의 토픽 타입에 대해 매칭되는 플러그인을 실행하여 subscription 및 publisher를 플러그인으로 전달합니다.

이로써 사용자가 작성한 토픽에 따라 각각의 플러그인이 생성되고 플러그인에는 Moth 서버의 url과 토픽의 pub/sub에 따른 publisher 및 subscription이 전달됩니다.

### 2-2. receiver.cpp

`config.yaml`파일과 `topic_types.yaml`파일을 로드하여 각각의 토픽에 대해 subscription과 publisher를 생성합니다.

생성된 publisher/subscription은 `receiver.yaml`파일에 작성된 `host`, `port`, `endpoint`와 매칭하여 receiver_plugin에 전달합니다.

### 2-3. plugins

`main.cpp`로부터 subscription/publisher와 `host`, `port`, `endpoint`를 전달받습니다.

`host`, `port`, `endpoint`를 활용하여 Moth 서버와의 연결을 유지합니다.

subscription을 전달받은 플러그인은 메시지 데이터를 handler를 통해 전달받게 됩니다.

전달받은 메시지는 각각의 타입에 해당하는 Deserialization을 진행합니다.

메시지 형식에 맞춰 Moth 서버에 전달할 데이터를 JSON 형식으로 파싱하여 송신합니다.

이와 반대로 publisher를 전달받은 플러그인은 Moth 서버로부터 데이터를 읽어들입니다.

읽어들인 데이터를 JSON 형식으로 Deserialization을 진행하고, 각각의 데이터를 토픽 타입에 맞춰 작성합니다. (예시 Twist)

```
json j = json::parse(message);

// 메시지 생성
auto msg = std::make_shared<geometry_msgs::msg::Twist>();
msg->linear.x = j["linear"]["x"];
msg->linear.y = j["linear"]["y"];
msg->linear.z = j["linear"]["z"];
msg->angular.x = j["angular"]["x"];
msg->angular.y = j["angular"]["y"];
msg->angular.z = j["angular"]["z"];
rclcpp::SerializedMessage serialized_msg;
rclcpp::Serialization<geometry_msgs::msg::Twist> serializer;
serializer.serialize_message(msg.get(), &serialized_msg);
```

작성된 데이터를 전달받은 publisher를 통해 송신하여 브릿지 역할을 수행합니다.

### 2-4. receiver_plugins

`receiver.cpp`로부터 subscription/publisher와 `host`, `port`, `endpoint`를 전달받습니다.

`host`, `port`, `endpoint`를 활용하여 Moth 서버와의 연결을 유지합니다.

plugins와 마찬가지로 subscription을 전달받은 플러그인은 메시지 데이터를 handler를 통해 전달받습니다.

전달받은 메시지는 각각의 타입에 맞춰 Deserialization을 진행하고, JSON 형태로 파싱하여 Moth 서버로 전달합니다.

반대로 publisher를 전달받은 플러그인은 Moth 서버로부터 데이터를 읽어들입니다.

JSON으로 수신된 메시지를 토픽 타입에 맞춰 publisher로 송신합니다.

# 빠른 시작 가이드

## 1. 로봇 세팅

Linux to Linux 시스템을 활용하기 위해서 로봇 한대와 관제할 다른 컴퓨터를 준비합니다.

예시를 위해 다음의 토픽들을 정의합니다.

|토픽|타입|
|----|----|
|/scan|sensor_msgs/msg/LaserScan|
|/rgb|sensor_msgs/msg/Image|
|/map|nav_msgs/msg/OccupancyGrid|
|/odom|nav_msgs/msg/Odometry|
|/imu|sensor_msgs/msg/Imu|
|/cmd_vel|geometry_msgs/msg/Twist|
|/goal_pose|geometry_msgs/msg/PoseStamped|

**가장 먼저 SDK를 통해 관제 혹은 제어하고싶은 토픽을 `config.yaml`파일에 다음과 같이 작성합니다. (이 예시에서는 로봇이 원격 컴퓨터로부터 `/cmd_vel`토픽과 `/goal_pose`토픽을 제공받습니다.)**

```
subscribers:
  - /scan
  - /rgb
  - /map
  - /odom
  - /imu

publishers:
  - /cmd_vel
  - /goal_pose
```

이후 `request.yaml`파일에 해당하는 토픽에 대한 preset을 작성해주어야합니다.

preset을 세팅하는 방법은 아래와 같습니다.

먼저 CoBiz Website를 방문하여 로그인을 진행합니다. (팀별로 다른 도메인의 웹사이트가 주어집니다. TeamGRIT에 문의하세요.)

![CoBiz_1](https://github.com/teamgrit-lab/ICRA-2025-QRC/blob/cobiz-ros2-bridge/images/CoBiz_1.png)

**좌측 메뉴의 관리 텝을 클릭하고, 프리셋 텝으로 이동합니다.**

![CoBiz_2](https://github.com/teamgrit-lab/ICRA-2025-QRC/blob/cobiz-ros2-bridge/images/CoBiz_2.png)

**우측의 프리셋 등록을 통해 아래와 같이 JSON 형태의 프리셋을 입력합니다.**

```
{
  "name": "teamgrit_test",
  "battery": false,
  "modules": [
    {
      "name": "/scan",
      "type": "FREE"
    },
    {
      "name": "/rgb",
      "type": "FREE"
    },
    {
      "name": "/map",
      "type": "FREE"
    },
    {
      "name": "/odom",
      "type": "FREE"
    },
    {
      "name": "/imu",
      "type": "FREE"
    },
    {
      "name": "/cmd_vel",
      "type": "FREE"
    },
    {
      "name": "/goal_pose",
      "type": "FREE"
    }
  ]
}
```

이 예시의 프리셋 이름은 `teamgrit_test`이고, 배터리 데이터의 유무는 기본 False로 설정합니다.

"name"에는 사용할 토픽의 이름을 적어주세요.

Linux to Linux에서는 타입에 상관없이 특정 토픽을 주고받기 위한 프리셋이기에 모든 modules의 타입을 FREE로 설정합니다.

이후 프리셋을 저장 한 뒤 `request.yaml`파일의 preset에 본인이 설정한 프리셋 이름을 적어넣습니다. 이 예제에서는 `teamgrit_test`라는 이름을 사용했으므로 preset에 `teamgrit_test`를 적습니다.

이후 cobiz-bridge를 빌드하면 로봇의 준비는 마무리됩니다. **아래의 명령을 실행하여 나오는 로그를 통해 각각의 토픽별 `host`, `port`, `endpoint`를 확인합니다.**

```
cd cobiz-ros2-bridge/
colcon build
source install/setup.bash
ros2 run cobiz-bridge cobiz-bridge
```

## 2. 원격 컴퓨터 세팅

로봇의 데이터를 받을 원격 컴퓨터에서도 SDK를 클론하여 사용합니다.

**`config.yaml`파일은 앞선 로봇의 세팅과 동일하게 작성해주세요.**

**이후 각 토픽별 타입에 대한 정보를 `topic_types.yaml`파일에 작성해줍니다.**

```
topic_types:
  /imu: "sensor_msgs/msg/Imu"
  /scan: "sensor_msgs/msg/LaserScan"
  ...
```

앞서 로봇에서 실행했던 cobiz-bridge를 통해 우리는 각각의 토픽별 `host`, `port`, `endpoint`를 얻었습니다. (로봇의 url입니다.)

**이 정보를 바탕으로 `receiver.yaml`파일에 아래와 같이 작성합니다.**

```
endpoints:
  /scan:
    host: "your.host"
    port: "your.port"
    endpoint: "/pang/ws/sub? ... /scan&mode=bundle"
  /rgb:
    ...
```

> **Warning**
> 
> endpoint에는 /pang/ws/pub이라는 url이 적혀있습니다. 이 부분을 전부 /pang/ws/sub으로 바꿔주세요.

YAML파일의 작성이 완료되면 빌드를 진행한 후 다음 명령을 통해 실행합니다.

```
cd cobiz-ros2-bridge/
colcon build
source install/setup.bash
ros2 run cobiz-bridge receiver
```

# 플러그인 개발 및 등록

CoBiz ROS2 Bridge는 현재 개발 진행중에 있는 SDK입니다.

때문에 아직 모든 ROS2 Topic에 대한 플러그인이 준비되어있지 않습니다.

플러그인의 역할은 어느 한 타입(예를 들어 geometry_msgs/msg/Twist)으로 pub/sub중인 토픽을 원격 송수신 하기 위해 웹소켓을 연결하고 압축, 파싱등을 진행하며 데이터를 처리하는 과정을 다룹니다.

다음 가이드를 통해 사용자가 직접 플러그인을 작성하여 Moth 서버를 이용할 수 있습니다.

## 1. plugin 작성 가이드

로봇에서 사용될 플러그인은 2가지가 존재합니다.

Subscriber를 생성하여 원격 컴퓨터로 데이터를 송신할 플러그인과 Publisher를 생성하여 원격 컴퓨터로부터 수신받은 데이터를 publish할 플러그인이 존재합니다.

### 1-1. Subscription용 plugin

기본 예제를 확인하려면 [src/plugins/odom_handler.cpp](https://github.com/teamgrit-lab/ICRA-2025-QRC/blob/cobiz-ros2-bridge/cobiz-bridge/src/plugins/odom_handler.cpp)을 확인할 수 있습니다.

이 핸들러 파일은 `nav_msgs/msg/Odometry`토픽에 대한 핸들러입니다.

**(Line 2)** 가장 먼저 ROS2에서 제공하는 `nav_msgs/msg/odometry.hpp`헤더파일을 include하여 메시지 토픽을 Deserialize 할 수 있도록 합니다. (만약 Message 타입을 직접 정의하여 사용중이라면 `CMakeLists.txt`파일에 등록해야합니다.)

**(Line 17)** `plugin_init()`함수에는 `mime_`라는 변수가 정의되어있습니다.

원격 컴퓨터에서 웹소켓을 실행 할 때 가장 처음으로 보내지는 String타입의 메시지를 `mime_`변수에 정의합니다. (보통 메시지에 대한 고정적인 정보를 전달할 때 사용합니다.)

이 예제에서는 "text/json"이라는 메시지를 보내 수신자에게 데이터의 파싱 방법에 대해 알려주고 있습니다.

**(Line 23)** `handle`함수에서는 `main.cpp`에서 전달하는 토픽을 serialized_msg라는 인자로 사용합니다.

try문을 통해 `nav_msgs::msg::Odometry`메시지 변수를 생성하고, serialized_msg 데이터를 `nav_msgs::msg::Odometry`메시지로 Deserialization을 진행합니다.

이 예제에서는 데이터를 JSON으로 파싱하여 전달하기에 json 변수를 생성하여 msg 데이터들을 하나씩 파싱해줍니다.

MessageHandlerBase라는 상위 클래스의 `send_message()`함수를 통해 json_str 데이터를 원격 컴퓨터로 전달하게 됩니다.

**(Line 65)** `main.cpp`함수에서 각각의 토픽별 플러그인을 찾기위해 `getMessageType()`이라는 함수를 사용합니다.

이 예제에서는 `nav_msgs/msg/Odometry`타입의 토픽을 사용하므로 다음과 같이 작성됩니다.

```
std::string getMessageType() const override {
  return "nav_msgs/msg/Odometry";
}
```

**(Line 71)** 플러그인 클래스를 등록하여 `main.cpp`코드에서 식별할 수 있도록 OdomHandler 클래스를 등록하는 코드입니다.

플러그인 파일 작성이 완료되면 `plugin.xml`파일에 플러그인 클래스를 추가해 핸들러를 등록해줍니다.

```
<library path="message_handlers">
  <class type="OdomHandler" base_class_type="cobiz_bridge::MessageHandlerBase">
    <description>Odometry 메시지 처리 핸들러</description>
  </class>
  ...
</library>
```

### 1-2. publisher용 plugin

기본 예제를 확인하려면 [src/plugins/twist_handler.cpp](https://github.com/teamgrit-lab/ICRA-2025-QRC/blob/cobiz-ros2-bridge/cobiz-bridge/src/plugins/twist_handler.cpp)를 확인할 수 있습니다.

이 핸들러 파일은 `geometry_msgs/msg/Twist`토픽에 대한 핸들러입니다.

**(Line 2)** ROS2에서 제공하는 `geometry_msgs/msg/twist.hpp`헤더파일을 include하여 msg 데이터를 생성할 수 있게 해줍니다.

**(Line 17)** `plugin_init()`함수에는 웹소켓 데이터를 읽어들이기 위한 비동기 딜레이가 세팅되어있습니다. 데이터의 hz에 따라 read_timer_ 주기를 세팅해주세요. (100hz 이상의 데이터는 여러개의 데이터를 하나로 모아서 보내는 것을 권장합니다. [src/plugins/tf_message_handler.cpp](https://github.com/teamgrit-lab/ICRA-2025-QRC/blob/cobiz-ros2-bridge/cobiz-bridge/src/plugins/tf_message_handler.cpp)을 참고하세요.)

```
read_timer = std::make_shared<boost::asio::steady_timer>(get_io_context(), std::chrono::milliseconds(100)); //   < 10hz
```

**(Line 55)** `on_read()`함수에서는 원격 컴퓨터로부터 데이터를 수신하여 `geometry_msgs::msg::Twist`토픽으로 publish하는 코드가 작성되어있습니다.

이 예제에서도 마찬가지로 JSON 데이터로 파싱된 Twist 메시지가 원격 컴퓨터로부터 수신됩니다.

플러그인이 실행되면 앞서 `config.yaml`에 작성된 `/cmd_vel`토픽이 생성되고, 원격 컴퓨터에서 보내는 Twist 메시지가 publish되는 것을 확인 할 수 있습니다.

**(Line 100)** `on_write_timeout`에서 사용중인 async_write를 통해 원격 컴퓨터와의 통신을 주기적으로 업데이트합니다. (10초에 한번씩 메시지를 송신하여 Moth 서버 연결을 유지합니다.)

**(Line 123)** 플러그인 클래스를 틍록하여 `main.cpp`에서 식별할 수 있도록 TwistHandler 클래스를 등록합니다.

앞선 subscription 플러그인과 마찬가지로 `plugin.xml`파일에 TwistHandler 클래스를 등록해줍니다.

```
<library path="message_handlers">
  ...
  <class type="TwistHandler" base_class_type="cobiz_bridge::MessageHandlerBase">
    <description>Twist 메시지 처리 핸들러</description>
  </class>
</library>
```

## 2. receiver_plugin 작성 가이드

### 2-1. subscription용 receiver_plugin

기본 예제를 확인하려면 [src/receiver_plugins/receiver_twist_handler.cpp](https://github.com/teamgrit-lab/ICRA-2025-QRC/blob/cobiz-ros2-bridge/cobiz-bridge/src/receiver_plugins/receiver_twist_handler.cpp)를 확인 할 수 있습니다.

**(Line 2)** 앞선 plugin 예제와 마찬가지로 `geometry_msgs/msg/twist.hpp`헤더 파일을 include합니다.

**(Line 17)** `plugin_init()`함수에는 read_timer_ 변수를 통해 웹소켓 수신 딜레이를 설정 할 수 있습니다. (100hz 이상의 데이터를 송수신 하는것은 권장하지 않습니다. 한번의 송신에 여러 데이터를 모아서 송신하는 예제는 [src/plugins/tf_message_handler.cpp](https://github.com/teamgrit-lab/ICRA-2025-QRC/blob/cobiz-ros2-bridge/cobiz-bridge/src/plugins/tf_message_handler.cpp)을 참고하세요.)

```
(Line 21) read_timer_ = std::make_shared<boost::asio::steady_timer>(get_io_context(), std::chrono::milliseconds(100)); // < 10hz
(Line 40) read_timer_->expires_after(std::chrono::milliseconds(100));
```

**(Line 37)** `receiver.cpp`의 Subscription에서의 핸들링을 통해 handle함수에 serialized_msg가 수신됩니다. (YAML파일에서 설정한 토픽 수신)

try문 안에는 마찬가지로 `geometry_msgs::msg::Twist`타입의 메시지 변수 msg를 생성하여 serialized_msg를 Deserialization합니다.

msg 데이터를 JSON 형식으로 파싱하여 웹소켓으로 로봇쪽에 송신하게 됩니다.

**(Line 146)** 플러그인 클래스를 등록하여 `receiver.cpp`에서 이 플러그인을 식별 할 수 있도록 합니다.

파일 작성이 완료되면 `plugin.xml`의 receiver_message_handlers 라이브러리에 다음과 같이 플러그인을 등록합니다.

```
<library path="receiver_message_handlers">
  <class type="ReceiverTwistHandler" base_class_type="cobiz_bridge::ReceiverMessageHandlerBase">
    <description>Twist 메시지 수신 핸들러</description>
  </class>
</library>
```

### 2-2. publisher용 receiver_plugin

기본 예제를 확인하려면 [src/receiver_plugins/receiver_odom_handler.cpp](https://github.com/teamgrit-lab/ICRA-2025-QRC/blob/cobiz-ros2-bridge/cobiz-bridge/src/receiver_plugins/receiver_odom_handler.cpp)을 확인 할 수 있습니다.

**(Line 18)** `plugin_init()`함수에서 read_timer_ 변수를 통해 웹소켓 데이터 수신 딜레이를 설정 할 수 있습니다. (송신 딜레이보다 뒤쳐지지 않도록 수신 딜레이 값을 더 낮게 세팅해주세요.)

```
(Line 23) read_timer_ = std::make_shared<boost::asio::steady_timer>(get_io_context(), std::chrono::milliseconds(10)); //   < 100hz
(Line 44) read_timer_->expires_after(std::chrono::milliseconds(10));
```

**(Line 62)** `on_read`함수를 통해 로봇으로부터 수신받은 데이터를 ROS2에 publish 할 수 있습니다.

이 예제에서는 JSON으로 파싱된 데이터를 `nav_msgs::msg::Odometry`형식으로 변환하여 publish하는 예제를 다루고 있습니다.

**(Line 157)** 플러그인 클래스를 등록하여 `receiver.cpp`에서 이 플러그인(ReceiverOdomHandler)을 식별 할 수 있도록 합니다.

파일 작성이 완료되면 `plugin.xml`의 receiver_message_handlers 라이브러리에 다음과 같이 플러그인을 등록합니다.

```
<library path="receiver_message_handlers">
  <class type="ReceiverOdomeHandler" base_class_type="cobiz_bridge::ReceiverMessageHandlerBase">
    <description>Odom 메시지 수신 핸들러</description>
  </class>
</library>
```

# 데이터 압축

Moth 서버를 활용한 통신은 ROS2와는 달리 외부 서버와 통신을 하기에 네트워크가 필수적으로 사용됩니다.

그렇기 때문에 이미지 raw 데이터 및 포인트 클라우드와 같은 큰 데이터는 압축이 반드시 필요합니다.

그렇기 때문에 [src/plugins/image_handler.cpp](https://github.com/teamgrit-lab/ICRA-2025-QRC/blob/cobiz-ros2-bridge/cobiz-bridge/src/plugins/image_handler.cpp)과 [src/plugins/point_cloud2_handler.cpp](https://github.com/teamgrit-lab/ICRA-2025-QRC/blob/cobiz-ros2-bridge/cobiz-bridge/src/plugins/point_cloud2_handler.cpp)같은 플러그인은 데이터를 압축하여 전송하는 방식으로 코드가 작성되었습니다.

두 플러그인을 참고하면 GStreamer를 활용한 이미지 인코딩과 Draco를 사용한 PointCloud2 압축 방법을 확인 할 수 있습니다.
