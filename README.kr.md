# TeamGRIT Agent

TeamGRIT Agent SDK는 ROS2에서 사용가능한 패키지로, 메시지 형식을 하나로 통합하여 어떤 데이터든지 Topic을 통해 [Moth](https://docs.cobiz.kr/docs/advanced-guides/protocol/protocol-rssp) 서버로 전송할 수 있도록 설계되었습니다.

## 목차

1. [주요 기능](#주요-기능)
2. [메시지 Topic 정의](#메시지-Topic-정의)
3. [코드 구조 설명](#코드-구조-설명)
4. [yaml 파일 작성](#yaml-파일-작성)
5. [빌드 및 실행](#빌드-및-실행)
6. [linux to linux](#linux-to-linux)

## 주요 기능

+ 통합 메시지 형식
  + 모든 데이터를 `uint8[]` 형식으로 전송 가능한 인터페이스 제공
  + ROS2 Topic을 통해 일관된 인터페이스 제공
+ 범용적인 Moth 서버 연동
  + `config` 디렉토리에 작성된 파라미터를 기반으로 Moth 서버와 동적으로 연결

## 메시지 Topic 정의

- [`AgentMsg.msg`](https://github.com/teamgrit-lab/teamgrit-agent/blob/main/teamgrit_agent_msgs/msg/AgentMsg.msg)에서는 범용적인 Moth 서버 연동을 위해 모든 데이터 형식을`uint8[]`로 통합하여 사용합니다.

```
uint8[] data
```

- [`AgentControl.msg`](https://github.com/teamgrit-lab/teamgrit-agent/blob/main/teamgrit_agent_msgs/msg/AgentControl.msg)에서는 4족 보행 로봇에서 사용될 조이스틱 Float32값 4개와 그 이외의 버튼 입력을 감지하는 Float32 배열이 정의되어있습니다.

```
Float32 lx
Float32 ly
Float32 rx
Float32 ry
Float32[16] button
```

- 위에 적힌 조이스틱과 버튼 배열은 [Gamepad Tester](https://hardwaretester.com/gamepad) 페이지에서 사용하는 스틱 및 버튼 배열입니다.

- 원격 조종을 사용하길 원하는 참가자들은 `AgentControl.msg`형식을 참고하세요.

## 코드 구조 설명

아래의 그림은 TeamGRIT Agent SDK에 대한 구조를 간략하게 나타낸 그림입니다.

![CoBiz Agent](https://github.com/teamgrit-lab/teamgrit-agent/blob/main/image/CoBiz%20Agent%20(Linux%20to%20Client).png)

중앙에 있는 Agent 노드에서는 원격 관제소(CoBiz WebSite)와 로봇(User Node)을 연결하기 위한 브릿지 역할을 수행합니다.

범용적인 브릿지 역할을 수행하기 위해, Agent 노드에서 사용할 메시지 형식을 두 가지로 정의합니다.

+ [AgentMsg](https://github.com/teamgrit-lab/teamgrit-agent/blob/main/teamgrit_agent_msgs/msg/AgentMsg.msg)
+ [AgentControl](https://github.com/teamgrit-lab/teamgrit-agent/blob/main/teamgrit_agent_msgs/msg/AgentControl.msg)

사용자는 각각의 노드에서 원격 관제가 필요한 데이터를 `AgentMsg`형식에 맞춰 Agent 노드로 전송합니다.

Agent 노드에서는 [config/config.yaml](https://github.com/teamgrit-lab/teamgrit-agent/blob/main/teamgrit_agent_sdk/config/config.yaml) 파일에 정의된 Topic 데이터를 읽어들이고, Moth 서버에 전달하는 브릿지 역할을 수행합니다.

또한 Agent 노드에서는 Control 노드에 원격 조종을 위한 `AgentControl`메시지를 제공합니다.

이 모든 역할을 수행하기 위해 사용자는 Agent 노드에 제공될 yaml 파일들을 작성해야 합니다.

## yaml 파일 작성

Agent SDK를 사용하기에 앞서 사용자는 두가지의 yaml 파일을 작성해야 합니다.

### config.yaml

[`config.yaml`](https://github.com/teamgrit-lab/teamgrit-agent/blob/main/teamgrit_agent_sdk/config/config.yaml)파일에는 User Node에서 사용할 Topic 정보와 사용자 지정 이름 및 mime에 대한 정보가 기재되어 있습니다.

아래는 예제를 통한 설명입니다.

```
topics:
  - topic: "/teamgrit/image/h264"
    name: "front_camera"
    mime: "video/h264;width=640;height=480;framerate=30;codecs=avc1.42002A"
  - topic: "/teamgrit/image/h264_2"
    name: "back_camera"
    mime: "video/h264;width=640;height=480;framerate=30;codecs=avc1.42002A"
  - topic: "/teamgrit/lidar"
    name: "lidar"
    mime: "lidar/draco"
  - topic: "/teamgrit/sensor"
    name: "sensor"
    mime: "text/json"
  - topic: "/teamgrit/control"
    name: "control"
    mime: "text/json"
```

`front_camera` 노드에서는 압축된 이미지 데이터를 `AgentMsg`형식에 맞춰 `/teamgrit/image/h264`라는 이름의 Topic으로 전송합니다.

해당 노드에 대한 이름을 `front_camera`라고 정의했으며, mime에는 Topic에 대한 정보를 기재합니다.

mime에 대한 작성 방법은 [mime 및 Type](https://docs.cobiz.kr/docs/advanced-guides/type-and-mime)에 대한 문서를 참고하세요.

mime에서 알 수 있듯이 `front_camera`는 이미지 데이터를 h.264 코덱으로 압축했으며, 원본 데이터는 VGA(640*480) 화질이라는 것을 알 수 있습니다.

`back_camera` 노드 또한 `front_camera` 노드와 비슷하게 작성이 되어있습니다.

주의해야 할 것은 name의 중복 유무입니다.

앞선 `front_camera` 노드에서 `front_camera`라는 이름을 사용했다면 `back_camera` 노드에서는 `front_camera`라는 이름을 사용할 수 없습니다.

topic도 마찬가지입니다.

`back_camera` 노드에서는 `front_camera`노드와는 달리 `/teamgrit/image/h264_2`토픽과 `back_camera`라는 이름을 사용합니다.

`lidar` 노드에서는 Google의 Draco를 사용하여 PointCloud 압축 데이터를 제공합니다.

Agent에서는 Draco 이외의 압축 방법은 제공하지 않습니다.

`sensor` 노드와 `control` 노드에서는 JSON 형식의 데이터를 사용합니다.

### request.yaml

[`request.yaml`](https://github.com/teamgrit-lab/teamgrit-agent/blob/main/teamgrit_agent_sdk/config/request.yaml)파일에는 [CoBiz](https://docs.cobiz.kr/docs/introduction) 서버에 전송될 `preset` 정보와 `secret_key`및 `server_address` 정보가 들어있습니다.

```
state: "NotRegistered"
id: null
token: null
preset: "teamgrit"
secret_key: "1234567890"
server_address: "teamgrit_address"
```

사용자는 TeamGRIT에서 제공하는 `preset`이름과 `secret_key`및 `server_address`를 작성합니다. (TeamGRIT에서 제공하는 값들입니다.)

## 빌드 및 실행

sdk를 시작하기에 앞서 `teamgrit_agent_msgs` 를 빌드해줍니다.

```
cd teamgrit-agent/teamgrit_agent_msgs
colcon build
```

메시지 패키지의 빌드가 끝나면 환경 변수에 패키지를 추가해줍니다.

```
source install/setup.bash
```

이제 `teamgrit_agent_sdk` 디렉토리로 이동하여 sdk를 빌드 할 수 있습니다.

sdk를 빌드하기 위한 dependency를 확인하세요.

```
sudo apt install nlohmann-json3-dev
sudo apt install libcurl4-openssl-dev
sudo apt install libssl-dev
sudo apt install ros-${ROS_DISTRO}-yaml-cpp-vendor
sudo apt install ros-${ROS_DISTRO}-ament-index-cpp
```

dependency를 모두 설치했다면 colcon 명령을 활용하여 sdk를 빌드 할 수 있습니다.

```
cd teamgrit-agent/teamgrit_agent_sdk
colcon build
```

`teamgrit_agent_sdk`를 사용하기 위해 환경 변수에 패키지를 추가해줍니다.

```
source install/setup.bash
```

모든 세팅이 끝나면 아래의 명령을 사용하여 `teamgrit_agnet_sdk`를 실행하세요.

```
ros2 run teamgrit_agent_sdk teamgrit_agent_sdk
```

## linux to linux

웹 페이지에서와 마찬가지로 또 다른 리눅스 환경에서 `teamgrit_agent_receiver`를 통해 데이터를 수신 할 수 있습니다.

`Agent` 패키지와 마찬가지로 수신된 데이터는 `teamgrit_agent_msg`형식에 맞춰 ROS2 Topic으로 송신됩니다.

![CoBiz Agent linux to linux](https://github.com/teamgrit-lab/teamgrit-agent/blob/main/image/CoBiz%20Agent%20(Linux%20to%20Linux).png)

config 파일을 작성하기 전 [Mime 및 Type](https://docs.cobiz.kr/docs/advanced-guides/type-and-mime)에 대한 문서를 확인해주세요.

문서를 참조한 뒤 [`config/config.yaml`](https://github.com/teamgrit-lab/teamgrit-agent/blob/main/teamgrit_agent_receiver/config/config.yaml)파일을 수정해줍니다.

```
topics:
  - topic: "/teamgrit/image/raw"
    name: "front_camera"
    host: "your_host"
    port: "your_port"
    endpoint: "your_endpoint"
    type: "VIDEO"
  - topic: "/teamgrit/image/raw"
    name: "back_camera"
    host: "your_host"
    port: "your_port"
    endpoint: "your_endpoint"
    type: "VIDEO"
  - topic: "/teamgrit/lidar"
    name: "lidar"
    host: "your_host"
    port: "your_port"
    endpoint: "your_endpoint"
    type: "LIDAR"
  - topic: "/teamgrit/sensor"
    name: "sensor"
    host: "your_host"
    port: "your_port"
    endpoint: "your_endpoint"
    type: "SENSOR"
  - topic: "/teamgrit/control"
    name: "control"
    host: "your_host"
    port: "your_port"
    endpoint: "your_endpoint"
    type: "CONTROL"
```

앞선 `teamgrit_agent_sdk`의 세팅과 비슷하게 6개의 파라미터가 존재합니다.

`topic`: 웹소켓으로 수신된 데이터가 이 파라미터의 이름과 `teamgrit_agent_msg`형식으로 송신됩니다.
- `CONTROL` 타입과 `SPEAKER` 타입의 경우 반대로 이 토픽에서 수신되는 데이터를 Moth 서버로 전송합니다.

`host`: 웹소켓으로부터 데이터를 수신하기 위한 URL의 호스트 주소입니다.

`port`: 웹소켓으로부터 데이터를 수신하기 위한 URL의 포트 번호입니다.

`endpoint`: 웹소켓으로부터 데이터를 수신하기 위한 URL의 경로입니다.

'type': type에 대한 내용은 [Mime 및 Type](https://docs.cobiz.kr/docs/advanced-guides/type-and-mime)에 대한 문서를 참고하세요

빌드 및 실행 방법은 아래와 같습니다.

```
source teamgrit_agent/teamgrit_agent_msgs/install/setup.bash
colcon build
```

```
source teamgrit_agent/teamgrit_agent_receiver/install/setup.bash
ros2 run teamgrit_agent_receiver teamgrit_agent_receiver
```
