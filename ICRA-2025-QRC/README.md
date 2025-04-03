## TeamGRIT Agent

TeamGRIT Agent SDK is a ROS2 compatible package designed to integrate message formats, allowing seamless transmission of any data to the [Moth](https://docs.cobiz.kr/docs/advanced-guides/protocol/protocol-rssp) server via Topics.

## Table of Contents

1.  [Key Features](#key-features)
2.  [Message Topic Definitions](#message-topic-definitions)
3.  [Code Structure Explanation](#code-structure-explanation)
4.  [YAML File Creation](#yaml-file-creation)
5.  [Build and Execution](#build-and-execution)
6.  [Linux to Linux](#linux-to-linux)

## Key Features

  * Integrated Message Format
      * Provides an interface to transmit all data in `uint8[]` format.
      * Offers a consistent interface via ROS2 Topics.
  * Universal Moth Server Integration
      * Dynamically connects to the Moth server based on parameters defined in the `config` directory.

## Message Topic Definitions

  * [`AgentMsg.msg`](https://github.com/teamgrit-lab/ICRA-2025-QRC/blob/main/ICRA-2025-QRC/teamgrit_agent_msgs/msg/AgentMsg.msg) integrates all data formats into `uint8[]` for universal Moth server compatibility.

<!-- end list -->

```
uint8[] data
```

  * [`AgentControl.msg`](https://github.com/teamgrit-lab/ICRA-2025-QRC/blob/main/ICRA-2025-QRC/teamgrit_agent_msgs/msg/AgentControl.msg) defines four Float32 values for joysticks used in quadruped robots, along with a Float32 array for button inputs.

<!-- end list -->

```
Float32 lx
Float32 ly
Float32 rx
Float32 ry
Float32[16] button
```

  * The joystick and button array specified above match the stick and button arrangement used on the [Gamepad Tester](https://hardwaretester.com/gamepad) page.

  * Participants intending to utilize remote control should refer to the `AgentControl.msg` format.

## Code Structure Explanation

The diagram below provides a simplified overview of the TeamGRIT Agent SDK architecture.

![CoBiz Agent](https://github.com/teamgrit-lab/ICRA-2025-QRC/blob/main/ICRA-2025-QRC/image/CoBiz%20Agent%20(Linux%20to%20Client).png)

The central Agent node acts as a bridge between the remote control station (CoBiz Website) and the robot (User Node).

To serve as a universal bridge, two message formats are defined for use by the Agent node:

  * [AgentMsg](https://github.com/teamgrit-lab/ICRA-2025-QRC/blob/main/ICRA-2025-QRC/teamgrit_agent_msgs/msg/AgentMsg.msg)
  * [AgentControl](https://github.com/teamgrit-lab/ICRA-2025-QRC/blob/main/ICRA-2025-QRC/teamgrit_agent_msgs/msg/AgentControl.msg)

Users transmit the data needed for remote control from their respective nodes to the Agent node in the `AgentMsg` format.

The Agent node reads the Topic data defined in the [config/config.yaml](https://github.com/teamgrit-lab/ICRA-2025-QRC/blob/main/ICRA-2025-QRC/teamgrit_agent_sdk/config/config.yaml) file and acts as a bridge to forward it to the Moth server.

The Agent node also provides `AgentControl` messages to the Control node for remote operation.

Users must create YAML files for the Agent node to facilitate all these functions.

## YAML File Creation

Users must create two YAML files before utilizing the Agent SDK.

### config.yaml

The [`config.yaml`](https://github.com/teamgrit-lab/ICRA-2025-QRC/blob/main/ICRA-2025-QRC/teamgrit_agent_sdk/config/config.yaml) file includes Topic information for the User Node, along with user-specified names and MIME details.

An example is shown below:

```yaml
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

The `front_camera` node transmits compressed image data in `AgentMsg` format to the Topic named `/teamgrit/image/h264`.

The name of this node is defined as `front_camera`, and MIME information about the Topic is provided.

For MIME creation details, refer to the [MIME and Type](https://docs.cobiz.kr/docs/advanced-guides/type-and-mime/) documentation.

As indicated by the MIME, `front_camera` compresses image data using the H.264 codec, and the original data is in VGA (640\*480) resolution.

The `back_camera` node is configured similarly to the `front_camera` node.

Note the uniqueness of the names.

If `front_camera` is already used by the `front_camera` node, it cannot be reused by the `back_camera` node.

The same applies to Topics.

Unlike the `front_camera` node, the `back_camera` node utilizes the `/teamgrit/image/h264_2` Topic and is named `back_camera`.

The `lidar` node provides PointCloud compressed data using Google's Draco.

The Agent does not provide compression methods other than Draco.

The `sensor` and `control` nodes use JSON formatted data.

### request.yaml

The [`request.yaml`](https://github.com/teamgrit-lab/ICRA-2025-QRC/blob/main/ICRA-2025-QRC/teamgrit_agent_sdk/config/request.yaml) file contains `preset`, `secret_key`, and `server_address` information for transmission to the [CoBiz](https://docs.cobiz.kr/docs/introduction) server.

```yaml
state: "NotRegistered"
id: null
token: null
preset: "teamgrit"
secret_key: "1234567890"
server_address: "teamgrit_address"
```

Users enter the `preset` name, `secret_key`, and `server_address` provided by TeamGRIT. (These values are provided by TeamGRIT.)

## Build and Execution

Before starting the SDK, build `teamgrit_agent_msgs`.

```bash
cd teamgrit-agent/teamgrit_agent_msgs
colcon build
```

After building the message package, add the package to the environment variables.

```bash
source install/setup.bash
```

Navigate to the `teamgrit_agent_sdk` directory to build the SDK.

Check the dependencies for building the SDK.

```bash
sudo apt install nlohmann-json3-dev
sudo apt install libcurl4-openssl-dev
sudo apt install libssl-dev
sudo apt install ros-${ROS_DISTRO}-yaml-cpp-vendor
sudo apt install ros-${ROS_DISTRO}-ament-index-cpp
```

Once all dependencies are installed, you can build the SDK using the colcon command.

```bash
cd teamgrit-agent/teamgrit_agent_sdk
colcon build
```

Add the package to the environment variables to use `teamgrit_agent_sdk`.

```bash
source install/setup.bash
```

After completing all settings, run `teamgrit_agent_sdk` using the command below.

```bash
ros2 run teamgrit_agent_sdk teamgrit_agent_sdk
```

## Linux to Linux

Data can be received in another Linux environment via `teamgrit_agent_receiver` similar to the web page.

Like the `Agent` package, received data is transmitted as ROS2 Topics in the `teamgrit_agent_msg` format.

![CoBiz Agent linux to linux](https://github.com/teamgrit-lab/ICRA-2025-QRC/blob/main/ICRA-2025-QRC/image/CoBiz%20Agent%20(Linux%20to%20Linux).png)

Before creating the config file, review the [MIME and Type](https://docs.cobiz.kr/docs/advanced-guides/type-and-mime/) documentation.

Modify the [`config/config.yaml`](https://github.com/teamgrit-lab/ICRA-2025-QRC/blob/main/ICRA-2025-QRC/teamgrit_agent_receiver/config/config.yaml) file based on the documentation.

```yaml
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

Similar to the previous `teamgrit_agent_sdk` setup, there are six parameters.

`topic`: Data received via WebSocket is transmitted with the name of this parameter in the `teamgrit_agent_msg` format.

  * For `CONTROL` and `SPEAKER` types, data received from this topic is forwarded to the Moth server.

`host`: The host address of the URL for receiving data from the WebSocket.

`port`: The port number of the URL for receiving data from the WebSocket.

`endpoint`: The path of the URL for receiving data from the WebSocket.

`type`: For information on types, refer to the [MIME and Type](https://www.google.com/search?q=https://docs.cobiz.kr/docs/advanced-guides/type-and-mime) document.

Build and execution methods are as follows:

```bash
source teamgrit_agent/teamgrit_agent_msgs/install/setup.bash
colcon build
```

```bash
source teamgrit_agent/teamgrit_agent_receiver/install/setup.bash
ros2 run teamgrit_agent_receiver teamgrit_agent_receiver
```
