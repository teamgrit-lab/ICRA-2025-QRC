cmake_minimum_required(VERSION 3.8)
project(cobiz-bridge)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(pluginlib REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(OpenCV REQUIRED)

find_package(Boost REQUIRED COMPONENTS system thread)
find_package(OpenSSL REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(CURL REQUIRED)

find_package(PkgConfig REQUIRED)
pkg_check_modules(GSTREAMER REQUIRED gstreamer-1.0)
pkg_check_modules(GSTREAMER_APP REQUIRED gstreamer-app-1.0)

# 헤더 파일 경로 설정
include_directories(
  include
)

# 플러그인 라이브러리 빌드
file(GLOB PLUGIN_SOURCES "src/plugins/*_handler.cpp")
add_library(message_handlers SHARED ${PLUGIN_SOURCES}
)

target_include_directories(message_handlers PUBLIC
  ${GSTREAMER_INCLUDE_DIRS}
  ${GSTREAMER_APP_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)
# 플러그인 라이브러리 종속성
target_link_libraries(message_handlers
  OpenSSL::SSL
  OpenSSL::Crypto
  nlohmann_json::nlohmann_json
  Boost::system
  Boost::thread
  pthread
  ${GSTREAMER_LIBRARIES}
  ${GSTREAMER_APP_LIBRARIES}
  /usr/local/lib/libdraco.a
  ${OpenCV_LIBRARIES}
)
ament_target_dependencies(message_handlers
  rclcpp
  sensor_msgs
  nav_msgs
  tf2_msgs
  pluginlib
)

# 수신기 플러그인 빌드
file(GLOB RECEIVER_PLUGIN_SOURCES "src/receiver_plugins/*_handler.cpp")
add_library(receiver_message_handlers SHARED ${RECEIVER_PLUGIN_SOURCES})

target_include_directories(receiver_message_handlers PUBLIC
  ${GSTREAMER_INCLUDE_DIRS}
  ${GSTREAMER_APP_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

target_link_libraries(receiver_message_handlers
  OpenSSL::SSL
  OpenSSL::Crypto
  nlohmann_json::nlohmann_json
  Boost::system
  Boost::thread
  pthread
  ${GSTREAMER_LIBRARIES}
  ${GSTREAMER_APP_LIBRARIES}
  /usr/local/lib/libdraco.a
  ${OpenCV_LIBRARIES}
)

ament_target_dependencies(receiver_message_handlers
  rclcpp
  sensor_msgs
  nav_msgs
  tf2_msgs
  pluginlib
)

# 플러그인 XML 파일 내보내기 - 추가 필요
pluginlib_export_plugin_description_file(cobiz-bridge plugin.xml)

# 실행 파일 생성
add_executable(${PROJECT_NAME} src/main.cpp)
target_link_libraries(${PROJECT_NAME} yaml-cpp OpenSSL::SSL OpenSSL::Crypto nlohmann_json::nlohmann_json Boost::system Boost::thread pthread CURL::libcurl nlohmann_json::nlohmann_json)
ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  ament_index_cpp
  pluginlib
)

# # Receiver 실행 파일 생성
add_executable(receiver src/receiver.cpp)
target_link_libraries(receiver yaml-cpp OpenSSL::SSL OpenSSL::Crypto nlohmann_json::nlohmann_json Boost::system Boost::thread pthread)
ament_target_dependencies(receiver
  rclcpp
  ament_index_cpp
  pluginlib
)

# 설치
install(TARGETS message_handlers
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS receiver_message_handlers
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS ${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS receiver
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  include/
  DESTINATION include
)

install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}
)

install(FILES
  plugin.xml  # plugins.xml에서 수정
  DESTINATION share/${PROJECT_NAME}
)

install(FILES
  config/receiver.yaml
  config/topic_types.yaml
  DESTINATION share/${PROJECT_NAME}/config
)

ament_package()
