cmake_minimum_required(VERSION 3.8)
project(ired_refbox)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

find_package(Protobuf REQUIRED)
find_package(OpenSSL REQUIRED)
find_package(Boost REQUIRED COMPONENTS system thread)

set(THIRD_PARTY_DIR "${CMAKE_CURRENT_SOURCE_DIR}/third_party")
set(PROTOBUF_DIR "${THIRD_PARTY_DIR}/generated")
set(PROTOBUF_COMM_DIR "${THIRD_PARTY_DIR}/protobuf_comm")

if(NOT EXISTS "${PROTOBUF_DIR}/BeaconSignal.pb.h")
  message(FATAL_ERROR "Missing vendored protobuf files in ${PROTOBUF_DIR}")
endif()

if(NOT EXISTS "${PROTOBUF_COMM_DIR}/src/peer.cpp")
  message(
    FATAL_ERROR "Missing vendored protobuf_comm source in ${PROTOBUF_COMM_DIR}")
endif()

file(GLOB PROTOBUF_SOURCES CONFIGURE_DEPENDS "${PROTOBUF_DIR}/*.pb.cc")

set(PROTOBUF_COMM_SOURCES
    "${PROTOBUF_COMM_DIR}/src/crypto.cpp"
    "${PROTOBUF_COMM_DIR}/src/message_register.cpp"
    "${PROTOBUF_COMM_DIR}/src/peer.cpp")

add_executable(ired_refbox_node src/refbox_node.cpp ${PROTOBUF_SOURCES}
                                ${PROTOBUF_COMM_SOURCES})

target_compile_features(ired_refbox_node PRIVATE cxx_std_17)

target_compile_definitions(ired_refbox_node PRIVATE HAVE_LIBCRYPTO)

target_include_directories(
  ired_refbox_node
  PRIVATE "${THIRD_PARTY_DIR}" "${PROTOBUF_DIR}" "${PROTOBUF_COMM_DIR}/include"
          ${Protobuf_INCLUDE_DIRS})

ament_target_dependencies(ired_refbox_node rclcpp std_msgs geometry_msgs)

target_link_libraries(ired_refbox_node ${Protobuf_LIBRARIES} OpenSSL::Crypto
                      Boost::system Boost::thread)

install(TARGETS ired_refbox_node DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

ament_package()
