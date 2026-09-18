cmake_minimum_required(VERSION 3.16)
project(franka_server)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(realtime_tools REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Float32Array.msg"
  "srv/GetValue.srv"
  "srv/SetValue.srv"
)

add_executable(franka_server_node
  src/franka_server_node.cpp
  src/thunder_franka.cpp
  src/franka_gen.cpp
)
target_compile_features(franka_server_node PUBLIC cxx_std_17)
target_include_directories(franka_server_node PRIVATE include include/${PROJECT_NAME} src)
target_link_libraries(franka_server_node Eigen3::Eigen yaml-cpp)
ament_target_dependencies(franka_server_node rclcpp std_msgs realtime_tools)
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(franka_server_node "${cpp_typesupport_target}")

install(TARGETS franka_server_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})
ament_export_dependencies(rosidl_default_runtime)
ament_package()
