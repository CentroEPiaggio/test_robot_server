cmake_minimum_required(VERSION 3.16)
project(server_benchmarks)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(realtime_tools REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(thunder_franka REQUIRED)
find_package(franka_server REQUIRED)

foreach(node bench_server_node bench_client_node load_subscriber_node)
  add_executable(${node} src/${node}.cpp)
  target_compile_features(${node} PUBLIC cxx_std_17)
  target_link_libraries(${node} Eigen3::Eigen thunder_franka::thunder_franka)
  ament_target_dependencies(${node} rclcpp realtime_tools franka_server)
  install(TARGETS ${node} DESTINATION lib/${PROJECT_NAME})
endforeach()

add_executable(eval_benchmark src/eval_benchmark.cpp)
target_compile_features(eval_benchmark PUBLIC cxx_std_17)
target_link_libraries(eval_benchmark Eigen3::Eigen thunder_franka::thunder_franka)
install(TARGETS eval_benchmark DESTINATION lib/${PROJECT_NAME})

install(PROGRAMS scripts/run_benchmarks.bash scripts/summarise_benchmarks.py
  DESTINATION lib/${PROJECT_NAME})
ament_package()
