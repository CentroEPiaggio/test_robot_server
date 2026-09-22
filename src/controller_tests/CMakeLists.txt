cmake_minimum_required(VERSION 3.16)
project(controller_tests)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(mact_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

add_library(lissajous_trajectory src/lissajous_trajectory.cpp)
target_compile_features(lissajous_trajectory PUBLIC cxx_std_17)
target_include_directories(lissajous_trajectory PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(lissajous_trajectory PUBLIC Eigen3::Eigen)

add_executable(trajectory_publisher_node src/trajectory_publisher_node.cpp)
target_link_libraries(trajectory_publisher_node lissajous_trajectory)
ament_target_dependencies(trajectory_publisher_node
  rclcpp mact_msgs sensor_msgs std_srvs trajectory_msgs)

install(TARGETS lissajous_trajectory
  ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION bin)
install(TARGETS trajectory_publisher_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})
install(PROGRAMS
  scripts/analyse_run.py
  scripts/plot_abstract_figure.py
  scripts/thunder_ctypes.py
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_lissajous_trajectory test/test_lissajous_trajectory.cpp)
  target_link_libraries(test_lissajous_trajectory lissajous_trajectory)
endif()

ament_package()
