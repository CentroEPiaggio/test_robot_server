cmake_minimum_required(VERSION 3.16)
project(thunder_franka)

# Release by default: the control loop runs at 1 kHz and the CasADi generated
# code is the single most expensive thing in it.
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)

add_library(${PROJECT_NAME} SHARED
  src/thunder_franka.cpp
  src/franka_gen.cpp
)
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)

# Consumers include <thunder_franka/thunder_franka.h>; the generated sources
# include the headers flatly ("thunder_franka.h", "franka_gen.h"), hence the
# private entry.
target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include/${PROJECT_NAME}
)
target_link_libraries(${PROJECT_NAME} PUBLIC Eigen3::Eigen yaml-cpp)

# franka_gen.cpp is a ~70 MB single translation unit of straight-line CasADi
# code. -O2 keeps the evaluation fast without the compile-time/memory blow-up
# that -O3 causes on a file this size.
set_source_files_properties(src/franka_gen.cpp PROPERTIES COMPILE_OPTIONS "-O2")

install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(Eigen3 yaml-cpp)
ament_package()
