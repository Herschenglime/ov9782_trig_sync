cmake_minimum_required(VERSION 3.8)
project(ov9782_trig_sync)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

find_path(GPIOD_INCLUDE_DIR gpiod.h)
find_library(GPIOD_LIBRARY NAMES gpiod)

if(NOT GPIOD_INCLUDE_DIR OR NOT GPIOD_LIBRARY)
  message(FATAL_ERROR "libgpiod not found. Install libgpiod-dev on target system.")
endif()

add_executable(trig_sync_node src/trig_sync_node.cpp)
target_include_directories(trig_sync_node PRIVATE ${GPIOD_INCLUDE_DIR})
target_link_libraries(trig_sync_node ${GPIOD_LIBRARY})
ament_target_dependencies(trig_sync_node rclcpp sensor_msgs)

install(TARGETS
  trig_sync_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  option(OV9782_TRIG_SYNC_ENABLE_LINT "Enable ament lint checks" OFF)
  if(OV9782_TRIG_SYNC_ENABLE_LINT)
    find_package(ament_lint_auto REQUIRED)
    ament_lint_auto_find_test_dependencies()
  endif()
endif()

ament_package()
