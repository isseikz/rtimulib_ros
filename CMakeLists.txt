cmake_minimum_required(VERSION 3.5)
project(rtimulib_ros)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

link_directories(/usr/local/lib)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

add_executable(${PROJECT_NAME} src/${PROJECT_NAME}.cpp)
target_link_libraries(${PROJECT_NAME} RTIMULib)

ament_target_dependencies(${PROJECT_NAME}
  RTIMULib
  sensor_msgs
  rclcpp
  rclcpp_lifecycle
)

install(TARGETS ${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME}
) 

install(DIRECTORY config
  DESTINATION config
)

ament_package()
