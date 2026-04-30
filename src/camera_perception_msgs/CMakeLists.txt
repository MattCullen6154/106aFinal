cmake_minimum_required(VERSION 3.8)
project(camera_perception_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(camera_perception_msgs REQUIRED)

rosidl_generate_interfaces(${ROBOT_WAITER}
  "msg/ObstacleState.msg"
  DEPENDENCIES std_msgs
)

rosidl_get_typesupport_target(cpp_type_support_target 
  "camera_perception_msgs" "rosidl_get_typesupport_cpp"
)

target_link_libraries(camera_perception "${cpp_type_support_target}")

ament_package()