cmake_minimum_required(VERSION 3.8)
project(msg_interface)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies : 사용하는 외부 패키지 작성
################################################################################
# Find and load build settings from external packages
################################################################################
find_package(ament_cmake REQUIRED) 
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)
################################################################################
# Declare ROS messages, services and actions
################################################################################

# Message(msg/.msg)
set(msg_files
  "msg/ArucoDetection.msg"
)

# Service(srv/.srv)
set(srv_files
  "srv/SetDetectTarget.srv"
)

# Action
set(action_files
  "action/FindItem.action"
  "action/GoToId.action"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}    # message
  ${srv_files}    # service
  ${action_files} # action
  DEPENDENCIES builtin_interfaces
) 

################################################################################
# Macro for ament package
################################################################################
ament_export_dependencies(rosidl_default_runtime geometry_msgs)
ament_package()

