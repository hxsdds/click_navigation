cmake_minimum_required(VERSION 3.5)

project(px4_msgs)

list(INSERT CMAKE_MODULE_PATH 0 "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
	add_compile_options(-Wall -Wextra)
endif()

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# ##############################################################################
# Generate ROS messages, ROS2 interfaces and IDL files #
# ##############################################################################

# get all msg files
set(MSGS_DIR "${CMAKE_CURRENT_SOURCE_DIR}/msg")
file(GLOB PX4_MSGS RELATIVE "${CMAKE_CURRENT_SOURCE_DIR}" "${MSGS_DIR}/*.msg")

# get all srv files
set(SRVS_DIR "${CMAKE_CURRENT_SOURCE_DIR}/srv")
file(GLOB PX4_SRVS RELATIVE "${CMAKE_CURRENT_SOURCE_DIR}" "${SRVS_DIR}/*.srv")

# The extracted demo uses six unmodified upstream wire definitions. Keep all
# source definitions available, but avoid generating hundreds of unused types
# during the default click-navigation build (especially on WSL mounted drives).
option(PX4_MSGS_CLICK_NAVIGATION_ONLY "Build interfaces required by click navigation" OFF)
if(PX4_MSGS_CLICK_NAVIGATION_ONLY)
  set(PX4_MSGS
    msg/OffboardControlMode.msg
    msg/TrajectorySetpoint.msg
    msg/VehicleCommand.msg
    msg/VehicleCommandAck.msg
    msg/VehicleLocalPosition.msg
    msg/VehicleStatus.msg)
  set(PX4_SRVS srv/VehicleCommand.srv)
endif()

# Generate introspection typesupport for C and C++ and IDL files
rosidl_generate_interfaces(${PROJECT_NAME}
	${PX4_MSGS}
	${PX4_SRVS}
	DEPENDENCIES builtin_interfaces
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
