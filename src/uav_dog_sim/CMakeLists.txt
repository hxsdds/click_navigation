cmake_minimum_required(VERSION 3.8)
project(uav_dog_sim)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY launch worlds
  DESTINATION share/${PROJECT_NAME}
)

install(FILES ../../config/gz_server.config DESTINATION share/${PROJECT_NAME}/config)

ament_package()
