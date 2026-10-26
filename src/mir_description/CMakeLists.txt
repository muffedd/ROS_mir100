cmake_minimum_required(VERSION 3.8)
project(mir_description)

find_package(ament_cmake REQUIRED)

install(DIRECTORY
  launch
  meshes
  rviz
  urdf
  worlds
  DESTINATION share/${PROJECT_NAME}
)

install(FILES
  model.config
  model.sdf
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
