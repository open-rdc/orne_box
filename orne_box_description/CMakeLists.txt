cmake_minimum_required(VERSION 2.8.3)
project(orne_box_description)

find_package(catkin REQUIRED COMPONENTS
  icart_mini_description
  xacro
)

catkin_package()

install(DIRECTORY meshes rviz urdf
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

