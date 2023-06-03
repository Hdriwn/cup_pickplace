cmake_minimum_required(VERSION 3.1.3)
project(ur5_pack_moveit)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  moveit_core
  moveit_ros_planning_interface
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(pickandplace scripts/pickandplace.cpp)
target_link_libraries(pickandplace ${catkin_LIBRARIES})
install(TARGETS pickandplace DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

add_executable(pick_place_tutorial scripts/pick_place_tutorial.cpp)
target_link_libraries(pick_place_tutorial ${catkin_LIBRARIES} ${Boost_LIBRARIES})
install(TARGETS pick_place_tutorial DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
