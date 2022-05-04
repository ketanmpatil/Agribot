cmake_minimum_required(VERSION 3.1.3)
project(agri_bot_moveit)


find_package(catkin REQUIRED COMPONENTS
roscpp
rospy
std_msgs
geometry_msgs
message_generation
actionlib
moveit_core 
moveit_ros_planning 
moveit_ros_planning_interface 
moveit_msgs
tf2
tf2_eigen
tf2_geometry_msgs
tf2_kdl
tf2_bullet
tf2_msgs
tf2_py
tf2_ros
tf2_sensor_msgs
tf2_tools
tf
)

catkin_package()

include_directories(${catkin_INCLUDE_DIRS}
${Boost_INCLUDE_DIRS}
${GSTREAMER_INCLUDE_DIRS}
)
add_executable(manipulation src/manipulation.cpp)
add_dependencies(manipulation ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(manipulation
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)


install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  PATTERN "setup_assistant.launch" EXCLUDE)
install(DIRECTORY config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(TARGETS manipulation
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})


# cmake_minimum_required(VERSION 3.1.3)
# project(agri_bot_moveit)

# find_package(catkin REQUIRED
# roscpp
# rospy
# std_msgs
# geometry_msgs
# message_generation
# actionlib
# moveit_core 
# moveit_ros_planning 
# moveit_ros_planning_interface 
# moveit_msgs
# tf2
# tf2_eigen
# tf2_geometry_msgs
# tf2_kdl
# tf2_bullet
# tf2_msgs
# tf2_py
# tf2_ros
# tf2_sensor_msgs
# tf2_tools
# tf
# )
# include_directories(${catkin_INCLUDE_DIRS})


# add_executable(agri_bot_moveit src/manipulation.cpp)

#  add_dependencies(manipulation.cpp ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
 
# target_link_libraries(agri_bot_moveit ${catkin_LIBRARIES})
# catkin_package()

# install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
#   PATTERN "setup_assistant.launch" EXCLUDE)
# install(DIRECTORY config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
