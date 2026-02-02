cmake_minimum_required(VERSION 3.0.2)
project(loco_planning)

find_package(catkin REQUIRED std_msgs message_generation COMPONENTS)



add_message_files(
  DIRECTORY
  msg
  FILES
  Reference.msg
)

generate_messages(DEPENDENCIES std_msgs)


catkin_package(
  CATKIN_DEPENDS rospy message_generation std_msgs
)

catkin_install_python(
  PROGRAMS
    # ONLY list scripts here that you run with "rosrun"
    scripts/controller.py
    scripts/spawn_controllers.py
    
    # REMOVE params.py (It is a library, not a node)
    # REMOVE lyapunov.py (Assuming this is imported by controller, not run alone)
    # REMOVE planner_base.py (Assuming this is a base class)
    # REMOVE planner_voronoi.py (Assuming this is imported)
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
  


install(DIRECTORY msg/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/msg
  PATTERN ".svn" EXCLUDE)


install(DIRECTORY scripts/utils
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  PATTERN ".svn" EXCLUDE)

install(DIRECTORY scripts/geometry
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  PATTERN ".svn" EXCLUDE)

install(DIRECTORY scripts/controllers
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  PATTERN ".svn" EXCLUDE)

install(DIRECTORY scripts/planners
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  PATTERN ".svn" EXCLUDE)
  
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  PATTERN ".svn" EXCLUDE)


install(DIRECTORY rviz/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/rviz
  PATTERN ".svn" EXCLUDE)

 install(DIRECTORY config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  PATTERN ".svn" EXCLUDE)
      

