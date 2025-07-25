cmake_minimum_required(VERSION 2.8.3)
project(topological_navigation_msgs)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS 
  message_generation 
  std_msgs 
  geometry_msgs
  actionlib_msgs
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/groovy/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

#######################################
## Declare ROS messages and services ##
#######################################

# Generate messages in the 'msg' folder
add_message_files(
  FILES
  ClosestEdges.msg
  TopologicalMap.msg
  TopologicalNode.msg
  Vertex.msg
  Edge.msg
  NavRoute.msg
  CurrentEdge.msg
  NavStatistics.msg
  TopologicalRoute.msg
  AddEdgeReq.msg
  AddNodeReq.msg
  SetInfluenceZoneReq.msg
  UpdateEdgeConfigReq.msg
)

#Generate services in the 'srv' folder
add_service_files(
  FILES
  WriteTopologicalMap.srv
  UpdateEdgeConfig.srv
  UpdateRestrictions.srv
  UpdateEdge.srv
  UpdateAction.srv
  RestrictMap.srv
  EvaluateNode.srv
  EvaluateEdge.srv
  AddDatum.srv
  GetTaggedNodes.srv
  GetTags.srv
  GetNodeTags.srv
  GetEdgesBetweenNodes.srv
  AddNode.srv
  RmvNode.srv
  AddEdge.srv
  AddContent.srv
  UpdateNodeName.srv
  UpdateNodeTolerance.srv
  ModifyTag.srv
  AddTag.srv
  GetTopologicalMap.srv
  UpdateEdgeLegacy.srv
  LocalisePose.srv
  GetRouteTo.srv
  GetRouteBetween.srv
  EstimateTravelTime.srv
  PredictEdgeState.srv
  UpdateFailPolicy.srv
  ReconfAtEdges.srv
  LoadTopoNavTestScenario.srv
  RunTopoNavTestScenario.srv
  SetInfluenceZone.srv
  AddEdgeArray.srv
  AddNodeArray.srv
  SetInfluenceZoneArray.srv
  UpdateEdgeConfigArray.srv
  SetVersion.srv
  InsertNode.srv
)



find_package(catkin REQUIRED genmsg actionlib_msgs)
add_action_files(
	DIRECTORY action 
	FILES 
	GotoNode.action
        ExecutePolicyMode.action
        BuildTopPrediction.action
)


## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  actionlib_msgs 
)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES strands_perception_people_msgs
  CATKIN_DEPENDS message_runtime
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})

## Declare a cpp library
# add_library(strands_perception_people_msgs
#   src/${PROJECT_NAME}/strands_perception_people_msgs.cpp
# )

## Declare a cpp executable
# add_executable(strands_perception_people_msgs_node src/strands_perception_people_msgs_node.cpp)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
# add_dependencies(strands_perception_people_msgs_node strands_perception_people_msgs_generate_messages_cpp)

## Specify libraries to link a library or executable target against
# target_link_libraries(strands_perception_people_msgs_node
#   ${catkin_LIBRARIES}
# )

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/groovy/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS strands_perception_people_msgs strands_perception_people_msgs_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_strands_perception_people_msgs.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
