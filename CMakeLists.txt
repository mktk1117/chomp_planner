cmake_minimum_required(VERSION 2.8.3)
project(chomp_planner)

find_package(catkin_simple REQUIRED)
find_package(Boost REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)
set(CMAKE_BUILD_TYPE Release)

add_definitions(-std=c++11)
#############
# LIBRARIES #
#############
cs_add_library(${PROJECT_NAME}_library
  src/ChompPlannerBase.cpp
  src/SimpleChompPlanner.cpp
  src/ChompPlannerNode.cpp
)

############
# BINARIES #
############
cs_add_executable(${PROJECT_NAME}_node
  src/chomp_planner_node.cpp)
add_library(${PROJECT_NAME}_nodelet SHARED
  src/chomp_planner_nodelet.cc)
target_link_libraries(${PROJECT_NAME}_node ${PROJECT_NAME}_library)
target_link_libraries(
  ${PROJECT_NAME}_nodelet 
  ${PROJECT_NAME}_library
  ${catkin_LIBRARIES}
  )

##########
# EXPORT #
##########
cs_install()
cs_export()


