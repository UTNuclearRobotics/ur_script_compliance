cmake_minimum_required(VERSION 2.8.3)
project(ur_script_compliance)


add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
)

catkin_package(
  CATKIN_DEPENDS
    roscpp
  DEPENDS
)

###########
## Build ##
###########

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(ur_script_compliance src/ur_script_compliance.cpp)
add_dependencies(ur_script_compliance ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} )
target_link_libraries(ur_script_compliance ${catkin_LIBRARIES} )


