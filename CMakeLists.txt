cmake_minimum_required(VERSION 2.8.3)
project(ur_script_interface)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
)

# What should we make available to other packages that depend on this one?
catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp
  DEPENDS
  LIBRARIES ur_script_interface
)

###########
## Build ##
###########

include_directories(include ${catkin_INCLUDE_DIRS})

add_library(ur_script_interface src/ur_script_interface.cpp)
add_dependencies(ur_script_interface ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} )
target_link_libraries(ur_script_interface ${catkin_LIBRARIES} )

add_executable(enable_compliance src/enable_compliance.cpp)
add_dependencies(enable_compliance ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} )
target_link_libraries(enable_compliance ${catkin_LIBRARIES} ur_script_interface )

add_executable(set_digital_output src/set_digital_output.cpp)
add_dependencies(set_digital_output ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} )
target_link_libraries(set_digital_output ${catkin_LIBRARIES} ur_script_interface )
