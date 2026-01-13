cmake_minimum_required(VERSION 3.0.2)
project(bpmp_tracker)

#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -std=c++17")
#set(CMAKE_BUILD_TYPE Release)
##set(CMAKE_BUILD_TYPE Debug)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -std=c++17")
set(CMAKE_POSITION_INDEPENDENT_CODE ON)
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fPIE")
set(CMAKE_BUILD_TYPE Release)

find_package(Eigen3  REQUIRED NO_MODULE)
include_directories(${EIGEN3_INCLUDE_DIRS})

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  decomp_util
  decomp_ros_utils
  tf
  pcl_conversions
  message_filters
  geometry_msgs
  nav_msgs
)

include_directories(include ${catkin_INCLUDE_DIRS} ${DECOMP_UTIL_INCLUDE_DIRS})
include_directories(${EIGEN3_INCLUDE_DIRS})
find_package(Boost REQUIRED COMPONENTS thread)
include_directories(${Boost_INCLUD_DIRS})

find_package(PCL REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_message_files(FILES ControlInput.msg ControlInputList.msg ObjectState.msg ObjectStateList.msg PolyState.msg RobotState.msg UnicycleInput.msg UnicycleState.msg)
generate_messages(DEPENDENCIES std_msgs)

catkin_package(
	LIBRARIES
	INCLUDE_DIRS include
	CATKIN_DEPENDS
	std_msgs
	message_runtime
)

file(GLOB BPMP_UTILS_SRC src/bpmp_utils/BernsteinUtils.cpp src/bpmp_utils/Utils.cpp)
file(GLOB BPMP_SIMULATOR_SRC src/bpmp_simulator/Simulator.cpp)
file(GLOB BPMP_ROSTYPE_SRC src/bpmp_simulator/RosTypeConverter.cpp)
file(GLOB BPMP_PREDICTOR_SRC src/bpmp_predictor/*.cpp exe/main_predictor.cpp)
file(GLOB BPMP_TRACKER_SRC src/bpmp_tracker/Wrapper.cpp src/bpmp_tracker/PlannerBase.cpp src/bpmp_tracker/Tracker.cpp src/bpmp_tracker/PlanningVisualizer.cpp src/bpmp_tracker/RosWrapper.cpp )
file(GLOB BPMP_COMPARISON_SRC src/comparison/*.cpp)
file(GLOB BPMP_BASELINE_SRC src/comparison/Baseline.cpp)


add_executable(bpmp_simulator exe/main_simulator.cpp ${BPMP_SIMULATOR_SRC} ${BPMP_UTILS_SRC})
target_link_libraries(bpmp_simulator ${catkin_LIBRARIES} -lstdc++fs)

add_executable(bpmp_ros_type_converter exe/main_type_converter.cpp ${BPMP_ROSTYPE_SRC} ${BPMP_UTILS_SRC})
target_link_libraries(bpmp_ros_type_converter ${catkin_LIBRARIES} -lstdc++fs)

add_executable(bpmp_predictor ${BPMP_PREDICTOR_SRC} ${BPMP_UTILS_SRC})
target_link_libraries(bpmp_predictor ${catkin_LIBRARIES} -lstdc++fs)

add_executable(bpmp_tracker exe/main_tracker.cpp ${BPMP_TRACKER_SRC} ${BPMP_UTILS_SRC})
target_link_libraries(bpmp_tracker ${catkin_LIBRARIES} -lstdc++fs)

add_executable(comparison_controller exe/main_tracking_controller.cpp ${BPMP_COMPARISON_SRC} ${BPMP_UTILS_SRC})
target_link_libraries(comparison_controller ${catkin_LIBRARIES} -lstdc++fs)

add_executable(baseline exe/main_baseline.cpp ${BPMP_COMPARISON_SRC} ${BPMP_UTILS_SRC})
target_link_libraries(baseline ${catkin_LIBRARIES} -lstdc++fs)

add_dependencies(comparison_controller ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(bpmp_ros_type_converter ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(bpmp_tracker ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(bpmp_simulator ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(bpmp_predictor ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

install(TARGETS
 bpmp_predictor
 bpmp_simulator
 bpmp_ros_type_converter
 bpmp_tracker
 comparison_controller
 DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

install(DIRECTORY param/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/param
)

install(DIRECTORY world/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/world
)