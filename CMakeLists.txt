cmake_minimum_required(VERSION 3.0.2)
project(bpmp_tracker)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -std=c++14")
set(CMAKE_BUILD_TYPE Release)
#set(CMAKE_BUILD_TYPE Debug)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  decomp_util
  decomp_ros_utils
  tf
  pcl_conversions
)

include_directories(include ${catkin_INCLUDE_DIRS} ${DECOMP_UTIL_INCLUDE_DIRS})
find_package(Boost REQUIRED COMPONENTS thread)
include_directories(${Boost_INCLUD_DIRS})

add_message_files(FILES ControlInput.msg ControlInputList.msg ObjectState.msg ObjectStateList.msg PolyState.msg RobotState.msg)
generate_messages(DEPENDENCIES std_msgs)

catkin_package(
	LIBRARIES
	INCLUDE_DIRS include
	CATKIN_DEPENDS
	std_msgs
	message_runtime
)

file(GLOB BPMP_UTILS_SRC src/bpmp_utils/BernsteinUtils.cpp src/bpmp_utils/Utils.cpp)
file(GLOB BPMP_SIMULATOR_SRC src/bpmp_simulator/*.cpp)
file(GLOB BPMP_PREDICTOR_SRC src/bpmp_predictor/*.cpp exe/main_predictor.cpp)

add_executable(bpmp_simulator exe/main_simulator.cpp ${BPMP_SIMULATOR_SRC} ${BPMP_UTILS_SRC})
target_link_libraries(bpmp_simulator ${catkin_LIBRARIES} -lstdc++fs)

add_executable(bpmp_ros_type_converter exe/main_type_converter.cpp ${BPMP_SIMULATOR_SRC} ${BPMP_UTILS_SRC})
target_link_libraries(bpmp_ros_type_converter ${catkin_LIBRARIES} -lstdc++fs)


add_executable(bpmp_predictor ${BPMP_PREDICTOR_SRC} ${BPMP_UTILS_SRC})
target_link_libraries(bpmp_predictor ${catkin_LIBRARIES} -lstdc++fs)