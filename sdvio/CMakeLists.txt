cmake_minimum_required(VERSION 2.8.3)
project(sdvio)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++11")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")
set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  cmake_modules
  cv_bridge
  geometry_msgs
  image_transport
  nav_msgs
  camera_model
  sensor_msgs
  std_msgs
  tf
)

add_definitions("-DUSE_ROS")
add_definitions("-DENABLE_SSE")

find_package(OpenCV REQUIRED)
find_package(Ceres REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Sophus REQUIRED)
FIND_PACKAGE(fast REQUIRED)
find_package(Boost REQUIRED COMPONENTS thread system)

include_directories(
	include/sdvio
	${catkin_INCLUDE_DIRS} 
	${CERES_INCLUDE_DIRS}
	${OpenCV_INCLUDE_DIRS}
	${Boost_INCLUDE_DIRS}
	${EIGEN3_INCLUDE_DIR}
	${Sophus_INCLUDE_DIRS}
	${fast_INCLUDE_DIRS}
)

list(APPEND SOURCE_FILES
    ${PROJECT_SOURCE_DIR}/src/global.cpp
    ${PROJECT_SOURCE_DIR}/src/LiveSDVIO_Core.cpp
    ${PROJECT_SOURCE_DIR}/src/frameHandler.cpp
    ${PROJECT_SOURCE_DIR}/src/sparseImgAlign.cpp
    ${PROJECT_SOURCE_DIR}/src/depthFilter.cpp
    ${PROJECT_SOURCE_DIR}/src/data_structure/feature.cpp
    ${PROJECT_SOURCE_DIR}/src/data_structure/point.cpp
    ${PROJECT_SOURCE_DIR}/src/data_structure/frame.cpp
    ${PROJECT_SOURCE_DIR}/src/data_structure/map.cpp
    ${PROJECT_SOURCE_DIR}/src/data_structure/feature_manager.cpp
    ${PROJECT_SOURCE_DIR}/src/utilities/utility.cpp
    ${PROJECT_SOURCE_DIR}/src/feature_detector.cpp
    ${PROJECT_SOURCE_DIR}/src/matcher.cpp
    ${PROJECT_SOURCE_DIR}/src/feature_alignment.cpp
    ${PROJECT_SOURCE_DIR}/src/initialization.cpp
)

# Set link libraries
list(APPEND LINK_LIBS 
  ${OpenCV_LIBS}
  ${Sophus_LIBRARIES}
  ${Boost_LIBRARIES}
  ${catkin_LIBRARIES}
  ${CERES_LIBRARIES}
  ${fast_LIBRARIES}
)                         

catkin_package()

add_library(sdvio SHARED ${SOURCE_FILES})
add_dependencies(sdvio ${${PROJECT_NAME}_EXPORTED_TARGETS}})
target_link_libraries(sdvio ${LINK_LIBS})

add_executable(sdvio_node src/sdvio_node.cpp)
target_link_libraries(sdvio_node sdvio)


