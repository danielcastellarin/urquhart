cmake_minimum_required(VERSION 3.1)
project(urquhart)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(catkin REQUIRED COMPONENTS
  sensor_msgs
  std_msgs
  roscpp
  pcl_conversions
  pcl_ros
)

find_package(OpenCV REQUIRED)
find_package(Boost REQUIRED)

SET(qhullDir "/usr/local/")
INCLUDE_DIRECTORIES(${qhullDir}/include/libqhullcpp)
INCLUDE_DIRECTORIES(${qhullDir}/include/libqhull_r)
LINK_DIRECTORIES(${qhullDir}/lib)
SET(qhullLibs qhullcpp qhull_r)

INCLUDE_DIRECTORIES(include include/urquhart include/sim)

catkin_package(
   INCLUDE_DIRS
   include
   LIBRARIES
   ${PROJECT_NAME}
   CATKIN_DEPENDS
   pcl_ros
   pcl_conversions
   roscpp
   DEPENDS)

add_executable(UrquhartExample
    example/example.cpp
    src/observation.cpp  
    src/tree.cpp  
    src/descriptor.cpp  
    src/matching.cpp  
    )

add_executable(ForestSim
    src/old/path.cpp
    src/observation.cpp
    src/tree.cpp
    src/descriptor.cpp
    src/matching.cpp
    )

add_executable(CreateForest src/old/poisson_disk_sampling.cpp)

add_executable(Homog src/homography.cpp)

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(ultimate_path src/ultimate_path.cpp)
target_link_libraries(ultimate_path ${catkin_LIBRARIES})

target_include_directories(UrquhartExample PUBLIC include ${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS} ${Boost_INCLUDE_DIR})
target_link_libraries(UrquhartExample PUBLIC ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${OpenCV_LIBS} ${qhullLibs})
SET_TARGET_PROPERTIES(UrquhartExample PROPERTIES COMPILE_DEFINITIONS "qh_QHpointer")

target_include_directories(ForestSim PUBLIC include ${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS} ${Boost_INCLUDE_DIR})
target_link_libraries(ForestSim PUBLIC ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${OpenCV_LIBS} ${qhullLibs})
SET_TARGET_PROPERTIES(ForestSim PROPERTIES COMPILE_DEFINITIONS "qh_QHpointer")
