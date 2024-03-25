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

# TODO SEE if this works
# PATH "/usr/local/include/eigen3/"
# https://eigen.tuxfamily.org/dox/TopicCMakeGuide.html
find_package(Eigen3 3.4.90 REQUIRED NO_MODULE)
include_directories( EIGEN3_INCLUDE_DIR )
# # find_package(Eigen3 3.4 REQUIRED NO_MODULE)
# # find_package(Eigen3 3.4 NO_MODULE)
# # if (TARGET Eigen3::Eigen)
# #   message("Boo")
# # endif (TARGET Eigen3::Eigen)
# message("${Eigen3_INCLUDE_DIRS}")
# message("${Eigen_LIBRARIES}")

find_package(OpenCV REQUIRED)
find_package(Boost REQUIRED)


SET(qhullDir "/usr/local/")
INCLUDE_DIRECTORIES(${qhullDir}/include/libqhullcpp)
INCLUDE_DIRECTORIES(${qhullDir}/include/libqhull_r)
# SET(qhullDir "/home/drc3203/thirdparty/qhull-2020.2/")
# SET(qhullDir "/home/drc3203/thirdparty/qhull/")
# INCLUDE_DIRECTORIES(${qhullDir}/src/libqhullcpp)
# INCLUDE_DIRECTORIES(${qhullDir}/src/libqhull_r)
LINK_DIRECTORIES(${qhullDir}/lib)
SET(qhullLibs qhullcpp qhull_r)

INCLUDE_DIRECTORIES(include include/urquhart include/sim include/my_urq)

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

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(ultimate_path src/ultimate_path.cpp)
target_link_libraries(ultimate_path ${catkin_LIBRARIES})

add_executable(KeyframeGen
    src/keyframe_maker.cpp
    src/observation.cpp
    src/tree.cpp
    src/descriptor.cpp
    src/matching.cpp
    )
target_include_directories(KeyframeGen PUBLIC include ${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS} ${Boost_INCLUDE_DIR})
target_link_libraries(KeyframeGen PUBLIC ${Boost_LIBRARIES} ${OpenCV_LIBS} ${qhullLibs} ${catkin_LIBRARIES})
SET_TARGET_PROPERTIES(KeyframeGen PROPERTIES COMPILE_DEFINITIONS "qh_QHpointer")

# TODO when making graph_builder or any executable with the new eigen version, make sure to link the library catkin found above (put it up front):
# target_link_libraries (example Eigen3::Eigen)

# INCLUDE_DIRECTORIES(/usr/local/include/eigen3)
# LINK_DIRECTORIES(/usr/local/include/eigen3)
add_executable(Stu src/stupid.cpp)

# target_link_libraries(Stu Eigen3::Eigen ${qhullLibs})

# target_link_libraries(Stu ${Eigen3_LIBRARIES})
# SET_TARGET_PROPERTIES(Stu PROPERTIES COMPILE_DEFINITIONS "qh_QHpointer")

target_include_directories(UrquhartExample PUBLIC include ${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS} ${Boost_INCLUDE_DIR})
# target_link_libraries(UrquhartExample PUBLIC ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${OpenCV_LIBS} ${qhullLibs})
target_link_libraries(UrquhartExample PUBLIC ${Boost_LIBRARIES} ${OpenCV_LIBS} ${qhullLibs} ${catkin_LIBRARIES})
SET_TARGET_PROPERTIES(UrquhartExample PROPERTIES COMPILE_DEFINITIONS "qh_QHpointer")

target_include_directories(ForestSim PUBLIC include ${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS} ${Boost_INCLUDE_DIR})
target_link_libraries(ForestSim PUBLIC ${Boost_LIBRARIES} ${OpenCV_LIBS} ${qhullLibs} ${catkin_LIBRARIES})
SET_TARGET_PROPERTIES(ForestSim PROPERTIES COMPILE_DEFINITIONS "qh_QHpointer")


add_executable(ObsRanger
    src/obs_range_testing.cpp
    src/observation.cpp
    src/tree.cpp
    src/descriptor.cpp
    src/matching.cpp
    )
target_include_directories(ObsRanger PUBLIC include ${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS} ${Boost_INCLUDE_DIR})
target_link_libraries(ObsRanger PUBLIC ${Boost_LIBRARIES} ${OpenCV_LIBS} ${qhullLibs} ${catkin_LIBRARIES})
SET_TARGET_PROPERTIES(ObsRanger PROPERTIES COMPILE_DEFINITIONS "qh_QHpointer")

# TODO compile my version of urquhart and try the example.cpp executable with my backend
# use my backend for normal observations, make sure behavior is same as vanilla
# begin incremental construction code


add_executable(NewBackendExample
    example/new_example.cpp
    src/newthings/hype_observation.cpp  
    src/newthings/hype_tree.cpp  
    src/newthings/hype_descriptor.cpp  
    src/newthings/hype_matching.cpp  
    )
target_include_directories(NewBackendExample PUBLIC include ${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS} ${Boost_INCLUDE_DIR})
target_link_libraries(NewBackendExample PUBLIC ${Boost_LIBRARIES} ${OpenCV_LIBS} ${qhullLibs} ${catkin_LIBRARIES})
SET_TARGET_PROPERTIES(NewBackendExample PROPERTIES COMPILE_DEFINITIONS "qh_QHpointer")