cmake_minimum_required(VERSION 3.1)
project(urquhart)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)


find_package(PCL 1.14.0 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})


find_package(catkin REQUIRED COMPONENTS
  sensor_msgs
  std_msgs
  roscpp
  geometry_msgs
  roslib
#   pcl_conversions
#   pcl_ros
)
# Need roslib in order to use ros::package?

# find_package(Eigen3 3.4.90 REQUIRED NO_MODULE)
# include_directories( EIGEN3_INCLUDE_DIR )

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

INCLUDE_DIRECTORIES(include include/urquhart include/sim)

catkin_package(
   INCLUDE_DIRS
   include
   LIBRARIES
   ${PROJECT_NAME}
   CATKIN_DEPENDS
   geometry_msgs
#    pcl_ros
#    pcl_conversions
   roscpp
   roslib
   DEPENDS)





# Non ROS robot path simulation
add_executable(ForestSim
    src/my_old/path.cpp
    src/urquhart/observation.cpp
    src/urquhart/tree.cpp
    src/urquhart/descriptor.cpp
    src/urquhart/matching.cpp
    )
target_include_directories(ForestSim PUBLIC include ${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS} ${Boost_INCLUDE_DIR})
target_link_libraries(ForestSim PUBLIC ${Boost_LIBRARIES} ${OpenCV_LIBS} ${qhullLibs} ${catkin_LIBRARIES})
SET_TARGET_PROPERTIES(ForestSim PROPERTIES COMPILE_DEFINITIONS "qh_QHpointer")


# Non ROS forest generator
add_executable(CreateForest src/my_old/poisson_disk_sampling.cpp)


# include_directories(include ${catkin_INCLUDE_DIRS})
# Robot Path Simulation ROS Node
add_executable(ultimate_path src/ultimate_path.cpp)
target_link_libraries(ultimate_path ${PCL_LIBRARIES} ${catkin_LIBRARIES})
target_include_directories(ultimate_path PUBLIC include ${catkin_INCLUDE_DIRS})


# Garbage test program
add_executable(Stu src/stupid.cpp)
# target_link_libraries(Stu ${qhullLibs})
# SET_TARGET_PROPERTIES(Stu PROPERTIES COMPILE_DEFINITIONS "qh_QHpointer")


# Match observations with varying sensor ranges
add_executable(ObsRanger
    src/obs_range_testing.cpp
    src/urquhart/observation.cpp
    src/urquhart/tree.cpp
    src/urquhart/descriptor.cpp
    src/urquhart/matching.cpp
    )
target_include_directories(ObsRanger PUBLIC include ${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS} ${Boost_INCLUDE_DIR})
target_link_libraries(ObsRanger PUBLIC ${Boost_LIBRARIES} ${OpenCV_LIBS} ${qhullLibs} ${catkin_LIBRARIES})
SET_TARGET_PROPERTIES(ObsRanger PROPERTIES COMPILE_DEFINITIONS "qh_QHpointer")



# Geometric Hierarchy Example Construction and Matching (Original example)
add_executable(UrquhartExample
    example/example.cpp
    src/urquhart/observation.cpp  
    src/urquhart/tree.cpp  
    src/urquhart/descriptor.cpp  
    src/urquhart/matching.cpp  
    )
target_include_directories(UrquhartExample PUBLIC include ${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS} ${Boost_INCLUDE_DIR})
target_link_libraries(UrquhartExample PUBLIC ${Boost_LIBRARIES} ${OpenCV_LIBS} ${qhullLibs} ${catkin_LIBRARIES})
SET_TARGET_PROPERTIES(UrquhartExample PROPERTIES COMPILE_DEFINITIONS "qh_QHpointer")

# Geometric Hierarchy Example Construction and Matching for Two Simulated Observations
add_executable(MyExample
    example/other_form_example.cpp
    src/urquhart/observation.cpp  
    src/urquhart/tree.cpp  
    src/urquhart/descriptor.cpp  
    src/urquhart/matching.cpp  
    )
target_include_directories(MyExample PUBLIC include ${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS} ${Boost_INCLUDE_DIR})
target_link_libraries(MyExample PUBLIC ${Boost_LIBRARIES} ${OpenCV_LIBS} ${qhullLibs} ${catkin_LIBRARIES})
SET_TARGET_PROPERTIES(MyExample PROPERTIES COMPILE_DEFINITIONS "qh_QHpointer")



# Local Bundle Adjustment Node
# PCL takes a minute to compile, so only uncomment when actively developing
# add_executable(KeyframeGen
#     src/keyframe_maker.cpp
#     src/urquhart/observation.cpp
#     src/urquhart/tree.cpp
#     src/urquhart/descriptor.cpp
#     src/urquhart/matching.cpp
#     )
# target_include_directories(KeyframeGen PUBLIC include ${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS} ${Boost_INCLUDE_DIR})
# # target_link_libraries(KeyframeGen PUBLIC ${Boost_LIBRARIES} ${OpenCV_LIBS} ${qhullLibs} ${catkin_LIBRARIES})
# target_link_libraries(KeyframeGen PUBLIC ${PCL_LIBRARIES} ${Boost_LIBRARIES} ${OpenCV_LIBS} ${qhullLibs} ${catkin_LIBRARIES})
# SET_TARGET_PROPERTIES(KeyframeGen PROPERTIES COMPILE_DEFINITIONS "qh_QHpointer")


# GraphSLAM Node
# PCL takes a minute to compile, so only uncomment when actively developing
add_executable(PolygonSLAM
    src/graph_builder.cpp
    src/urquhart/observation.cpp
    src/urquhart/tree.cpp
    src/urquhart/descriptor.cpp
    src/urquhart/matching.cpp
    )
target_include_directories(PolygonSLAM PUBLIC include ${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS} ${Boost_INCLUDE_DIR})
target_link_libraries(PolygonSLAM PUBLIC ${PCL_LIBRARIES} ${Boost_LIBRARIES} ${OpenCV_LIBS} ${qhullLibs} ${catkin_LIBRARIES})
SET_TARGET_PROPERTIES(PolygonSLAM PROPERTIES COMPILE_DEFINITIONS "qh_QHpointer")
