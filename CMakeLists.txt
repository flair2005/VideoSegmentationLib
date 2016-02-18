cmake_minimum_required(VERSION 2.6)
project (VideoSegmentation)

set(CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake/Modules/" ${CMAKE_MODULE_PATH})

#set(OpenCV_DIR "~/workspace/opencv/build")
find_package( OpenCV 2.4 REQUIRED   ml core gpu highgui)
#set(PCL_DIR "~/workspace/pcl/build/")
find_package(PCL REQUIRED)
include_directories(${PCL_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})
#add_definitions(${PCL_DEFINITIONS})


find_package(Boost REQUIRED graph program_options)
find_package(GraphViz REQUIRED)


MESSAGE( STATUS "CMAKE at VisualRepresentation")


add_definitions(-Wall -g  -fopenmp -lgomp -march=native -std=c++11 -O4)

#link the Segmentation lib
set(SEG_LIB_PATH /home/martin/workspace/SegmentationLib)
ADD_LIBRARY(segLib SHARED IMPORTED)
SET_TARGET_PROPERTIES(segLib PROPERTIES
    IMPORTED_LOCATION ${SEG_LIB_PATH}/build/libSEGMENTATION.so)
include_directories(${SEG_LIB_PATH}/include )

#MRSmap
#set(MRSmap_LIB_PATH /home/martin/workspace/mrsmap/)
#ADD_LIBRARY(mrsmapLib SHARED IMPORTED)
#SET_TARGET_PROPERTIES(mrsmapLib PROPERTIES
    #IMPORTED_LOCATION ${MRSmap_LIB_PATH}/build/devel/lib/libmrsmap.so)
#include_directories(${MRSmap_LIB_PATH}/include)

#MRSSlam
#set(MRSslam_LIB_PATH /home/martin/workspace/mrsmap/)
#ADD_LIBRARY(mrsslamLib SHARED IMPORTED)
#SET_TARGET_PROPERTIES(mrsslamLib PROPERTIES
    #IMPORTED_LOCATION ${MRSslam_LIB_PATH}/build/devel/lib/libmrsslam.so)
#include_directories(${MRSslam_LIB_PATH}/include)


# OctreeLib required
#find_package(OctreeLib REQUIRED)
#include_directories(${OCTREELIB_INCLUDE_DIRS})

# G2O required
#find_package(G2O REQUIRED)
#include_directories(${G2O_INCLUDE_DIR})

# GSL required
#find_package(GSL REQUIRED)
#include_directories(${GSL_INCLUDE_DIRS})
#link_directories(${GSL_LIBRARY_DIRS})

# TBB required
#find_package(TBB REQUIRED)
#include_directories(${TBB_INCLUDE_DIRS})

# For building the CHOLMOD / CSPARSE solvers
#find_package(Cholmod)
#find_package(BLAS)
#find_package(LAPACK)
#find_package(CSparse)
#include_directories(${CSPARSE_INCLUDE_DIR})


set(PROJECT_SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/src)
set(PROJECT_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/include)

#Include Required Directories
include_directories("${PROJECT_INCLUDE_DIR}")
include_directories("${PROJECT_SOURCE_DIR}")
include_directories("${PROJECT_BINARY_DIR}")

# Create a shared library
add_library(VR_LIBRARY SHARED
${PROJECT_SOURCE_DIR}/VideoSegmentation.cpp
${PROJECT_SOURCE_DIR}/GraphsMatch.cpp
${PROJECT_SOURCE_DIR}/visual_representation.cpp
${PROJECT_SOURCE_DIR}/atom.cpp
${PROJECT_SOURCE_DIR}/visualisation.cpp
${PROJECT_SOURCE_DIR}/hungarian/hungarian.c
)


#add_executable (test src/testgraphs.cpp)
#target_link_libraries (test VR_LIBRARY ${GRAPHVIZ_CDT_LIBRARY} ${GRAPHVIZ_GVC_LIBRARY} ${GRAPHVIZ_CGRAPH_LIBRARY}  segLib mrsmapLib ${Boost_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBS})

#add_executable (compare src/compare.cpp)
#target_link_libraries (compare VR_LIBRARY  ${GRAPHVIZ_CDT_LIBRARY} ${GRAPHVIZ_GVC_LIBRARY} ${GRAPHVIZ_CGRAPH_LIBRARY}  segLib mrsmapLib  ${Boost_LIBRARIES}  ${PCL_LIBRARIES} ${OpenCV_LIBS})

#add_executable (track src/track_sequence.cpp)
#target_link_libraries (track VR_LIBRARY   ${GRAPHVIZ_CDT_LIBRARY} ${GRAPHVIZ_GVC_LIBRARY} ${GRAPHVIZ_CGRAPH_LIBRARY}  segLib mrsmapLib  ${Boost_LIBRARIES}  ${PCL_LIBRARIES} ${OpenCV_LIBS})

#add_executable (testinterface src/testinterface.cpp)
#target_link_libraries (testinterface VR_LIBRARY   ${GRAPHVIZ_CDT_LIBRARY} ${GRAPHVIZ_GVC_LIBRARY} ${GRAPHVIZ_CGRAPH_LIBRARY}  segLib mrsmapLib  ${Boost_LIBRARIES}  ${PCL_LIBRARIES} ${OpenCV_LIBS})

add_executable (train_detector src/train_object_sequence.cpp)
target_link_libraries (train_detector VR_LIBRARY   ${GRAPHVIZ_CDT_LIBRARY} ${GRAPHVIZ_GVC_LIBRARY} ${GRAPHVIZ_CGRAPH_LIBRARY} segLib  ${Boost_LIBRARIES}  ${PCL_LIBRARIES} ${OpenCV_LIBS} )#mrsmapLib mrsslamLib)

#add_executable (eff_train_detector src/efficient_train_object_sequence.cpp)
#target_link_libraries (eff_train_detector VR_LIBRARY   ${GRAPHVIZ_CDT_LIBRARY} ${GRAPHVIZ_GVC_LIBRARY} ${GRAPHVIZ_CGRAPH_LIBRARY}  segLib mrsmapLib  ${Boost_LIBRARIES}  ${PCL_LIBRARIES} ${OpenCV_LIBS})

add_executable (test_detector src/test_object_sequence.cpp)
target_link_libraries (test_detector VR_LIBRARY   ${GRAPHVIZ_CDT_LIBRARY} ${GRAPHVIZ_GVC_LIBRARY} ${GRAPHVIZ_CGRAPH_LIBRARY}  segLib  ${Boost_LIBRARIES}  ${PCL_LIBRARIES} ${OpenCV_LIBS} )#mrsmapLib mrsslamLib)

#add_executable (test_pcls src/test_pcls.cpp)
#target_link_libraries (test_pcls VR_LIBRARY   ${GRAPHVIZ_CDT_LIBRARY} ${GRAPHVIZ_GVC_LIBRARY} ${GRAPHVIZ_CGRAPH_LIBRARY}  segLib mrsmapLib  ${Boost_LIBRARIES}  ${PCL_LIBRARIES} ${OpenCV_LIBS})


#Continue with subdirectory
add_subdirectory(GUIMultiThread)
