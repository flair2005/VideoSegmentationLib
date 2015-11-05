cmake_minimum_required(VERSION 2.6)
project (VisualRepresentation)

#set(OpenCV_DIR "~/workspace/opencv/build")
find_package( OpenCV 2.4 REQUIRED)
set(PCL_DIR "~/workspace/pcl/build/")
find_package(PCL REQUIRED)
include_directories(${PCL_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})
#add_definitions(${PCL_DEFINITIONS})

set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR})
find_package(Boost REQUIRED graph)
find_package(GraphViz REQUIRED)



MESSAGE( STATUS "CMAKE at VisualRepresentation")


add_definitions(-Wall -g  -fopenmp -lgomp -march=native -std=c++11 -O4)

#link the Discrete Relaxation Lib
set(RELAX_LIB_PATH /home/martin/workspace/DiscreteRelaxation)
ADD_LIBRARY(relaxationLib STATIC IMPORTED)
SET_TARGET_PROPERTIES(relaxationLib PROPERTIES
    IMPORTED_LOCATION ${RELAX_LIB_PATH}/build/libSEGMENTATION.a)
include_directories(${RELAX_LIB_PATH}/src/include)

#link the Segmentation lib
set(SEG_LIB_PATH /home/martin/workspace/SegmentationLib)
ADD_LIBRARY(segLib SHARED IMPORTED)
SET_TARGET_PROPERTIES(segLib PROPERTIES
    IMPORTED_LOCATION ${SEG_LIB_PATH}/build/libSEGMENTATION.so)
include_directories(${SEG_LIB_PATH}/include )

set(PROJECT_SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/src)
set(PROJECT_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/include)

#Include Required Directories
include_directories("${PROJECT_INCLUDE_DIR}")
include_directories("${PROJECT_SOURCE_DIR}")
include_directories("${PROJECT_BINARY_DIR}")

# Create a shared library
add_library(VR_LIBRARY
${PROJECT_SOURCE_DIR}/GraphsMatch.cpp
${PROJECT_SOURCE_DIR}/visual_representation.cpp
${PROJECT_SOURCE_DIR}/atom.cpp
${PROJECT_SOURCE_DIR}/visualisation.cpp
)


add_executable (test src/testgraphs.cpp)
target_link_libraries (test VR_LIBRARY  ${Boost_GRAPH_LIBRARY} ${GRAPHVIZ_CDT_LIBRARY} ${GRAPHVIZ_GVC_LIBRARY} ${GRAPHVIZ_CGRAPH_LIBRARY}  segLib  ${PCL_LIBRARIES} ${OpenCV_LIBS})

add_executable (compare src/compare.cpp)
target_link_libraries (compare VR_LIBRARY  ${Boost_GRAPH_LIBRARY} ${GRAPHVIZ_CDT_LIBRARY} ${GRAPHVIZ_GVC_LIBRARY} ${GRAPHVIZ_CGRAPH_LIBRARY}  segLib  ${PCL_LIBRARIES} ${OpenCV_LIBS})

add_executable (track src/track_sequence.cpp)
target_link_libraries (track VR_LIBRARY  ${Boost_GRAPH_LIBRARY} ${GRAPHVIZ_CDT_LIBRARY} ${GRAPHVIZ_GVC_LIBRARY} ${GRAPHVIZ_CGRAPH_LIBRARY}  segLib  ${PCL_LIBRARIES} ${OpenCV_LIBS})


#Continue with subdirectory
#add_subdirectory(Compare)
#add_subdirectory(GUI)

