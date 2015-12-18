cmake_minimum_required(VERSION 2.8)
project(GUI)

#set(PROJECT_NAME            "qt-opencv-multithreaded")
set(PROJECT_VERSION         "1.4.0")
set(PROJECT_AUTHOR_NAME     "Nick D'Ademo")
set(PROJECT_AUTHOR_EMAIL    "nickdademo@gmail.com")
set(PROJECT_AUTHOR_WEBSITE  "www.nickdademo.com")

# Find includes in corresponding build directories
set(CMAKE_INCLUDE_CURRENT_DIR ON)
# Instruct CMake to run moc automatically when needed.
set(CMAKE_AUTOMOC ON)

########
# Qt 5 #
########
#find_package(Qt5Widgets REQUIRED)
find_package(Qt4 REQUIRED)
include(${QT_USE_FILE})

##########
# OpenCV #
##########
#find_package(OpenCV REQUIRED)
#include_directories(${OpenCV_INCLUDE_DIRS})

add_definitions(-Wall -g  -fopenmp -lgomp -march=native -std=c++11 -O4)

set(PROJECT_SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/src)
set(PROJECT_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/include)
include_directories("${PROJECT_BINARY_DIR}")
include_directories("${CMAKE_CURRENT_SOURCE_DIR}/include")

###############
# Application #
###############
file(GLOB_RECURSE UI_FILES ${PROJECT_SOURCE_DIR}/*.ui)
file(GLOB_RECURSE SOURCE_FILES ${PROJECT_SOURCE_DIR}/*.cpp)
file(GLOB_RECURSE HEADER_FILES ${PROJECT_INCLUDE_DIR}/*.h*)

#qt5_wrap_ui(UI_HEADERS ${UI_FILES})
qt4_wrap_ui(UI_HEADERS ${UI_FILES})

configure_file(${PROJECT_SOURCE_DIR}/Config.h.cmake
  ${PROJECT_SOURCE_DIR}/Config.h
)

add_executable(gui
  ${UI_HEADERS}
  ${SOURCE_FILES}
  ${HEADER_FILES}
)

target_link_libraries(gui
  #Qt5::Widgets
    ${QT_QTCORE_LIBRARY}
    ${QT_QTGUI_LIBRARY}
	segLib 
	${Boost_LIBRARIES}
	${PCL_LIBRARIES}
	${OpenCV_LIBS}
	
#  ${OpenCV_LIBS}
)
