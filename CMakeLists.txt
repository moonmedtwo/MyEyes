cmake_minimum_required(VERSION 2.8.11)

project(MyEyes)

# init_qt: Let's do the CMake job for us
set(CMAKE_AUTOMOC ON) # For meta object compiler
set(CMAKE_AUTORCC ON) # Resource files
set(CMAKE_AUTOUIC ON) # UI files

# Find includes in corresponding build directories
set(CMAKE_INCLUDE_CURRENT_DIR ON)
# Set Error when Warning
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -lboost_system -lstdc++fs -Wall -Wno-comment")

# Find the QtWidgets library
find_package(Qt5 REQUIRED Widgets Core Gui SerialPort)

find_package(VTK REQUIRED)
find_package(PCL REQUIRED)

# Fix a compilation bug under ubuntu 16.04 (Xenial)
list(REMOVE_ITEM PCL_LIBRARIES "vtkproj4")

include_directories(${PCL_INCLUDE_DIRS}
                    "/usr/include/ni/"
                    "../metslib/")
link_directories(${PCL_LIBRARY_DIRS})
message( "PCL_LIBRARY_DIRS " ${PCL_LIBRARY_DIRS})

add_definitions(${PCL_DEFINITIONS})

set(project_SOURCES main.cpp
    mainwindow.cpp
    userrecognizer_thread.cpp
    userrecognizer_window.cpp
    usercomm.cpp
    usertracking.cpp
    setposition_window.cpp
    util.cpp)

set(project_HEADERS mainwindow.h)


add_executable(${PROJECT_NAME} ${project_SOURCES})

target_link_libraries(${PROJECT_NAME} ${PCL_LIBRARIES})
#set_target_properties(${PROJECT_NAME} PROPERTIES LINK_FLAGS "-static" )

qt5_use_modules(${PROJECT_NAME} Widgets Core Gui SerialPort)
