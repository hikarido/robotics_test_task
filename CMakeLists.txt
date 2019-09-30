cmake_minimum_required(VERSION 2.8)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package( OpenCV REQUIRED )
include_directories( ${OpenCV_INCLUDE_DIRS} )

project(tra_test_task)
add_executable(${PROJECT_NAME} "main.cpp" "nanosvg.h" "easylogging++.h" "easylogging++.cc")
target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS})
