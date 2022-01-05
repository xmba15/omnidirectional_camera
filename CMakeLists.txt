cmake_minimum_required(VERSION 3.10)

project(omnidirectional_camera)
set(CMAKE_CXX_STANDARD 17)

list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

find_package(OpenCV REQUIRED)

include(Findrapidjson)
if(NOT RAPIDJSON_FOUND)
  message(STATUS "rapidjson not found")
  return()
endif()

add_executable(${PROJECT_NAME}_calibration
  ${PROJECT_SOURCE_DIR}/src/CalibrationApp.cpp
  ${PROJECT_SOURCE_DIR}/src/CalibrationHandler.cpp
)

target_include_directories(${PROJECT_NAME}_calibration
  SYSTEM PUBLIC
    ${OpenCV_INCLUDE_DIRS}
)

target_link_libraries(${PROJECT_NAME}_calibration
  PUBLIC
    ${OpenCV_LIBRARIES}
)

add_executable(${PROJECT_NAME}_undistortion
  ${PROJECT_SOURCE_DIR}/src/UndistortionApp.cpp
)

target_include_directories(${PROJECT_NAME}_undistortion
  SYSTEM PUBLIC
    ${OpenCV_INCLUDE_DIRS}
)

target_link_libraries(${PROJECT_NAME}_undistortion
  PUBLIC
    ${OpenCV_LIBRARIES}
)
