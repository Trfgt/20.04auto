# Install script for directory: /home/trobo/lidar_ws/src/rviz_car_model

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/trobo/lidar_ws/install")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Install shared libraries without execute permission?
if(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  set(CMAKE_INSTALL_SO_NO_EXE "1")
endif()

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "FALSE")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/rviz_car_model/README.md;/rviz_car_model/demo.launch;/rviz_car_model/_51225ex1diffuse.jpeg;/rviz_car_model/_51225ex2diffuse.jpeg;/rviz_car_model/CAR_original.dae;/rviz_car_model/default.urdf;/rviz_car_model/demo.rviz")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/rviz_car_model" TYPE FILE FILES
    "/home/trobo/lidar_ws/src/rviz_car_model/README.md"
    "/home/trobo/lidar_ws/src/rviz_car_model/launch/demo.launch"
    "/home/trobo/lidar_ws/src/rviz_car_model/rviz/car_model/_51225ex1diffuse.jpeg"
    "/home/trobo/lidar_ws/src/rviz_car_model/rviz/car_model/_51225ex2diffuse.jpeg"
    "/home/trobo/lidar_ws/src/rviz_car_model/rviz/car_model/CAR_original.dae"
    "/home/trobo/lidar_ws/src/rviz_car_model/rviz/car_model/default.urdf"
    "/home/trobo/lidar_ws/src/rviz_car_model/rviz/demo.rviz"
    )
endif()

