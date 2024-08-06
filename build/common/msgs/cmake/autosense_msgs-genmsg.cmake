# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "autosense_msgs: 2 messages, 0 services")

set(MSG_I_FLAGS "-Iautosense_msgs:/home/trobo/lidar_ws/src/common/msgs/autosense_msgs;-Istd_msgs:/opt/ros/noetic/share/std_msgs/cmake/../msg;-Isensor_msgs:/opt/ros/noetic/share/sensor_msgs/cmake/../msg;-Igeometry_msgs:/opt/ros/noetic/share/geometry_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(autosense_msgs_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/trobo/lidar_ws/src/common/msgs/autosense_msgs/PointCloud2Array.msg" NAME_WE)
add_custom_target(_autosense_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "autosense_msgs" "/home/trobo/lidar_ws/src/common/msgs/autosense_msgs/PointCloud2Array.msg" "sensor_msgs/PointField:std_msgs/Header:sensor_msgs/PointCloud2"
)

get_filename_component(_filename "/home/trobo/lidar_ws/src/common/msgs/autosense_msgs/TrackingFixedTrajectoryArray.msg" NAME_WE)
add_custom_target(_autosense_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "autosense_msgs" "/home/trobo/lidar_ws/src/common/msgs/autosense_msgs/TrackingFixedTrajectoryArray.msg" "std_msgs/Header"
)

#
#  langs = gencpp;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(autosense_msgs
  "/home/trobo/lidar_ws/src/common/msgs/autosense_msgs/PointCloud2Array.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/sensor_msgs/cmake/../msg/PointField.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/sensor_msgs/cmake/../msg/PointCloud2.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/autosense_msgs
)
_generate_msg_cpp(autosense_msgs
  "/home/trobo/lidar_ws/src/common/msgs/autosense_msgs/TrackingFixedTrajectoryArray.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/autosense_msgs
)

### Generating Services

### Generating Module File
_generate_module_cpp(autosense_msgs
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/autosense_msgs
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(autosense_msgs_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(autosense_msgs_generate_messages autosense_msgs_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/trobo/lidar_ws/src/common/msgs/autosense_msgs/PointCloud2Array.msg" NAME_WE)
add_dependencies(autosense_msgs_generate_messages_cpp _autosense_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/trobo/lidar_ws/src/common/msgs/autosense_msgs/TrackingFixedTrajectoryArray.msg" NAME_WE)
add_dependencies(autosense_msgs_generate_messages_cpp _autosense_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(autosense_msgs_gencpp)
add_dependencies(autosense_msgs_gencpp autosense_msgs_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS autosense_msgs_generate_messages_cpp)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(autosense_msgs
  "/home/trobo/lidar_ws/src/common/msgs/autosense_msgs/PointCloud2Array.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/sensor_msgs/cmake/../msg/PointField.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/sensor_msgs/cmake/../msg/PointCloud2.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/autosense_msgs
)
_generate_msg_py(autosense_msgs
  "/home/trobo/lidar_ws/src/common/msgs/autosense_msgs/TrackingFixedTrajectoryArray.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/autosense_msgs
)

### Generating Services

### Generating Module File
_generate_module_py(autosense_msgs
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/autosense_msgs
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(autosense_msgs_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(autosense_msgs_generate_messages autosense_msgs_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/trobo/lidar_ws/src/common/msgs/autosense_msgs/PointCloud2Array.msg" NAME_WE)
add_dependencies(autosense_msgs_generate_messages_py _autosense_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/trobo/lidar_ws/src/common/msgs/autosense_msgs/TrackingFixedTrajectoryArray.msg" NAME_WE)
add_dependencies(autosense_msgs_generate_messages_py _autosense_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(autosense_msgs_genpy)
add_dependencies(autosense_msgs_genpy autosense_msgs_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS autosense_msgs_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/autosense_msgs)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/autosense_msgs
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(autosense_msgs_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()
if(TARGET sensor_msgs_generate_messages_cpp)
  add_dependencies(autosense_msgs_generate_messages_cpp sensor_msgs_generate_messages_cpp)
endif()
if(TARGET geometry_msgs_generate_messages_cpp)
  add_dependencies(autosense_msgs_generate_messages_cpp geometry_msgs_generate_messages_cpp)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/autosense_msgs)
  install(CODE "execute_process(COMMAND \"/usr/bin/python3\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/autosense_msgs\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/autosense_msgs
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(autosense_msgs_generate_messages_py std_msgs_generate_messages_py)
endif()
if(TARGET sensor_msgs_generate_messages_py)
  add_dependencies(autosense_msgs_generate_messages_py sensor_msgs_generate_messages_py)
endif()
if(TARGET geometry_msgs_generate_messages_py)
  add_dependencies(autosense_msgs_generate_messages_py geometry_msgs_generate_messages_py)
endif()
