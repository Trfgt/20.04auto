# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.16

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:


#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:


# Remove some rules from gmake that .SUFFIXES does not remove.
SUFFIXES =

.SUFFIXES: .hpux_make_needs_suffix_list


# Suppress display of executed commands.
$(VERBOSE).SILENT:


# A target that is always out of date.
cmake_force:

.PHONY : cmake_force

#=============================================================================
# Set environment variables for the build.

# The shell in which to execute make rules.
SHELL = /bin/sh

# The CMake executable.
CMAKE_COMMAND = /usr/bin/cmake

# The command to remove a file.
RM = /usr/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/trobo/lidar_ws/src

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/trobo/lidar_ws/build

# Utility rule file for velodyne_msgs_generate_messages_cpp.

# Include the progress variables for this target.
include velodyne/velodyne_msgs/CMakeFiles/velodyne_msgs_generate_messages_cpp.dir/progress.make

velodyne/velodyne_msgs/CMakeFiles/velodyne_msgs_generate_messages_cpp: /home/trobo/lidar_ws/devel/include/velodyne_msgs/VelodynePacket.h
velodyne/velodyne_msgs/CMakeFiles/velodyne_msgs_generate_messages_cpp: /home/trobo/lidar_ws/devel/include/velodyne_msgs/VelodyneScan.h


/home/trobo/lidar_ws/devel/include/velodyne_msgs/VelodynePacket.h: /opt/ros/noetic/lib/gencpp/gen_cpp.py
/home/trobo/lidar_ws/devel/include/velodyne_msgs/VelodynePacket.h: /home/trobo/lidar_ws/src/velodyne/velodyne_msgs/msg/VelodynePacket.msg
/home/trobo/lidar_ws/devel/include/velodyne_msgs/VelodynePacket.h: /opt/ros/noetic/share/gencpp/msg.h.template
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/trobo/lidar_ws/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Generating C++ code from velodyne_msgs/VelodynePacket.msg"
	cd /home/trobo/lidar_ws/src/velodyne/velodyne_msgs && /home/trobo/lidar_ws/build/catkin_generated/env_cached.sh /usr/bin/python3 /opt/ros/noetic/share/gencpp/cmake/../../../lib/gencpp/gen_cpp.py /home/trobo/lidar_ws/src/velodyne/velodyne_msgs/msg/VelodynePacket.msg -Ivelodyne_msgs:/home/trobo/lidar_ws/src/velodyne/velodyne_msgs/msg -Istd_msgs:/opt/ros/noetic/share/std_msgs/cmake/../msg -p velodyne_msgs -o /home/trobo/lidar_ws/devel/include/velodyne_msgs -e /opt/ros/noetic/share/gencpp/cmake/..

/home/trobo/lidar_ws/devel/include/velodyne_msgs/VelodyneScan.h: /opt/ros/noetic/lib/gencpp/gen_cpp.py
/home/trobo/lidar_ws/devel/include/velodyne_msgs/VelodyneScan.h: /home/trobo/lidar_ws/src/velodyne/velodyne_msgs/msg/VelodyneScan.msg
/home/trobo/lidar_ws/devel/include/velodyne_msgs/VelodyneScan.h: /opt/ros/noetic/share/std_msgs/msg/Header.msg
/home/trobo/lidar_ws/devel/include/velodyne_msgs/VelodyneScan.h: /home/trobo/lidar_ws/src/velodyne/velodyne_msgs/msg/VelodynePacket.msg
/home/trobo/lidar_ws/devel/include/velodyne_msgs/VelodyneScan.h: /opt/ros/noetic/share/gencpp/msg.h.template
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/trobo/lidar_ws/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Generating C++ code from velodyne_msgs/VelodyneScan.msg"
	cd /home/trobo/lidar_ws/src/velodyne/velodyne_msgs && /home/trobo/lidar_ws/build/catkin_generated/env_cached.sh /usr/bin/python3 /opt/ros/noetic/share/gencpp/cmake/../../../lib/gencpp/gen_cpp.py /home/trobo/lidar_ws/src/velodyne/velodyne_msgs/msg/VelodyneScan.msg -Ivelodyne_msgs:/home/trobo/lidar_ws/src/velodyne/velodyne_msgs/msg -Istd_msgs:/opt/ros/noetic/share/std_msgs/cmake/../msg -p velodyne_msgs -o /home/trobo/lidar_ws/devel/include/velodyne_msgs -e /opt/ros/noetic/share/gencpp/cmake/..

velodyne_msgs_generate_messages_cpp: velodyne/velodyne_msgs/CMakeFiles/velodyne_msgs_generate_messages_cpp
velodyne_msgs_generate_messages_cpp: /home/trobo/lidar_ws/devel/include/velodyne_msgs/VelodynePacket.h
velodyne_msgs_generate_messages_cpp: /home/trobo/lidar_ws/devel/include/velodyne_msgs/VelodyneScan.h
velodyne_msgs_generate_messages_cpp: velodyne/velodyne_msgs/CMakeFiles/velodyne_msgs_generate_messages_cpp.dir/build.make

.PHONY : velodyne_msgs_generate_messages_cpp

# Rule to build all files generated by this target.
velodyne/velodyne_msgs/CMakeFiles/velodyne_msgs_generate_messages_cpp.dir/build: velodyne_msgs_generate_messages_cpp

.PHONY : velodyne/velodyne_msgs/CMakeFiles/velodyne_msgs_generate_messages_cpp.dir/build

velodyne/velodyne_msgs/CMakeFiles/velodyne_msgs_generate_messages_cpp.dir/clean:
	cd /home/trobo/lidar_ws/build/velodyne/velodyne_msgs && $(CMAKE_COMMAND) -P CMakeFiles/velodyne_msgs_generate_messages_cpp.dir/cmake_clean.cmake
.PHONY : velodyne/velodyne_msgs/CMakeFiles/velodyne_msgs_generate_messages_cpp.dir/clean

velodyne/velodyne_msgs/CMakeFiles/velodyne_msgs_generate_messages_cpp.dir/depend:
	cd /home/trobo/lidar_ws/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/trobo/lidar_ws/src /home/trobo/lidar_ws/src/velodyne/velodyne_msgs /home/trobo/lidar_ws/build /home/trobo/lidar_ws/build/velodyne/velodyne_msgs /home/trobo/lidar_ws/build/velodyne/velodyne_msgs/CMakeFiles/velodyne_msgs_generate_messages_cpp.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : velodyne/velodyne_msgs/CMakeFiles/velodyne_msgs_generate_messages_cpp.dir/depend

