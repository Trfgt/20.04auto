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

# Utility rule file for velodyne_msgs_gencpp.

# Include the progress variables for this target.
include velodyne/velodyne_msgs/CMakeFiles/velodyne_msgs_gencpp.dir/progress.make

velodyne_msgs_gencpp: velodyne/velodyne_msgs/CMakeFiles/velodyne_msgs_gencpp.dir/build.make

.PHONY : velodyne_msgs_gencpp

# Rule to build all files generated by this target.
velodyne/velodyne_msgs/CMakeFiles/velodyne_msgs_gencpp.dir/build: velodyne_msgs_gencpp

.PHONY : velodyne/velodyne_msgs/CMakeFiles/velodyne_msgs_gencpp.dir/build

velodyne/velodyne_msgs/CMakeFiles/velodyne_msgs_gencpp.dir/clean:
	cd /home/trobo/lidar_ws/build/velodyne/velodyne_msgs && $(CMAKE_COMMAND) -P CMakeFiles/velodyne_msgs_gencpp.dir/cmake_clean.cmake
.PHONY : velodyne/velodyne_msgs/CMakeFiles/velodyne_msgs_gencpp.dir/clean

velodyne/velodyne_msgs/CMakeFiles/velodyne_msgs_gencpp.dir/depend:
	cd /home/trobo/lidar_ws/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/trobo/lidar_ws/src /home/trobo/lidar_ws/src/velodyne/velodyne_msgs /home/trobo/lidar_ws/build /home/trobo/lidar_ws/build/velodyne/velodyne_msgs /home/trobo/lidar_ws/build/velodyne/velodyne_msgs/CMakeFiles/velodyne_msgs_gencpp.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : velodyne/velodyne_msgs/CMakeFiles/velodyne_msgs_gencpp.dir/depend
