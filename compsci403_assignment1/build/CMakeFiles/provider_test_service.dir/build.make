# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.5

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
CMAKE_SOURCE_DIR = /home/wyliao/Desktop/compsci403_assignment1

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/wyliao/Desktop/compsci403_assignment1/build

# Include any dependencies generated for this target.
include CMakeFiles/provider_test_service.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/provider_test_service.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/provider_test_service.dir/flags.make

CMakeFiles/provider_test_service.dir/src/provider_test_service.cpp.o: CMakeFiles/provider_test_service.dir/flags.make
CMakeFiles/provider_test_service.dir/src/provider_test_service.cpp.o: ../src/provider_test_service.cpp
CMakeFiles/provider_test_service.dir/src/provider_test_service.cpp.o: ../manifest.xml
CMakeFiles/provider_test_service.dir/src/provider_test_service.cpp.o: /opt/ros/kinetic/share/cpp_common/package.xml
CMakeFiles/provider_test_service.dir/src/provider_test_service.cpp.o: /opt/ros/kinetic/share/rostime/package.xml
CMakeFiles/provider_test_service.dir/src/provider_test_service.cpp.o: /opt/ros/kinetic/share/roscpp_traits/package.xml
CMakeFiles/provider_test_service.dir/src/provider_test_service.cpp.o: /opt/ros/kinetic/share/roscpp_serialization/package.xml
CMakeFiles/provider_test_service.dir/src/provider_test_service.cpp.o: /opt/ros/kinetic/share/catkin/package.xml
CMakeFiles/provider_test_service.dir/src/provider_test_service.cpp.o: /opt/ros/kinetic/share/genmsg/package.xml
CMakeFiles/provider_test_service.dir/src/provider_test_service.cpp.o: /opt/ros/kinetic/share/genpy/package.xml
CMakeFiles/provider_test_service.dir/src/provider_test_service.cpp.o: /opt/ros/kinetic/share/message_runtime/package.xml
CMakeFiles/provider_test_service.dir/src/provider_test_service.cpp.o: /opt/ros/kinetic/share/std_msgs/package.xml
CMakeFiles/provider_test_service.dir/src/provider_test_service.cpp.o: /opt/ros/kinetic/share/geometry_msgs/package.xml
CMakeFiles/provider_test_service.dir/src/provider_test_service.cpp.o: /opt/ros/kinetic/share/sensor_msgs/package.xml
CMakeFiles/provider_test_service.dir/src/provider_test_service.cpp.o: /opt/ros/kinetic/share/visualization_msgs/package.xml
CMakeFiles/provider_test_service.dir/src/provider_test_service.cpp.o: /opt/ros/kinetic/share/gencpp/package.xml
CMakeFiles/provider_test_service.dir/src/provider_test_service.cpp.o: /opt/ros/kinetic/share/geneus/package.xml
CMakeFiles/provider_test_service.dir/src/provider_test_service.cpp.o: /opt/ros/kinetic/share/gennodejs/package.xml
CMakeFiles/provider_test_service.dir/src/provider_test_service.cpp.o: /opt/ros/kinetic/share/genlisp/package.xml
CMakeFiles/provider_test_service.dir/src/provider_test_service.cpp.o: /opt/ros/kinetic/share/message_generation/package.xml
CMakeFiles/provider_test_service.dir/src/provider_test_service.cpp.o: /opt/ros/kinetic/share/rosbuild/package.xml
CMakeFiles/provider_test_service.dir/src/provider_test_service.cpp.o: /opt/ros/kinetic/share/rosconsole/package.xml
CMakeFiles/provider_test_service.dir/src/provider_test_service.cpp.o: /opt/ros/kinetic/share/rosgraph_msgs/package.xml
CMakeFiles/provider_test_service.dir/src/provider_test_service.cpp.o: /opt/ros/kinetic/share/xmlrpcpp/package.xml
CMakeFiles/provider_test_service.dir/src/provider_test_service.cpp.o: /opt/ros/kinetic/share/roscpp/package.xml
CMakeFiles/provider_test_service.dir/src/provider_test_service.cpp.o: /opt/ros/kinetic/share/rosgraph/package.xml
CMakeFiles/provider_test_service.dir/src/provider_test_service.cpp.o: /opt/ros/kinetic/share/rospack/package.xml
CMakeFiles/provider_test_service.dir/src/provider_test_service.cpp.o: /opt/ros/kinetic/share/roslib/package.xml
CMakeFiles/provider_test_service.dir/src/provider_test_service.cpp.o: /opt/ros/kinetic/share/rospy/package.xml
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/wyliao/Desktop/compsci403_assignment1/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/provider_test_service.dir/src/provider_test_service.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/provider_test_service.dir/src/provider_test_service.cpp.o -c /home/wyliao/Desktop/compsci403_assignment1/src/provider_test_service.cpp

CMakeFiles/provider_test_service.dir/src/provider_test_service.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/provider_test_service.dir/src/provider_test_service.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/wyliao/Desktop/compsci403_assignment1/src/provider_test_service.cpp > CMakeFiles/provider_test_service.dir/src/provider_test_service.cpp.i

CMakeFiles/provider_test_service.dir/src/provider_test_service.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/provider_test_service.dir/src/provider_test_service.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/wyliao/Desktop/compsci403_assignment1/src/provider_test_service.cpp -o CMakeFiles/provider_test_service.dir/src/provider_test_service.cpp.s

CMakeFiles/provider_test_service.dir/src/provider_test_service.cpp.o.requires:

.PHONY : CMakeFiles/provider_test_service.dir/src/provider_test_service.cpp.o.requires

CMakeFiles/provider_test_service.dir/src/provider_test_service.cpp.o.provides: CMakeFiles/provider_test_service.dir/src/provider_test_service.cpp.o.requires
	$(MAKE) -f CMakeFiles/provider_test_service.dir/build.make CMakeFiles/provider_test_service.dir/src/provider_test_service.cpp.o.provides.build
.PHONY : CMakeFiles/provider_test_service.dir/src/provider_test_service.cpp.o.provides

CMakeFiles/provider_test_service.dir/src/provider_test_service.cpp.o.provides.build: CMakeFiles/provider_test_service.dir/src/provider_test_service.cpp.o


# Object files for target provider_test_service
provider_test_service_OBJECTS = \
"CMakeFiles/provider_test_service.dir/src/provider_test_service.cpp.o"

# External object files for target provider_test_service
provider_test_service_EXTERNAL_OBJECTS =

../bin/provider_test_service: CMakeFiles/provider_test_service.dir/src/provider_test_service.cpp.o
../bin/provider_test_service: CMakeFiles/provider_test_service.dir/build.make
../bin/provider_test_service: CMakeFiles/provider_test_service.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/wyliao/Desktop/compsci403_assignment1/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable ../bin/provider_test_service"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/provider_test_service.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/provider_test_service.dir/build: ../bin/provider_test_service

.PHONY : CMakeFiles/provider_test_service.dir/build

CMakeFiles/provider_test_service.dir/requires: CMakeFiles/provider_test_service.dir/src/provider_test_service.cpp.o.requires

.PHONY : CMakeFiles/provider_test_service.dir/requires

CMakeFiles/provider_test_service.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/provider_test_service.dir/cmake_clean.cmake
.PHONY : CMakeFiles/provider_test_service.dir/clean

CMakeFiles/provider_test_service.dir/depend:
	cd /home/wyliao/Desktop/compsci403_assignment1/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/wyliao/Desktop/compsci403_assignment1 /home/wyliao/Desktop/compsci403_assignment1 /home/wyliao/Desktop/compsci403_assignment1/build /home/wyliao/Desktop/compsci403_assignment1/build /home/wyliao/Desktop/compsci403_assignment1/build/CMakeFiles/provider_test_service.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/provider_test_service.dir/depend

