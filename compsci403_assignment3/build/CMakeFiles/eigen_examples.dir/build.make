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
CMAKE_SOURCE_DIR = /home/wyliao/Desktop/compsci403_assignment3

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/wyliao/Desktop/compsci403_assignment3/build

# Include any dependencies generated for this target.
include CMakeFiles/eigen_examples.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/eigen_examples.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/eigen_examples.dir/flags.make

CMakeFiles/eigen_examples.dir/src/eigen_examples.cpp.o: CMakeFiles/eigen_examples.dir/flags.make
CMakeFiles/eigen_examples.dir/src/eigen_examples.cpp.o: ../src/eigen_examples.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/wyliao/Desktop/compsci403_assignment3/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/eigen_examples.dir/src/eigen_examples.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/eigen_examples.dir/src/eigen_examples.cpp.o -c /home/wyliao/Desktop/compsci403_assignment3/src/eigen_examples.cpp

CMakeFiles/eigen_examples.dir/src/eigen_examples.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/eigen_examples.dir/src/eigen_examples.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/wyliao/Desktop/compsci403_assignment3/src/eigen_examples.cpp > CMakeFiles/eigen_examples.dir/src/eigen_examples.cpp.i

CMakeFiles/eigen_examples.dir/src/eigen_examples.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/eigen_examples.dir/src/eigen_examples.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/wyliao/Desktop/compsci403_assignment3/src/eigen_examples.cpp -o CMakeFiles/eigen_examples.dir/src/eigen_examples.cpp.s

CMakeFiles/eigen_examples.dir/src/eigen_examples.cpp.o.requires:

.PHONY : CMakeFiles/eigen_examples.dir/src/eigen_examples.cpp.o.requires

CMakeFiles/eigen_examples.dir/src/eigen_examples.cpp.o.provides: CMakeFiles/eigen_examples.dir/src/eigen_examples.cpp.o.requires
	$(MAKE) -f CMakeFiles/eigen_examples.dir/build.make CMakeFiles/eigen_examples.dir/src/eigen_examples.cpp.o.provides.build
.PHONY : CMakeFiles/eigen_examples.dir/src/eigen_examples.cpp.o.provides

CMakeFiles/eigen_examples.dir/src/eigen_examples.cpp.o.provides.build: CMakeFiles/eigen_examples.dir/src/eigen_examples.cpp.o


# Object files for target eigen_examples
eigen_examples_OBJECTS = \
"CMakeFiles/eigen_examples.dir/src/eigen_examples.cpp.o"

# External object files for target eigen_examples
eigen_examples_EXTERNAL_OBJECTS =

../bin/eigen_examples: CMakeFiles/eigen_examples.dir/src/eigen_examples.cpp.o
../bin/eigen_examples: CMakeFiles/eigen_examples.dir/build.make
../bin/eigen_examples: CMakeFiles/eigen_examples.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/wyliao/Desktop/compsci403_assignment3/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable ../bin/eigen_examples"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/eigen_examples.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/eigen_examples.dir/build: ../bin/eigen_examples

.PHONY : CMakeFiles/eigen_examples.dir/build

CMakeFiles/eigen_examples.dir/requires: CMakeFiles/eigen_examples.dir/src/eigen_examples.cpp.o.requires

.PHONY : CMakeFiles/eigen_examples.dir/requires

CMakeFiles/eigen_examples.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/eigen_examples.dir/cmake_clean.cmake
.PHONY : CMakeFiles/eigen_examples.dir/clean

CMakeFiles/eigen_examples.dir/depend:
	cd /home/wyliao/Desktop/compsci403_assignment3/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/wyliao/Desktop/compsci403_assignment3 /home/wyliao/Desktop/compsci403_assignment3 /home/wyliao/Desktop/compsci403_assignment3/build /home/wyliao/Desktop/compsci403_assignment3/build /home/wyliao/Desktop/compsci403_assignment3/build/CMakeFiles/eigen_examples.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/eigen_examples.dir/depend
