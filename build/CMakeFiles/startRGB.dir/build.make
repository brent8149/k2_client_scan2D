# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.2

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
CMAKE_COMMAND = /usr/local/bin/cmake

# The command to remove a file.
RM = /usr/local/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/brent/ros_ws/k2_client

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/brent/ros_ws/k2_client/build

# Include any dependencies generated for this target.
include CMakeFiles/startRGB.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/startRGB.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/startRGB.dir/flags.make

CMakeFiles/startRGB.dir/src/startRGB.cpp.o: CMakeFiles/startRGB.dir/flags.make
CMakeFiles/startRGB.dir/src/startRGB.cpp.o: ../src/startRGB.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/brent/ros_ws/k2_client/build/CMakeFiles $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/startRGB.dir/src/startRGB.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/startRGB.dir/src/startRGB.cpp.o -c /home/brent/ros_ws/k2_client/src/startRGB.cpp

CMakeFiles/startRGB.dir/src/startRGB.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/startRGB.dir/src/startRGB.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/brent/ros_ws/k2_client/src/startRGB.cpp > CMakeFiles/startRGB.dir/src/startRGB.cpp.i

CMakeFiles/startRGB.dir/src/startRGB.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/startRGB.dir/src/startRGB.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/brent/ros_ws/k2_client/src/startRGB.cpp -o CMakeFiles/startRGB.dir/src/startRGB.cpp.s

CMakeFiles/startRGB.dir/src/startRGB.cpp.o.requires:
.PHONY : CMakeFiles/startRGB.dir/src/startRGB.cpp.o.requires

CMakeFiles/startRGB.dir/src/startRGB.cpp.o.provides: CMakeFiles/startRGB.dir/src/startRGB.cpp.o.requires
	$(MAKE) -f CMakeFiles/startRGB.dir/build.make CMakeFiles/startRGB.dir/src/startRGB.cpp.o.provides.build
.PHONY : CMakeFiles/startRGB.dir/src/startRGB.cpp.o.provides

CMakeFiles/startRGB.dir/src/startRGB.cpp.o.provides.build: CMakeFiles/startRGB.dir/src/startRGB.cpp.o

# Object files for target startRGB
startRGB_OBJECTS = \
"CMakeFiles/startRGB.dir/src/startRGB.cpp.o"

# External object files for target startRGB
startRGB_EXTERNAL_OBJECTS =

../bin/startRGB: CMakeFiles/startRGB.dir/src/startRGB.cpp.o
../bin/startRGB: CMakeFiles/startRGB.dir/build.make
../bin/startRGB: CMakeFiles/startRGB.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --red --bold "Linking CXX executable ../bin/startRGB"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/startRGB.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/startRGB.dir/build: ../bin/startRGB
.PHONY : CMakeFiles/startRGB.dir/build

CMakeFiles/startRGB.dir/requires: CMakeFiles/startRGB.dir/src/startRGB.cpp.o.requires
.PHONY : CMakeFiles/startRGB.dir/requires

CMakeFiles/startRGB.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/startRGB.dir/cmake_clean.cmake
.PHONY : CMakeFiles/startRGB.dir/clean

CMakeFiles/startRGB.dir/depend:
	cd /home/brent/ros_ws/k2_client/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/brent/ros_ws/k2_client /home/brent/ros_ws/k2_client /home/brent/ros_ws/k2_client/build /home/brent/ros_ws/k2_client/build /home/brent/ros_ws/k2_client/build/CMakeFiles/startRGB.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/startRGB.dir/depend

