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
include CMakeFiles/startScan2D.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/startScan2D.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/startScan2D.dir/flags.make

CMakeFiles/startScan2D.dir/src/startScan2D.cpp.o: CMakeFiles/startScan2D.dir/flags.make
CMakeFiles/startScan2D.dir/src/startScan2D.cpp.o: ../src/startScan2D.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/brent/ros_ws/k2_client/build/CMakeFiles $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/startScan2D.dir/src/startScan2D.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/startScan2D.dir/src/startScan2D.cpp.o -c /home/brent/ros_ws/k2_client/src/startScan2D.cpp

CMakeFiles/startScan2D.dir/src/startScan2D.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/startScan2D.dir/src/startScan2D.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/brent/ros_ws/k2_client/src/startScan2D.cpp > CMakeFiles/startScan2D.dir/src/startScan2D.cpp.i

CMakeFiles/startScan2D.dir/src/startScan2D.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/startScan2D.dir/src/startScan2D.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/brent/ros_ws/k2_client/src/startScan2D.cpp -o CMakeFiles/startScan2D.dir/src/startScan2D.cpp.s

CMakeFiles/startScan2D.dir/src/startScan2D.cpp.o.requires:
.PHONY : CMakeFiles/startScan2D.dir/src/startScan2D.cpp.o.requires

CMakeFiles/startScan2D.dir/src/startScan2D.cpp.o.provides: CMakeFiles/startScan2D.dir/src/startScan2D.cpp.o.requires
	$(MAKE) -f CMakeFiles/startScan2D.dir/build.make CMakeFiles/startScan2D.dir/src/startScan2D.cpp.o.provides.build
.PHONY : CMakeFiles/startScan2D.dir/src/startScan2D.cpp.o.provides

CMakeFiles/startScan2D.dir/src/startScan2D.cpp.o.provides.build: CMakeFiles/startScan2D.dir/src/startScan2D.cpp.o

# Object files for target startScan2D
startScan2D_OBJECTS = \
"CMakeFiles/startScan2D.dir/src/startScan2D.cpp.o"

# External object files for target startScan2D
startScan2D_EXTERNAL_OBJECTS =

../bin/startScan2D: CMakeFiles/startScan2D.dir/src/startScan2D.cpp.o
../bin/startScan2D: CMakeFiles/startScan2D.dir/build.make
../bin/startScan2D: CMakeFiles/startScan2D.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --red --bold "Linking CXX executable ../bin/startScan2D"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/startScan2D.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/startScan2D.dir/build: ../bin/startScan2D
.PHONY : CMakeFiles/startScan2D.dir/build

CMakeFiles/startScan2D.dir/requires: CMakeFiles/startScan2D.dir/src/startScan2D.cpp.o.requires
.PHONY : CMakeFiles/startScan2D.dir/requires

CMakeFiles/startScan2D.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/startScan2D.dir/cmake_clean.cmake
.PHONY : CMakeFiles/startScan2D.dir/clean

CMakeFiles/startScan2D.dir/depend:
	cd /home/brent/ros_ws/k2_client/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/brent/ros_ws/k2_client /home/brent/ros_ws/k2_client /home/brent/ros_ws/k2_client/build /home/brent/ros_ws/k2_client/build /home/brent/ros_ws/k2_client/build/CMakeFiles/startScan2D.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/startScan2D.dir/depend
