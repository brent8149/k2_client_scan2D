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
include CMakeFiles/startIR.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/startIR.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/startIR.dir/flags.make

CMakeFiles/startIR.dir/src/startIR.cpp.o: CMakeFiles/startIR.dir/flags.make
CMakeFiles/startIR.dir/src/startIR.cpp.o: ../src/startIR.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/brent/ros_ws/k2_client/build/CMakeFiles $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/startIR.dir/src/startIR.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/startIR.dir/src/startIR.cpp.o -c /home/brent/ros_ws/k2_client/src/startIR.cpp

CMakeFiles/startIR.dir/src/startIR.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/startIR.dir/src/startIR.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/brent/ros_ws/k2_client/src/startIR.cpp > CMakeFiles/startIR.dir/src/startIR.cpp.i

CMakeFiles/startIR.dir/src/startIR.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/startIR.dir/src/startIR.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/brent/ros_ws/k2_client/src/startIR.cpp -o CMakeFiles/startIR.dir/src/startIR.cpp.s

CMakeFiles/startIR.dir/src/startIR.cpp.o.requires:
.PHONY : CMakeFiles/startIR.dir/src/startIR.cpp.o.requires

CMakeFiles/startIR.dir/src/startIR.cpp.o.provides: CMakeFiles/startIR.dir/src/startIR.cpp.o.requires
	$(MAKE) -f CMakeFiles/startIR.dir/build.make CMakeFiles/startIR.dir/src/startIR.cpp.o.provides.build
.PHONY : CMakeFiles/startIR.dir/src/startIR.cpp.o.provides

CMakeFiles/startIR.dir/src/startIR.cpp.o.provides.build: CMakeFiles/startIR.dir/src/startIR.cpp.o

# Object files for target startIR
startIR_OBJECTS = \
"CMakeFiles/startIR.dir/src/startIR.cpp.o"

# External object files for target startIR
startIR_EXTERNAL_OBJECTS =

../bin/startIR: CMakeFiles/startIR.dir/src/startIR.cpp.o
../bin/startIR: CMakeFiles/startIR.dir/build.make
../bin/startIR: CMakeFiles/startIR.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --red --bold "Linking CXX executable ../bin/startIR"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/startIR.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/startIR.dir/build: ../bin/startIR
.PHONY : CMakeFiles/startIR.dir/build

CMakeFiles/startIR.dir/requires: CMakeFiles/startIR.dir/src/startIR.cpp.o.requires
.PHONY : CMakeFiles/startIR.dir/requires

CMakeFiles/startIR.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/startIR.dir/cmake_clean.cmake
.PHONY : CMakeFiles/startIR.dir/clean

CMakeFiles/startIR.dir/depend:
	cd /home/brent/ros_ws/k2_client/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/brent/ros_ws/k2_client /home/brent/ros_ws/k2_client /home/brent/ros_ws/k2_client/build /home/brent/ros_ws/k2_client/build /home/brent/ros_ws/k2_client/build/CMakeFiles/startIR.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/startIR.dir/depend

