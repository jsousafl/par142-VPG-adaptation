# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.12

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
CMAKE_SOURCE_DIR = /home/ecl/robot/visual-pushing-grasping/realsense

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/ecl/robot/visual-pushing-grasping/realsense/build

# Include any dependencies generated for this target.
include CMakeFiles/realsense.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/realsense.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/realsense.dir/flags.make

CMakeFiles/realsense.dir/realsense.cpp.o: CMakeFiles/realsense.dir/flags.make
CMakeFiles/realsense.dir/realsense.cpp.o: ../realsense.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/ecl/robot/visual-pushing-grasping/realsense/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/realsense.dir/realsense.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/realsense.dir/realsense.cpp.o -c /home/ecl/robot/visual-pushing-grasping/realsense/realsense.cpp

CMakeFiles/realsense.dir/realsense.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/realsense.dir/realsense.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/ecl/robot/visual-pushing-grasping/realsense/realsense.cpp > CMakeFiles/realsense.dir/realsense.cpp.i

CMakeFiles/realsense.dir/realsense.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/realsense.dir/realsense.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/ecl/robot/visual-pushing-grasping/realsense/realsense.cpp -o CMakeFiles/realsense.dir/realsense.cpp.s

# Object files for target realsense
realsense_OBJECTS = \
"CMakeFiles/realsense.dir/realsense.cpp.o"

# External object files for target realsense
realsense_EXTERNAL_OBJECTS =

realsense: CMakeFiles/realsense.dir/realsense.cpp.o
realsense: CMakeFiles/realsense.dir/build.make
realsense: /usr/lib/x86_64-linux-gnu/libGL.so
realsense: /usr/lib/x86_64-linux-gnu/libGLU.so
realsense: /usr/lib/x86_64-linux-gnu/libglfw.so
realsense: CMakeFiles/realsense.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/ecl/robot/visual-pushing-grasping/realsense/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable realsense"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/realsense.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/realsense.dir/build: realsense

.PHONY : CMakeFiles/realsense.dir/build

CMakeFiles/realsense.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/realsense.dir/cmake_clean.cmake
.PHONY : CMakeFiles/realsense.dir/clean

CMakeFiles/realsense.dir/depend:
	cd /home/ecl/robot/visual-pushing-grasping/realsense/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/ecl/robot/visual-pushing-grasping/realsense /home/ecl/robot/visual-pushing-grasping/realsense /home/ecl/robot/visual-pushing-grasping/realsense/build /home/ecl/robot/visual-pushing-grasping/realsense/build /home/ecl/robot/visual-pushing-grasping/realsense/build/CMakeFiles/realsense.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/realsense.dir/depend

