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
CMAKE_SOURCE_DIR = /home/ws/spark/hand-eye2

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/ws/spark/hand-eye2/build

# Include any dependencies generated for this target.
include CMakeFiles/pop.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/pop.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/pop.dir/flags.make

CMakeFiles/pop.dir/pop_ideas.cpp.o: CMakeFiles/pop.dir/flags.make
CMakeFiles/pop.dir/pop_ideas.cpp.o: ../pop_ideas.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/ws/spark/hand-eye2/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/pop.dir/pop_ideas.cpp.o"
	/bin/x86_64-linux-gnu-g++-9  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/pop.dir/pop_ideas.cpp.o -c /home/ws/spark/hand-eye2/pop_ideas.cpp

CMakeFiles/pop.dir/pop_ideas.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/pop.dir/pop_ideas.cpp.i"
	/bin/x86_64-linux-gnu-g++-9 $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/ws/spark/hand-eye2/pop_ideas.cpp > CMakeFiles/pop.dir/pop_ideas.cpp.i

CMakeFiles/pop.dir/pop_ideas.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/pop.dir/pop_ideas.cpp.s"
	/bin/x86_64-linux-gnu-g++-9 $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/ws/spark/hand-eye2/pop_ideas.cpp -o CMakeFiles/pop.dir/pop_ideas.cpp.s

# Object files for target pop
pop_OBJECTS = \
"CMakeFiles/pop.dir/pop_ideas.cpp.o"

# External object files for target pop
pop_EXTERNAL_OBJECTS =

../bin/pop: CMakeFiles/pop.dir/pop_ideas.cpp.o
../bin/pop: CMakeFiles/pop.dir/build.make
../bin/pop: CMakeFiles/pop.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/ws/spark/hand-eye2/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable ../bin/pop"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/pop.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/pop.dir/build: ../bin/pop

.PHONY : CMakeFiles/pop.dir/build

CMakeFiles/pop.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/pop.dir/cmake_clean.cmake
.PHONY : CMakeFiles/pop.dir/clean

CMakeFiles/pop.dir/depend:
	cd /home/ws/spark/hand-eye2/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/ws/spark/hand-eye2 /home/ws/spark/hand-eye2 /home/ws/spark/hand-eye2/build /home/ws/spark/hand-eye2/build /home/ws/spark/hand-eye2/build/CMakeFiles/pop.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/pop.dir/depend

