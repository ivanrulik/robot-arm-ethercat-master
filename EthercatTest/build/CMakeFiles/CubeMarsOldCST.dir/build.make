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
CMAKE_SOURCE_DIR = /home/ivanrulik/thesis_ws/ETHERCAT_PROJECT-main/EthercatTest

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/ivanrulik/thesis_ws/ETHERCAT_PROJECT-main/EthercatTest/build

# Include any dependencies generated for this target.
include CMakeFiles/CubeMarsOldCST.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/CubeMarsOldCST.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/CubeMarsOldCST.dir/flags.make

CMakeFiles/CubeMarsOldCST.dir/EthercatCubemarsOLD.cpp.o: CMakeFiles/CubeMarsOldCST.dir/flags.make
CMakeFiles/CubeMarsOldCST.dir/EthercatCubemarsOLD.cpp.o: ../EthercatCubemarsOLD.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/ivanrulik/thesis_ws/ETHERCAT_PROJECT-main/EthercatTest/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/CubeMarsOldCST.dir/EthercatCubemarsOLD.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/CubeMarsOldCST.dir/EthercatCubemarsOLD.cpp.o -c /home/ivanrulik/thesis_ws/ETHERCAT_PROJECT-main/EthercatTest/EthercatCubemarsOLD.cpp

CMakeFiles/CubeMarsOldCST.dir/EthercatCubemarsOLD.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/CubeMarsOldCST.dir/EthercatCubemarsOLD.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/ivanrulik/thesis_ws/ETHERCAT_PROJECT-main/EthercatTest/EthercatCubemarsOLD.cpp > CMakeFiles/CubeMarsOldCST.dir/EthercatCubemarsOLD.cpp.i

CMakeFiles/CubeMarsOldCST.dir/EthercatCubemarsOLD.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/CubeMarsOldCST.dir/EthercatCubemarsOLD.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/ivanrulik/thesis_ws/ETHERCAT_PROJECT-main/EthercatTest/EthercatCubemarsOLD.cpp -o CMakeFiles/CubeMarsOldCST.dir/EthercatCubemarsOLD.cpp.s

# Object files for target CubeMarsOldCST
CubeMarsOldCST_OBJECTS = \
"CMakeFiles/CubeMarsOldCST.dir/EthercatCubemarsOLD.cpp.o"

# External object files for target CubeMarsOldCST
CubeMarsOldCST_EXTERNAL_OBJECTS =

CubeMarsOldCST: CMakeFiles/CubeMarsOldCST.dir/EthercatCubemarsOLD.cpp.o
CubeMarsOldCST: CMakeFiles/CubeMarsOldCST.dir/build.make
CubeMarsOldCST: /usr/local/lib/libsoem.a
CubeMarsOldCST: CMakeFiles/CubeMarsOldCST.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/ivanrulik/thesis_ws/ETHERCAT_PROJECT-main/EthercatTest/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable CubeMarsOldCST"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/CubeMarsOldCST.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/CubeMarsOldCST.dir/build: CubeMarsOldCST

.PHONY : CMakeFiles/CubeMarsOldCST.dir/build

CMakeFiles/CubeMarsOldCST.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/CubeMarsOldCST.dir/cmake_clean.cmake
.PHONY : CMakeFiles/CubeMarsOldCST.dir/clean

CMakeFiles/CubeMarsOldCST.dir/depend:
	cd /home/ivanrulik/thesis_ws/ETHERCAT_PROJECT-main/EthercatTest/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/ivanrulik/thesis_ws/ETHERCAT_PROJECT-main/EthercatTest /home/ivanrulik/thesis_ws/ETHERCAT_PROJECT-main/EthercatTest /home/ivanrulik/thesis_ws/ETHERCAT_PROJECT-main/EthercatTest/build /home/ivanrulik/thesis_ws/ETHERCAT_PROJECT-main/EthercatTest/build /home/ivanrulik/thesis_ws/ETHERCAT_PROJECT-main/EthercatTest/build/CMakeFiles/CubeMarsOldCST.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/CubeMarsOldCST.dir/depend
