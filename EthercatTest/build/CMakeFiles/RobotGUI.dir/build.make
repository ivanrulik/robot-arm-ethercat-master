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
include CMakeFiles/RobotGUI.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/RobotGUI.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/RobotGUI.dir/flags.make

CMakeFiles/RobotGUI.dir/robotGUI.cpp.o: CMakeFiles/RobotGUI.dir/flags.make
CMakeFiles/RobotGUI.dir/robotGUI.cpp.o: ../robotGUI.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/ivanrulik/thesis_ws/ETHERCAT_PROJECT-main/EthercatTest/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/RobotGUI.dir/robotGUI.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/RobotGUI.dir/robotGUI.cpp.o -c /home/ivanrulik/thesis_ws/ETHERCAT_PROJECT-main/EthercatTest/robotGUI.cpp

CMakeFiles/RobotGUI.dir/robotGUI.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/RobotGUI.dir/robotGUI.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/ivanrulik/thesis_ws/ETHERCAT_PROJECT-main/EthercatTest/robotGUI.cpp > CMakeFiles/RobotGUI.dir/robotGUI.cpp.i

CMakeFiles/RobotGUI.dir/robotGUI.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/RobotGUI.dir/robotGUI.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/ivanrulik/thesis_ws/ETHERCAT_PROJECT-main/EthercatTest/robotGUI.cpp -o CMakeFiles/RobotGUI.dir/robotGUI.cpp.s

# Object files for target RobotGUI
RobotGUI_OBJECTS = \
"CMakeFiles/RobotGUI.dir/robotGUI.cpp.o"

# External object files for target RobotGUI
RobotGUI_EXTERNAL_OBJECTS =

RobotGUI: CMakeFiles/RobotGUI.dir/robotGUI.cpp.o
RobotGUI: CMakeFiles/RobotGUI.dir/build.make
RobotGUI: /usr/local/lib/libsoem.a
RobotGUI: CMakeFiles/RobotGUI.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/ivanrulik/thesis_ws/ETHERCAT_PROJECT-main/EthercatTest/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable RobotGUI"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/RobotGUI.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/RobotGUI.dir/build: RobotGUI

.PHONY : CMakeFiles/RobotGUI.dir/build

CMakeFiles/RobotGUI.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/RobotGUI.dir/cmake_clean.cmake
.PHONY : CMakeFiles/RobotGUI.dir/clean

CMakeFiles/RobotGUI.dir/depend:
	cd /home/ivanrulik/thesis_ws/ETHERCAT_PROJECT-main/EthercatTest/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/ivanrulik/thesis_ws/ETHERCAT_PROJECT-main/EthercatTest /home/ivanrulik/thesis_ws/ETHERCAT_PROJECT-main/EthercatTest /home/ivanrulik/thesis_ws/ETHERCAT_PROJECT-main/EthercatTest/build /home/ivanrulik/thesis_ws/ETHERCAT_PROJECT-main/EthercatTest/build /home/ivanrulik/thesis_ws/ETHERCAT_PROJECT-main/EthercatTest/build/CMakeFiles/RobotGUI.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/RobotGUI.dir/depend
