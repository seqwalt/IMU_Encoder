# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.25

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:

#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:

# Disable VCS-based implicit rules.
% : %,v

# Disable VCS-based implicit rules.
% : RCS/%

# Disable VCS-based implicit rules.
% : RCS/%,v

# Disable VCS-based implicit rules.
% : SCCS/s.%

# Disable VCS-based implicit rules.
% : s.%

.SUFFIXES: .hpux_make_needs_suffix_list

# Command-line flag to silence nested $(MAKE).
$(VERBOSE)MAKESILENT = -s

#Suppress display of executed commands.
$(VERBOSE).SILENT:

# A target that is always out of date.
cmake_force:
.PHONY : cmake_force

#=============================================================================
# Set environment variables for the build.

# The shell in which to execute make rules.
SHELL = /bin/sh

# The CMake executable.
CMAKE_COMMAND = /usr/local/Cellar/cmake/3.25.2/bin/cmake

# The command to remove a file.
RM = /usr/local/Cellar/cmake/3.25.2/bin/cmake -E rm -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /Users/sequoyah/Documents/Arduino/libraries/ImuEKF

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /Users/sequoyah/Documents/Arduino/libraries/ImuEKF/build

# Include any dependencies generated for this target.
include CMakeFiles/imu_ekf.dir/depend.make
# Include any dependencies generated by the compiler for this target.
include CMakeFiles/imu_ekf.dir/compiler_depend.make

# Include the progress variables for this target.
include CMakeFiles/imu_ekf.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/imu_ekf.dir/flags.make

CMakeFiles/imu_ekf.dir/ImuEKF.cpp.o: CMakeFiles/imu_ekf.dir/flags.make
CMakeFiles/imu_ekf.dir/ImuEKF.cpp.o: /Users/sequoyah/Documents/Arduino/libraries/ImuEKF/ImuEKF.cpp
CMakeFiles/imu_ekf.dir/ImuEKF.cpp.o: CMakeFiles/imu_ekf.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/Users/sequoyah/Documents/Arduino/libraries/ImuEKF/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/imu_ekf.dir/ImuEKF.cpp.o"
	/Library/Developer/CommandLineTools/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/imu_ekf.dir/ImuEKF.cpp.o -MF CMakeFiles/imu_ekf.dir/ImuEKF.cpp.o.d -o CMakeFiles/imu_ekf.dir/ImuEKF.cpp.o -c /Users/sequoyah/Documents/Arduino/libraries/ImuEKF/ImuEKF.cpp

CMakeFiles/imu_ekf.dir/ImuEKF.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/imu_ekf.dir/ImuEKF.cpp.i"
	/Library/Developer/CommandLineTools/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /Users/sequoyah/Documents/Arduino/libraries/ImuEKF/ImuEKF.cpp > CMakeFiles/imu_ekf.dir/ImuEKF.cpp.i

CMakeFiles/imu_ekf.dir/ImuEKF.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/imu_ekf.dir/ImuEKF.cpp.s"
	/Library/Developer/CommandLineTools/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /Users/sequoyah/Documents/Arduino/libraries/ImuEKF/ImuEKF.cpp -o CMakeFiles/imu_ekf.dir/ImuEKF.cpp.s

# Object files for target imu_ekf
imu_ekf_OBJECTS = \
"CMakeFiles/imu_ekf.dir/ImuEKF.cpp.o"

# External object files for target imu_ekf
imu_ekf_EXTERNAL_OBJECTS =

libimu_ekf.a: CMakeFiles/imu_ekf.dir/ImuEKF.cpp.o
libimu_ekf.a: CMakeFiles/imu_ekf.dir/build.make
libimu_ekf.a: CMakeFiles/imu_ekf.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/Users/sequoyah/Documents/Arduino/libraries/ImuEKF/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX static library libimu_ekf.a"
	$(CMAKE_COMMAND) -P CMakeFiles/imu_ekf.dir/cmake_clean_target.cmake
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/imu_ekf.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/imu_ekf.dir/build: libimu_ekf.a
.PHONY : CMakeFiles/imu_ekf.dir/build

CMakeFiles/imu_ekf.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/imu_ekf.dir/cmake_clean.cmake
.PHONY : CMakeFiles/imu_ekf.dir/clean

CMakeFiles/imu_ekf.dir/depend:
	cd /Users/sequoyah/Documents/Arduino/libraries/ImuEKF/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /Users/sequoyah/Documents/Arduino/libraries/ImuEKF /Users/sequoyah/Documents/Arduino/libraries/ImuEKF /Users/sequoyah/Documents/Arduino/libraries/ImuEKF/build /Users/sequoyah/Documents/Arduino/libraries/ImuEKF/build /Users/sequoyah/Documents/Arduino/libraries/ImuEKF/build/CMakeFiles/imu_ekf.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/imu_ekf.dir/depend
