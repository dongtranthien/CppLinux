# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.10

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
CMAKE_SOURCE_DIR = /home/idea/Documents/CppLinux/005_Serial

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/idea/Documents/CppLinux/005_Serial/build

# Include any dependencies generated for this target.
include CMakeFiles/ExampleProject.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/ExampleProject.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/ExampleProject.dir/flags.make

CMakeFiles/ExampleProject.dir/main.cpp.o: CMakeFiles/ExampleProject.dir/flags.make
CMakeFiles/ExampleProject.dir/main.cpp.o: ../main.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/idea/Documents/CppLinux/005_Serial/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/ExampleProject.dir/main.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/ExampleProject.dir/main.cpp.o -c /home/idea/Documents/CppLinux/005_Serial/main.cpp

CMakeFiles/ExampleProject.dir/main.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/ExampleProject.dir/main.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/idea/Documents/CppLinux/005_Serial/main.cpp > CMakeFiles/ExampleProject.dir/main.cpp.i

CMakeFiles/ExampleProject.dir/main.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/ExampleProject.dir/main.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/idea/Documents/CppLinux/005_Serial/main.cpp -o CMakeFiles/ExampleProject.dir/main.cpp.s

CMakeFiles/ExampleProject.dir/main.cpp.o.requires:

.PHONY : CMakeFiles/ExampleProject.dir/main.cpp.o.requires

CMakeFiles/ExampleProject.dir/main.cpp.o.provides: CMakeFiles/ExampleProject.dir/main.cpp.o.requires
	$(MAKE) -f CMakeFiles/ExampleProject.dir/build.make CMakeFiles/ExampleProject.dir/main.cpp.o.provides.build
.PHONY : CMakeFiles/ExampleProject.dir/main.cpp.o.provides

CMakeFiles/ExampleProject.dir/main.cpp.o.provides.build: CMakeFiles/ExampleProject.dir/main.cpp.o


# Object files for target ExampleProject
ExampleProject_OBJECTS = \
"CMakeFiles/ExampleProject.dir/main.cpp.o"

# External object files for target ExampleProject
ExampleProject_EXTERNAL_OBJECTS =

ExampleProject: CMakeFiles/ExampleProject.dir/main.cpp.o
ExampleProject: CMakeFiles/ExampleProject.dir/build.make
ExampleProject: CMakeFiles/ExampleProject.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/idea/Documents/CppLinux/005_Serial/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable ExampleProject"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/ExampleProject.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/ExampleProject.dir/build: ExampleProject

.PHONY : CMakeFiles/ExampleProject.dir/build

CMakeFiles/ExampleProject.dir/requires: CMakeFiles/ExampleProject.dir/main.cpp.o.requires

.PHONY : CMakeFiles/ExampleProject.dir/requires

CMakeFiles/ExampleProject.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/ExampleProject.dir/cmake_clean.cmake
.PHONY : CMakeFiles/ExampleProject.dir/clean

CMakeFiles/ExampleProject.dir/depend:
	cd /home/idea/Documents/CppLinux/005_Serial/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/idea/Documents/CppLinux/005_Serial /home/idea/Documents/CppLinux/005_Serial /home/idea/Documents/CppLinux/005_Serial/build /home/idea/Documents/CppLinux/005_Serial/build /home/idea/Documents/CppLinux/005_Serial/build/CMakeFiles/ExampleProject.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/ExampleProject.dir/depend

