# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.7

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
CMAKE_SOURCE_DIR = /home/multipurpose/Design2/OpenCV

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/multipurpose/Design2/OpenCV/build

# Include any dependencies generated for this target.
include samples/cpp/CMakeFiles/example_filestorage.dir/depend.make

# Include the progress variables for this target.
include samples/cpp/CMakeFiles/example_filestorage.dir/progress.make

# Include the compile flags for this target's objects.
include samples/cpp/CMakeFiles/example_filestorage.dir/flags.make

samples/cpp/CMakeFiles/example_filestorage.dir/filestorage.cpp.o: samples/cpp/CMakeFiles/example_filestorage.dir/flags.make
samples/cpp/CMakeFiles/example_filestorage.dir/filestorage.cpp.o: ../samples/cpp/filestorage.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/multipurpose/Design2/OpenCV/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object samples/cpp/CMakeFiles/example_filestorage.dir/filestorage.cpp.o"
	cd /home/multipurpose/Design2/OpenCV/build/samples/cpp && /usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/example_filestorage.dir/filestorage.cpp.o -c /home/multipurpose/Design2/OpenCV/samples/cpp/filestorage.cpp

samples/cpp/CMakeFiles/example_filestorage.dir/filestorage.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/example_filestorage.dir/filestorage.cpp.i"
	cd /home/multipurpose/Design2/OpenCV/build/samples/cpp && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/multipurpose/Design2/OpenCV/samples/cpp/filestorage.cpp > CMakeFiles/example_filestorage.dir/filestorage.cpp.i

samples/cpp/CMakeFiles/example_filestorage.dir/filestorage.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/example_filestorage.dir/filestorage.cpp.s"
	cd /home/multipurpose/Design2/OpenCV/build/samples/cpp && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/multipurpose/Design2/OpenCV/samples/cpp/filestorage.cpp -o CMakeFiles/example_filestorage.dir/filestorage.cpp.s

samples/cpp/CMakeFiles/example_filestorage.dir/filestorage.cpp.o.requires:

.PHONY : samples/cpp/CMakeFiles/example_filestorage.dir/filestorage.cpp.o.requires

samples/cpp/CMakeFiles/example_filestorage.dir/filestorage.cpp.o.provides: samples/cpp/CMakeFiles/example_filestorage.dir/filestorage.cpp.o.requires
	$(MAKE) -f samples/cpp/CMakeFiles/example_filestorage.dir/build.make samples/cpp/CMakeFiles/example_filestorage.dir/filestorage.cpp.o.provides.build
.PHONY : samples/cpp/CMakeFiles/example_filestorage.dir/filestorage.cpp.o.provides

samples/cpp/CMakeFiles/example_filestorage.dir/filestorage.cpp.o.provides.build: samples/cpp/CMakeFiles/example_filestorage.dir/filestorage.cpp.o


# Object files for target example_filestorage
example_filestorage_OBJECTS = \
"CMakeFiles/example_filestorage.dir/filestorage.cpp.o"

# External object files for target example_filestorage
example_filestorage_EXTERNAL_OBJECTS =

bin/cpp-example-filestorage: samples/cpp/CMakeFiles/example_filestorage.dir/filestorage.cpp.o
bin/cpp-example-filestorage: samples/cpp/CMakeFiles/example_filestorage.dir/build.make
bin/cpp-example-filestorage: /usr/lib/i386-linux-gnu/libGLU.so
bin/cpp-example-filestorage: /usr/lib/i386-linux-gnu/libGL.so
bin/cpp-example-filestorage: /usr/lib/i386-linux-gnu/libtbb.so
bin/cpp-example-filestorage: lib/libopencv_shape.so.3.2.0
bin/cpp-example-filestorage: lib/libopencv_stitching.so.3.2.0
bin/cpp-example-filestorage: lib/libopencv_superres.so.3.2.0
bin/cpp-example-filestorage: lib/libopencv_videostab.so.3.2.0
bin/cpp-example-filestorage: lib/libopencv_viz.so.3.2.0
bin/cpp-example-filestorage: lib/libopencv_objdetect.so.3.2.0
bin/cpp-example-filestorage: lib/libopencv_photo.so.3.2.0
bin/cpp-example-filestorage: lib/libopencv_calib3d.so.3.2.0
bin/cpp-example-filestorage: lib/libopencv_features2d.so.3.2.0
bin/cpp-example-filestorage: lib/libopencv_flann.so.3.2.0
bin/cpp-example-filestorage: lib/libopencv_highgui.so.3.2.0
bin/cpp-example-filestorage: lib/libopencv_ml.so.3.2.0
bin/cpp-example-filestorage: lib/libopencv_videoio.so.3.2.0
bin/cpp-example-filestorage: lib/libopencv_imgcodecs.so.3.2.0
bin/cpp-example-filestorage: lib/libopencv_video.so.3.2.0
bin/cpp-example-filestorage: lib/libopencv_imgproc.so.3.2.0
bin/cpp-example-filestorage: lib/libopencv_core.so.3.2.0
bin/cpp-example-filestorage: /usr/lib/i386-linux-gnu/libGLU.so
bin/cpp-example-filestorage: /usr/lib/i386-linux-gnu/libGL.so
bin/cpp-example-filestorage: samples/cpp/CMakeFiles/example_filestorage.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/multipurpose/Design2/OpenCV/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable ../../bin/cpp-example-filestorage"
	cd /home/multipurpose/Design2/OpenCV/build/samples/cpp && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/example_filestorage.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
samples/cpp/CMakeFiles/example_filestorage.dir/build: bin/cpp-example-filestorage

.PHONY : samples/cpp/CMakeFiles/example_filestorage.dir/build

samples/cpp/CMakeFiles/example_filestorage.dir/requires: samples/cpp/CMakeFiles/example_filestorage.dir/filestorage.cpp.o.requires

.PHONY : samples/cpp/CMakeFiles/example_filestorage.dir/requires

samples/cpp/CMakeFiles/example_filestorage.dir/clean:
	cd /home/multipurpose/Design2/OpenCV/build/samples/cpp && $(CMAKE_COMMAND) -P CMakeFiles/example_filestorage.dir/cmake_clean.cmake
.PHONY : samples/cpp/CMakeFiles/example_filestorage.dir/clean

samples/cpp/CMakeFiles/example_filestorage.dir/depend:
	cd /home/multipurpose/Design2/OpenCV/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/multipurpose/Design2/OpenCV /home/multipurpose/Design2/OpenCV/samples/cpp /home/multipurpose/Design2/OpenCV/build /home/multipurpose/Design2/OpenCV/build/samples/cpp /home/multipurpose/Design2/OpenCV/build/samples/cpp/CMakeFiles/example_filestorage.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : samples/cpp/CMakeFiles/example_filestorage.dir/depend

