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
include samples/cpp/CMakeFiles/tutorial_bg_sub.dir/depend.make

# Include the progress variables for this target.
include samples/cpp/CMakeFiles/tutorial_bg_sub.dir/progress.make

# Include the compile flags for this target's objects.
include samples/cpp/CMakeFiles/tutorial_bg_sub.dir/flags.make

samples/cpp/CMakeFiles/tutorial_bg_sub.dir/tutorial_code/video/bg_sub.cpp.o: samples/cpp/CMakeFiles/tutorial_bg_sub.dir/flags.make
samples/cpp/CMakeFiles/tutorial_bg_sub.dir/tutorial_code/video/bg_sub.cpp.o: ../samples/cpp/tutorial_code/video/bg_sub.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/multipurpose/Design2/OpenCV/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object samples/cpp/CMakeFiles/tutorial_bg_sub.dir/tutorial_code/video/bg_sub.cpp.o"
	cd /home/multipurpose/Design2/OpenCV/build/samples/cpp && /usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/tutorial_bg_sub.dir/tutorial_code/video/bg_sub.cpp.o -c /home/multipurpose/Design2/OpenCV/samples/cpp/tutorial_code/video/bg_sub.cpp

samples/cpp/CMakeFiles/tutorial_bg_sub.dir/tutorial_code/video/bg_sub.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/tutorial_bg_sub.dir/tutorial_code/video/bg_sub.cpp.i"
	cd /home/multipurpose/Design2/OpenCV/build/samples/cpp && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/multipurpose/Design2/OpenCV/samples/cpp/tutorial_code/video/bg_sub.cpp > CMakeFiles/tutorial_bg_sub.dir/tutorial_code/video/bg_sub.cpp.i

samples/cpp/CMakeFiles/tutorial_bg_sub.dir/tutorial_code/video/bg_sub.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/tutorial_bg_sub.dir/tutorial_code/video/bg_sub.cpp.s"
	cd /home/multipurpose/Design2/OpenCV/build/samples/cpp && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/multipurpose/Design2/OpenCV/samples/cpp/tutorial_code/video/bg_sub.cpp -o CMakeFiles/tutorial_bg_sub.dir/tutorial_code/video/bg_sub.cpp.s

samples/cpp/CMakeFiles/tutorial_bg_sub.dir/tutorial_code/video/bg_sub.cpp.o.requires:

.PHONY : samples/cpp/CMakeFiles/tutorial_bg_sub.dir/tutorial_code/video/bg_sub.cpp.o.requires

samples/cpp/CMakeFiles/tutorial_bg_sub.dir/tutorial_code/video/bg_sub.cpp.o.provides: samples/cpp/CMakeFiles/tutorial_bg_sub.dir/tutorial_code/video/bg_sub.cpp.o.requires
	$(MAKE) -f samples/cpp/CMakeFiles/tutorial_bg_sub.dir/build.make samples/cpp/CMakeFiles/tutorial_bg_sub.dir/tutorial_code/video/bg_sub.cpp.o.provides.build
.PHONY : samples/cpp/CMakeFiles/tutorial_bg_sub.dir/tutorial_code/video/bg_sub.cpp.o.provides

samples/cpp/CMakeFiles/tutorial_bg_sub.dir/tutorial_code/video/bg_sub.cpp.o.provides.build: samples/cpp/CMakeFiles/tutorial_bg_sub.dir/tutorial_code/video/bg_sub.cpp.o


# Object files for target tutorial_bg_sub
tutorial_bg_sub_OBJECTS = \
"CMakeFiles/tutorial_bg_sub.dir/tutorial_code/video/bg_sub.cpp.o"

# External object files for target tutorial_bg_sub
tutorial_bg_sub_EXTERNAL_OBJECTS =

bin/cpp-tutorial-bg_sub: samples/cpp/CMakeFiles/tutorial_bg_sub.dir/tutorial_code/video/bg_sub.cpp.o
bin/cpp-tutorial-bg_sub: samples/cpp/CMakeFiles/tutorial_bg_sub.dir/build.make
bin/cpp-tutorial-bg_sub: /usr/lib/i386-linux-gnu/libGLU.so
bin/cpp-tutorial-bg_sub: /usr/lib/i386-linux-gnu/libGL.so
bin/cpp-tutorial-bg_sub: /usr/lib/i386-linux-gnu/libtbb.so
bin/cpp-tutorial-bg_sub: lib/libopencv_shape.so.3.2.0
bin/cpp-tutorial-bg_sub: lib/libopencv_stitching.so.3.2.0
bin/cpp-tutorial-bg_sub: lib/libopencv_superres.so.3.2.0
bin/cpp-tutorial-bg_sub: lib/libopencv_videostab.so.3.2.0
bin/cpp-tutorial-bg_sub: lib/libopencv_viz.so.3.2.0
bin/cpp-tutorial-bg_sub: lib/libopencv_objdetect.so.3.2.0
bin/cpp-tutorial-bg_sub: lib/libopencv_photo.so.3.2.0
bin/cpp-tutorial-bg_sub: lib/libopencv_calib3d.so.3.2.0
bin/cpp-tutorial-bg_sub: lib/libopencv_features2d.so.3.2.0
bin/cpp-tutorial-bg_sub: lib/libopencv_flann.so.3.2.0
bin/cpp-tutorial-bg_sub: lib/libopencv_highgui.so.3.2.0
bin/cpp-tutorial-bg_sub: lib/libopencv_ml.so.3.2.0
bin/cpp-tutorial-bg_sub: lib/libopencv_videoio.so.3.2.0
bin/cpp-tutorial-bg_sub: lib/libopencv_imgcodecs.so.3.2.0
bin/cpp-tutorial-bg_sub: lib/libopencv_video.so.3.2.0
bin/cpp-tutorial-bg_sub: lib/libopencv_imgproc.so.3.2.0
bin/cpp-tutorial-bg_sub: lib/libopencv_core.so.3.2.0
bin/cpp-tutorial-bg_sub: /usr/lib/i386-linux-gnu/libGLU.so
bin/cpp-tutorial-bg_sub: /usr/lib/i386-linux-gnu/libGL.so
bin/cpp-tutorial-bg_sub: samples/cpp/CMakeFiles/tutorial_bg_sub.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/multipurpose/Design2/OpenCV/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable ../../bin/cpp-tutorial-bg_sub"
	cd /home/multipurpose/Design2/OpenCV/build/samples/cpp && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/tutorial_bg_sub.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
samples/cpp/CMakeFiles/tutorial_bg_sub.dir/build: bin/cpp-tutorial-bg_sub

.PHONY : samples/cpp/CMakeFiles/tutorial_bg_sub.dir/build

samples/cpp/CMakeFiles/tutorial_bg_sub.dir/requires: samples/cpp/CMakeFiles/tutorial_bg_sub.dir/tutorial_code/video/bg_sub.cpp.o.requires

.PHONY : samples/cpp/CMakeFiles/tutorial_bg_sub.dir/requires

samples/cpp/CMakeFiles/tutorial_bg_sub.dir/clean:
	cd /home/multipurpose/Design2/OpenCV/build/samples/cpp && $(CMAKE_COMMAND) -P CMakeFiles/tutorial_bg_sub.dir/cmake_clean.cmake
.PHONY : samples/cpp/CMakeFiles/tutorial_bg_sub.dir/clean

samples/cpp/CMakeFiles/tutorial_bg_sub.dir/depend:
	cd /home/multipurpose/Design2/OpenCV/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/multipurpose/Design2/OpenCV /home/multipurpose/Design2/OpenCV/samples/cpp /home/multipurpose/Design2/OpenCV/build /home/multipurpose/Design2/OpenCV/build/samples/cpp /home/multipurpose/Design2/OpenCV/build/samples/cpp/CMakeFiles/tutorial_bg_sub.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : samples/cpp/CMakeFiles/tutorial_bg_sub.dir/depend

