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
include samples/cpp/CMakeFiles/example_detect_blob.dir/depend.make

# Include the progress variables for this target.
include samples/cpp/CMakeFiles/example_detect_blob.dir/progress.make

# Include the compile flags for this target's objects.
include samples/cpp/CMakeFiles/example_detect_blob.dir/flags.make

samples/cpp/CMakeFiles/example_detect_blob.dir/detect_blob.cpp.o: samples/cpp/CMakeFiles/example_detect_blob.dir/flags.make
samples/cpp/CMakeFiles/example_detect_blob.dir/detect_blob.cpp.o: ../samples/cpp/detect_blob.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/multipurpose/Design2/OpenCV/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object samples/cpp/CMakeFiles/example_detect_blob.dir/detect_blob.cpp.o"
	cd /home/multipurpose/Design2/OpenCV/build/samples/cpp && /usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/example_detect_blob.dir/detect_blob.cpp.o -c /home/multipurpose/Design2/OpenCV/samples/cpp/detect_blob.cpp

samples/cpp/CMakeFiles/example_detect_blob.dir/detect_blob.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/example_detect_blob.dir/detect_blob.cpp.i"
	cd /home/multipurpose/Design2/OpenCV/build/samples/cpp && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/multipurpose/Design2/OpenCV/samples/cpp/detect_blob.cpp > CMakeFiles/example_detect_blob.dir/detect_blob.cpp.i

samples/cpp/CMakeFiles/example_detect_blob.dir/detect_blob.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/example_detect_blob.dir/detect_blob.cpp.s"
	cd /home/multipurpose/Design2/OpenCV/build/samples/cpp && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/multipurpose/Design2/OpenCV/samples/cpp/detect_blob.cpp -o CMakeFiles/example_detect_blob.dir/detect_blob.cpp.s

samples/cpp/CMakeFiles/example_detect_blob.dir/detect_blob.cpp.o.requires:

.PHONY : samples/cpp/CMakeFiles/example_detect_blob.dir/detect_blob.cpp.o.requires

samples/cpp/CMakeFiles/example_detect_blob.dir/detect_blob.cpp.o.provides: samples/cpp/CMakeFiles/example_detect_blob.dir/detect_blob.cpp.o.requires
	$(MAKE) -f samples/cpp/CMakeFiles/example_detect_blob.dir/build.make samples/cpp/CMakeFiles/example_detect_blob.dir/detect_blob.cpp.o.provides.build
.PHONY : samples/cpp/CMakeFiles/example_detect_blob.dir/detect_blob.cpp.o.provides

samples/cpp/CMakeFiles/example_detect_blob.dir/detect_blob.cpp.o.provides.build: samples/cpp/CMakeFiles/example_detect_blob.dir/detect_blob.cpp.o


# Object files for target example_detect_blob
example_detect_blob_OBJECTS = \
"CMakeFiles/example_detect_blob.dir/detect_blob.cpp.o"

# External object files for target example_detect_blob
example_detect_blob_EXTERNAL_OBJECTS =

bin/cpp-example-detect_blob: samples/cpp/CMakeFiles/example_detect_blob.dir/detect_blob.cpp.o
bin/cpp-example-detect_blob: samples/cpp/CMakeFiles/example_detect_blob.dir/build.make
bin/cpp-example-detect_blob: /usr/lib/i386-linux-gnu/libGLU.so
bin/cpp-example-detect_blob: /usr/lib/i386-linux-gnu/libGL.so
bin/cpp-example-detect_blob: /usr/lib/i386-linux-gnu/libtbb.so
bin/cpp-example-detect_blob: lib/libopencv_shape.so.3.2.0
bin/cpp-example-detect_blob: lib/libopencv_stitching.so.3.2.0
bin/cpp-example-detect_blob: lib/libopencv_superres.so.3.2.0
bin/cpp-example-detect_blob: lib/libopencv_videostab.so.3.2.0
bin/cpp-example-detect_blob: lib/libopencv_viz.so.3.2.0
bin/cpp-example-detect_blob: lib/libopencv_objdetect.so.3.2.0
bin/cpp-example-detect_blob: lib/libopencv_photo.so.3.2.0
bin/cpp-example-detect_blob: lib/libopencv_calib3d.so.3.2.0
bin/cpp-example-detect_blob: lib/libopencv_features2d.so.3.2.0
bin/cpp-example-detect_blob: lib/libopencv_flann.so.3.2.0
bin/cpp-example-detect_blob: lib/libopencv_highgui.so.3.2.0
bin/cpp-example-detect_blob: lib/libopencv_ml.so.3.2.0
bin/cpp-example-detect_blob: lib/libopencv_videoio.so.3.2.0
bin/cpp-example-detect_blob: lib/libopencv_imgcodecs.so.3.2.0
bin/cpp-example-detect_blob: lib/libopencv_video.so.3.2.0
bin/cpp-example-detect_blob: lib/libopencv_imgproc.so.3.2.0
bin/cpp-example-detect_blob: lib/libopencv_core.so.3.2.0
bin/cpp-example-detect_blob: /usr/lib/i386-linux-gnu/libGLU.so
bin/cpp-example-detect_blob: /usr/lib/i386-linux-gnu/libGL.so
bin/cpp-example-detect_blob: samples/cpp/CMakeFiles/example_detect_blob.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/multipurpose/Design2/OpenCV/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable ../../bin/cpp-example-detect_blob"
	cd /home/multipurpose/Design2/OpenCV/build/samples/cpp && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/example_detect_blob.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
samples/cpp/CMakeFiles/example_detect_blob.dir/build: bin/cpp-example-detect_blob

.PHONY : samples/cpp/CMakeFiles/example_detect_blob.dir/build

samples/cpp/CMakeFiles/example_detect_blob.dir/requires: samples/cpp/CMakeFiles/example_detect_blob.dir/detect_blob.cpp.o.requires

.PHONY : samples/cpp/CMakeFiles/example_detect_blob.dir/requires

samples/cpp/CMakeFiles/example_detect_blob.dir/clean:
	cd /home/multipurpose/Design2/OpenCV/build/samples/cpp && $(CMAKE_COMMAND) -P CMakeFiles/example_detect_blob.dir/cmake_clean.cmake
.PHONY : samples/cpp/CMakeFiles/example_detect_blob.dir/clean

samples/cpp/CMakeFiles/example_detect_blob.dir/depend:
	cd /home/multipurpose/Design2/OpenCV/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/multipurpose/Design2/OpenCV /home/multipurpose/Design2/OpenCV/samples/cpp /home/multipurpose/Design2/OpenCV/build /home/multipurpose/Design2/OpenCV/build/samples/cpp /home/multipurpose/Design2/OpenCV/build/samples/cpp/CMakeFiles/example_detect_blob.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : samples/cpp/CMakeFiles/example_detect_blob.dir/depend

