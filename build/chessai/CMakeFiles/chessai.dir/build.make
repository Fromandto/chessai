# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 2.8

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

# The program to use to edit the cache.
CMAKE_EDIT_COMMAND = /usr/bin/cmake-gui

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /root/chessai_ws/src

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /root/chessai_ws/build

# Include any dependencies generated for this target.
include chessai/CMakeFiles/chessai.dir/depend.make

# Include the progress variables for this target.
include chessai/CMakeFiles/chessai.dir/progress.make

# Include the compile flags for this target's objects.
include chessai/CMakeFiles/chessai.dir/flags.make

chessai/CMakeFiles/chessai.dir/src/AI.cpp.o: chessai/CMakeFiles/chessai.dir/flags.make
chessai/CMakeFiles/chessai.dir/src/AI.cpp.o: /root/chessai_ws/src/chessai/src/AI.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /root/chessai_ws/build/CMakeFiles $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object chessai/CMakeFiles/chessai.dir/src/AI.cpp.o"
	cd /root/chessai_ws/build/chessai && /usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/chessai.dir/src/AI.cpp.o -c /root/chessai_ws/src/chessai/src/AI.cpp

chessai/CMakeFiles/chessai.dir/src/AI.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/chessai.dir/src/AI.cpp.i"
	cd /root/chessai_ws/build/chessai && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /root/chessai_ws/src/chessai/src/AI.cpp > CMakeFiles/chessai.dir/src/AI.cpp.i

chessai/CMakeFiles/chessai.dir/src/AI.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/chessai.dir/src/AI.cpp.s"
	cd /root/chessai_ws/build/chessai && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /root/chessai_ws/src/chessai/src/AI.cpp -o CMakeFiles/chessai.dir/src/AI.cpp.s

chessai/CMakeFiles/chessai.dir/src/AI.cpp.o.requires:
.PHONY : chessai/CMakeFiles/chessai.dir/src/AI.cpp.o.requires

chessai/CMakeFiles/chessai.dir/src/AI.cpp.o.provides: chessai/CMakeFiles/chessai.dir/src/AI.cpp.o.requires
	$(MAKE) -f chessai/CMakeFiles/chessai.dir/build.make chessai/CMakeFiles/chessai.dir/src/AI.cpp.o.provides.build
.PHONY : chessai/CMakeFiles/chessai.dir/src/AI.cpp.o.provides

chessai/CMakeFiles/chessai.dir/src/AI.cpp.o.provides.build: chessai/CMakeFiles/chessai.dir/src/AI.cpp.o

# Object files for target chessai
chessai_OBJECTS = \
"CMakeFiles/chessai.dir/src/AI.cpp.o"

# External object files for target chessai
chessai_EXTERNAL_OBJECTS =

/root/chessai_ws/devel/lib/chessai/chessai: chessai/CMakeFiles/chessai.dir/src/AI.cpp.o
/root/chessai_ws/devel/lib/chessai/chessai: /opt/ros/hydro/lib/libcv_bridge.so
/root/chessai_ws/devel/lib/chessai/chessai: /opt/ros/hydro/lib/libopencv_videostab.so.2.4.9
/root/chessai_ws/devel/lib/chessai/chessai: /opt/ros/hydro/lib/libopencv_video.so.2.4.9
/root/chessai_ws/devel/lib/chessai/chessai: /opt/ros/hydro/lib/libopencv_superres.so.2.4.9
/root/chessai_ws/devel/lib/chessai/chessai: /opt/ros/hydro/lib/libopencv_stitching.so.2.4.9
/root/chessai_ws/devel/lib/chessai/chessai: /opt/ros/hydro/lib/libopencv_photo.so.2.4.9
/root/chessai_ws/devel/lib/chessai/chessai: /opt/ros/hydro/lib/libopencv_ocl.so.2.4.9
/root/chessai_ws/devel/lib/chessai/chessai: /opt/ros/hydro/lib/libopencv_objdetect.so.2.4.9
/root/chessai_ws/devel/lib/chessai/chessai: /opt/ros/hydro/lib/libopencv_nonfree.so.2.4.9
/root/chessai_ws/devel/lib/chessai/chessai: /opt/ros/hydro/lib/libopencv_ml.so.2.4.9
/root/chessai_ws/devel/lib/chessai/chessai: /opt/ros/hydro/lib/libopencv_legacy.so.2.4.9
/root/chessai_ws/devel/lib/chessai/chessai: /opt/ros/hydro/lib/libopencv_imgproc.so.2.4.9
/root/chessai_ws/devel/lib/chessai/chessai: /opt/ros/hydro/lib/libopencv_highgui.so.2.4.9
/root/chessai_ws/devel/lib/chessai/chessai: /opt/ros/hydro/lib/libopencv_gpu.so.2.4.9
/root/chessai_ws/devel/lib/chessai/chessai: /opt/ros/hydro/lib/libopencv_flann.so.2.4.9
/root/chessai_ws/devel/lib/chessai/chessai: /opt/ros/hydro/lib/libopencv_features2d.so.2.4.9
/root/chessai_ws/devel/lib/chessai/chessai: /opt/ros/hydro/lib/libopencv_core.so.2.4.9
/root/chessai_ws/devel/lib/chessai/chessai: /opt/ros/hydro/lib/libopencv_contrib.so.2.4.9
/root/chessai_ws/devel/lib/chessai/chessai: /opt/ros/hydro/lib/libopencv_calib3d.so.2.4.9
/root/chessai_ws/devel/lib/chessai/chessai: /opt/ros/hydro/lib/libimage_transport.so
/root/chessai_ws/devel/lib/chessai/chessai: /opt/ros/hydro/lib/libmessage_filters.so
/root/chessai_ws/devel/lib/chessai/chessai: /usr/lib/libtinyxml.so
/root/chessai_ws/devel/lib/chessai/chessai: /opt/ros/hydro/lib/libclass_loader.so
/root/chessai_ws/devel/lib/chessai/chessai: /usr/lib/libPocoFoundation.so
/root/chessai_ws/devel/lib/chessai/chessai: /usr/lib/x86_64-linux-gnu/libdl.so
/root/chessai_ws/devel/lib/chessai/chessai: /opt/ros/hydro/lib/libroscpp.so
/root/chessai_ws/devel/lib/chessai/chessai: /usr/lib/libboost_signals-mt.so
/root/chessai_ws/devel/lib/chessai/chessai: /usr/lib/libboost_filesystem-mt.so
/root/chessai_ws/devel/lib/chessai/chessai: /opt/ros/hydro/lib/librosconsole.so
/root/chessai_ws/devel/lib/chessai/chessai: /opt/ros/hydro/lib/librosconsole_log4cxx.so
/root/chessai_ws/devel/lib/chessai/chessai: /opt/ros/hydro/lib/librosconsole_backend_interface.so
/root/chessai_ws/devel/lib/chessai/chessai: /usr/lib/liblog4cxx.so
/root/chessai_ws/devel/lib/chessai/chessai: /usr/lib/libboost_regex-mt.so
/root/chessai_ws/devel/lib/chessai/chessai: /opt/ros/hydro/lib/libxmlrpcpp.so
/root/chessai_ws/devel/lib/chessai/chessai: /opt/ros/hydro/lib/libroslib.so
/root/chessai_ws/devel/lib/chessai/chessai: /opt/ros/hydro/lib/libroscpp_serialization.so
/root/chessai_ws/devel/lib/chessai/chessai: /opt/ros/hydro/lib/librostime.so
/root/chessai_ws/devel/lib/chessai/chessai: /usr/lib/libboost_date_time-mt.so
/root/chessai_ws/devel/lib/chessai/chessai: /usr/lib/libboost_system-mt.so
/root/chessai_ws/devel/lib/chessai/chessai: /usr/lib/libboost_thread-mt.so
/root/chessai_ws/devel/lib/chessai/chessai: /usr/lib/x86_64-linux-gnu/libpthread.so
/root/chessai_ws/devel/lib/chessai/chessai: /opt/ros/hydro/lib/libcpp_common.so
/root/chessai_ws/devel/lib/chessai/chessai: /opt/ros/hydro/lib/libconsole_bridge.so
/root/chessai_ws/devel/lib/chessai/chessai: chessai/CMakeFiles/chessai.dir/build.make
/root/chessai_ws/devel/lib/chessai/chessai: chessai/CMakeFiles/chessai.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --red --bold "Linking CXX executable /root/chessai_ws/devel/lib/chessai/chessai"
	cd /root/chessai_ws/build/chessai && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/chessai.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
chessai/CMakeFiles/chessai.dir/build: /root/chessai_ws/devel/lib/chessai/chessai
.PHONY : chessai/CMakeFiles/chessai.dir/build

chessai/CMakeFiles/chessai.dir/requires: chessai/CMakeFiles/chessai.dir/src/AI.cpp.o.requires
.PHONY : chessai/CMakeFiles/chessai.dir/requires

chessai/CMakeFiles/chessai.dir/clean:
	cd /root/chessai_ws/build/chessai && $(CMAKE_COMMAND) -P CMakeFiles/chessai.dir/cmake_clean.cmake
.PHONY : chessai/CMakeFiles/chessai.dir/clean

chessai/CMakeFiles/chessai.dir/depend:
	cd /root/chessai_ws/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /root/chessai_ws/src /root/chessai_ws/src/chessai /root/chessai_ws/build /root/chessai_ws/build/chessai /root/chessai_ws/build/chessai/CMakeFiles/chessai.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : chessai/CMakeFiles/chessai.dir/depend

