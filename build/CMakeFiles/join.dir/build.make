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

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/kleiber/Desktop/VisionProject

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/kleiber/Desktop/VisionProject/build

# Include any dependencies generated for this target.
include CMakeFiles/join.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/join.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/join.dir/flags.make

CMakeFiles/join.dir/join.cpp.o: CMakeFiles/join.dir/flags.make
CMakeFiles/join.dir/join.cpp.o: ../join.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/kleiber/Desktop/VisionProject/build/CMakeFiles $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/join.dir/join.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/join.dir/join.cpp.o -c /home/kleiber/Desktop/VisionProject/join.cpp

CMakeFiles/join.dir/join.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/join.dir/join.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/kleiber/Desktop/VisionProject/join.cpp > CMakeFiles/join.dir/join.cpp.i

CMakeFiles/join.dir/join.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/join.dir/join.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/kleiber/Desktop/VisionProject/join.cpp -o CMakeFiles/join.dir/join.cpp.s

CMakeFiles/join.dir/join.cpp.o.requires:
.PHONY : CMakeFiles/join.dir/join.cpp.o.requires

CMakeFiles/join.dir/join.cpp.o.provides: CMakeFiles/join.dir/join.cpp.o.requires
	$(MAKE) -f CMakeFiles/join.dir/build.make CMakeFiles/join.dir/join.cpp.o.provides.build
.PHONY : CMakeFiles/join.dir/join.cpp.o.provides

CMakeFiles/join.dir/join.cpp.o.provides.build: CMakeFiles/join.dir/join.cpp.o

# Object files for target join
join_OBJECTS = \
"CMakeFiles/join.dir/join.cpp.o"

# External object files for target join
join_EXTERNAL_OBJECTS =

join: CMakeFiles/join.dir/join.cpp.o
join: CMakeFiles/join.dir/build.make
join: ../lib/libVisionProject.so
join: /usr/local/lib/libopencv_xphoto.so.3.0.0
join: /usr/local/lib/libopencv_ximgproc.so.3.0.0
join: /usr/local/lib/libopencv_tracking.so.3.0.0
join: /usr/local/lib/libopencv_surface_matching.so.3.0.0
join: /usr/local/lib/libopencv_saliency.so.3.0.0
join: /usr/local/lib/libopencv_rgbd.so.3.0.0
join: /usr/local/lib/libopencv_reg.so.3.0.0
join: /usr/local/lib/libopencv_optflow.so.3.0.0
join: /usr/local/lib/libopencv_line_descriptor.so.3.0.0
join: /usr/local/lib/libopencv_latentsvm.so.3.0.0
join: /usr/local/lib/libopencv_datasets.so.3.0.0
join: /usr/local/lib/libopencv_text.so.3.0.0
join: /usr/local/lib/libopencv_face.so.3.0.0
join: /usr/local/lib/libopencv_cvv.so.3.0.0
join: /usr/local/lib/libopencv_ccalib.so.3.0.0
join: /usr/local/lib/libopencv_bioinspired.so.3.0.0
join: /usr/local/lib/libopencv_bgsegm.so.3.0.0
join: /usr/local/lib/libopencv_adas.so.3.0.0
join: /usr/local/lib/libopencv_xobjdetect.so.3.0.0
join: /usr/local/lib/libopencv_videostab.so.3.0.0
join: /usr/local/lib/libopencv_superres.so.3.0.0
join: /usr/local/lib/libopencv_stitching.so.3.0.0
join: /usr/local/lib/libopencv_xfeatures2d.so.3.0.0
join: /usr/local/lib/libopencv_shape.so.3.0.0
join: /usr/local/lib/libopencv_video.so.3.0.0
join: /usr/local/lib/libopencv_photo.so.3.0.0
join: /usr/local/lib/libopencv_objdetect.so.3.0.0
join: /usr/local/lib/libopencv_calib3d.so.3.0.0
join: /usr/local/lib/libopencv_features2d.so.3.0.0
join: /usr/local/lib/libopencv_ml.so.3.0.0
join: /usr/local/lib/libopencv_highgui.so.3.0.0
join: /usr/local/lib/libopencv_videoio.so.3.0.0
join: /usr/local/lib/libopencv_imgcodecs.so.3.0.0
join: /usr/local/lib/libopencv_imgproc.so.3.0.0
join: /usr/local/lib/libopencv_flann.so.3.0.0
join: /usr/local/lib/libopencv_core.so.3.0.0
join: /usr/local/lib/libopencv_hal.a
join: /usr/local/share/OpenCV/3rdparty/lib/libippicv.a
join: /usr/lib/x86_64-linux-gnu/libvtkImagingMath-6.0.so.6.0.0
join: /usr/lib/x86_64-linux-gnu/libvtkFiltersProgrammable-6.0.so.6.0.0
join: /usr/lib/x86_64-linux-gnu/libvtkViewsGeovis-6.0.so.6.0.0
join: /usr/lib/x86_64-linux-gnu/libvtkGeovisCore-6.0.so.6.0.0
join: /usr/lib/x86_64-linux-gnu/libvtkproj4-6.0.so.6.0.0
join: /usr/lib/x86_64-linux-gnu/libvtkInteractionImage-6.0.so.6.0.0
join: /usr/lib/x86_64-linux-gnu/libvtkImagingStencil-6.0.so.6.0.0
join: /usr/lib/x86_64-linux-gnu/libvtkRenderingVolumeAMR-6.0.so.6.0.0
join: /usr/lib/x86_64-linux-gnu/libvtkFiltersGeneric-6.0.so.6.0.0
join: /usr/lib/x86_64-linux-gnu/libvtkTestingGenericBridge-6.0.so.6.0.0
join: /usr/lib/x86_64-linux-gnu/libvtkIOAMR-6.0.so.6.0.0
join: /usr/lib/x86_64-linux-gnu/libvtkIOVPIC-6.0.so.6.0.0
join: /usr/lib/x86_64-linux-gnu/libvtkVPIC-6.0.so.6.0.0
join: /usr/lib/x86_64-linux-gnu/libvtkFiltersVerdict-6.0.so.6.0.0
join: /usr/lib/x86_64-linux-gnu/libvtkverdict-6.0.so.6.0.0
join: /usr/lib/x86_64-linux-gnu/libvtkFiltersParallelStatistics-6.0.so.6.0.0
join: /usr/lib/x86_64-linux-gnu/libvtkRenderingFreeTypeFontConfig-6.0.so.6.0.0
join: /usr/lib/x86_64-linux-gnu/libfontconfig.so
join: /usr/lib/x86_64-linux-gnu/libvtkFiltersCosmo-6.0.so.6.0.0
join: /usr/lib/x86_64-linux-gnu/libvtkCosmo-6.0.so.6.0.0
join: /usr/lib/x86_64-linux-gnu/libvtkFiltersParallelImaging-6.0.so.6.0.0
join: /usr/lib/x86_64-linux-gnu/libvtkRenderingImage-6.0.so.6.0.0
join: /usr/lib/x86_64-linux-gnu/libvtkIOPostgreSQL-6.0.so.6.0.0
join: /usr/lib/libpq.so
join: /usr/lib/x86_64-linux-gnu/libvtkRenderingQt-6.0.so.6.0.0
join: /usr/lib/x86_64-linux-gnu/libvtkFiltersTexture-6.0.so.6.0.0
join: /usr/lib/x86_64-linux-gnu/libvtkRenderingHybridOpenGL-6.0.so.6.0.0
join: /usr/lib/x86_64-linux-gnu/libvtkImagingStatistics-6.0.so.6.0.0
join: /usr/lib/x86_64-linux-gnu/libvtkIOParallelExodus-6.0.so.6.0.0
join: /usr/lib/x86_64-linux-gnu/libvtkIOExodus-6.0.so.6.0.0
join: /usr/lib/x86_64-linux-gnu/libvtkRenderingParallel-6.0.so.6.0.0
join: /usr/lib/x86_64-linux-gnu/libvtkIOInfovis-6.0.so.6.0.0
join: /usr/lib/x86_64-linux-gnu/libvtkIOVideo-6.0.so.6.0.0
join: /usr/lib/x86_64-linux-gnu/libvtkImagingMorphological-6.0.so.6.0.0
join: /usr/lib/x86_64-linux-gnu/libvtkFiltersParallelGeometry-6.0.so.6.0.0
join: /usr/lib/x86_64-linux-gnu/libvtkIOMPIParallel-6.0.so.6.0.0
join: /usr/lib/x86_64-linux-gnu/libvtkGUISupportQtSQL-6.0.so.6.0.0
join: /usr/lib/x86_64-linux-gnu/libQtSql.so
join: /usr/lib/libvtkWrappingTools-6.0.a
join: /usr/lib/x86_64-linux-gnu/libvtkLocalExample-6.0.so.6.0.0
join: /usr/lib/x86_64-linux-gnu/libvtkIOMySQL-6.0.so.6.0.0
join: /usr/lib/x86_64-linux-gnu/libmysqlclient.so
join: /usr/lib/x86_64-linux-gnu/libvtkRenderingMatplotlib-6.0.so.6.0.0
join: /usr/lib/x86_64-linux-gnu/libvtkPythonInterpreter-6.0.so.6.0.0
join: /usr/lib/x86_64-linux-gnu/libvtkTestingIOSQL-6.0.so.6.0.0
join: /usr/lib/x86_64-linux-gnu/libvtkRenderingLOD-6.0.so.6.0.0
join: /usr/lib/x86_64-linux-gnu/libvtkGUISupportQtOpenGL-6.0.so.6.0.0
join: /usr/lib/x86_64-linux-gnu/libQtOpenGL.so
join: /usr/lib/x86_64-linux-gnu/libvtkIOParallelNetCDF-6.0.so.6.0.0
join: /usr/lib/x86_64-linux-gnu/libvtkIOXdmf2-6.0.so.6.0.0
join: /usr/lib/x86_64-linux-gnu/libvtkxdmf2-6.0.so.6.0.0
join: /usr/lib/x86_64-linux-gnu/libvtkFiltersParallelFlowPaths-6.0.so.6.0.0
join: /usr/lib/x86_64-linux-gnu/libvtkFiltersAMR-6.0.so.6.0.0
join: /usr/lib/x86_64-linux-gnu/libvtkFiltersFlowPaths-6.0.so.6.0.0
join: /usr/lib/x86_64-linux-gnu/libvtkIOParallel-6.0.so.6.0.0
join: /usr/lib/x86_64-linux-gnu/libvtkexoIIc-6.0.so.6.0.0
join: /usr/lib/x86_64-linux-gnu/libvtkIONetCDF-6.0.so.6.0.0
join: /usr/lib/x86_64-linux-gnu/libvtkRenderingVolumeOpenGL-6.0.so.6.0.0
join: /usr/lib/x86_64-linux-gnu/libvtkjsoncpp-6.0.so.6.0.0
join: /usr/lib/x86_64-linux-gnu/libvtkIOODBC-6.0.so.6.0.0
join: /usr/lib/x86_64-linux-gnu/libvtkIOSQL-6.0.so.6.0.0
join: /usr/lib/x86_64-linux-gnu/libvtksqlite-6.0.so.6.0.0
join: /usr/lib/x86_64-linux-gnu/libodbc.so
join: /usr/lib/x86_64-linux-gnu/libvtkRenderingFreeTypeOpenGL-6.0.so.6.0.0
join: /usr/lib/x86_64-linux-gnu/libvtkIOExport-6.0.so.6.0.0
join: /usr/lib/x86_64-linux-gnu/libvtkRenderingGL2PS-6.0.so.6.0.0
join: /usr/lib/x86_64-linux-gnu/libvtkIOFFMPEG-6.0.so.6.0.0
join: /usr/lib/x86_64-linux-gnu/libvtkIOMovie-6.0.so.6.0.0
join: /usr/lib/x86_64-linux-gnu/libvtkViewsContext2D-6.0.so.6.0.0
join: /usr/lib/x86_64-linux-gnu/libvtkIOGDAL-6.0.so.6.0.0
join: /usr/lib/libgdal.so
join: /usr/lib/x86_64-linux-gnu/libvtkFiltersHyperTree-6.0.so.6.0.0
join: /usr/lib/x86_64-linux-gnu/libvtkFiltersSelection-6.0.so.6.0.0
join: /usr/lib/x86_64-linux-gnu/libvtkTestingRendering-6.0.so.6.0.0
join: /usr/lib/x86_64-linux-gnu/libvtkDomainsChemistry-6.0.so.6.0.0
join: /usr/lib/x86_64-linux-gnu/libvtkIOEnSight-6.0.so.6.0.0
join: /usr/lib/x86_64-linux-gnu/libvtkIOParallelLSDyna-6.0.so.6.0.0
join: /usr/lib/x86_64-linux-gnu/libvtkIOLSDyna-6.0.so.6.0.0
join: /usr/lib/x86_64-linux-gnu/libvtkIOXML-6.0.so.6.0.0
join: /usr/lib/x86_64-linux-gnu/libvtkIOPLY-6.0.so.6.0.0
join: /usr/lib/x86_64-linux-gnu/libvtkWrappingJava-6.0.so.6.0.0
join: /usr/lib/x86_64-linux-gnu/libvtkInfovisBoostGraphAlgorithms-6.0.so.6.0.0
join: /usr/lib/x86_64-linux-gnu/libvtkFiltersParallelMPI-6.0.so.6.0.0
join: /usr/lib/x86_64-linux-gnu/libvtkFiltersParallel-6.0.so.6.0.0
join: /usr/lib/x86_64-linux-gnu/libvtkIOImport-6.0.so.6.0.0
join: /usr/lib/x86_64-linux-gnu/libvtkWrappingPython27Core-6.0.so.6.0.0
join: /usr/lib/x86_64-linux-gnu/libvtkIOMPIImage-6.0.so.6.0.0
join: /usr/lib/x86_64-linux-gnu/libvtkParallelMPI-6.0.so.6.0.0
join: /usr/lib/x86_64-linux-gnu/libvtkParallelCore-6.0.so.6.0.0
join: /usr/lib/x86_64-linux-gnu/libvtkIOLegacy-6.0.so.6.0.0
join: /usr/lib/libmpi.so
join: /usr/lib/x86_64-linux-gnu/libhwloc.so
join: /usr/lib/libmpi_cxx.so
join: /usr/lib/libmpi.so
join: /usr/lib/x86_64-linux-gnu/libhwloc.so
join: /usr/lib/libmpi_cxx.so
join: /usr/lib/x86_64-linux-gnu/libvtkFiltersReebGraph-6.0.so.6.0.0
join: /usr/lib/x86_64-linux-gnu/libvtkChartsCore-6.0.so.6.0.0
join: /usr/lib/x86_64-linux-gnu/libvtkCommonColor-6.0.so.6.0.0
join: /usr/lib/x86_64-linux-gnu/libvtkIOMINC-6.0.so.6.0.0
join: /usr/lib/x86_64-linux-gnu/libvtkNetCDF_cxx-6.0.so.6.0.0
join: /usr/lib/x86_64-linux-gnu/libvtkNetCDF-6.0.so.6.0.0
join: /usr/lib/x86_64-linux-gnu/libvtkIOGeometry-6.0.so.6.0.0
join: /usr/lib/x86_64-linux-gnu/libvtkGUISupportQtWebkit-6.0.so.6.0.0
join: /usr/lib/x86_64-linux-gnu/libvtkViewsQt-6.0.so.6.0.0
join: /usr/lib/x86_64-linux-gnu/libvtkViewsInfovis-6.0.so.6.0.0
join: /usr/lib/x86_64-linux-gnu/libvtkInfovisLayout-6.0.so.6.0.0
join: /usr/lib/x86_64-linux-gnu/libvtkInfovisCore-6.0.so.6.0.0
join: /usr/lib/x86_64-linux-gnu/libvtkViewsCore-6.0.so.6.0.0
join: /usr/lib/x86_64-linux-gnu/libvtkInteractionWidgets-6.0.so.6.0.0
join: /usr/lib/x86_64-linux-gnu/libvtkFiltersModeling-6.0.so.6.0.0
join: /usr/lib/x86_64-linux-gnu/libvtkFiltersHybrid-6.0.so.6.0.0
join: /usr/lib/x86_64-linux-gnu/libvtkRenderingAnnotation-6.0.so.6.0.0
join: /usr/lib/x86_64-linux-gnu/libvtkImagingColor-6.0.so.6.0.0
join: /usr/lib/x86_64-linux-gnu/libvtkRenderingVolume-6.0.so.6.0.0
join: /usr/lib/x86_64-linux-gnu/libvtkFiltersImaging-6.0.so.6.0.0
join: /usr/lib/x86_64-linux-gnu/libvtkImagingGeneral-6.0.so.6.0.0
join: /usr/lib/x86_64-linux-gnu/libvtkImagingSources-6.0.so.6.0.0
join: /usr/lib/x86_64-linux-gnu/libvtkRenderingContext2D-6.0.so.6.0.0
join: /usr/lib/x86_64-linux-gnu/libvtkRenderingLabel-6.0.so.6.0.0
join: /usr/lib/x86_64-linux-gnu/libvtkRenderingFreeType-6.0.so.6.0.0
join: /usr/lib/x86_64-linux-gnu/libvtkftgl-6.0.so.6.0.0
join: /usr/lib/x86_64-linux-gnu/libvtkGUISupportQt-6.0.so.6.0.0
join: /usr/lib/x86_64-linux-gnu/libvtkInteractionStyle-6.0.so.6.0.0
join: /usr/lib/x86_64-linux-gnu/libvtkRenderingOpenGL-6.0.so.6.0.0
join: /usr/lib/x86_64-linux-gnu/libvtkImagingHybrid-6.0.so.6.0.0
join: /usr/lib/x86_64-linux-gnu/libvtkRenderingCore-6.0.so.6.0.0
join: /usr/lib/x86_64-linux-gnu/libvtkFiltersSources-6.0.so.6.0.0
join: /usr/lib/x86_64-linux-gnu/libvtkIOXMLParser-6.0.so.6.0.0
join: /usr/lib/x86_64-linux-gnu/libvtkIOImage-6.0.so.6.0.0
join: /usr/lib/x86_64-linux-gnu/libvtkIOCore-6.0.so.6.0.0
join: /usr/lib/x86_64-linux-gnu/libvtkDICOMParser-6.0.so.6.0.0
join: /usr/lib/x86_64-linux-gnu/libvtkmetaio-6.0.so.6.0.0
join: /usr/lib/x86_64-linux-gnu/libvtkFiltersExtraction-6.0.so.6.0.0
join: /usr/lib/x86_64-linux-gnu/libvtkFiltersGeneral-6.0.so.6.0.0
join: /usr/lib/x86_64-linux-gnu/libvtkCommonComputationalGeometry-6.0.so.6.0.0
join: /usr/lib/x86_64-linux-gnu/libvtkFiltersStatistics-6.0.so.6.0.0
join: /usr/lib/x86_64-linux-gnu/libvtkImagingFourier-6.0.so.6.0.0
join: /usr/lib/x86_64-linux-gnu/libvtkImagingCore-6.0.so.6.0.0
join: /usr/lib/x86_64-linux-gnu/libvtkalglib-6.0.so.6.0.0
join: /usr/lib/x86_64-linux-gnu/libvtkFiltersGeometry-6.0.so.6.0.0
join: /usr/lib/x86_64-linux-gnu/libvtkFiltersCore-6.0.so.6.0.0
join: /usr/lib/x86_64-linux-gnu/libvtkCommonExecutionModel-6.0.so.6.0.0
join: /usr/lib/x86_64-linux-gnu/libvtkCommonDataModel-6.0.so.6.0.0
join: /usr/lib/x86_64-linux-gnu/libvtkCommonMisc-6.0.so.6.0.0
join: /usr/lib/x86_64-linux-gnu/libvtkCommonSystem-6.0.so.6.0.0
join: /usr/lib/x86_64-linux-gnu/libvtkCommonTransforms-6.0.so.6.0.0
join: /usr/lib/x86_64-linux-gnu/libvtkCommonMath-6.0.so.6.0.0
join: /usr/lib/x86_64-linux-gnu/libvtkCommonCore-6.0.so.6.0.0
join: /usr/lib/x86_64-linux-gnu/libvtksys-6.0.so.6.0.0
join: /usr/lib/x86_64-linux-gnu/libXt.so
join: /usr/lib/x86_64-linux-gnu/libSM.so
join: /usr/lib/x86_64-linux-gnu/libICE.so
join: /usr/lib/x86_64-linux-gnu/libQtWebKit.so
join: /usr/lib/x86_64-linux-gnu/libQtXmlPatterns.so
join: /usr/lib/x86_64-linux-gnu/libQtGui.so
join: /usr/lib/x86_64-linux-gnu/libQtNetwork.so
join: /usr/lib/x86_64-linux-gnu/libQtCore.so
join: /usr/lib/x86_64-linux-gnu/libboost_system.so
join: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so
join: /usr/lib/x86_64-linux-gnu/libboost_thread.so
join: /usr/lib/x86_64-linux-gnu/libboost_date_time.so
join: /usr/lib/x86_64-linux-gnu/libboost_iostreams.so
join: /usr/lib/x86_64-linux-gnu/libboost_serialization.so
join: /usr/local/lib/libpcl_common.so
join: /usr/lib/x86_64-linux-gnu/libflann_cpp_s.a
join: /usr/local/lib/libpcl_kdtree.so
join: /usr/local/lib/libpcl_octree.so
join: /usr/local/lib/libpcl_search.so
join: /usr/lib/x86_64-linux-gnu/libqhull.so
join: /usr/local/lib/libpcl_surface.so
join: /usr/local/lib/libpcl_sample_consensus.so
join: /usr/local/lib/libpcl_filters.so
join: /usr/lib/libOpenNI.so
join: /usr/lib/libOpenNI2.so
join: /usr/local/lib/libpcl_io.so
join: /usr/local/lib/libpcl_features.so
join: /usr/local/lib/libpcl_keypoints.so
join: /usr/local/lib/libpcl_ml.so
join: /usr/local/lib/libpcl_segmentation.so
join: /usr/local/lib/libpcl_registration.so
join: /usr/local/lib/libpcl_recognition.so
join: /usr/local/lib/libpcl_visualization.so
join: /usr/local/lib/libpcl_outofcore.so
join: /usr/local/lib/libpcl_stereo.so
join: /usr/local/lib/libpcl_tracking.so
join: /usr/local/lib/libpcl_people.so
join: /usr/lib/x86_64-linux-gnu/libboost_system.so
join: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so
join: /usr/lib/x86_64-linux-gnu/libboost_thread.so
join: /usr/lib/x86_64-linux-gnu/libboost_date_time.so
join: /usr/lib/x86_64-linux-gnu/libboost_iostreams.so
join: /usr/lib/x86_64-linux-gnu/libboost_serialization.so
join: /usr/local/lib/libpcl_common.so
join: /usr/lib/x86_64-linux-gnu/libflann_cpp_s.a
join: /usr/local/lib/libpcl_kdtree.so
join: /usr/local/lib/libpcl_octree.so
join: /usr/local/lib/libpcl_search.so
join: /usr/lib/x86_64-linux-gnu/libqhull.so
join: /usr/local/lib/libpcl_surface.so
join: /usr/local/lib/libpcl_sample_consensus.so
join: /usr/local/lib/libpcl_filters.so
join: /usr/lib/libOpenNI.so
join: /usr/lib/libOpenNI2.so
join: /usr/local/lib/libpcl_io.so
join: /usr/local/lib/libpcl_features.so
join: /usr/local/lib/libpcl_keypoints.so
join: /usr/local/lib/libpcl_ml.so
join: /usr/local/lib/libpcl_segmentation.so
join: /usr/local/lib/libpcl_registration.so
join: /usr/local/lib/libpcl_recognition.so
join: /usr/local/lib/libpcl_visualization.so
join: /usr/local/lib/libpcl_outofcore.so
join: /usr/local/lib/libpcl_stereo.so
join: /usr/local/lib/libpcl_tracking.so
join: /usr/local/lib/libpcl_people.so
join: /usr/lib/x86_64-linux-gnu/libpthread.so
join: /usr/lib/x86_64-linux-gnu/libhdf5.so
join: /usr/lib/x86_64-linux-gnu/libm.so
join: /usr/lib/x86_64-linux-gnu/libhdf5_hl.so
join: /usr/lib/x86_64-linux-gnu/libpthread.so
join: /usr/lib/x86_64-linux-gnu/libhdf5.so
join: /usr/lib/x86_64-linux-gnu/libm.so
join: /usr/lib/x86_64-linux-gnu/libhdf5_hl.so
join: /usr/lib/x86_64-linux-gnu/libdl.so
join: /usr/lib/x86_64-linux-gnu/libexpat.so
join: /usr/lib/x86_64-linux-gnu/libfreetype.so
join: /usr/lib/libgl2ps.so
join: /usr/lib/x86_64-linux-gnu/libxml2.so
join: /usr/lib/x86_64-linux-gnu/libogg.so
join: /usr/lib/x86_64-linux-gnu/libtheoraenc.so
join: /usr/lib/x86_64-linux-gnu/libtheoradec.so
join: /usr/local/lib/liboctomap.so
join: /usr/local/lib/liboctomath.so
join: /usr/local/lib/libpangolin.so
join: /usr/lib/x86_64-linux-gnu/libGLU.so
join: /usr/lib/x86_64-linux-gnu/libGL.so
join: /usr/lib/x86_64-linux-gnu/libGLEW.so
join: /usr/lib/x86_64-linux-gnu/libpython2.7.so
join: /usr/lib/x86_64-linux-gnu/libX11.so
join: /usr/lib/x86_64-linux-gnu/libXext.so
join: /usr/lib/x86_64-linux-gnu/libdc1394.so
join: /usr/lib/x86_64-linux-gnu/libjpeg.so
join: /usr/lib/x86_64-linux-gnu/libpng.so
join: /usr/lib/x86_64-linux-gnu/libz.so
join: /usr/lib/x86_64-linux-gnu/libtiff.so
join: /usr/lib/x86_64-linux-gnu/libavformat.so
join: /usr/lib/x86_64-linux-gnu/libavcodec.so
join: /usr/lib/x86_64-linux-gnu/libavutil.so
join: /usr/lib/x86_64-linux-gnu/libswscale.so
join: /usr/lib/x86_64-linux-gnu/libIlmImf.so
join: /usr/lib/x86_64-linux-gnu/libglut.so
join: /usr/lib/x86_64-linux-gnu/libXmu.so
join: /usr/lib/x86_64-linux-gnu/libXi.so
join: ../Thirdparty/DBoW2/lib/libDBoW2.so
join: ../Thirdparty/g2o/lib/libg2o.so
join: CMakeFiles/join.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --red --bold "Linking CXX executable join"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/join.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/join.dir/build: join
.PHONY : CMakeFiles/join.dir/build

CMakeFiles/join.dir/requires: CMakeFiles/join.dir/join.cpp.o.requires
.PHONY : CMakeFiles/join.dir/requires

CMakeFiles/join.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/join.dir/cmake_clean.cmake
.PHONY : CMakeFiles/join.dir/clean

CMakeFiles/join.dir/depend:
	cd /home/kleiber/Desktop/VisionProject/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/kleiber/Desktop/VisionProject /home/kleiber/Desktop/VisionProject /home/kleiber/Desktop/VisionProject/build /home/kleiber/Desktop/VisionProject/build /home/kleiber/Desktop/VisionProject/build/CMakeFiles/join.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/join.dir/depend
