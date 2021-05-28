cmake_minimum_required(VERSION 2.8)
project(VisionProject)

IF(NOT CMAKE_BUILD_TYPE)
  SET(CMAKE_BUILD_TYPE Release)
ENDIF()

MESSAGE("Build type: " ${CMAKE_BUILD_TYPE})

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  -Wall  -O3 -march=native ")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall   -O3 -march=native")

# Check C++11 or C++0x support
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
   set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
   add_definitions(-DCOMPILEDWITHC11)
   message(STATUS "Using flag -std=c++11.")
elseif(COMPILER_SUPPORTS_CXX0X)
   set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
   add_definitions(-DCOMPILEDWITHC0X)
   message(STATUS "Using flag -std=c++0x.")
else()
   message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

LIST(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules)

find_package(PCL REQUIRED)
find_package(OpenCV  REQUIRED )
find_package(octomap REQUIRED )
find_package(Eigen3 REQUIRED )
find_package(Pangolin REQUIRED)
find_package(FREEGLUT QUIET)
find_package(GLUT QUIET)
find_package(OpenMP)
if (OPENMP_FOUND)
    set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()

include_directories(
${PROJECT_SOURCE_DIR}
${PCL_INCLUDE_DIRS}
${OCTOMAP_INCLUDE_DIRS}
${EIGEN3_INCLUDE_DIR}
${Pangolin_INCLUDE_DIRS}
)

set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)

set(HDRS_3dkht
#Algebra 
Thirdparty/3dkht/Mathematic.h
Thirdparty/3dkht/Vector4d.h
Thirdparty/3dkht/Matrix4d.h
#Planos
Thirdparty/3dkht/settings.h
Thirdparty/3dkht/plane_t.h
Thirdparty/3dkht/octree_t.h
Thirdparty/3dkht/accum_cell_t.h
Thirdparty/3dkht/accum_ball_cell_t.h
Thirdparty/3dkht/accumulatorball_t.h
Thirdparty/3dkht/kernel_t.h
Thirdparty/3dkht/bin_t.h
Thirdparty/3dkht/voting.h
Thirdparty/3dkht/peak_detection.h
Thirdparty/3dkht/hough.h
Thirdparty/3dkht/reader_file.h
)
set(SRCS_3dkht
#Algebra 
Thirdparty/3dkht/Vector4d.cpp
Thirdparty/3dkht/Matrix4d.cpp
#Planos
Thirdparty/3dkht/octree_t.cpp
Thirdparty/3dkht/accumulatorball_t.cpp
Thirdparty/3dkht/voting.cpp
Thirdparty/3dkht/hough.cpp
)

add_library(${PROJECT_NAME} SHARED
alignmentdirect.cpp
epipolargeometry.cpp
planefiltering.cpp
depthestimation.cpp
seed.cpp
se3.cpp
pinholecamera.cpp
visualizer.cpp
visualizerframe.cpp
visualizermap.cpp
System.cpp
Tracking.cpp
LocalMapping.cpp
LoopClosing.cpp
ORBextractor.cpp
ORBmatcher.cpp
FrameDrawer.cpp
Converter.cpp
MapPoint.cpp
KeyFrame.cpp
Map.cpp
MapDrawer.cpp
Optimizer.cpp
PnPsolver.cpp
Frame.cpp
KeyFrameDatabase.cpp
Sim3Solver.cpp
Initializer.cpp
Viewer.cpp
${SRCS_3dkht}
)

target_link_libraries(${PROJECT_NAME}
${OpenCV_LIBS}
${PCL_LIBRARIES}
${OCTOMAP_LIBRARIES}
${Pangolin_LIBRARIES}
${GLUT_LIBRARIES}
${FREEGLUT_LIBRARY}
${PROJECT_SOURCE_DIR}/Thirdparty/DBoW2/lib/libDBoW2.so
${PROJECT_SOURCE_DIR}/Thirdparty/g2o/lib/libg2o.so
)

# Build main
add_executable(main main.cpp)
target_link_libraries(main ${PROJECT_NAME})

# Build test
add_executable(test test.cpp)
target_link_libraries(test ${PROJECT_NAME})

# Build test
add_executable(testSeed testSeed.cpp)
target_link_libraries(testSeed ${PROJECT_NAME})

# Build Fusion
add_executable(fusion fusion.cpp)
target_link_libraries(fusion ${PROJECT_NAME})

# Build Octomap
add_executable(octomap octomap.cpp)
target_link_libraries(octomap ${PROJECT_NAME})

# Build Octomap
add_executable(join join.cpp)
target_link_libraries(join ${PROJECT_NAME})

