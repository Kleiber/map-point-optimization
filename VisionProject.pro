#-------------------------------------------------
#
# Project created by QtCreator 2016-04-08T16:22:43
#
#-------------------------------------------------

QT       += core gui

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

CONFIG += c++11
CONFIG += link_pkgconfig

TARGET = VisionProject
TEMPLATE = app


SOURCES += main.cpp \
    Converter.cpp \
    Frame.cpp \
    FrameDrawer.cpp \
    Initializer.cpp \
    KeyFrame.cpp \
    KeyFrameDatabase.cpp \
    LocalMapping.cpp \
    LoopClosing.cpp \
    Map.cpp \
    MapDrawer.cpp \
    MapPoint.cpp \
    Optimizer.cpp \
    ORBextractor.cpp \
    ORBmatcher.cpp \
    PnPsolver.cpp \
    Sim3Solver.cpp \
    System.cpp \
    Tracking.cpp \
    Viewer.cpp \
    planefiltering.cpp \
    epipolargeometry.cpp \
    test.cpp \
    alignmentdirect.cpp \
    seed.cpp \
    depthestimation.cpp \
    pinholecamera.cpp \
    se3.cpp \
    testSeed.cpp \
    visualizer.cpp \
    visualizerframe.cpp \
    visualizermap.cpp \
    octomap.cpp \
    fusion.cpp \
    join.cpp

HEADERS  += \
    Converter.h \
    Frame.h \
    FrameDrawer.h \
    Initializer.h \
    KeyFrame.h \
    KeyFrameDatabase.h \
    LocalMapping.h \
    LoopClosing.h \
    Map.h \
    MapDrawer.h \
    MapPoint.h \
    Optimizer.h \
    ORBextractor.h \
    ORBmatcher.h \
    ORBVocabulary.h \
    PnPsolver.h \
    Sim3Solver.h \
    System.h \
    Tracking.h \
    Viewer.h \
    planefiltering.h \
    epipolargeometry.h \
    alignmentdirect.h \
    seed.h \
    depthestimation.h \
    pinholecamera.h \
    se3.h \
    visualizer.h \
    visualizerframe.h \
    visualizermap.h

FORMS    +=

INCLUDEPATH += -I/usr/local/include/opencv2 /usr/include/eigen3 /usr/local/include/pcl-1.8 /usr/include /usr/include/vtk-6.0
INCLUDEPATH += /home/kleiber/octomap/octomap/include
INCLUDEPATH += /home/kleiber/octomap/octovis/include

INCLUDEPATH += /home/kleiber/Pangolin/include
INCLUDEPATH += /home/kleiber/Pangolin/build/src/include

LIBS += -L/usr/local/lib `pkg-config opencv --libs` -lboost_system -lboost_filesystem -lboost_thread
LIBS += -L/home/kleiber/octomap/lib -loctomap -loctomath -ldynamicedt3d -loctovis
LIBS += -L/home/kleiber/Desktop/VisionProject/Thirdparty/DBoW2/lib -lDBoW2
LIBS += -L/home/kleiber/Desktop/VisionProject/Thirdparty/g2o/lib -lg2o
LIBS += -L/home/kleiber/Desktop/VisionProject/Thirdparty/3dkht/dlib/build -llib.so
LIBS += -L/home/kleiber/Pangolin/build/src -lpangolin

#sudo apt-get install libv4l-dev
#cd /usr/include/linux
#sudo ln -s ../libv4l1-videodev.h videodev.h

#gksudo gedit /usr/include/vtk-6.0/vtkMath.h
#sudo chmod +x /usr/include/vtk-6.0/vtkMath.h

#sudo apt-get install libtbb-dev libeigen2-dev libqt4-dev libqt4-opengl-dev build-essential checkinstall cmake pkg-config yasm libtiff4-dev libjpeg-dev libjasper-dev libavcodec-dev libavformat-dev libswscale-dev libdc1394-22-dev libxine-dev libgstreamer0.10-dev libgstreamer-plugins-base0.10-dev libv4l-dev python-dev python-numpy libtbb-dev libqt4-dev libgtk2.0-dev

#cmake -D CMAKE_BUILD_TYPE=RELEASE -D CMAKE_INSTALL_PREFIX=/usr/local -D WITH_TBB=ON -D WITH_V4L=ON -D WITH_QT=ON -D WITH_OPENGL=ON -D BUILD_TIFF=ON -D WITH_OPENCL=OFF -D WITH_VTK=OFF -D INSTALL_C_EXAMPLES=ON -D INSTALL_PYTHON_EXAMPLES=ON -D OPENCV_EXTRA_MODULES_PATH=~/opencv_contrib-3.0.0/modules -D BUILD_EXAMPLES=ON ..



