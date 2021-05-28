#ifndef SYSTEM_H
#define SYSTEM_H

#include<string>
#include<thread>
#include<opencv2/core/core.hpp>

#include "Tracking.h"
#include "FrameDrawer.h"
#include "MapDrawer.h"
#include "Map.h"
#include "LocalMapping.h"
#include "LoopClosing.h"
#include "KeyFrameDatabase.h"
#include "ORBVocabulary.h"
#include "Viewer.h"

namespace ORB_SLAM2
{

class Viewer;
class FrameDrawer;
class Map;
class Tracking;
class LocalMapping;
class LoopClosing;

class System
{
public:
    // Input sensor
    //tipo de sensor que se esta usando
    enum eSensor{
        MONOCULAR=0,
        STEREO=1,
        RGBD=2
    };

public:

    // Initialize the SLAM system. It launches the Local Mapping, Loop Closing and Viewer threads.
    //vocabulario, configuracion, tipo de sensor, usar la vizualizacion
    //crear los threads y realizar la configuracion de sus parametros
    System(const string &strVocFile, const string &strSettingsFile, const eSensor sensor, const bool bUseViewer = true);

    // Proccess the given stereo frame. Images must be synchronized and rectified.
    // Input images: RGB (CV_8UC3) or grayscale (CV_8U). RGB is converted to grayscale.
    // Returns the camera pose (empty if tracking fails).
    cv::Mat TrackStereo(const cv::Mat &imLeft, const cv::Mat &imRight, const double &timestamp);

    // Process the given rgbd frame. Depthmap must be registered to the RGB frame.
    // Input image: RGB (CV_8UC3) or grayscale (CV_8U). RGB is converted to grayscale.
    // Input depthmap: Float (CV_32F).
    // Returns the camera pose (empty if tracking fails).
    //retorna la posicion de la camara y verifica si esta en proceso de localizacion o mappeamiento
    cv::Mat TrackRGBD(const cv::Mat &im, const cv::Mat &depthmap, const double &timestamp);

    // Proccess the given monocular frame
    // Input images: RGB (CV_8UC3) or grayscale (CV_8U). RGB is converted to grayscale.
    // Returns the camera pose (empty if tracking fails).
    //retorna la posicion de la camara y verifica si esta en proceso de localizacion o mappeamiento
    cv::Mat TrackMonocular(const cv::Mat &im, const double &timestamp);

    // This stops local mapping thread (map building) and performs only camera tracking.
    //activa la variable del modo localizacion en TRUE
    void ActivateLocalizationMode();

    // This resumes local mapping thread and performs SLAM again.
    //desactiva la variable del modo localizacion TRUE
    void DeactivateLocalizationMode();

    // Reset the system (clear map)
    //activa la variable de reset en TRUE
    void Reset();

    // All threads will be requested to finish.
    // It waits until all threads have finished.
    // This function must be called before saving the trajectory.
    //para los threads y espera a que finalizen los procesos para apagarlos
    void Shutdown();

    // Save camera trajectory in the TUM RGB-D dataset format.
    // Call first Shutdown()
    // See format details at: http://vision.in.tum.de/data/datasets/rgbd-dataset
    void SaveTrajectoryTUM(const string &filename);

    // Save keyframe poses in the TUM RGB-D dataset format.
    // Use this function in the monocular case.
    // Call first Shutdown()
    // See format details at: http://vision.in.tum.de/data/datasets/rgbd-dataset
    void SaveKeyFrameTrajectoryTUM(const string &filename);

    // Save camera trajectory in the KITTI dataset format.
    // Call first Shutdown()
    // See format details at: http://www.cvlibs.net/datasets/kitti/eval_odometry.php
    void SaveTrajectoryKITTI(const string &filename);


    void SaveVisualOdometry(const string &filename);
    // TODO: Save/Load functions
    // SaveMap(const string &filename);
    // LoadMap(const string &filename);

    void SaveTrajectory(const string &filename);
    void SaveKeyFrameTrajectory(const string &filename);
    void saveCloudPoint(const string &filename);
    void saveOtherInformations(const string &filename);
    void ChangeCalibration(const string &strSettingPath);

private:

    // Input sensor
    //tipo de sensor que se esta utilizando
    eSensor mSensor;

    // ORB vocabulary used for place recognition and feature matching.
    //vocabulario cargado para usar el reconocimiento de lugares
    ORBVocabulary* mpVocabulary;

    // KeyFrame database for place recognition (relocalization and loop detection).
    //base de datos de los keyframes para la deteccion de loops o la relocalizacion
    KeyFrameDatabase* mpKeyFrameDatabase;

    // Map structure that stores the pointers to all KeyFrames and MapPoints.
    //mapa de puntos estructura que almacena los punteros de los puntos de todos los keyframes
    Map* mpMap;

    // Tracker. It receives a frame and computes the associated camera pose.
    // It also decides when to insert a new keyframe, create some new MapPoints and
    // performs relocalization if tracking fails.
    //thread tracking se utiliza para
    //computar la posicion de la camara
    //decide cuando insertar un nuevo keyframe
    //crea nuevas nuves de puntos
    //realiza la localizacion cuando el tracking falla
    Tracking* mpTracker;

    // Local Mapper. It manages the local map and performs local bundle adjustment.
    //maneja el mapeamiento local
    //realiza la optimizacion del mapeamiento local
    LocalMapping* mpLocalMapper;

    // Loop Closer. It searches loops with every new keyframe. If there is a loop it performs
    // a pose graph optimization and full bundle adjustment (in a new thread) afterwards.
    //busca loops para cada keyframe
    //si halla un loop realiza una optimizacion global
    LoopClosing* mpLoopCloser;

    // The viewer draws the map and the current camera pose. It uses Pangolin.
    //dibuja la imagen del frame con alguna informacion
    //dibuja el mapa de puntos se mapearon
    Viewer* mpViewer;

    FrameDrawer* mpFrameDrawer; //dibuja la imagen del frame con informacion adicionada
    MapDrawer* mpMapDrawer; //dibuja los puntos del mapa construido

    // System threads: Local Mapping, Loop Closing, Viewer.
    // The Tracking thread "lives" in the main execution thread that creates the System object.
    std::thread* mptLocalMapping; //thread que controla el mapeamiento local(RUN)
    std::thread* mptLoopClosing; //thread que controla la busqueda de loops(RUN)
    std::thread* mptViewer; //thread que dibuja el mapa y muestra el frame observado

    // Reset flag
    std::mutex mMutexReset; //mutex que controla los threads para resetear
    bool mbReset; // variable para resetear FALSE(limpiar el mapa)

    // Change mode flags
    std::mutex mMutexMode; //mutex que controla los threads para cambiar de modo localizacion a mapeamiento
    bool mbActivateLocalizationMode; //variable para activar la localizacion FALSE(solo tracking)
    bool mbDeactivateLocalizationMode; //variable para desactivar la localizacion FALSE(tranking y mapping)
};

}// namespace ORB_SLAM

#endif // SYSTEM_H
