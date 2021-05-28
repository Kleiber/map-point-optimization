#ifndef TRACKING_H
#define TRACKING_H

#include<opencv2/core/core.hpp>
#include<opencv2/features2d/features2d.hpp>

#include"Viewer.h"
#include"FrameDrawer.h"
#include"Map.h"
#include"LocalMapping.h"
#include"LoopClosing.h"
#include"Frame.h"
#include "ORBVocabulary.h"
#include"KeyFrameDatabase.h"
#include"ORBextractor.h"
#include "Initializer.h"
#include "MapDrawer.h"
#include "System.h"
#include "planefiltering.h"

#include <mutex>

namespace ORB_SLAM2
{

class Viewer;
class FrameDrawer;
class Map;
class LocalMapping;
class LoopClosing;
class System;

class Tracking
{  

public:

    //configuraciones del sustema, el vocabulario para convertir en palabras, el graficador de la imagen mas la informacion,
    //el graficador de la nube de puntos, el mapa contruido, la base de datos de los keyframes que se va contruyendo,
    //la ruta de las configuraciones, el tipo de sensor
    Tracking(System* pSys, ORBVocabulary* pVoc, FrameDrawer* pFrameDrawer, MapDrawer* pMapDrawer, Map* pMap,
             KeyFrameDatabase* pKFDB, const string &strSettingPath, const int sensor);

    // Preprocess the input and call Track(). Extract features and performs stereo matching.
    //preprocesamiento de las imagenes antes del tracking, convertir a tonos gris y crear los frames con la extrancion de features y las configuraciones iniciales
    cv::Mat GrabImageStereo(const cv::Mat &imRectLeft,const cv::Mat &imRectRight, const double &timestamp);
    cv::Mat GrabImageRGBD(const cv::Mat &imRGB,const cv::Mat &imD, const double &timestamp);
    //convierte imagen a escala de grises , si el sistema aun no se ha inicializado entonces usa un orbextractor inicial
    //caso contrario usa un orbextractor normal y retorna la posicion actual de la camara en el mundo
    cv::Mat GrabImageMonocular(const cv::Mat &im, const double &timestamp);

    void SetLocalMapper(LocalMapping* pLocalMapper); //asigna el puntero del thread que maneja el mapa (mapping )
    void SetLoopClosing(LoopClosing* pLoopClosing); //asigna el puntero del thread que busca loops (loops closing)
    void SetViewer(Viewer* pViewer); //asigna el puntero del thread de vizualizacion

    // Load new settings
    // The focal lenght should be similar or scale prediction will fail when projecting points
    // TODO: Modify MapPoint::PredictScale to take into account focal lenght
    //carga las configuraciones de la matriz de calibracion
    //marca la variable INITIAL-COMPUTATION como TRUE (variable global de la clase frame)
    void ChangeCalibration(const string &strSettingPath);

    // Use this function if you have deactivated local mapping and you only want to localize the camera.
    //asigna el valor a la variable ONLY-TRACKING el valor de TRUE si se quiere solo la localizacion y false caso contrario
    void InformOnlyTracking(const bool &flag);


public:

    // Tracking states
    // -1 El sistema no esta aun listo para inicializar
    //  0 No existen imagenes
    //  1 No se consiguio inicializar
    //  2 Cuando se consiguio inicializar y se realiza el tracking y/o mapping
    //  3 Cuando se perdio el tracking y se intenta localizar
    enum eTrackingState{
        SYSTEM_NOT_READY=-1,
        NO_IMAGES_YET=0,
        NOT_INITIALIZED=1,
        OK=2,
        LOST=3
    };

    eTrackingState mState; //ESTADO ACTUAL DEL TRACKING
    eTrackingState mLastProcessedState; //ESTADO ANTERIOR DEL TRACKING

    // Input sensor
    int mSensor; //TIPO DE SENSOR QUE SE UTILIZA

    // Current Frame
    Frame mCurrentFrame; //FRAME ACTUAL QUE SE VIENE PROCESANDO
    cv::Mat mImGray; //IMAGEN EN ESCALA DE GRISES QUE SE VIENE PROCESANDO

    // Initialization Variables (Monocular)
    std::vector<int> mvIniLastMatches;
    std::vector<int> mvIniMatches; //MATCHES ENTRE EL FRAME INICIAL Y EL FRAME ACTUAL
    std::vector<cv::Point2f> mvbPrevMatched; //PUNTOS DEL FRAME INCIAL QUE SE USARAN PARA BUSCAR LOS OTROS MATCHED
    std::vector<cv::Point3f> mvIniP3D; //POSICION 3D EN EL MAPA DE LOS PUNTOS
    std::vector<float> mvUnTriangulated; //INCERTIDUMBRE DE LOS PUNTOS
    Frame mInitialFrame; //FRAME INICIAL O FRAME DE REFERENCIA

    // Lists used to recover the full camera trajectory at the end of the execution.
    // Basically we store the reference keyframe for each frame and its relative transformation
    list<cv::Mat> mlRelativeFramePoses;
    list<KeyFrame*> mlpReferences;
    list<double> mlFrameTimes;
    list<bool> mlbLost;

    // True if local mapping is deactivated and we are performing only localization
    bool mbOnlyTracking; //variable ONLY-TRACKING , si esta en TRUE el sistema solo realiza localizacion sino hace mapeamiento

    void Reset(); //resetea el sistema, pide solicitud de resetear a los threads y limpia toda la informacion

protected:

    // Main tracking function. It is independent of the input sensor.
    void Track();

    // Map initialization for stereo and RGB-D
    void StereoInitialization(); //inicializacion sabiendo la profundidad tanto en estereo como en rgb-d

    // Map initialization for monocular
    void MonocularInitialization();//se halla las poses iniciales de initFrame y el currentFrame y se manda a crear un mapa inicial
    void CreateInitialMapMonocular();//crea el mapa inicial de puntos y realiza algunas configuraciones en el tracking

    void CheckReplacedInLastFrame(); //verifica que los puntos no hayan sido reemplazados en el ultimo frame procesado, caso lo hayan sido actualizo la informacion de ultimo frame procesado
    bool TrackReferenceKeyFrame();
    void UpdateLastFrame();
    bool TrackWithMotionModel();

    bool Relocalization();//realiza la relocalizacion

    void UpdateLocalMap(); //actualiza el mapa loca, la lista de puntos locales y la lista de keyframes locales
    void UpdateLocalPoints();//con la lista de keyframes locales busca la lista de puntos locales se actualiza
    void UpdateLocalKeyFrames();//actualiza la lista de keyframes locales y busca el mejor keyframe para hacer el tracking(mas puntos compartidos)

    bool TrackLocalMap();//verifica cuantos inliers esxisten para realizar el tracking y si es bueno?
    void SearchLocalPoints(); //se busca en el mapa local puntos que tambien sean visto desde la camara y se busca su matching

    bool NeedNewKeyFrame(); //verifica si se necesita crear un nuevo keyframe o no
    void CreateNewKeyFrame(); //crea el nuevo keyframe y actualiza los datos de la referenceKeyframe

    // In case of performing only localization, this flag is true when there are no matches to
    // points in the map. Still tracking will continue if there are enough matches with temporal points.
    // In that case we are doing visual odometry. The system will try to do relocalization to recover
    // "zero-drift" localization to the map.
    bool mbVO;

    //Plane Filtering
    PlaneFiltering* Filtering;


    //Other Thread Pointers
    LocalMapping* mpLocalMapper; //puntero al thread para el manejador del mapa(mapping)
    LoopClosing* mpLoopClosing; //puntero al thread buscador de loops

    //ORB
    //extractor de features ORB para la imagen izquierda y la imagen derecha (en monocular solo se usa la izquierda)
    ORBextractor* mpORBextractorLeft, *mpORBextractorRight;
    //extractor de features ORB que se usa solo para la inicializacion
    ORBextractor* mpIniORBextractor;

    //BoW
    ORBVocabulary* mpORBVocabulary; //puntero al vocabulario para convertir el frame en palabras
    KeyFrameDatabase* mpKeyFrameDB; //base de datos de todos los frame que hasta el momento se vieron

    // Initalization (only for monocular)
    Initializer* mpInitializer; //retorna la rotacion y traslacion de un frame para el frame de referencia

    //Local Map
    KeyFrame* mpReferenceKF;//KEYFRAME DE REFERENCIA PARA REALIZAR EL  TRACKING
    std::vector<KeyFrame*> mvpLocalKeyFrames; //lista de keyframes locales
    std::vector<MapPoint*> mvpLocalMapPoints; //lista de puntos procesados en el momento  (local map)
    
    // System
    System* mpSystem; //puntero thread del sistema
    
    //Drawers
    Viewer* mpViewer; //puntero thread para el visualizador
    FrameDrawer* mpFrameDrawer; //puntero para el graficador de la imagen
    MapDrawer* mpMapDrawer; // puntero para el graficador de los puntos del mapa

    //Map
    Map* mpMap; //puntero para toda la informacion del mapa

    //Calibration matrix
    cv::Mat mK; //matriz de calibracion
    cv::Mat mDistCoef;//matriz de coeficientes de distorcion
    float mbf; //distancia base, separacion entre dos camaras(base line)

    //New KeyFrame rules (according to fps)
    int mMinFrames; //el numero minimo de frames
    int mMaxFrames; //el numero maximo de frames

    // Threshold close/far points
    // Points seen as close by the stereo/RGBD sensor are considered reliable
    // and inserted from just one frame. Far points requiere a match in two keyframes.
    float mThDepth;

    // For RGB-D inputs only. For some datasets (e.g. TUM) the depthmap values are scaled.
    float mDepthMapFactor; //factor de escala de la imagen de profundidad 5000

    //Current matches in frame
    int mnMatchesInliers;

    //Last Frame, KeyFrame and Relocalisation Info
    KeyFrame* mpLastKeyFrame; //ultimo keyFrame procesado
    Frame mLastFrame; //ultimo Frame procesado
    unsigned int mnLastKeyFrameId; //id del ultimo keyFrame procesado
    unsigned int mnLastRelocFrameId;

    //Motion Model
    cv::Mat mVelocity; //transformacion del lastframe al currentframe

    //Color order (true RGB, false BGR, ignored if grayscale)
    bool mbRGB;

    list<MapPoint*> mlpTemporalPoints;
};

} //namespace ORB_SLAM

#endif // TRACKING_H
