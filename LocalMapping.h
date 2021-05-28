#ifndef LOCALMAPPING_H
#define LOCALMAPPING_H

#include "MapPoint.h"
#include "KeyFrame.h"
#include "Map.h"
#include "LoopClosing.h"
#include "Tracking.h"
#include "KeyFrameDatabase.h"

#include <mutex>


namespace ORB_SLAM2
{

class Tracking;
class LoopClosing;
class Map;

class LocalMapping
{
public:
    //puntero hacia la informacion del mapa y si el sistema es monocular o no bool
    LocalMapping(Map* pMap, const float bMonocular);

    //asiganmos el puntero del thread para buscar loops
    void SetLoopCloser(LoopClosing* pLoopCloser);

    //asignamos el puntero del thread para hacer el tacking(crear keyframe, crear nuve de puntos, hallar pose, hacer localizacion)
    void SetTracker(Tracking* pTracker);

    // Main function
    void Run();

    //inserta un nuevo keyframe en la cola de keyframes y varia el valor de ABORT-BA para TRUE
    void InsertKeyFrame(KeyFrame* pKF);

    // Thread Synch
    void RequestStop(); //pide parar varia la variable STOP-REQUEST para TRUE y la variable ABORT-BA para TRUE
    void RequestReset(); //pide resetear varia la variable RESET-REQUEST para TRUE y espera que cambie de valor
    bool Stop(); //verifica si paro de thread mirando la variable STOP-REQUEST y NOT-STOP
    void Release(); //verifica si paro FINISHED y asigana las variables STOP-REQUEST y STOPPED en FALSE y limpia la cola de newkeyframes
    bool isStopped(); //esta parado? retorna variable STOPPED
    bool stopRequested(); //se hizo la solicitud para parar? se retorna la variable STOP-REQUEST
    bool AcceptKeyFrames(); //verifica si acepta keyframes retorna variable ACCEPT-KEYFRAME
    void SetAcceptKeyFrames(bool flag); //asigna el valor a si acepta keyframes ACCEPT-KEYFRAME
    bool SetNotStop(bool flag);//asigna valor a parar NOT-STOP, si el sistema esta parado y la bandera enviada es TRUE retorna FALSE

    void InterruptBA(); //asigna a la variable ABORT-BA con TRUE

    void RequestFinish();//pide terminar, asigna a la variable FINISH-REQUEST con TRUE
    bool isFinished();//esta finalizado? retorna la variable FINISHED

    int KeyframesInQueue(){ //tamano de elementos en la cola de keyframes
        unique_lock<std::mutex> lock(mMutexNewKFs);
        return mlNewKeyFrames.size();
    }

    //INCERTIDUMBRE DE TRIANGULIZACION PARA GENERAR EL RUIDO Y REPRESENTARLO COMO UNA GAUSIANA
    //****************************************************************************************//

    //Filtro de planos en los puntos del keyframe
     uint32_t random;
     int maxPoints;
     int numSamples;
     int numLocalSamples;
     float maxError;
     float minInlierFraction;
     float maxDepthDiff;
     int numRetries;
     int planeSize;

     int BSD();

     bool sampleLocation(int &indice, vector<MapPoint*> &vpMapPoints);
     bool sampleLocation(int &indice, vector<MapPoint*> &vpMapPoints, const vector<size_t> &vpIndMapPoints);
     void filteredMapPoints();
     float TriangulationUncertainty(const Mat &T21, const float &depth1, const cv::Mat &ray1, const float &px_error_angle);
     float ErrorPixelAngle(KeyFrame* pKF);

protected:

    bool CheckNewKeyFrames(); // retorna si la cola de keyframes esta vazia retorna FALSE sino TRUE
    void ProcessNewKeyFrame(); //asigna como actual keyframe el front de la cola de keyframes y se procesa(adicionar keyframe al mapa, asignar puntos , crear grafo de covisibilidad)
    void CreateNewMapPoints(); //crear nuevos map points

    void MapPointCulling(); //a partir de los puntos creados recientemente se verifica para hacer una limpieza de los malos puntos
    void SearchInNeighbors(); //buscar los puntos en los vecinos y realizar la fusion

    void KeyFrameCulling(); //remueve los keyframes que tengan informacion redundante


    cv::Mat ComputeF12(KeyFrame* &pKF1, KeyFrame* &pKF2); //computa la matriz fundamental

    cv::Mat SkewSymmetricMatrix(const cv::Mat &v); //computa la matriz simetrica del vector3d

    bool mbMonocular; //variable TRUE si el sensor es monocular

    void ResetIfRequested(); //si RESET-REQUEST esta activado en TRUE se limpia la cola de keyframes y puntos recientes y se muda la variable REQUEST-RESET para falso
    bool mbResetRequested; //variable RESET-REQUEST que comienza en TRUE(se hizo una solicitud de reseteo?)
    std::mutex mMutexReset;

    bool CheckFinish(); //si finalizo el proceso?retorna variable FINISH-REQUEST
    void SetFinish(); //asigna TRUE a las variables FINISHED y STOPPED
    bool mbFinishRequested; //variable FINIS-REQUEST comienza en FALSO(se hizo una solicitud de finalizacion?)
    bool mbFinished; //variable FINISHED comienza en TRUE (esta finalizado?)
    std::mutex mMutexFinish;

    Map* mpMap; //puntero hacia la representacion del mapa donde estan los puntos

    LoopClosing* mpLoopCloser; //puntero al thread para buscar loops
    Tracking* mpTracker; //puntero al thread para hacer el tracking, crear nuevos puntos, localizar, estimar pose

    std::list<KeyFrame*> mlNewKeyFrames; //lista de nuevos keyframes

    KeyFrame* mpCurrentKeyFrame; //actual keyframe que esta siendo la referencia

    std::list<MapPoint*> mlpRecentAddedMapPoints; // puntos recientemente agregados y creados

    std::mutex mMutexNewKFs;

    bool mbAbortBA; //variable ABORT-BA (cancelamos el proceso de BA?)

    bool mbStopped; //variable STOPPED comienza en FALSO (esta parado?)
    bool mbStopRequested; //variable STOP-REQUEST comienza en FALSO (se hizo una solicitud para parar?)
    bool mbNotStop; //variable NOT-STOP comienza en FALSO(no parar?)
    std::mutex mMutexStop;

    bool mbAcceptKeyFrames; //variable ACCEPT-NEWKEYFRAME (acepta nuevo keyframe?)
    std::mutex mMutexAccept;
};

} //namespace ORB_SLAM

#endif // LOCALMAPPING_H
