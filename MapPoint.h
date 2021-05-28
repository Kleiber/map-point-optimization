#ifndef MAPPOINT_H
#define MAPPOINT_H

#include"KeyFrame.h"
#include"Frame.h"
#include"Map.h"

#include<opencv2/core/core.hpp>
#include<boost/math/distributions/normal.hpp>
#include<mutex>

#define SEED_NOT_INIT -1
#define SEED_DIVERGED 0
#define SEED_CONVERGED 1
#define SEED_UPDATE 2

#define SEED_DEPTH 1
#define SEED_INVERSE_DEPTH 2


namespace ORB_SLAM2
{

class KeyFrame;
class Map;
class Frame;

class MapPoint
{
public:
    MapPoint(const cv::Mat &Pos){ // crea un punto con una posicion en el mundo y supone que el punto es bueno
        Pos.copyTo(mWorldPos);
        mbBad = false;
    }

    MapPoint(const cv::Mat &Pos, KeyFrame* pRefKF, Map* pMap); //crea un punto con una posicion en el mundo el frame y keyframe de referencia y el mapa a donde pertenece
    MapPoint(const cv::Mat &Pos,  Map* pMap, Frame* pFrame, const int &idxF);// crea un punto con la posicion y el frame de referencia ademas sabe su ubicacion en la lista de keypoints

    void SetWorldPos(const cv::Mat &Pos); // asigna la posicion del punto  en el mundo
    cv::Mat GetWorldPos(); // recupera la posicion del punto en el mundo

    float getWorldPosUncertainty(); //asigna la incerteza de la pose
    void setWorldPosUncertainty(float PosUncertainty); //recupera la incerteza de la pose

    cv::Mat GetNormal(); // recupera el vector normal del punto (posicion - centro del keyframe, frame de referencia)
    KeyFrame* GetReferenceKeyFrame(); // recuperar el keyframe de referencia del punto

    std::map<KeyFrame*,size_t> GetObservations(); // recupera la lista de keyframes que observan a este punto y el pixel que lo genero en cada keyframe como valor
    int Observations(); // recupera numero de observaciones que se realizaron sobre este punto

    void AddObservation(KeyFrame* pKF,size_t idx); //agrega a las observaciones el puntero del keyframe como clave y y el pixel que lo genero en el keyframe
    void EraseObservation(KeyFrame* pKF); //elimina de las observaciones el puntero del keyframe y si es el frame de referencia se actualiza por uno nuevo

    int GetIndexInKeyFrame(KeyFrame* pKF); // retorna el indice del keyframe en la lista de observaciones solo si existe
    bool IsInKeyFrame(KeyFrame* pKF); //verifica si en el keyframe a sido observado este punto

    void SetBadFlag(); //elimina el punto de las observaciones y del mapa
    bool isBad(); // retorna si el punto es malo

    void Replace(MapPoint* pMP); // reemplaza el punto por otro, eliminado el punto del mapa y reemplazandolo en todos los keyframes que ha sido observado
    MapPoint* GetReplaced(); // retorna el puntero del punto con el cual a sido reemplazado

    void IncreaseVisible(int n=1); //incrementa la variable VISIBLE en 1 por defecto o N como parametro
    void IncreaseFound(int n=1); //incrementa la variable FOUND en 1 por defecto o N como parametro
    float GetFoundRatio(); // una relacion entre FOUND/VISIBLE
    inline int GetFound(){ // recupera la variable FOUND
        return mnFound;
    }

    void ComputeDistinctiveDescriptors(); //calculamos el mejor descriptor para este punto a partir de la informacion de los descriptores en los otros keyframes observados

    cv::Mat GetDescriptor(); //recupera el descriptor del punto

    void UpdateNormalAndDepth(); //actualiza la normal y max y min distancia (profundidad), basado en un promedio de las normales a los keyframes que observa a este punto

    float GetMinDistanceInvariance(); //el 80% de la mininma distancia
    float GetMaxDistanceInvariance(); //el 120%  de la maxima distancia
    int PredictScale(const float &currentDist, const float &logScaleFactor); //prediccion de la escala

public:
    long unsigned int mnId; // id del punto
    static long unsigned int nNextId; // siguiente id del nuevo punto
    long int mnFirstKFid; //id del primer keyframe de referencia
    long int mnFirstFrame; //id del primer frame de referencia
    int nObs; //numero de observaciones del punto

    // Variables used by the tracking
    float mTrackProjX;
    float mTrackProjY;
    float mTrackProjXR;
    bool mbTrackInView; //punto que no se usa para el tracking
    int mnTrackScaleLevel;
    float mTrackViewCos;
    long unsigned int mnTrackReferenceForFrame;
    long unsigned int mnLastFrameSeen; //ultimo frame donde se vio el punto

    // Variables used by local mapping
    long unsigned int mnBALocalForKF;
    long unsigned int mnFuseCandidateForKF;

    // Variables used by loop closing
    long unsigned int mnLoopPointForKF;
    long unsigned int mnCorrectedByKF;
    long unsigned int mnCorrectedReference;    
    cv::Mat mPosGBA;
    long unsigned int mnBAGlobalForKF;


    static std::mutex mGlobalMutex;


    //MAPPOINT COMPORTANDOSE COMO UN SEED PARA ACTUALIZAR LAS MEDICIONES DE LA PROFUNDIDAD
    //**********************************************************************************//

    long int mninitSeedKFid; //puntero al keyframe donde se inicio ek seed (todo las profundidades se proyectaran sobre este keyframe)
    long int mnupdateSeedKFid; //puntero al keyframe donde se hizo la ultima actualizacion del seed
    long int mnupdates; //numero de actualizaciones

    float mrange; //rango de la gaussiana creada sobre la profundidad del punto
    float ma; //parametro alpha de la distribucion beta
    float mb; //parametro beta de la distribucion beta
    float mmu; //media de la gaussiana (inicada con la profundidad media y intentamos convergir para la profundidad cierta)
    float msigma2; //variance de la gaussiana(indica cuando convergio la actualizacion)

    int mstate; //estado del seed

    float minlier; //reestriccion de inlier
    float moutlier; //reestriccion de outlier
    float mconvergence_sigma2_thresh; //restricion de convergencia

    void IncreaseUpdates(long int n);
    long int getUpdates();
    int getState();

    float normpdf(float x, float mu, float sigma2);

    void initSeed(float minDepth, float meanDepth, float medDepth); //inicializa las variables de la distribucion normal y la beta en el seed
    bool updateSeed(float pmu, float psigma2); //actualiza el seed
    void checkSeed(); //verifica el estado despues de la actualizacion

    void UpdatePos();
    void copySeed(MapPoint* pMP);
    bool fuseSeed(float pmu, float psigma, long int n);

    std::mutex mMutexSeeds;//controla las actualizaciones de los seeds

protected:
     // Position in absolute coordinates
     cv::Mat mWorldPos;  // posicion del punto en el world
     float mWorldPosUncertainty; //incerteza de la triangulizacion que genero este punto

     // Keyframes observing the point and associated index in keyframe
     std::map<KeyFrame*,size_t> mObservations; // lista de keyframes que tienen a este punto en su lista

     // Mean viewing direction
     cv::Mat mNormalVector; // vector normal del punto

     // Best descriptor to fast matching
     cv::Mat mDescriptor; // descriptor del punto

     // Reference KeyFrame
     KeyFrame* mpRefKF; // Keyframe a donde pertenece el punto

     // Tracking counters
     int mnVisible; //variable VISIBLE (cuantas veces a sido vista)
     int mnFound; //variable FOUND (cuantas veces a sido encontrada)

     // Bad flag (we do not currently erase MapPoint from memory)
     bool mbBad; // verifica que el punto no sea malo
     MapPoint* mpReplaced; // punto con el cual ha sido reemplazado si lo ha sido

     // Scale invariance distances
     float mfMinDistance; // min distancia  que se ubica a un 20% menos de la original
     float mfMaxDistance; // max distancia que se ubica a un 20% mas de la original

     Map* mpMap; // mapa de todos los puntos

     std::mutex mMutexPos;
     std::mutex mMutexFeatures;
};

} //namespace ORB_SLAM

#endif // MAPPOINT_H
