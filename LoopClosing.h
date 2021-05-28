#ifndef LOOPCLOSING_H
#define LOOPCLOSING_H

#include "KeyFrame.h"
#include "LocalMapping.h"
#include "Map.h"
#include "ORBVocabulary.h"
#include "Tracking.h"

#include "KeyFrameDatabase.h"

#include <thread>
#include <mutex>
#include "Thirdparty/g2o/g2o/types/types_seven_dof_expmap.h"

namespace ORB_SLAM2
{

class Tracking;
class LocalMapping;
class KeyFrameDatabase;


class LoopClosing
{
public:

    typedef pair<set<KeyFrame*>,int> ConsistentGroup;    
    typedef map<KeyFrame*,g2o::Sim3,std::less<KeyFrame*>,
        Eigen::aligned_allocator<std::pair<const KeyFrame*, g2o::Sim3> > > KeyFrameAndPose;

public:

    //puntero del mapa, puntero de la base de datos de keyframes, puntero del vocabulario, es escala fija?
    LoopClosing(Map* pMap, KeyFrameDatabase* pDB, ORBVocabulary* pVoc,const bool bFixScale);

    void SetTracker(Tracking* pTracker); //asigna puntero del thread tracking (estimacion de pose, localizacion)

    void SetLocalMapper(LocalMapping* pLocalMapper); //asigna puntero del thread Mapping (maneja el mapa)

    // Main function
    void Run();

    void InsertKeyFrame(KeyFrame *pKF); //insertar frame en la cola de loops keyframes

    void RequestReset(); //se ha solicitado un reset? asigna la variable RESET-REQUEST como TRUE

    // This function will run in a separate thread
    void RunGlobalBundleAdjustment(unsigned long nLoopKF);

    bool isRunningGBA(){ //esta ejecutandose Global BA? retorna variable RUNNING-GBA
        unique_lock<std::mutex> lock(mMutexGBA);
        return mbRunningGBA;
    }
    bool isFinishedGBA(){ //finalizo el Global BA? retorna varibale FINISHED-GBA
        unique_lock<std::mutex> lock(mMutexGBA);
        return mbFinishedGBA;
    }   

    void RequestFinish(); //se ha solicitado una finalizacion? asigna la variable FINISH-REQUEST como TRUE

    bool isFinished();//esta finalizado? retorna variable FINISHED

protected:

    bool CheckNewKeyFrames();//si la cola de loops keyframes es vacia retorna FALSO sino TRUE

    bool DetectLoop(); //detecto loop?

    bool ComputeSim3();//para cada candidato a un loops computamos sim3

    void SearchAndFuse(const KeyFrameAndPose &CorrectedPosesMap); //fusionar los datos del loop

    void CorrectLoop(); //loop correcto

    void ResetIfRequested(); //si se solicito un reset, limpia la cola de loops keyframes
    bool mbResetRequested;// variable RESET-REQUEST (hay una solicitud de resetear?)
    std::mutex mMutexReset;

    bool CheckFinish(); //retorna la variable FINISH-REQUEST
    void SetFinish(); //asigna valor a la variable FINISHED
    bool mbFinishRequested; //variable solicitud de finalizacion FINISH-REQUEST
    bool mbFinished; //varible finalizado FINISHED
    std::mutex mMutexFinish;

    Map* mpMap; //puntero para la estructura del mapa
    Tracking* mpTracker; //puntero para el thread Tracking

    KeyFrameDatabase* mpKeyFrameDB; //puntero a la base de datos de keyframes
    ORBVocabulary* mpORBVocabulary; //puntero al vocabulario de palabras

    LocalMapping *mpLocalMapper; //puntero para el thread del Mapping

    std::list<KeyFrame*> mlpLoopKeyFrameQueue; //cola de los loop keyframes

    std::mutex mMutexLoopQueue;

    // Loop detector parameters
    float mnCovisibilityConsistencyTh; //prametro para verificar si es un loop

    // Loop detector variables
    KeyFrame* mpCurrentKF; //keyframe actual
    KeyFrame* mpMatchedKF; //
    std::vector<ConsistentGroup> mvConsistentGroups;
    std::vector<KeyFrame*> mvpEnoughConsistentCandidates;
    std::vector<KeyFrame*> mvpCurrentConnectedKFs;
    std::vector<MapPoint*> mvpCurrentMatchedPoints;
    std::vector<MapPoint*> mvpLoopMapPoints;
    cv::Mat mScw;
    g2o::Sim3 mg2oScw;

    long unsigned int mLastLoopKFid;

    // Variables related to Global Bundle Adjustment
    bool mbRunningGBA; //ejecutando GBA
    bool mbFinishedGBA; //finalizado GBA
    bool mbStopGBA; //parar GBA
    std::mutex mMutexGBA;
    std::thread* mpThreadGBA; //thread de GBA

    // Fix scale in the stereo/RGB-D case
    bool mbFixScale;
};

} //namespace ORB_SLAM

#endif // LOOPCLOSING_H
