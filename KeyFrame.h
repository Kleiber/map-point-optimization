#ifndef KEYFRAME_H
#define KEYFRAME_H

#include "MapPoint.h"
#include "Thirdparty/DBoW2/DBoW2/BowVector.h"
#include "Thirdparty/DBoW2/DBoW2/FeatureVector.h"
#include "ORBVocabulary.h"
#include "ORBextractor.h"
#include "Frame.h"
#include "KeyFrameDatabase.h"

#include <mutex>

namespace ORB_SLAM2
{

class Map;
class MapPoint;
class Frame;
class KeyFrameDatabase;

class KeyFrame
{
public:
    KeyFrame(Frame &F, Map* pMap, KeyFrameDatabase* pKFDB);

    // Pose functions
    void SetPose(const cv::Mat &Tcw); // asignamos la transformacion del keyframe para el world
    cv::Mat GetPose(); // recuperar la transformacion del world para la camara (pose en el mundo)
    cv::Mat GetPoseInverse(); // recuperar la transformacion de la camara para el mundo
    cv::Mat GetCameraCenter(); // recuperar el centro del keyframe en el world
    cv::Mat GetStereoCenter();
    cv::Mat GetRotation(); // recuperar la rotacion de la camara con respecto al mundo
    cv::Mat GetTranslation(); // recuperar la traslacion de la camara con respecto el mundo

    // Bag of Words Representation
    void ComputeBoW(); // descriptor del keyframe en palabras

    // Covisibility graph functions
    void AddConnection(KeyFrame* pKF, const int &weight); //agrega o modifica puntero del keyframe como clave y valor ṕeso y actualiza la lista de mejor covisibles
    void EraseConnection(KeyFrame* pKF); //si existe el puntero del keyframe, lo elimina y actualiza la lista de mejor covisibles
    void UpdateConnections(); // actualiza el grafo de covisibilidad, lista de keyframes y pesos del grafo, verifica los frames que ven mas puntos
    void UpdateBestCovisibles(); // actualiza la lista de mejor covisibles segun los pesos(cantidad de puntos visibles)
    std::set<KeyFrame *> GetConnectedKeyFrames(); // recupera el conjunto de keyframes conectados en el grafo de coviibilidad
    std::vector<KeyFrame* > GetVectorCovisibleKeyFrames(); // recupera la lista de los keyframes conectados de forma ordenada por los pesos (puntos visibles)
    std::vector<KeyFrame*> GetBestCovisibilityKeyFrames(const int &N); // recupera una lista de N keyframes del grafo de covisibilidad
    std::vector<KeyFrame*> GetCovisiblesByWeight(const int &w); // recuperara una lista de keyframes indicando el maximo peso que deberian tener
    int GetWeight(KeyFrame* pKF); // recuperar el peso(cantidad de puntos que vio) de un keyframe

    // Spanning tree functions
    void AddChild(KeyFrame* pKF); // agrega el puntero de un keyframe al arbol de expansion
    void EraseChild(KeyFrame* pKF); // borra un puntero de un keyframe del arbol de expansion
    void ChangeParent(KeyFrame* pKF); // cambia el padre del arbol por el nuevo puntero del keyframe y adiciona este keyframe como hijo
    std::set<KeyFrame*> GetChilds(); // retorna la lista de punteros de los keyframes pertenecientes al arbol de expansion
    KeyFrame* GetParent(); // retorna el puntero para el padre del arbol de extension
    bool hasChild(KeyFrame* pKF); // verifica si el puntero del keyframe se encuentra en el arbol de exapnsion

    // Loop Edges
    void AddLoopEdge(KeyFrame* pKF); //agrega a la lista de loops el puntero del keyframe
    std::set<KeyFrame*> GetLoopEdges(); // recupera todos los keyframes que pertenecen a la lista de loops

    // MapPoint observation functions
    void AddMapPoint(MapPoint* pMP, const size_t &idx); // agrega un puntero de un punto en el keyframe y una posicion "id"(su respectivo match)
    void EraseMapPointMatch(const size_t &idx); // eliminar un puntero de un punto en el keyframe y su posicion "id"(respectivo match)
    void EraseMapPointMatch(MapPoint* pMP); //eliminar el puntero de un punto en el keyframe, buscar su "id"(respectivo match posicion en el keyframe) y eliminarlos
    void ReplaceMapPointMatch(const size_t &idx, MapPoint* pMP); //reemplazar un puntero de un punto por otro en la posicion "id" (respectivo match)
    std::set<MapPoint*> GetMapPoints(); // recuperar la lista de punteros de los puntos en el keyframe excepto los malos
    std::vector<MapPoint*> GetMapPointMatches(); //recuperar toda la lista de punteros de los puntos incluyendo los malos
    int TrackedMapPoints(const int &minObs); //cantidad de puntos con una cantidad minima de obsevaciones
    MapPoint* GetMapPoint(const size_t &idx);//recuperar un puntero de un punto en el keyframe en la posicion "id" (respectivo match)

    // KeyPoint functions
    std::vector<size_t> GetFeaturesInArea(const float &x, const float  &y, const float  &r) const;
    cv::Mat UnprojectStereo(int i); // reprojeccion de un punto del keyframe estereo

    // Image
    bool IsInImage(const float &x, const float &y) const; // verifica si el punto (x,y) pertenecen a la region de la imagen

    // Enable/Disable bad flag changes
    void SetNotErase(); //variable NOTERASE = true
    void SetErase(); //si no hay loops NOTERASE =false y verifica TOBEERASE para empezar a eliminar y actualizar

    // Set/check bad flag
    void SetBadFlag(); // caso TOBEERASE=true, si exite loops no se elimina las conecciones del keyframe caso contrario se elimina y se actualiza creando otros enlaces para los hijos
    bool isBad(); // retorna variable BAD (es malo el keyframe?)

    // Compute Scene Depth (q=2 median). Used in monocular.
    float ComputeSceneMedianDepth(const int q); // calcula la media de todas las profundidades de los puntos en el keyframe
    void ComputeSceneDepth(float &minDepth, float &maXDepth,  float &medDepth); //calcula la minima y la maxima profundidad

    static bool weightComp( int a, int b){  // sobrecarga para el metodo sort ordena de mayor a menor
        return a>b;
    }

    static bool lId(KeyFrame* pKF1, KeyFrame* pKF2){ //  sobrecarga para el metodo sort ordena quien tiene el menor id , es mas antiguo keyframe
        return pKF1->mnId<pKF2->mnId;
    }

    // The following variables are accesed from only 1 thread or never change (no mutex needed).
public:

    static long unsigned int nNextId; // proximo identificador
    long unsigned int mnId; // identificador del keyframe
    const long unsigned int mnFrameId; // identificador del frame

    const double mTimeStamp; // tiempo que fue adquirida el keyframe

    // Grid (to speed up feature matching)
    const int mnGridCols;
    const int mnGridRows;
    const float mfGridElementWidthInv;
    const float mfGridElementHeightInv;

    // Variables used by the tracking
    long unsigned int mnTrackReferenceForFrame;
    long unsigned int mnFuseTargetForKF;

    // Variables used by the local mapping
    long unsigned int mnBALocalForKF; // variable que almacena el id del keyframe donde se hace la optimizacion local
    long unsigned int mnBAFixedForKF; // variable que almacena el id del keyframe que es el fijo para la optimizacion

    // Variables used by the keyframe database
    long unsigned int mnLoopQuery;//indice del keyframe donde se hizo una solicitud de loop
    int mnLoopWords; //cuantas palabras en comun tienen dos frames
    float mLoopScore; //score que indica cuan parecidos son para detectar loop
    long unsigned int mnRelocQuery; //indice del keyframe donde se hizo una solicitud de localizacion
    int mnRelocWords; //cuantos palabras en comun tienen dos frames
    float mRelocScore; //score que indica cuan parecidos son dos frames en la localizacion

    // Variables used by loop closing
    cv::Mat mTcwGBA; //tranformacion cuando se hizo una optimizacion global y se encontro un loop
    cv::Mat mTcwBefGBA; // transformacion inversa cuando se hizo una optimizacion y se encontro un loop
    long unsigned int mnBAGlobalForKF; //variable que almacena el loop

    // Calibration parameters
    const float fx, fy, cx, cy, invfx, invfy, mbf, mb, mThDepth;

    // Number of KeyPoints
    const int N;

    // KeyPoints, stereo coordinate and descriptors (all associated by an index)
    const std::vector<cv::KeyPoint> mvKeys;  // lista de keypoints del keyframe
    const std::vector<cv::KeyPoint> mvKeysUn;
    const std::vector<float> mvuRight; // negative value for monocular points
    const std::vector<float> mvDepth; // negative value for monocular points
    const cv::Mat mDescriptors;

    //BoW
    DBoW2::BowVector mBowVec; // representacion del keyframe como un vector de palabras
    DBoW2::FeatureVector mFeatVec; // representacion del keyframe como un feature de palabras

    // Pose relative to parent (this is computed when bad flag is activated)
    cv::Mat mTcp;

    // Scale
    const int mnScaleLevels;
    const float mfScaleFactor;
    const float mfLogScaleFactor;
    const std::vector<float> mvScaleFactors;
    const std::vector<float> mvLevelSigma2;
    const std::vector<float> mvInvLevelSigma2;

    // Image bounds and calibration
    const int mnMinX; //limite inferior de imagen en x
    const int mnMinY; //limite inferior de imagen en y
    const int mnMaxX; //limite superior de imagen en x
    const int mnMaxY; //limite superior de imagen en y
    const cv::Mat mK; //matriz de calibracion focos y centro optico

    // The following variables need to be accessed trough a mutex to be thread safe.
//protected:

    // SE3 Pose and camera center
    cv::Mat Tcw; // transformacion del mundo para la camara
    cv::Mat Twc; // transformacion de la camara para el mundo
    cv::Mat Ow;  // centro de la camara en el mundo

    cv::Mat Cw; // Stereo middel point. Only for visualization

    // MapPoints associated to keypoints
    std::vector<MapPoint*> mvpMapPoints; // conjunto de punteros de los puntos en el keyframe

    // BoW
    KeyFrameDatabase* mpKeyFrameDB;  // base de datos de los keyframes en palabras
    ORBVocabulary* mpORBvocabulary;  // vocabulario para representar un keyframe en palabras

    // Grid over the image to speed up feature matching
    std::vector< std::vector <std::vector<size_t> > > mGrid;

    std::map<KeyFrame*,int> mConnectedKeyFrameWeights; //grafo de covisilidad de los keyframes
    std::vector<KeyFrame*> mvpOrderedConnectedKeyFrames; // vector ordenado contiene los keyframes que mejor covisibles son
    std::vector<int> mvOrderedWeights; // vector contiene los pesos de los keyframes que mejor covisibles son

    // Spanning Tree and Loop Edges
    bool mbFirstConnection; // variable para ver si el arbol de expansion aun no tiene ningun hijo (true) caso contrario(false)
    KeyFrame* mpParent; // punteros keyframe que es padre del arbol de expansion
    std::set<KeyFrame*> mspChildrens; // lista de punteros de los keyframes que son de hijos del arbol de expansion
    std::set<KeyFrame*> mspLoopEdges; //lista de punteros de los keyframes que pertenecen a un loops

    // Bad flags
    bool mbNotErase; //FALSO
    bool mbToBeErased;//FALSO
    bool mbBad;// FALSO

    float mHalfBaseline; // Only for visualization

    Map* mpMap;

    std::mutex mMutexPose;
    std::mutex mMutexConnections;
    std::mutex mMutexFeatures;

};

} //namespace ORB_SLAM

#endif // KEYFRAME_H
