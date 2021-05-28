#ifndef MAP_H
#define MAP_H

#include "MapPoint.h"
#include "KeyFrame.h"
#include "planefiltering.h"
#include <set>

#include <mutex>



namespace ORB_SLAM2
{

class MapPoint;
class KeyFrame;
class PlaneFiltering;

class Map
{
public:
    Map();

    void AddKeyFrame(KeyFrame* pKF); // agrega el puntero de un keyframe
    void AddMapPoint(MapPoint* pMP); // agrega el puntero de un punto
    void EraseMapPoint(MapPoint* pMP); // elimina el puntero de un punto
    void EraseKeyFrame(KeyFrame* pKF); // elimina el puntero de un keyframe
    void SetReferenceMapPoints(const std::vector<MapPoint*> &vpMPs); // asigna un puntero a la lista de referencia de puntos

    std::vector<KeyFrame*> GetAllKeyFrames(); // retorna un vector con los punteros de los keyframes
    std::vector<MapPoint*> GetAllMapPoints(); //retorna un vector con los punteros de los puntos
    std::vector<MapPoint*> GetReferenceMapPoints(); // retornan un vector con los punteros de los puntos en referencia

    long unsigned int MapPointsInMap(); //cantidad de puntos que existen en el mapa
    long unsigned  KeyFramesInMap(); //cantidad de keyframes que existen en el mapa

    long unsigned int GetMaxKFid(); // identificador maximo de uno de los keyframes

    void clear(); // limpiar las lista


    vector<KeyFrame*> mvpKeyFrameOrigins;

    std::mutex mMutexMapUpdate;

    // This avoid that two points are created simultaneously in separate threads (id conflict)
    std::mutex mMutexPointCreation;


    //LISTA DE MAPPOINTS EN PROCESO DE ACTUALIZACION DEL KEYFRAME DE REFERENCIA
    //**********************************************************************************//
    void setOption(int poption);
    int getOption();
    bool findPoint(MapPoint* pMP);
    long unsigned int SeedsInMap();
    std::vector<MapPoint*> GetAllSeeds();
    bool findSeed(MapPoint* pMP);
    void AddSeed(MapPoint* pMP);
    void EraseSeed(MapPoint* pMP);
    void initMapSeed(MapPoint* pMP, float &minDepth, float &meanDepth, float &medDepth, KeyFrame* pKF);
    bool updateMapSeed(MapPoint* pMP, float &pmu, float &psigma2);

    int option;
    std::set<MapPoint*> mspListSeeds; //lista de los puntos en el seed (clave punto, valor pixel)
    std::mutex mMutexSeeds; //controla ingresos a la lista de seed

protected:
    std::set<KeyFrame*> mspKeyFrames; //--conjunto de frames en el mapa
    std::set<MapPoint*> mspMapPoints; //--conjunto de puntos en el mapa
    std::vector<MapPoint*> mvpReferenceMapPoints; //--vector de puntos de referencia en un instante

    long unsigned int mnMaxKFid; //-- maximo keyframes

    std::mutex mMutexMap;

};

} //namespace ORB_SLAM

#endif // MAP_H
