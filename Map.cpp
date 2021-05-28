#include "Map.h"

#include<mutex>

namespace ORB_SLAM2
{

Map::Map():mnMaxKFid(0)
{
}

void Map::AddKeyFrame(KeyFrame *pKF)
{
    unique_lock<mutex> lock(mMutexMap);
    mspKeyFrames.insert(pKF);
    if(pKF->mnId>mnMaxKFid)
        mnMaxKFid=pKF->mnId;
}

void Map::AddMapPoint(MapPoint *pMP)
{
    unique_lock<mutex> lock(mMutexMap);
    mspMapPoints.insert(pMP);
}

void Map::EraseMapPoint(MapPoint *pMP)
{
    unique_lock<mutex> lock(mMutexMap);
    mspMapPoints.erase(pMP);

    // TODO: This only erase the pointer.
    // Delete the MapPoint
}

void Map::EraseKeyFrame(KeyFrame *pKF)
{
    unique_lock<mutex> lock(mMutexMap);
    mspKeyFrames.erase(pKF);

    // TODO: This only erase the pointer.
    // Delete the MapPoint
}

void Map::SetReferenceMapPoints(const vector<MapPoint *> &vpMPs)
{
    unique_lock<mutex> lock(mMutexMap);
    mvpReferenceMapPoints = vpMPs;
}

vector<KeyFrame*> Map::GetAllKeyFrames()
{
    unique_lock<mutex> lock(mMutexMap);
    return vector<KeyFrame*>(mspKeyFrames.begin(),mspKeyFrames.end());
}

vector<MapPoint*> Map::GetAllMapPoints()
{
    unique_lock<mutex> lock(mMutexMap);
    return vector<MapPoint*>(mspMapPoints.begin(),mspMapPoints.end());
}

long unsigned int Map::MapPointsInMap()
{
    unique_lock<mutex> lock(mMutexMap);
    return mspMapPoints.size();
}

long unsigned int Map::KeyFramesInMap()
{
    unique_lock<mutex> lock(mMutexMap);
    return mspKeyFrames.size();
}

vector<MapPoint*> Map::GetReferenceMapPoints()
{
    unique_lock<mutex> lock(mMutexMap);
    return mvpReferenceMapPoints;
}

long unsigned int Map::GetMaxKFid()
{
    unique_lock<mutex> lock(mMutexMap);
    return mnMaxKFid;
}

void Map::clear()
{
    for(set<MapPoint*>::iterator sit=mspMapPoints.begin(), send=mspMapPoints.end(); sit!=send; sit++)
        delete *sit;

    for(set<KeyFrame*>::iterator sit=mspKeyFrames.begin(), send=mspKeyFrames.end(); sit!=send; sit++)
        delete *sit;

    mspMapPoints.clear();
    mspKeyFrames.clear();
    mnMaxKFid = 0;
    mvpReferenceMapPoints.clear();
    mvpKeyFrameOrigins.clear();

    mspListSeeds.clear();
}


//LISTA DE MAPPOINTS EN PROCESO DE ACTUALIZACION DEL KEYFRAME DE REFERENCIA
//**********************************************************************************//

void Map::setOption(int poption){
    unique_lock<mutex> lock(mMutexMap);
    option = poption;
}

int Map::getOption(){
    unique_lock<mutex> lock(mMutexMap);
    return option;
}

bool Map::findPoint(MapPoint *pMP){
    unique_lock<mutex> lock(mMutexMap);
    return mspMapPoints.count(pMP) > 0;
}

long unsigned int Map::SeedsInMap(){
    unique_lock<mutex> lock(mMutexMap);
    return mspListSeeds.size();
}

vector<MapPoint*> Map::GetAllSeeds(){
    unique_lock<mutex> lock(mMutexMap);
    return vector<MapPoint*>(mspListSeeds.begin(),mspListSeeds.end());
}

bool Map:: findSeed(MapPoint* pMP){
    unique_lock<mutex> lock(mMutexSeeds);
    return mspListSeeds.count(pMP) > 0;
}

void Map::AddSeed(MapPoint* pMP){
    unique_lock<mutex> lock(mMutexSeeds);
    mspListSeeds.insert(pMP);
}

void Map::EraseSeed(MapPoint* pMP){
    unique_lock<mutex> lock(mMutexSeeds);
    mspListSeeds.erase(pMP);
}

void Map::initMapSeed(MapPoint* pMP, float &minDepth, float &meanDepth, float &medDepth, KeyFrame* pKF){
    pMP->initSeed(minDepth, meanDepth, medDepth);
    pMP->IncreaseUpdates(1);
    pMP->mninitSeedKFid = pKF->mnId;
    AddSeed(pMP);
}

bool Map::updateMapSeed(MapPoint* pMP, float &pmu, float &psigma2){
    if(pMP->updateSeed(pmu, psigma2)){
        pMP->IncreaseUpdates(1);
        pMP->checkSeed();
        return true;
    }else{
        pMP->checkSeed();
        return false;
    }
}

} //namespace ORB_SLAM
