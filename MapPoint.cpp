#include "MapPoint.h"
#include "ORBmatcher.h"

#include<mutex>

namespace ORB_SLAM2
{

long unsigned int MapPoint::nNextId=0;
mutex MapPoint::mGlobalMutex;

MapPoint::MapPoint(const cv::Mat &Pos, KeyFrame *pRefKF, Map* pMap):
    mnFirstKFid(pRefKF->mnId), mnFirstFrame(pRefKF->mnFrameId), nObs(0), mnTrackReferenceForFrame(0),
    mnLastFrameSeen(0), mnBALocalForKF(0), mnFuseCandidateForKF(0), mnLoopPointForKF(0), mnCorrectedByKF(0),
    mnCorrectedReference(0), mnBAGlobalForKF(0),mninitSeedKFid(-1), mnupdateSeedKFid(-1), mnupdates(-1), mstate(-1),
    mpRefKF(pRefKF), mnVisible(1), mnFound(1), mbBad(false), mpReplaced(static_cast<MapPoint*>(NULL)),
    mfMinDistance(0), mfMaxDistance(0), mpMap(pMap)
{
    Pos.copyTo(mWorldPos);
    mNormalVector = cv::Mat::zeros(3,1,CV_32F);

    // MapPoints can be created from Tracking and Local Mapping. This mutex avoid conflicts with id.
    unique_lock<mutex> lock(mpMap->mMutexPointCreation);
    mnId=nNextId++;
}

MapPoint::MapPoint(const cv::Mat &Pos, Map* pMap, Frame* pFrame, const int &idxF):
    mnFirstKFid(-1), mnFirstFrame(pFrame->mnId), nObs(0), mnTrackReferenceForFrame(0), mnLastFrameSeen(0),
    mnBALocalForKF(0), mnFuseCandidateForKF(0),mnLoopPointForKF(0), mnCorrectedByKF(0),
    mnCorrectedReference(0), mnBAGlobalForKF(0), mninitSeedKFid(-1), mnupdateSeedKFid(-1), mnupdates(-1),mstate(-1),
    mpRefKF(static_cast<KeyFrame*>(NULL)), mnVisible(1),mnFound(1), mbBad(false), mpReplaced(NULL), mpMap(pMap)
{
    Pos.copyTo(mWorldPos);
    cv::Mat Ow = pFrame->GetCameraCenter();
    mNormalVector = mWorldPos - Ow;
    mNormalVector = mNormalVector/cv::norm(mNormalVector);

    cv::Mat PC = Pos - Ow;
    const float dist = cv::norm(PC);
    const int level = pFrame->mvKeysUn[idxF].octave;
    const float levelScaleFactor =  pFrame->mvScaleFactors[level];
    const int nLevels = pFrame->mnScaleLevels;

    mfMaxDistance = dist*levelScaleFactor;
    mfMinDistance = mfMaxDistance/pFrame->mvScaleFactors[nLevels-1];

    pFrame->mDescriptors.row(idxF).copyTo(mDescriptor);

    // MapPoints can be created from Tracking and Local Mapping. This mutex avoid conflicts with id.
    unique_lock<mutex> lock(mpMap->mMutexPointCreation);
    mnId=nNextId++;
}

void MapPoint::SetWorldPos(const cv::Mat &Pos)
{
    unique_lock<mutex> lock2(mGlobalMutex);
    unique_lock<mutex> lock(mMutexPos);
    Pos.copyTo(mWorldPos);
}

cv::Mat MapPoint::GetWorldPos()
{
    unique_lock<mutex> lock(mMutexPos);
    return mWorldPos.clone();
}

cv::Mat MapPoint::GetNormal()
{
    unique_lock<mutex> lock(mMutexPos);
    return mNormalVector.clone();
}

KeyFrame* MapPoint::GetReferenceKeyFrame()
{
     unique_lock<mutex> lock(mMutexFeatures);
     return mpRefKF;
}

void MapPoint::AddObservation(KeyFrame* pKF, size_t idx)
{
    unique_lock<mutex> lock(mMutexFeatures);
    if(mObservations.count(pKF))
        return;
    mObservations[pKF]=idx;

    if(pKF->mvuRight[idx]>=0)
        nObs+=2;
    else
        nObs++;
}

void MapPoint::EraseObservation(KeyFrame* pKF)
{
    bool bBad=false;
    {
        unique_lock<mutex> lock(mMutexFeatures);
        if(mObservations.count(pKF))
        {
            int idx = mObservations[pKF];
            if(pKF->mvuRight[idx]>=0)
                nObs-=2;
            else
                nObs--;

            mObservations.erase(pKF);

            if(mpRefKF==pKF)
                mpRefKF=mObservations.begin()->first;

            // If only 2 observations or less, discard point
            if(nObs<=2)
                bBad=true;
        }
    }

    if(bBad)
        SetBadFlag();
}

map<KeyFrame*, size_t> MapPoint::GetObservations()
{
    unique_lock<mutex> lock(mMutexFeatures);
    return mObservations;
}

int MapPoint::Observations()
{
    unique_lock<mutex> lock(mMutexFeatures);
    return nObs;
}

void MapPoint::SetBadFlag()
{
    map<KeyFrame*,size_t> obs;
    {
        unique_lock<mutex> lock1(mMutexFeatures);
        unique_lock<mutex> lock2(mMutexPos);
        mbBad=true;
        obs = mObservations;
        mObservations.clear();
    }
    for(map<KeyFrame*,size_t>::iterator mit=obs.begin(), mend=obs.end(); mit!=mend; mit++)
    {
        KeyFrame* pKF = mit->first;
        pKF->EraseMapPointMatch(mit->second);
    }

    if(mpMap->findSeed(this)) mpMap->EraseSeed(this);
    mpMap->EraseMapPoint(this);
}

MapPoint* MapPoint::GetReplaced()
{
    unique_lock<mutex> lock1(mMutexFeatures);
    unique_lock<mutex> lock2(mMutexPos);
    return mpReplaced;
}

void MapPoint::Replace(MapPoint* pMP)
{
    if(pMP->mnId==this->mnId)
        return;

    int nvisible, nfound;

    float mu, sigma2, PosUncertainty;
    cv::Mat Pos;

    map<KeyFrame*,size_t> obs;
    {
        unique_lock<mutex> lock1(mMutexFeatures);
        unique_lock<mutex> lock2(mMutexPos);

        obs=mObservations;
        mObservations.clear();
        mbBad=true;
        nvisible = mnVisible;
        nfound = mnFound;
        mpReplaced = pMP;

        mu = mmu;
        sigma2 = msigma2;
        PosUncertainty = mWorldPosUncertainty;
        Pos = mWorldPos;
    }


    //fusionamos informacion
    int state = getState();
    int stateReplace = pMP->getState();
    bool ok = false;
    long int updates = 0;

    if(state != SEED_NOT_INIT){
        if(stateReplace == SEED_NOT_INIT){
            pMP->copySeed(this);
        }else{
            ok = pMP->fuseSeed(mu, sigma2, getUpdates());
            ok = pMP->fuseSeed(Pos.at<float>(2), PosUncertainty, 1);
            updates = pMP->getUpdates();

            stateReplace = pMP->getState();
            if(stateReplace == SEED_CONVERGED) mpMap->EraseSeed(pMP);
            if(stateReplace == SEED_DIVERGED) mpMap->EraseSeed(pMP);
        }
    }

    if(stateReplace == SEED_CONVERGED) pMP->UpdatePos();
    mpMap->EraseSeed(this);

    //reemplazamos con el nuevo punto o eliminamos el antiguo punto en las observaciones
    for(map<KeyFrame*,size_t>::iterator mit=obs.begin(), mend=obs.end(); mit!=mend; mit++)
    {
        KeyFrame* pKF = mit->first;

        if(!pMP->IsInKeyFrame(pKF))
        {
            pKF->ReplaceMapPointMatch(mit->second, pMP);
            pMP->AddObservation(pKF,mit->second);
        }
        else
        {
            pKF->EraseMapPointMatch(mit->second);
        }
    }

    pMP->IncreaseFound(nfound);
    pMP->IncreaseVisible(nvisible);
    pMP->ComputeDistinctiveDescriptors();

    mpMap->EraseMapPoint(this);
}

bool MapPoint::isBad()
{
    unique_lock<mutex> lock(mMutexFeatures);
    unique_lock<mutex> lock2(mMutexPos);
    return mbBad;
}

void MapPoint::IncreaseVisible(int n)
{
    unique_lock<mutex> lock(mMutexFeatures);
    mnVisible+=n;
}

void MapPoint::IncreaseFound(int n)
{
    unique_lock<mutex> lock(mMutexFeatures);
    mnFound+=n;
}

float MapPoint::GetFoundRatio()
{
    unique_lock<mutex> lock(mMutexFeatures);
    return static_cast<float>(mnFound)/mnVisible;
}

void MapPoint::ComputeDistinctiveDescriptors()
{
    // Retrieve all observed descriptors
    vector<cv::Mat> vDescriptors;

    map<KeyFrame*,size_t> observations;

    {
        unique_lock<mutex> lock1(mMutexFeatures);
        if(mbBad)
            return;
        observations=mObservations;
    }

    if(observations.empty())
        return;

    vDescriptors.reserve(observations.size());

    for(map<KeyFrame*,size_t>::iterator mit=observations.begin(), mend=observations.end(); mit!=mend; mit++)
    {
        KeyFrame* pKF = mit->first;

        if(!pKF->isBad())
            vDescriptors.push_back(pKF->mDescriptors.row(mit->second));
    }

    if(vDescriptors.empty())
        return;

    // Compute distances between them
    const size_t N = vDescriptors.size();

    float Distances[N][N];
    for(size_t i=0;i<N;i++)
    {
        Distances[i][i]=0;
        for(size_t j=i+1;j<N;j++)
        {
            int distij = ORBmatcher::DescriptorDistance(vDescriptors[i],vDescriptors[j]);
            Distances[i][j]=distij;
            Distances[j][i]=distij;
        }
    }

    // Take the descriptor with least median distance to the rest
    int BestMedian = INT_MAX;
    int BestIdx = 0;
    for(size_t i=0;i<N;i++)
    {
        vector<int> vDists(Distances[i],Distances[i]+N);
        sort(vDists.begin(),vDists.end());
        int median = vDists[0.5*(N-1)];

        if(median<BestMedian)
        {
            BestMedian = median;
            BestIdx = i;
        }
    }

    {
        unique_lock<mutex> lock(mMutexFeatures);
        mDescriptor = vDescriptors[BestIdx].clone();       
    }
}

cv::Mat MapPoint::GetDescriptor()
{
    unique_lock<mutex> lock(mMutexFeatures);
    return mDescriptor.clone();
}

int MapPoint::GetIndexInKeyFrame(KeyFrame *pKF)
{
    unique_lock<mutex> lock(mMutexFeatures);
    if(mObservations.count(pKF))
        return mObservations[pKF];
    else
        return -1;
}

bool MapPoint::IsInKeyFrame(KeyFrame *pKF)
{
    unique_lock<mutex> lock(mMutexFeatures);
    return (mObservations.count(pKF));
}

void MapPoint::UpdateNormalAndDepth()
{
    map<KeyFrame*,size_t> observations;
    KeyFrame* pRefKF;
    cv::Mat Pos;
    {
        unique_lock<mutex> lock1(mMutexFeatures);
        unique_lock<mutex> lock2(mMutexPos);
        if(mbBad)
            return;
        observations=mObservations;
        pRefKF=mpRefKF;
        Pos = mWorldPos.clone();
    }

    if(observations.empty())
        return;

    cv::Mat normal = cv::Mat::zeros(3,1,CV_32F);
    int n=0;
    for(map<KeyFrame*,size_t>::iterator mit=observations.begin(), mend=observations.end(); mit!=mend; mit++)
    {
        KeyFrame* pKF = mit->first;
        cv::Mat Owi = pKF->GetCameraCenter();
        cv::Mat normali = mWorldPos - Owi;
        normal = normal + normali/cv::norm(normali);
        n++;
    } 

    cv::Mat PC = Pos - pRefKF->GetCameraCenter();
    const float dist = cv::norm(PC);
    const int level = pRefKF->mvKeysUn[observations[pRefKF]].octave;
    const float levelScaleFactor =  pRefKF->mvScaleFactors[level];
    const int nLevels = pRefKF->mnScaleLevels;

    {
        unique_lock<mutex> lock3(mMutexPos);
        mfMaxDistance = dist*levelScaleFactor;
        mfMinDistance = mfMaxDistance/pRefKF->mvScaleFactors[nLevels-1];
        mNormalVector = normal/n;
    }
}

float MapPoint::GetMinDistanceInvariance()
{
    unique_lock<mutex> lock(mMutexPos);
    return 0.8f*mfMinDistance;
}

float MapPoint::GetMaxDistanceInvariance()
{
    unique_lock<mutex> lock(mMutexPos);
    return 1.2f*mfMaxDistance;
}

int MapPoint::PredictScale(const float &currentDist, const float &logScaleFactor)
{
    float ratio;
    {
        unique_lock<mutex> lock3(mMutexPos);
        ratio = mfMaxDistance/currentDist;
    }

    return ceil(log(ratio)/logScaleFactor);
}


//MAPPOINT COMPORTANDOSE COMO UN SEED PARA ACTUALIZAR LAS MEDICIONES DE LA PROFUNDIDAD
//**********************************************************************************//
float MapPoint::getWorldPosUncertainty(){
    unique_lock<mutex> lock(mMutexPos);
    return mWorldPosUncertainty;
}

void MapPoint::setWorldPosUncertainty(float PosUncertainty){
    unique_lock<mutex> lock(mMutexPos);
    mWorldPosUncertainty = PosUncertainty;
}

void MapPoint :: IncreaseUpdates(long int n){
    //incrementa el numero de actualizaciones
    unique_lock<mutex> lock(mMutexSeeds);
    mnupdates += (n > 0)? n : 0;
}

long int MapPoint :: getUpdates(){
    //retorna el numero de actualizaciones del seed
    unique_lock<mutex> lock(mMutexSeeds);
    return mnupdates;
}

int MapPoint :: getState(){
    //retorna el estado del seed
    unique_lock<mutex> lock(mMutexSeeds);
    return mstate;
}

float MapPoint :: normpdf(float x, float mu, float sigma2){
    //probabilistic density function
    return (expf(-(x-mu)*(x-mu) / (2.0f*sigma2))) * (1.0f/sqrtf(2.0f*M_PI*sigma2));
}

void MapPoint::initSeed(float minDepth, float maxDepth, float medDepth){
    //uno trabaja con la inversa y otro con la profundidad normal
    cv::Mat Pos;
    int option;
    {
        unique_lock<mutex> lock(mMutexPos);
        Pos = mWorldPos.clone();
        option = mpMap->getOption();
    }

    if(option == SEED_DEPTH){
        {
            unique_lock<mutex> lock(mMutexSeeds);

            mrange = maxDepth - minDepth;
            mmu = medDepth;
            msigma2 = mrange*mrange/36.0f;
            ma = 10.0f;
            mb = 10.0f;
            minlier = 0.7f;
            moutlier = 0.05f;
            mconvergence_sigma2_thresh = mrange/1000.0f;
            mnupdates = -1;
            mninitSeedKFid = -1;
            mstate = SEED_UPDATE;
        }
    }

    if(option == SEED_INVERSE_DEPTH){
        {
            unique_lock<mutex> lock(mMutexSeeds);
            mrange = 1.0/minDepth;
            mmu = 1.0/medDepth;
            msigma2 = mrange*mrange/36.0;
            ma = 10.0;
            mb = 10.0;
            minlier = 0.7;
            moutlier = 0.05;
            mconvergence_sigma2_thresh = mrange/200.0;
            mnupdates = -1;
            mninitSeedKFid = -1;
            mstate = SEED_UPDATE;
        }
    }
}

bool MapPoint::updateSeed(float pmu, float psigma2){
    //uno trabaja con la inversa y otro con la profundidad normal

    float range, mu, sigma2, a, b;
    int option;
    {
        unique_lock<mutex> lock1(mMutexSeeds);
        range = mrange;
        mu = mmu;
        sigma2 = msigma2;
        a = ma;
        b = mb;
        option = mpMap->getOption();
    }

    if(option == SEED_DEPTH){

        if(std::isnan(sigma2 + psigma2)) return false;

        float fusion_sigma2 = (psigma2 * sigma2) / (psigma2 + sigma2);
        float fusion_mu = fusion_sigma2*(mu/sigma2 + pmu/psigma2);

        float c1   = (a/(a+b)) * normpdf(pmu, mu, sigma2 + psigma2);
        float c2   = (b/(a+b)) * (1.0f/range);

        double normalize = c1 + c2;
        c1 = c1 / normalize;
        c2 = c2 / normalize;

        float f = c1*((a+1.0f)/(a+b+1.0f)) + c2*(a/(a+b+1.0f));
        float e = c1*(((a+1.0f)*(a+2.0f))/((a+b+1.0f)*(a+b+2.0f))) + c2*(a*(a+1.0f)/((a+b+1.0f)*(a+b+2.0f)));

        if(std::isnan(c1*fusion_mu)) return false;

        float new_mu = c1*fusion_mu + c2*mu;
        float new_sigma2 = c1*(fusion_sigma2 + fusion_mu*fusion_mu) + c2*(sigma2 + mu*mu) - new_mu*new_mu;
        float new_a = (e-f)/(f-e/f);
        float new_b = new_a*(1.0f-f)/f;

        //std::cout<<mnId<<" "<<mWorldPos.at<float>(2)<<" - ["<<mu<<" "<<sigma2<<"] ["<<pmu<<" "<<psigma2<<"] to ["<<new_mu<<" "<<new_sigma2<<"] ";

        {
            unique_lock<mutex> lock2(mMutexSeeds);
            mmu = new_mu;
            msigma2 = new_sigma2;
            ma = new_a;
            mb = new_b;
        }

        return true;
    }

    if(option == SEED_INVERSE_DEPTH){

         if(std::isnan(sigma2 + psigma2)) return false;

         double fusion_sigma2 = 1./(1./sigma2 + 1./psigma2);
         double fusion_mu = fusion_sigma2*(mu/sigma2 + pmu/psigma2);

         float c1 = a/(a + b) * normpdf(pmu, mu, sigma2 + psigma2);
         float c2 = b/(a + b) * 1./range;

         float normalization = c1 + c2;
         c1 /= normalization;
         c2 /= normalization;

         float f = c1*(a+1.)/(a+b+1.) + c2*a/(a+b+1.);
         float e = c1*(a+1.)*(a+2.)/((a+b+1.)*(a+b+2.)) + c2*a*(a+1.0f)/((a+b+1.0f)*(a+b+2.0f));

         if(std::isnan(c1 * fusion_mu)) return false;

         float new_mu = c1*fusion_mu + c2*mu;
         float new_sigma2 = c1*(fusion_sigma2 + fusion_mu*fusion_mu) + c2*(sigma2 + mu*mu) - new_mu*new_mu;
         float new_a = (e-f)/(f-e/f);
         float new_b = new_a*(1.0f-f)/f;

         {
             unique_lock<mutex> lock3(mMutexSeeds);
             mmu = new_mu;
             msigma2 = new_sigma2;
             ma = new_a;
             mb = new_b;
         }

         return true;
    }

    return false;
}

void MapPoint::checkSeed(){
    //uno trabaja con la inversa y otro con la profundidad normal

    float mu, sigma2,a, b, state;
    int option;
    {
        unique_lock<mutex> lock1(mMutexSeeds);
        mu = mmu;
        sigma2 = msigma2;
        a = ma;
        b = mb;
        state = mstate;
        option = mpMap->getOption();
    }

    if(option == SEED_DEPTH){

        float mu_beta = a/(a + b); // > minlier (CONVERGE)
        float mode_beta = (a - 1.0)/(a + b - 2.0); // < moutlier (DIVERGE)

        if(sigma2 < mconvergence_sigma2_thresh){
            state = SEED_CONVERGED;
        }else{
            if(std::isnan(mu + sigma2)) state = SEED_DIVERGED;
            else state = SEED_UPDATE;
        }

        //std::cout<<"- {"<<(sigma2<mconvergence_sigma2_thresh)<<"} {"<<mu_beta<<"} {"<<mode_beta<<"} u:"<<mnupdates<<" s:"<<state<<endl;

        {
            unique_lock<mutex> lock2(mMutexSeeds);
            mstate = state;
        }
    }

    if(option == SEED_INVERSE_DEPTH){

        if(sqrtf(sigma2) < mconvergence_sigma2_thresh){
            state = SEED_CONVERGED;
        }else{
            if(std::isnan(mu + sqrtf(sigma2))) state = SEED_DIVERGED;
            else state = SEED_UPDATE;
        }

        {
            unique_lock<mutex> lock3(mMutexSeeds);
            mstate = state;
        }
    }
}

void MapPoint :: UpdatePos(){
    int option;
    float mu;
    cv::Mat Pos;
    cv::Mat new_Pos;
    {
        unique_lock<mutex> lock1(mMutexSeeds);
        option = mpMap->getOption();
        Pos = mWorldPos;
        mu = mmu;
    }

    if(option == SEED_DEPTH) new_Pos = (Pos/Pos.at<float>(2))*mu;
    if(option == SEED_INVERSE_DEPTH) new_Pos = (Pos/Pos.at<float>(2))*1./mu;

    SetWorldPos(new_Pos);
}


void MapPoint :: copySeed(MapPoint *pMP){
    //copiamos los parametros del seed
    {
        unique_lock<mutex> lock(mMutexSeeds);
        mrange = pMP->mrange;
        mmu = pMP->mmu;
        msigma2 = pMP->msigma2;
        ma = pMP->ma;
        mb = pMP->mb;
        mnupdates = pMP->mnupdates;
        mstate = pMP->mstate;
        minlier = pMP->minlier;
        moutlier = pMP->moutlier;
        mninitSeedKFid = pMP->mninitSeedKFid;
        mconvergence_sigma2_thresh = pMP->mconvergence_sigma2_thresh;
    }
}

bool MapPoint ::fuseSeed(float pmu, float psigma, long int n){
    //realizamos la fusion de dos seeds

    float mu = 0.0, sigma = 0.0;
    int option;
    {
        unique_lock<mutex> lock1(mMutexSeeds);
        option = mpMap->getOption();
    }

    if(option == SEED_DEPTH){
        mu = pmu;
        sigma = psigma;
    }

    if(option == SEED_INVERSE_DEPTH){
        mu = 1./pmu;
        sigma = 0.5 * (1.0/max(0.0000001f, pmu-psigma) - 1.0/(pmu+psigma));
    }


    if(updateSeed(mu, sigma*sigma)){
        IncreaseUpdates(n);
        checkSeed();
        return true;
    }else{
        checkSeed();
        return false;
    }
}

} //namespace ORB_SLAM
