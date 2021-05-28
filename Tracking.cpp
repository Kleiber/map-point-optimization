#include "Tracking.h"

#include<opencv2/core/core.hpp>
#include<opencv2/features2d/features2d.hpp>

#include"ORBmatcher.h"
#include"FrameDrawer.h"
#include"Converter.h"
#include"Map.h"
#include"Initializer.h"

#include"Optimizer.h"
#include"PnPsolver.h"

#include<iostream>

#include<mutex>


using namespace std;

namespace ORB_SLAM2
{

Tracking::Tracking(System *pSys, ORBVocabulary* pVoc, FrameDrawer *pFrameDrawer, MapDrawer *pMapDrawer, Map *pMap, KeyFrameDatabase* pKFDB, const string &strSettingPath, const int sensor):
    mState(NO_IMAGES_YET), mSensor(sensor), mbOnlyTracking(false), mbVO(false), mpORBVocabulary(pVoc),
    mpKeyFrameDB(pKFDB), mpInitializer(static_cast<Initializer*>(NULL)), mpSystem(pSys),
    mpFrameDrawer(pFrameDrawer), mpMapDrawer(pMapDrawer), mpMap(pMap), mnLastRelocFrameId(0)
{
    // Load camera parameters from settings file

    cv::FileStorage fSettings(strSettingPath, cv::FileStorage::READ);
    float fx = fSettings["Camera.fx"];
    float fy = fSettings["Camera.fy"];
    float cx = fSettings["Camera.cx"];
    float cy = fSettings["Camera.cy"];

    cv::Mat K = cv::Mat::eye(3,3,CV_32F);
    K.at<float>(0,0) = fx;
    K.at<float>(1,1) = fy;
    K.at<float>(0,2) = cx;
    K.at<float>(1,2) = cy;
    K.copyTo(mK);

    cv::Mat DistCoef(4,1,CV_32F);
    DistCoef.at<float>(0) = fSettings["Camera.k1"];
    DistCoef.at<float>(1) = fSettings["Camera.k2"];
    DistCoef.at<float>(2) = fSettings["Camera.p1"];
    DistCoef.at<float>(3) = fSettings["Camera.p2"];
    const float k3 = fSettings["Camera.k3"];
    if(k3!=0)
    {
        DistCoef.resize(5);
        DistCoef.at<float>(4) = k3;
    }
    DistCoef.copyTo(mDistCoef);

    mbf = fSettings["Camera.bf"];

    float fps = fSettings["Camera.fps"];
    if(fps==0)
        fps=30;

    // Max/Min Frames to insert keyframes and to check relocalisation
    mMinFrames = 0;
    mMaxFrames = fps;

    cout << endl << "Camera Parameters: " << endl;
    cout << "- fx: " << fx << endl;
    cout << "- fy: " << fy << endl;
    cout << "- cx: " << cx << endl;
    cout << "- cy: " << cy << endl;
    cout << "- k1: " << DistCoef.at<float>(0) << endl;
    cout << "- k2: " << DistCoef.at<float>(1) << endl;
    if(DistCoef.rows==5)
        cout << "- k3: " << DistCoef.at<float>(4) << endl;
    cout << "- p1: " << DistCoef.at<float>(2) << endl;
    cout << "- p2: " << DistCoef.at<float>(3) << endl;
    cout << "- fps: " << fps << endl;


    int nRGB = fSettings["Camera.RGB"];
    mbRGB = nRGB;

    if(mbRGB)
        cout << "- color order: RGB (ignored if grayscale)" << endl;
    else
        cout << "- color order: BGR (ignored if grayscale)" << endl;

    // Load ORB parameters

    int nFeatures = fSettings["ORBextractor.nFeatures"];
    float fScaleFactor = fSettings["ORBextractor.scaleFactor"];
    int nLevels = fSettings["ORBextractor.nLevels"];
    int fIniThFAST = fSettings["ORBextractor.iniThFAST"];
    int fMinThFAST = fSettings["ORBextractor.minThFAST"];

    mpORBextractorLeft = new ORBextractor(nFeatures,fScaleFactor,nLevels,fIniThFAST,fMinThFAST);

    if(sensor==System::STEREO)
        mpORBextractorRight = new ORBextractor(nFeatures,fScaleFactor,nLevels,fIniThFAST,fMinThFAST);

    if(sensor==System::MONOCULAR)
        mpIniORBextractor = new ORBextractor(2*nFeatures,fScaleFactor,nLevels,fIniThFAST,fMinThFAST);

    cout << endl  << "ORB Extractor Parameters: " << endl;
    cout << "- Number of Features: " << nFeatures << endl;
    cout << "- Scale Levels: " << nLevels << endl;
    cout << "- Scale Factor: " << fScaleFactor << endl;
    cout << "- Initial Fast Threshold: " << fIniThFAST << endl;
    cout << "- Minimum Fast Threshold: " << fMinThFAST << endl;

    if(sensor==System::STEREO || sensor==System::RGBD)
    {
        mThDepth = mbf*(float)fSettings["ThDepth"]/fx;
        cout << endl << "Depth Threshold (Close/Far Points): " << mThDepth << endl;
    }

    if(sensor==System::RGBD)
    {
        mDepthMapFactor = fSettings["DepthMapFactor"];
        if(mDepthMapFactor==0)
            mDepthMapFactor=1;
        else
            mDepthMapFactor = 1.0f/mDepthMapFactor;
    }

    int option = fSettings["Filter.Option"];
    pMap->setOption(option);


    // Load Plane filtering Parameters
    int w = fSettings["Camera.width"];
    int h = fSettings["Camera.height"];
    float minDepth = fSettings["PlaneFilteringParameters.minDepth"];
    float maxDepth = fSettings["PlaneFilteringParameters.maxDepth"];
    int maxPoints = fSettings["PlaneFilteringParameters.maxPoints"];
    int numSamples = fSettings["PlaneFilteringParameters.numSamples"];
    int numLocalSamples = fSettings["PlaneFilteringParameters.numLocalSamples"];
    int planeSize = fSettings["PlaneFilteringParameters.planeSize"];
    float WorldPlaneSize = fSettings["PlaneFilteringParameters.WorldPlaneSize"];
    float maxError = fSettings["PlaneFilteringParameters.maxError"];
    float minInlierFraction = fSettings["PlaneFilteringParameters.minInlierFraction"];
    float maxDepthDiff = fSettings["PlaneFilteringParameters.maxDepthDiff"];
    int numRetries = fSettings["PlaneFilteringParameters.numRetries"];


    Filtering = new PlaneFiltering(w, h, fx, fy, cx, cy, minDepth, maxDepth, maxPoints, numSamples,
                                   numLocalSamples, planeSize, WorldPlaneSize,maxError,minInlierFraction,
                                   maxDepthDiff, numRetries);
}

void Tracking::SetLocalMapper(LocalMapping *pLocalMapper)
{
    mpLocalMapper=pLocalMapper;
}

void Tracking::SetLoopClosing(LoopClosing *pLoopClosing)
{
    mpLoopClosing=pLoopClosing;
}

void Tracking::SetViewer(Viewer *pViewer)
{
    mpViewer=pViewer;
}


cv::Mat Tracking::GrabImageStereo(const cv::Mat &imRectLeft, const cv::Mat &imRectRight, const double &timestamp)
{
    mImGray = imRectLeft;
    cv::Mat imGrayRight = imRectRight;

    if(mImGray.channels()==3)
    {
        if(mbRGB)
        {
            cvtColor(mImGray,mImGray,CV_RGB2GRAY);
            cvtColor(imGrayRight,imGrayRight,CV_RGB2GRAY);
        }
        else
        {
            cvtColor(mImGray,mImGray,CV_BGR2GRAY);
            cvtColor(imGrayRight,imGrayRight,CV_BGR2GRAY);
        }
    }
    else if(mImGray.channels()==4)
    {
        if(mbRGB)
        {
            cvtColor(mImGray,mImGray,CV_RGBA2GRAY);
            cvtColor(imGrayRight,imGrayRight,CV_RGBA2GRAY);
        }
        else
        {
            cvtColor(mImGray,mImGray,CV_BGRA2GRAY);
            cvtColor(imGrayRight,imGrayRight,CV_BGRA2GRAY);
        }
    }

    mCurrentFrame = Frame(mImGray,imGrayRight,timestamp,mpORBextractorLeft,mpORBextractorRight,mpORBVocabulary,mK,mDistCoef,mbf,mThDepth);

    Track();

    return mCurrentFrame.mTcw.clone();
}


cv::Mat Tracking::GrabImageRGBD(const cv::Mat &imRGB,const cv::Mat &imD, const double &timestamp)
{
    mImGray = imRGB;
    cv::Mat imDepth = imD;

    if(mImGray.channels()==3)
    {
        if(mbRGB)
            cvtColor(mImGray,mImGray,CV_RGB2GRAY);
        else
            cvtColor(mImGray,mImGray,CV_BGR2GRAY);
    }
    else if(mImGray.channels()==4)
    {
        if(mbRGB)
            cvtColor(mImGray,mImGray,CV_RGBA2GRAY);
        else
            cvtColor(mImGray,mImGray,CV_BGRA2GRAY);
    }

    if(mDepthMapFactor!=1 || imDepth.type()!=CV_32F);
    imDepth.convertTo(imDepth,CV_32F,mDepthMapFactor);

    mCurrentFrame = Frame(mImGray,imDepth,timestamp,mpORBextractorLeft,mpORBVocabulary,mK,mDistCoef,mbf,mThDepth);

    Track();

    return mCurrentFrame.mTcw.clone();
}


cv::Mat Tracking::GrabImageMonocular(const cv::Mat &im, const double &timestamp)
{
    mImGray = im;

    if(mImGray.channels()==3)
    {
        if(mbRGB)
            cvtColor(mImGray,mImGray,CV_RGB2GRAY);
        else
            cvtColor(mImGray,mImGray,CV_BGR2GRAY);
    }
    else if(mImGray.channels()==4)
    {
        if(mbRGB)
            cvtColor(mImGray,mImGray,CV_RGBA2GRAY);
        else
            cvtColor(mImGray,mImGray,CV_BGRA2GRAY);
    }

    if(mState==NOT_INITIALIZED || mState==NO_IMAGES_YET)
        mCurrentFrame = Frame(mImGray,timestamp,mpIniORBextractor,mpORBVocabulary,mK,mDistCoef,mbf,mThDepth);
    else
        mCurrentFrame = Frame(mImGray,timestamp,mpORBextractorLeft,mpORBVocabulary,mK,mDistCoef,mbf,mThDepth);

    Track();

    return mCurrentFrame.mTcw.clone();
}

void Tracking::Track()
{
    if(mState==NO_IMAGES_YET) //si no hay imagenes
    {
        mState = NOT_INITIALIZED; //asiganmos al estado como no esta inicializado
    }

    mLastProcessedState=mState; //actualizamos el estado del ultimo proceso

    // Get Map Mutex -> Map cannot be changed
    unique_lock<mutex> lock(mpMap->mMutexMapUpdate);

    if(mState==NOT_INITIALIZED) //caso no este inicializado
    {
        if(mSensor==System::STEREO || mSensor==System::RGBD)
            StereoInitialization(); //inicializacion sterero y rgb-d
        else
            MonocularInitialization(); //inicializacion monocular

        mpFrameDrawer->Update(this); //actualizamos el framedrawer con informacion del tracking

        if(mState!=OK) // si el estado resulta mal salimos
            return;
    }
    else
    {
        // System is initialized. Track Frame.
        bool bOK; //VARIABLE OK

        // Initial camera pose estimation using motion model or relocalization (if tracking is lost)
        if(!mbOnlyTracking) //si la variable ONLY-TRACKING esta en FALSO
        {
            // Local Mapping is activated. This is the normal behaviour, unless
            // you explicitly activate the "only tracking" mode.

            if(mState==OK) //si el estado del sistema esta perfectamente
            {
                // Local Mapping might have changed some MapPoints tracked in last frame
                CheckReplacedInLastFrame(); //puede tener algunos puntos tracked en el anterior frame

                //modelo de movimiento o el id del currentFrame menor el id del ultimo relocFrame + 2
                //para los 2  frames luego de una relocalizacion
                // y para el primer tracking
                if(mVelocity.empty() || mCurrentFrame.mnId<mnLastRelocFrameId+2)
                {
                    bOK = TrackReferenceKeyFrame(); //realiza un tracking entre el frameKeyReferencia y el currentFrame
                                                    //coloca la misma posicion de la camara por que asume que no se movio nada
                                                    //actualiza los matches entre el currentFrame y el referenceKeyFrame
                }
                else//si no hubo relocalizacion o la velocidad es vacia para el resto de tracking
                {
                    bOK = TrackWithMotionModel(); //tracking entre el currentFrame y el ultimoFrame procesado
                    if(!bOK){ //si el estado es FALSO
                        bOK = TrackReferenceKeyFrame(); //tracking entre currentFrame y referenceFrame
                    }
                }
            }
            else //caso el estado no sea OK es decir que la camara se perdio
            {
                bOK = Relocalization(); //lanzamos la tarea de relocalizacion
            }
        }
        else //si la variable ONLY-TRACKING esta en TRUE
        {
            // Only Tracking: Local Mapping is deactivated

            if(mState==LOST) //si el systema se perdio
            {
                bOK = Relocalization(); //entramos en la tarea relocalizacion
            }
            else //estado no esta perdido
            {
                if(!mbVO) //si hay mas de 10 matches en el tracking model match
                {
                    // In last frame we tracked enough MapPoints in the map

                    if(!mVelocity.empty())
                    {
                        bOK = TrackWithMotionModel(); //VO
                    }
                    else
                    {
                        bOK = TrackReferenceKeyFrame(); //RKF
                    }
                }
                else
                {
                    // In last frame we tracked mainly "visual odometry" points.

                    // We compute two camera poses, one from motion model and one doing relocalization.
                    // If relocalization is sucessfull we choose that solution, otherwise we retain
                    // the "visual odometry" solution.

                    bool bOKMM = false;
                    bool bOKReloc = false;
                    vector<MapPoint*> vpMPsMM;
                    vector<bool> vbOutMM;
                    cv::Mat TcwMM;
                    if(!mVelocity.empty())
                    {
                        bOKMM = TrackWithMotionModel(); //VO
                        vpMPsMM = mCurrentFrame.mvpMapPoints; //puntos del current frame
                        vbOutMM = mCurrentFrame.mvbOutlier; //outliers del current frame
                        TcwMM = mCurrentFrame.mTcw.clone(); //pose de la camara
                    }
                    bOKReloc = Relocalization(); //intentamos relocalizar

                    if(bOKMM && !bOKReloc) //si el VO dio cierto y la relocalizacion fallo
                    {
                        mCurrentFrame.SetPose(TcwMM); //asignamos la pose encontrada
                        mCurrentFrame.mvpMapPoints = vpMPsMM; //asignamos los puntos encontramos
                        mCurrentFrame.mvbOutlier = vbOutMM; //asiganmos los outliers encontrados

                        if(mbVO) //si hay menor que 10 matches en el tracking model
                        {
                            for(int i =0; i<mCurrentFrame.N; i++) //para cada punto en el currentFrame
                            {
                                //si existe un punto y no es outlier
                                if(mCurrentFrame.mvpMapPoints[i] && !mCurrentFrame.mvbOutlier[i])
                                {
                                    mCurrentFrame.mvpMapPoints[i]->IncreaseFound(); //incrementamos que lo encontramos
                                }
                            }
                        }
                    }
                    else if(bOKReloc) //si relocalizacion dio cierto
                    {
                        mbVO = false; //desactivamos VO significa que hay mas que 10 matches
                    }

                    bOK = bOKReloc || bOKMM; //uno de los dos dio cierto
                }
            }
        }

        mCurrentFrame.mpReferenceKF = mpReferenceKF;  //asiganmos el referenceKeyFrame del currentFrame

        // If we have an initial estimation of the camera pose and matching. Track the local map.
        if(!mbOnlyTracking) //si el solo tracking esta desactivado
        {
            if(bOK) //si se realizo correctamente el tracking
                bOK = TrackLocalMap(); //realizamos el track local map
        }
        else //si el solo tracking esta activado
        {
            // mbVO true means that there are few matches to MapPoints in the map. We cannot retrieve
            // a local map and therefore we do not perform TrackLocalMap(). Once the system relocalizes
            // the camera we will use the local map again.
            if(bOK && !mbVO) //si la localizacion dio cierta y el ha mas de 10 matches
                bOK = TrackLocalMap(); //realizamos el track local map
        }

        if(bOK) //si todo dio certo asiganmos nuestro estado
            mState = OK;
        else //sino aun estamos perdidos o entramos a este estado
            mState=LOST;

        // Update drawer
        //actualizamos nuestro dibujadoe de puntos
        mpFrameDrawer->Update(this);

        // If tracking were good, check if we insert a keyframe
        //si el estado va bien, verificamos si necesitamos un nuevo keyframe
        if(bOK)
        {
            // Update motion model
            if(!mLastFrame.mTcw.empty()) // si tenemos la pose del ultimo frame procesado
            {
                cv::Mat LastTwc = cv::Mat::eye(4,4,CV_32F);//identidad
                mLastFrame.GetRotationInverse().copyTo(LastTwc.rowRange(0,3).colRange(0,3));//copiamos la rotacion
                mLastFrame.GetCameraCenter().copyTo(LastTwc.rowRange(0,3).col(3));//copiamos la traslacion
                mVelocity = mCurrentFrame.mTcw*LastTwc;
                //actualizamos la velocidad (CurrentTransforacion * AnteriorTransformacion)
                //saca la transformacion inversa del ultimo frame (del ultimo frame al mundo)
                //actualiza la velocidad con una transformacion que es del ultimoframe al currentframe
            }
            else//si no hay una pose no hay velocidad
                mVelocity = cv::Mat();

            mpMapDrawer->SetCurrentCameraPose(mCurrentFrame.mTcw); //asignar la pose al graficador de puntos

            // Clean temporal point matches
            for(int i=0; i<mCurrentFrame.N; i++) //para cada punto en el currentFrame
            {
                MapPoint* pMP = mCurrentFrame.mvpMapPoints[i]; //recuperamos el punto
                if(pMP) //si existe el punto
                    if(pMP->Observations()<1) //si tiene 0 observaciones limpiamos
                    {
                        mCurrentFrame.mvbOutlier[i] = false;
                        mCurrentFrame.mvpMapPoints[i]=static_cast<MapPoint*>(NULL);
                    }
            }

            // Delete temporal MapPoints
            //para cada punto temporal
            for(list<MapPoint*>::iterator lit = mlpTemporalPoints.begin(), lend =  mlpTemporalPoints.end(); lit!=lend; lit++)
            {
                MapPoint* pMP = *lit; //recuperamos el punto y eliminamos
                delete pMP;
            }
            mlpTemporalPoints.clear();

            // Check if we need to insert a new keyframe
            //verificamos si necesitamos un nuevo keyframe
            if(NeedNewKeyFrame())
                CreateNewKeyFrame();//creamos nuevo keyframe

            // We allow points with high innovation (considererd outliers by the Huber Function)
            // pass to the new keyframe, so that bundle adjustment will finally decide
            // if they are outliers or not. We don't want next frame to estimate its position
            // with those points so we discard them in the frame.
            for(int i=0; i<mCurrentFrame.N;i++) //verificamos cada punto
            {
                //si existe un punto y no es outlier
                if(mCurrentFrame.mvpMapPoints[i] && mCurrentFrame.mvbOutlier[i])
                    mCurrentFrame.mvpMapPoints[i]=static_cast<MapPoint*>(NULL);
            }
        }

        // Reset if the camera get lost soon after initialization
        if(mState==LOST) //si el estado esta perdido
        {
            if(mpMap->KeyFramesInMap()<=5)//si el numero de keyframes es menor a 5 reseteamos el sistema dificil localizarse
            {
                cout << "Track lost soon after initialisation, reseting..." << endl;
                mpSystem->Reset();
                return;
            }
        }

        if(!mCurrentFrame.mpReferenceKF)//si el currentFrame no tiene referencia
            mCurrentFrame.mpReferenceKF = mpReferenceKF;//asignamos la referencia actual

        mLastFrame = Frame(mCurrentFrame); //creamos como ultimo frame esl current frame
    }



    // Store frame pose information to retrieve the complete camera trajectory afterwards.
    //si tenemos la posicion del currentFrame
    if(!mCurrentFrame.mTcw.empty())
    {
        cv::Mat Tcr = mCurrentFrame.mTcw*mCurrentFrame.mpReferenceKF->GetPoseInverse();//recuperamos transformacion del current al reference, las transformaciones se leen de ultimo para el primero
        mlRelativeFramePoses.push_back(Tcr); //agregamos en las poses relativas
        mlpReferences.push_back(mpReferenceKF);//agregamos el frame de referencia
        mlFrameTimes.push_back(mCurrentFrame.mTimeStamp);//tiempo que fue tomado
        mlbLost.push_back(mState==LOST);//si el estado es perdido
    }
    else //si no tenemos la posicion del currentFrame
    {
        // This can happen if tracking is lost
        mlRelativeFramePoses.push_back(mlRelativeFramePoses.back());//agregamos el que esta en el fondo de la lista
        mlpReferences.push_back(mlpReferences.back());//la referencia del fondo de la fila
        mlFrameTimes.push_back(mlFrameTimes.back());//el tiempo del fondo de la fila
        mlbLost.push_back(mState==LOST);//el estado esta perdido
    }

}


void Tracking::StereoInitialization()
{
    if(mCurrentFrame.N>500) //si currentFrame tiene mas de 500 puntos
    {
        // Set Frame pose to the origin
        mCurrentFrame.SetPose(cv::Mat::eye(4,4,CV_32F)); //asiganmos la pose de currentFrame como identidad

        // Create KeyFrame
        //el initKeyFrame es creado a partir del currentFrame, el puntero al mapa y el puntero a la base de datos de los keyframes
        KeyFrame* pKFini = new KeyFrame(mCurrentFrame,mpMap,mpKeyFrameDB);

        // Insert KeyFrame in the map
        mpMap->AddKeyFrame(pKFini); //agregamos el initKeyFrame al mapa

        // Create MapPoints and asscoiate to KeyFrame
        //para cada punto en el currentFrame , obtenemos su profundidad
        for(int i=0; i<mCurrentFrame.N;i++)
        {
            float z = mCurrentFrame.mvDepth[i];
            if(z>0) //si la profundidad es mayor que cero se considera
            {
                cv::Mat x3D = mCurrentFrame.UnprojectStereo(i); //calculamos el punto del currentFrame en el espacio
                MapPoint* pNewMP = new MapPoint(x3D,pKFini,mpMap); //iniciamos un punto con posicion espacial, el keyframe de referencia y el mapa al que pertenece
                pNewMP->AddObservation(pKFini,i);//agregamos init keyFrame y el pixel que genero el punto a sus observaciones
                pKFini->AddMapPoint(pNewMP,i);//agregamos el punto y el pixel que la genero a la lista de puntos del initKeyFrame
                pNewMP->ComputeDistinctiveDescriptors(); //creamos su descriptor a partir de las observaciones escojemos el mejor
                pNewMP->UpdateNormalAndDepth(); //actualizamos la normal y el rango de la profundidad
                mpMap->AddMapPoint(pNewMP); //adicionamos el punto al mapa

                mCurrentFrame.mvpMapPoints[i]=pNewMP; //adicionamos el punto al currentFrame en la posicion del pixel que lo genero
            }
        }

        cout << "New map created with " << mpMap->MapPointsInMap() << " points" << endl; //cantidad de puntos en el mapa

        mpLocalMapper->InsertKeyFrame(pKFini); //adicionamos a la cola de neyKeyFrames el initKeyFrame

        mLastFrame = Frame(mCurrentFrame); //actualizamos el ultimo frame procesado con el currentFrame
        mnLastKeyFrameId=mCurrentFrame.mnId; //actualizamos el id de ultimo keyframe procesado
        mpLastKeyFrame = pKFini; //actualizamos el ultimo keyframe procesado con el initKeyFrame

        mvpLocalKeyFrames.push_back(pKFini); //insertamos a la lista de local keyframes el initKeyFrame
        mvpLocalMapPoints=mpMap->GetAllMapPoints(); //asignamos a local map la actual lista de puntos en el mapa
        mpReferenceKF = pKFini; //actualizamos como reference keyframe con initKeyFrame
        mCurrentFrame.mpReferenceKF = pKFini;

        mpMap->SetReferenceMapPoints(mvpLocalMapPoints); //en el mapa asignamos como puntos de referencia a todos los puntos actuales del mapa

        mpMap->mvpKeyFrameOrigins.push_back(pKFini); //adicionamos a la lista de keyframes origenes el initKeyFrame

        mpMapDrawer->SetCurrentCameraPose(mCurrentFrame.mTcw); //informamos la posicion de la camara em el mundo al graficador del mapa

        mState=OK; //infomamos que esl sistema esta perfectamente
    }
}

void Tracking::MonocularInitialization()
{

    if(!mpInitializer)
    {
        //si el objecto Initializer es un elemento NULL
        //iniciamos el primer frame
        // Set Reference Frame
        if(mCurrentFrame.mvKeys.size()>100) //si tiene mas de 100 puntos
        {
            mInitialFrame = Frame(mCurrentFrame); //frame inicial asignado
            mLastFrame = Frame(mCurrentFrame); //ultimo frame asignado
            mvbPrevMatched.resize(mCurrentFrame.mvKeysUn.size());
            for(size_t i=0; i<mCurrentFrame.mvKeysUn.size(); i++)
                mvbPrevMatched[i]=mCurrentFrame.mvKeysUn[i].pt; //anteriores puntos

            if(mpInitializer)
                delete mpInitializer;

            mpInitializer =  new Initializer(mCurrentFrame,1.0,200); //configuramos nuestro objeto

            fill(mvIniMatches.begin(),mvIniMatches.end(),-1);//rellenamos todos los matcheds iniciales con -1

            return;
        }
    }
    else
    {
        // Intentamos inicializar
        // Try to initialize
        if((int)mCurrentFrame.mvKeys.size()<=100) //si tiene menor a 100 puntos
        {
            delete mpInitializer;
            mpInitializer = static_cast<Initializer*>(NULL); //descartamos el current frame para tomar el proximo
            fill(mvIniMatches.begin(),mvIniMatches.end(),-1); //rellenamos todos los matches iniciales con -1
            return;
        }

        // Find correspondences

        ORBmatcher matcher(0.9,true); //hallamos las correspondencias entre initialframe y currentframe, tambien el numero de correspondencias
        int nmatches = matcher.SearchForInitialization(mInitialFrame,mCurrentFrame,mvbPrevMatched,mvIniMatches,100);

        // Check if there are enough correspondences
        if(nmatches<100) //si hubieron menos de 100 matches , descartamos el current frame para tomar el proximo
        {
            delete mpInitializer;
            mpInitializer = static_cast<Initializer*>(NULL);
            return;
        }

        cv::Mat Rcw; // Current Camera Rotation
        cv::Mat tcw; // Current Camera Translation
        vector<bool> vbTriangulated; // Triangulated Correspondences (mvIniMatches)


        //geometria epipolar entre el initFrame y el currentFrame para hallar
        //se necesita de los keypoint1, keypoint2 y el matches entre 1(indices) y 2(valor)
        //la rotacion, traslacion, puntos en el espacio 3D, cuales puntos estan bien triangulados
        //Transformacion del currentFrame (CURRENT K[R|t]) al initFrame(REFERENCIA K[I|0])
        if(mpInitializer->Initialize(mCurrentFrame, mvIniMatches, Rcw, tcw, mvIniP3D, vbTriangulated))
        {
            for(size_t i=0, iend=mvIniMatches.size(); i<iend;i++)
            {
                if(mvIniMatches[i]>=0 && !vbTriangulated[i]) //si tiene un match y la triangulizacion fue correcta
                {
                    mvIniMatches[i]=-1; //borramos el match y disminuimos la cantidad que resta
                    nmatches--;
                }
            }

            // Set Frame Poses
            mInitialFrame.SetPose(cv::Mat::eye(4,4,CV_32F)); //initFrame(REFERENCE K[I|0])
            cv::Mat Tcw = cv::Mat::eye(4,4,CV_32F); //montamos la matriz de transformacion
            Rcw.copyTo(Tcw.rowRange(0,3).colRange(0,3)); //copiamos la matriz de rotacion al de transformacion
            tcw.copyTo(Tcw.rowRange(0,3).col(3));//copiamos la matriz de traslacion al de tranformacion
            mCurrentFrame.SetPose(Tcw); //currentFrame(CURRENT K[R|t]) (Tcw transformacion camera to world)

            CreateInitialMapMonocular(); //creamos el mapa inicial
        }
    }
}

void Tracking::CreateInitialMapMonocular()
{
    // Create KeyFrames
    //creamos los keyframes mandando enlace del mapa, de la base de datos y informacion del frame que incluye la pose
    KeyFrame* pKFini = new KeyFrame(mInitialFrame,mpMap,mpKeyFrameDB); //creamos el keyframe para el initFrame
    KeyFrame* pKFcur = new KeyFrame(mCurrentFrame,mpMap,mpKeyFrameDB); //creamos el keyframe para el CurrentFrame

    pKFini->ComputeBoW(); //generamos el descriptor de palabras para el initFrame
    pKFcur->ComputeBoW(); //generamos el descriptor de palabras para el currentFrame

    // Insert KFs in the map
    mpMap->AddKeyFrame(pKFini); //adicionamos el initFrame al mapa
    mpMap->AddKeyFrame(pKFcur); //adicionamos el currentFrame al mapa

    // Create MapPoints and asscoiate to keyframes
    for(size_t i=0; i<mvIniMatches.size();i++)
    {
        if(mvIniMatches[i]<0) // -1 si no se tiene un match para el punto i del initFrame
            continue;

        //Create MapPoint.
        cv::Mat worldPos(mvIniP3D[i]); //posicion del punto en 3D con respecto a initFrame

        //creamos el punto para adicionar al mapa
        //con la posicion del punto, el keyframe de referencia del punto y el enlace al mapa donde pertenece
        MapPoint* pMP = new MapPoint(worldPos,pKFcur,mpMap);

        pKFini->AddMapPoint(pMP,i); //adicionamos el punto a las lista de puntos onbservados desde el initKeyFrame con enlace de su posicion del pixel en el initFrame que genero esa observacion
        pKFcur->AddMapPoint(pMP,mvIniMatches[i]); //adicionamos el punto a las lista de puntos onbservados desde el currentKeyFrame con enlace de su posicion del pixel en el currentFrame que genero esa observacion

        pMP->AddObservation(pKFini,i); // adicionamos al punto que ha sido observado desde el initKeyFrame y que pixel lo genero en el initFrame
        pMP->AddObservation(pKFcur,mvIniMatches[i]); //adicionamos al punto que ha sido observado desde el currentKeyFrame y que pixel lo genero en el currentFrame

        pMP->ComputeDistinctiveDescriptors(); //escoger el mejor descriptor a partir de todos las observaciones que fueron hechas y escojer el mejor
        pMP->UpdateNormalAndDepth(); //actualiza la normal y intervalo de la profundidad

        //Fill Current Frame structure
        mCurrentFrame.mvpMapPoints[mvIniMatches[i]] = pMP; //adicionamos el punto a la lista de puntos en la posicion del pixel que lo genero en el currentFrame
        mCurrentFrame.mvbOutlier[mvIniMatches[i]] = false; //adicionamos la informacion que no fue un outlier

        //Add to Map
        mpMap->AddMapPoint(pMP); //adicionamos el punto en el mapa
    }

    // Update Connections
    pKFini->UpdateConnections(); //actualiza el grafo de covisibilidad desde el nodo initKeyFrame
    pKFcur->UpdateConnections(); //actualiza el grafo de covisibilidad desde el nodo currentKeyFrame

    // Bundle Adjustment
    cout << "Init New Map created with " << mpMap->MapPointsInMap() << " points" << endl;

    Optimizer::GlobalBundleAdjustemnt(mpMap,20); //Optimizacion global del mapa

    // Set median depth to 1
    float medianDepth = pKFini->ComputeSceneMedianDepth(2); //mediana de las profundidades en el initFrame
    float invMedianDepth = 1.0f/medianDepth; //inversa de la profundidad media

    if(medianDepth<0 || pKFcur->TrackedMapPoints(1)<100) //si se observaron menos de 100 puntos en el current frame o la profundidad media es menor a cero
    {
        cout << "Wrong initialization, reseting..." << endl;
        Reset();
        return;
    }

    // Scale initial baseline
    cv::Mat Tc2w = pKFcur->GetPose(); //recuperar pose del currentFrame
    Tc2w.col(3).rowRange(0,3) = Tc2w.col(3).rowRange(0,3)*invMedianDepth; //multiplica el vector traslacion por la inversa de la profundidad media escalandolo
    pKFcur->SetPose(Tc2w); //asignamos esa posicion alterada al current frame

    // Scale points
    vector<MapPoint*> vpAllMapPoints = pKFini->GetMapPointMatches(); //recuperar los puntos del initFrame
    for(size_t iMP=0; iMP<vpAllMapPoints.size(); iMP++)
    {
        if(vpAllMapPoints[iMP])
        {
            MapPoint* pMP = vpAllMapPoints[iMP]; //cada punto
            pMP->SetWorldPos(pMP->GetWorldPos()*invMedianDepth); //escalamos su ubicacion en el mapa al multiplicar por la inversa de la profundidad media
        }
    }

    /********************************************************************************************************/
    /* INICIALIZAMOS LAS VARIABLES SEED DE LOS PRIMEROS PUNTOS INICIALES NUEVO PUNTO Y AGREGAMOS A LA LISTA */
    float minDepth, maxDepth, medDepth;
    pKFcur->ComputeSceneDepth(minDepth, maxDepth, medDepth);

    for(size_t i = 0; i < mvIniMatches.size(); i++){
        if(mvIniMatches[i] < 0) continue;
        MapPoint* pMP = pKFcur->GetMapPoint(mvIniMatches[i]);
        mpMap->initMapSeed(pMP, minDepth, maxDepth, medDepth, pKFcur);
    }
    /********************************************************************************************************/

    mpLocalMapper->InsertKeyFrame(pKFini); //en el manejador del mapa colocamos a la cola de newframes el initFrame
    mpLocalMapper->InsertKeyFrame(pKFcur); //en el manejador del mapa colocamos a la cola de newframes el currentFrame

    mCurrentFrame.SetPose(pKFcur->GetPose()); //asignamos la pose al currentFrame con la misma pose del currentKeyFrame
    mnLastKeyFrameId=mCurrentFrame.mnId; //asignamos el id del currentFrame como el ultimo frame procesado
    mpLastKeyFrame = pKFcur; //asignamos el currentFrame como el ultimo frame procesado

    mvpLocalKeyFrames.push_back(pKFcur);//incluimos a la lista de local keyFrames el currentKeyFrame
    mvpLocalKeyFrames.push_back(pKFini);//incluimos a la lista de local keyFrames el initKeyFrame
    mvpLocalMapPoints=mpMap->GetAllMapPoints(); //recuperamos todos los puntos actuales en el mapa
    mpReferenceKF = pKFcur; //asiganmos como referenceFrame al currenKeyFrame
    mCurrentFrame.mpReferenceKF = pKFcur; //asiganmos como el referenceFrame del currentFrame  el puntero del currentKeyFrame

    mLastFrame = Frame(mCurrentFrame); //ultimo frame utilizado asiganmos el currentFrame

    mpMap->SetReferenceMapPoints(mvpLocalMapPoints); //conjunto de puntos de referencia asignamos todos los puntos actuales en el mapa

    mpMapDrawer->SetCurrentCameraPose(pKFcur->GetPose()); //le asignamos al dibujador del mapa la posicion de currentKeyFrame

    mpMap->mvpKeyFrameOrigins.push_back(pKFini); //adicionamos a la lista de origenes el initKeyFrame

    mState=OK; //informamos que el estado del sistema esta perfecto
}

void Tracking::CheckReplacedInLastFrame()
{
    for(int i =0; i<mLastFrame.N; i++) //para todos los puntos en el ultimo frame procesado
    {
        MapPoint* pMP = mLastFrame.mvpMapPoints[i];

        if(pMP)//si existe el punto tiene informacion
        {
            MapPoint* pRep = pMP->GetReplaced();
            if(pRep)//si el punto ha sido reemplazado
            {
                mLastFrame.mvpMapPoints[i] = pRep; //guardamos esa nueva informacionen el ultimo frame procesado
            }
        }
    }
}

bool Tracking::TrackReferenceKeyFrame()
{


    //busca los puntos en comun entre el referenceKeyFrame y el currentFrame
    //con los puntos encontrados y la ultima posicion del ultimo frame estima la nueva posicion
    //al estimar la nueva posicion muchos puntos se convierten en outliers, toca limpiarlos
    //se cuenta los puntos que no son outliers y que tienen al menos una observacion
    //la cantidad de puntos que pasen el filtro definen si el tracking estuvo bien o no

    // Compute Bag of Words vector
    mCurrentFrame.ComputeBoW(); //computa la palabra

    // We perform first an ORB matching with the reference keyframe
    // If enough matches are found we setup a PnP solver
    ORBmatcher matcher(0.7,true);
    vector<MapPoint*> vpMapPointMatches;//matches entre el referenceKeyFrame y el currentFrame

    int nmatches = matcher.SearchByBoW(mpReferenceKF,mCurrentFrame,vpMapPointMatches);//computamos matches

    if(nmatches<15) //si tiene nes de 15 matches el estado sera FALSO
        return false;

    mCurrentFrame.mvpMapPoints = vpMapPointMatches; //asignamos los matches encontrados al currentFrame
    mCurrentFrame.SetPose(mLastFrame.mTcw); //asignamos a la pose del currentFrame la pose del ultimo frame procesado

    Optimizer::PoseOptimization(&mCurrentFrame); //en base a lo que vemos y la pose anterior estimamos la nueva pose

    // Discard outliers
    int nmatchesMap = 0; //numero de matches en el mapa
    for(int i =0; i<mCurrentFrame.N; i++) //para todos los puntos en el currentFrame
    {
        if(mCurrentFrame.mvpMapPoints[i]) //si existe un punto
        {
            if(mCurrentFrame.mvbOutlier[i]) //si es outlier
            {
                MapPoint* pMP = mCurrentFrame.mvpMapPoints[i]; //recuperamos el punto del currentFrame

                mCurrentFrame.mvpMapPoints[i]=static_cast<MapPoint*>(NULL); //eliminamos de la lista del currentFrame
                mCurrentFrame.mvbOutlier[i]=false; //decimos que no es un outlier
                pMP->mbTrackInView = false;  //no se usa para el tracking
                pMP->mnLastFrameSeen = mCurrentFrame.mnId;//se actualiza id del ultimo frame donde se vio el punto
                nmatches--;
            }
            else if(mCurrentFrame.mvpMapPoints[i]->Observations()>0)//si no es outlier y el numero de observaciones es mayor que 0 se cuenta
                nmatchesMap++;
        }
    }

    //retornamos si hay mas de 10 puntos que se incluyen en el mapa
    return nmatchesMap>=10;
}

void Tracking::UpdateLastFrame()
{
    // Update pose according to reference keyframe
    KeyFrame* pRef = mLastFrame.mpReferenceKF; //recuperamos el ReferenceKeyFrame que fue usado con el ultimo frame procesado
    cv::Mat Tlr = mlRelativeFramePoses.back(); //una relativa posicion del fondo de la lista

    mLastFrame.SetPose(Tlr*pRef->GetPose());  //actualizamos la posicion del ultimo frame procesado con su verdadera posicion en el mundo, las trasnformaciones se leen del ultimo para el primero

    //si el id del ultimo keyFrame procesado es igual al ultimo frame procesado
    // o el sensor es monocular retornamos
    if(mnLastKeyFrameId==mLastFrame.mnId || mSensor==System::MONOCULAR)
        return;

    // Create "visual odometry" MapPoints
    // We sort points according to their measured depth by the stereo/RGB-D sensor
    vector<pair<float,int> > vDepthIdx; //lista de profundidades y indices del tamano de los puntos
    vDepthIdx.reserve(mLastFrame.N);
    for(int i=0; i<mLastFrame.N;i++) //para cada punto en el ultimo frame procesado
    {
        float z = mLastFrame.mvDepth[i]; //recupero la profundidad
        if(z>0) //verifico que sea positiva
        {
            vDepthIdx.push_back(make_pair(z,i)); //adiciono a mi lista la profundidad y el indice del punto que lo genera
        }
    }

    if(vDepthIdx.empty()) //si no tenemos profundidades validas retornamos
        return;

    sort(vDepthIdx.begin(),vDepthIdx.end()); //ordenamos las profundidades

    // We insert all close points (depth<mThDepth)
    // If less than 100 close points, we insert the 100 closest ones.
    int nPoints = 0; //contador de puntos
    for(size_t j=0; j<vDepthIdx.size();j++) //para todas las profundidades validas
    {
        int i = vDepthIdx[j].second; //recuperamos el indice del pixel que lo genera

        bool bCreateNew = false; //crear nuevo?

        MapPoint* pMP = mLastFrame.mvpMapPoints[i]; //recuperamos el punto del ultimo frame procesado
        if(!pMP) //si no tiene aun una estructura
            bCreateNew = true; //crea uno nuevo TRUE
        else if(pMP->Observations()<1) //si tiene una estructura verificamos las observaciones, si tiene negativas
        {
            bCreateNew = true; //crear uno nuevo
        }

        if(bCreateNew) //si decidimos crear un nuevo
        {
            cv::Mat x3D = mLastFrame.UnprojectStereo(i);  //creamos el punto 3d para el pixel con indice i
            MapPoint* pNewMP = new MapPoint(x3D,mpMap,&mLastFrame,i); //creamos la estructura

            mLastFrame.mvpMapPoints[i]=pNewMP; //agregamos el punto en el ultimo frame procesado

            mlpTemporalPoints.push_back(pNewMP); //adicionamos a la lista de puntos temporales
            nPoints++; //contamos el punto
        }
        else //caso no se necesite crear una estructura nueva solo contamos
        {
            nPoints++;
        }

        if(vDepthIdx[j].first>mThDepth && nPoints>100) //si se revisaron 100 puntos y la profundidad es mayor al limite salimos
            break;
    }
}

bool Tracking::TrackWithMotionModel()
{
    ORBmatcher matcher(0.9,true);

    // Update last frame pose according to its reference keyframe
    // Create "visual odometry" points

    //actualiza la posicion del ultimo frame procesado, inserta su posicion en el mundo
    UpdateLastFrame(); //solo para rgbd y stereo

    mCurrentFrame.SetPose(mVelocity*mLastFrame.mTcw);
    //modificamos la pose del currentFrame multiplicando por la velocidad
    //el ultimo frame tiene la transformacion del mundo al ultimo frame
    //la velocidad tiene la transformacion del ultimo frame al current frame
    //la multiplicacion sera la posicion del current frame en el mundo

    fill(mCurrentFrame.mvpMapPoints.begin(),mCurrentFrame.mvpMapPoints.end(),static_cast<MapPoint*>(NULL));//rellenamos el mapa de puntos del currentFrame con nulls

    // Project points seen in previous frame
    int th; //limite
    if(mSensor!=System::STEREO)
        th=15;
    else
        th=7;

    //buscamos los matches a traves de proyecciones entre el currentFrame y el LastFrame para un limite
    //rellena el mapa de puntos del current frame con los puntos encontrados en el ultimo frame
    int nmatches = matcher.SearchByProjection(mCurrentFrame,mLastFrame,th,mSensor==System::MONOCULAR);

    // If few matches, uses a wider window search
    //si se tiene menos de 20 matches
    if(nmatches<20)
    {
        //limpiamos el mapa de puntos del currentFrame con null y buscamos los matches para un limite doble
        fill(mCurrentFrame.mvpMapPoints.begin(),mCurrentFrame.mvpMapPoints.end(),static_cast<MapPoint*>(NULL));
        nmatches = matcher.SearchByProjection(mCurrentFrame,mLastFrame,2*th,mSensor==System::MONOCULAR);
    }

    if(nmatches<20) //si los matches aun no son muchos desistimos
        return false;

    // Optimize frame pose with all matches
    Optimizer::PoseOptimization(&mCurrentFrame);
    //con los puntos observados y la posicion estimada por el movimiento
    //buscamos optimizar la posicion y hallar la correcta

    // Discard outliers
    int nmatchesMap = 0; //cuantos puntos estan buenos para el mapa
    for(int i =0; i<mCurrentFrame.N; i++) //para cada punto del currentFrame
    {
        if(mCurrentFrame.mvpMapPoints[i]) //verificamos si tenemos un punto
        {
            if(mCurrentFrame.mvbOutlier[i]) //si es outlier
            {
                MapPoint* pMP = mCurrentFrame.mvpMapPoints[i]; //recuperamos el punto generado por el pixel en la posicion i

                mCurrentFrame.mvpMapPoints[i]=static_cast<MapPoint*>(NULL); //limpiamos en el mapa de puntos del currentFrame
                mCurrentFrame.mvbOutlier[i]=false; //asignamos que no es outlier
                pMP->mbTrackInView = false; //el punto no se utilizara en el tracking
                pMP->mnLastFrameSeen = mCurrentFrame.mnId; //asignamos el id del ultimo frame donde fue visto
                nmatches--;
            }
            else if(mCurrentFrame.mvpMapPoints[i]->Observations()>0) //caso es inlier verificamos si fue observado una vez o mas
                nmatchesMap++;
        }
    }    

    if(mbOnlyTracking) //si es solo tracking
    {
        mbVO = nmatchesMap<10; //VO es TRUE cuando la cantidad de puntos que se irian para el mapa es menor que 10
        return nmatches>20; //tiene mas de 20 matches
    }

    return nmatchesMap>=10; //tiene mas de 10 puntos que se van a ir al mapa
}

bool Tracking::TrackLocalMap()
{
    // We have an estimation of the camera pose and some map points tracked in the frame.
    // We retrieve the local map and try to find matches to points in the local map.

    UpdateLocalMap(); //actualizamos el mapa local

    SearchLocalPoints(); //se busca en los puntos locales puntos que son vistos desde la camara

    // Optimize Pose
    Optimizer::PoseOptimization(&mCurrentFrame); //optimizamos la posicion de la camara con los puntos borrados
    mnMatchesInliers = 0; //cuales de los matches son correctos

    // Update MapPoints Statistics
    for(int i=0; i<mCurrentFrame.N; i++) //para cada punto en el currentFrame
    {
        if(mCurrentFrame.mvpMapPoints[i]) //si se tiene un punto?
        {
            if(!mCurrentFrame.mvbOutlier[i]) //el no punto es outlier?
            {
                mCurrentFrame.mvpMapPoints[i]->IncreaseFound(); //incrementamos que ha sido encontrado
                if(!mbOnlyTracking) //no es solo tracking?
                {
                    if(mCurrentFrame.mvpMapPoints[i]->Observations()>0) //el punto tiene mas de una observacion?
                        mnMatchesInliers++;
                }
                else
                    mnMatchesInliers++;
            }
            else if(mSensor==System::STEREO)
                mCurrentFrame.mvpMapPoints[i] = static_cast<MapPoint*>(NULL);

        }
    }

    // Decide if the tracking was succesful
    // More restrictive if there was a relocalization recently
    //si tuvo una relozalizacion recientemente
    if(mCurrentFrame.mnId<mnLastRelocFrameId+mMaxFrames && mnMatchesInliers<50)
        return false;

    //si la cantidad de matches es menor a 30 no es bueno para el tracking caso contrario es bueno
    if(mnMatchesInliers<30)
        return false;
    else
        return true;
}


bool Tracking::NeedNewKeyFrame()
{
    if(mbOnlyTracking) //si es solo tracking no se necesita nuevo keyframe
        return false;

    // If Local Mapping is freezed by a Loop Closure do not insert keyframes
    //no se necesita keyframe si el processo de mapping fue parado por causa de un loop
    if(mpLocalMapper->isStopped() || mpLocalMapper->stopRequested())
        return false;

    const int nKFs = mpMap->KeyFramesInMap(); //numero de keyframes en el mapa

    // Do not insert keyframes if not enough frames have passed from last relocalisation
    //no se necesita keyframe si no paso ningun frame despues de la ultima localizacion
    if(mCurrentFrame.mnId<mnLastRelocFrameId+mMaxFrames && nKFs>mMaxFrames)
        return false;

    // Tracked MapPoints in the reference keyframe
    int nMinObs = 3;
    if(nKFs<=2)
        nMinObs=2;
    int nRefMatches = mpReferenceKF->TrackedMapPoints(nMinObs); //cantidad de puntos con un minimo de observaciones

    // Local Mapping accept keyframes?
    //esta aceptando nuevos keyframes el local mapping?
    bool bLocalMappingIdle = mpLocalMapper->AcceptKeyFrames();

    // Stereo & RGB-D: Ratio of close "matches to map"/"total matches"
    // "total matches = matches to map + visual odometry matches"
    // Visual odometry matches will become MapPoints if we insert a keyframe.
    // This ratio measures how many MapPoints we could create if we insert a keyframe.
    int nMap = 0; //numero de puntos con observaciones para incluir en el mapa
    int nTotal= 0; //numero de puntos validos
    if(mSensor!=System::MONOCULAR) //si es monocular
    {
        for(int i =0; i<mCurrentFrame.N; i++) //para cada punto el el currentFrame
        {
            if(mCurrentFrame.mvDepth[i]>0 && mCurrentFrame.mvDepth[i]<mThDepth) //si la profundidad esta en el intervalo
            {
                nTotal++;
                if(mCurrentFrame.mvpMapPoints[i]) //si existe un punto
                    if(mCurrentFrame.mvpMapPoints[i]->Observations()>0) //si el punto tiene por lo menos una observacion
                        nMap++;
            }
        }
    }
    else
    {
        // There are no visual odometry matches in the monocular case
        nMap=1;
        nTotal=1;
    }

    const float ratioMap = (float)nMap/fmax(1.0f,nTotal); //puntos vistos en el mapa entre los puntos del currentFrame

    // Thresholds
    float thRefRatio = 0.75f; //75% de los puntos
    if(nKFs<2)
        thRefRatio = 0.4f; //40% de los puntos

    if(mSensor==System::MONOCULAR)
        thRefRatio = 0.9f; //90% de los puntos

    float thMapRatio = 0.35f; //35% de los puntos
    if(mnMatchesInliers>300) //numero de matches inliers son mas de 300
        thMapRatio = 0.20f; //20% de los puntos

    // Condition 1a: More than "MaxFrames" have passed from last keyframe insertion
    //cuantos frames tiene que pasar como maximo para crear un nuevo keyframe
    const bool c1a = mCurrentFrame.mnId>=mnLastKeyFrameId+mMaxFrames;
    // Condition 1b: More than "MinFrames" have passed and Local Mapping is idle
    //cuantos frame que tiene que pasar como minimo despues del ultimo keyframe y el mapping debe aceptar keyframes
    const bool c1b = (mCurrentFrame.mnId>=mnLastKeyFrameId+mMinFrames && bLocalMappingIdle);
    //Condition 1c: tracking is weak
    const bool c1c =  mSensor!=System::MONOCULAR && (mnMatchesInliers<nRefMatches*0.25 || ratioMap<0.3f) ;
    // Condition 2: Few tracked points compared to reference keyframe. Lots of visual odometry compared to map matches.
    const bool c2 = ((mnMatchesInliers<nRefMatches*thRefRatio|| ratioMap<thMapRatio) && mnMatchesInliers>15);

    if((c1a||c1b||c1c)&&c2)
    {
        // If the mapping accepts keyframes, insert keyframe.
        // Otherwise send a signal to interrupt BA
        if(bLocalMappingIdle)
        {
            return true;
        }
        else
        {
            mpLocalMapper->InterruptBA();
            if(mSensor!=System::MONOCULAR)
            {
                if(mpLocalMapper->KeyframesInQueue()<3)
                    return true;
                else
                    return false;
            }
            else
                return false;
        }
    }
    else
        return false;
}

void Tracking::CreateNewKeyFrame()
{
    if(!mpLocalMapper->SetNotStop(true)) //informar al local mapping que no pare
        return;

    //creamos el keyframe con informacion del currentFrame , enlace al mapa y enlace a la base de datos de los keyframes
    KeyFrame* pKF = new KeyFrame(mCurrentFrame,mpMap,mpKeyFrameDB);

    mpReferenceKF = pKF; //sera el nuevo keyframe de referencia
    mCurrentFrame.mpReferenceKF = pKF; //sera el frame de referencia del currentFrame

    if(mSensor!=System::MONOCULAR) // si el sistema no es monocular
    {
        mCurrentFrame.UpdatePoseMatrices(); //modificamos la posicion de las matrices

        // We sort points by the measured depth by the stereo/RGBD sensor.
        // We create all those MapPoints whose depth < mThDepth.
        // If there are less than 100 close points we create the 100 closest.
        vector<pair<float,int> > vDepthIdx;
        vDepthIdx.reserve(mCurrentFrame.N);

        //para cada punto verificamos si la profundidad es positiva
        for(int i=0; i<mCurrentFrame.N; i++)
        {
            float z = mCurrentFrame.mvDepth[i];
            if(z>0)
            {
                vDepthIdx.push_back(make_pair(z,i));
            }
        }

        if(!vDepthIdx.empty())
        {
            sort(vDepthIdx.begin(),vDepthIdx.end()); //ordenamos por la profundidad

            int nPoints = 0;
            for(size_t j=0; j<vDepthIdx.size();j++) //para cada profundidad aceptada
            {
                int i = vDepthIdx[j].second;

                bool bCreateNew = false; //no necesitamos crear

                MapPoint* pMP = mCurrentFrame.mvpMapPoints[i]; //recuperamos el punto
                if(!pMP) //si no se tiene una estructura punto necesitamos crear
                    bCreateNew = true;
                else if(pMP->Observations()<1) //si no tiene observaciones necesitamos crear
                {
                    bCreateNew = true;
                    mCurrentFrame.mvpMapPoints[i] = static_cast<MapPoint*>(NULL);
                }

                if(bCreateNew) //creamos el punto y adicionamos al mapa y al keyframe
                {
                    cv::Mat x3D = mCurrentFrame.UnprojectStereo(i);
                    MapPoint* pNewMP = new MapPoint(x3D,pKF,mpMap);
                    pNewMP->AddObservation(pKF,i);
                    pKF->AddMapPoint(pNewMP,i);
                    pNewMP->ComputeDistinctiveDescriptors();
                    pNewMP->UpdateNormalAndDepth();
                    mpMap->AddMapPoint(pNewMP);

                    mCurrentFrame.mvpMapPoints[i]=pNewMP;
                    nPoints++;
                }
                else
                {
                    nPoints++;
                }

                if(vDepthIdx[j].first>mThDepth && nPoints>100) //minimo 100 puntos
                    break;
            }
        }
    }

    //insertamos el keyframe al local mapping
    mpLocalMapper->InsertKeyFrame(pKF);

    //liberamos el local mapping
    mpLocalMapper->SetNotStop(false);

    mnLastKeyFrameId = mCurrentFrame.mnId; //el ultimo id del keyFrame sera el actual
    mpLastKeyFrame = pKF; //asignamos el puntero del keyframe
}

void Tracking::SearchLocalPoints()
{
    // Do not search map points already matched
    //para todos los puntos en el currentFrame
    for(vector<MapPoint*>::iterator vit=mCurrentFrame.mvpMapPoints.begin(), vend=mCurrentFrame.mvpMapPoints.end(); vit!=vend; vit++)
    {
        MapPoint* pMP = *vit; //recuperamos el punto
        if(pMP)
        {
            if(pMP->isBad()) //el puntos es malo?
            {
                *vit = static_cast<MapPoint*>(NULL);
            }
            else
            {
                pMP->IncreaseVisible(); //incrementamos la visibilidad
                pMP->mnLastFrameSeen = mCurrentFrame.mnId; //visto por ultima vez desde este frame
                pMP->mbTrackInView = false;
            }
        }
    }

    int nToMatch=0;

    // Project points in frame and check its visibility
    //para todos los puntos en el mapa local
    for(vector<MapPoint*>::iterator vit=mvpLocalMapPoints.begin(), vend=mvpLocalMapPoints.end(); vit!=vend; vit++)
    {
        MapPoint* pMP = *vit; //recuperamos el punto
        if(pMP->mnLastFrameSeen == mCurrentFrame.mnId) //si el ultimo frame donde se vio es este continuamos
            continue;
        if(pMP->isBad()) //si el punto es malo continuamos
            continue;
        // Project (this fills MapPoint variables for matching)
        if(mCurrentFrame.isInFrustum(pMP,0.5)) //verificamos si se puede ver el punto desde este frame
        {
            pMP->IncreaseVisible(); //incrementamos visibilidad
            nToMatch++; //contamos para hacer el matching
        }
    }

    if(nToMatch>0) //si hallamos mas de un punto para hacer el matching
    {
        ORBmatcher matcher(0.8);
        int th = 1;
        if(mSensor==System::RGBD)
            th=3;
        // If the camera has been relocalised recently, perform a coarser search
        if(mCurrentFrame.mnId<mnLastRelocFrameId+2)
            th=5;
        matcher.SearchByProjection(mCurrentFrame,mvpLocalMapPoints,th); //buscamos el match por la proyeccion
    }
}

void Tracking::UpdateLocalMap()
{
    // This is for visualization
    mpMap->SetReferenceMapPoints(mvpLocalMapPoints); //asigamos el mapa local al mapa

    // Update
    UpdateLocalKeyFrames(); //actualizamos los keyframes locales
    UpdateLocalPoints(); //actualizamos los puntos locales
}

void Tracking::UpdateLocalPoints()
{
    mvpLocalMapPoints.clear();

    //para todos los keyframes locales
    for(vector<KeyFrame*>::const_iterator itKF=mvpLocalKeyFrames.begin(), itEndKF=mvpLocalKeyFrames.end(); itKF!=itEndKF; itKF++)
    {
        KeyFrame* pKF = *itKF; //recuperamos el keyframe
        const vector<MapPoint*> vpMPs = pKF->GetMapPointMatches();//recuperamos la lista de puntos

        //para todos los puntos en un keyframe
        for(vector<MapPoint*>::const_iterator itMP=vpMPs.begin(), itEndMP=vpMPs.end(); itMP!=itEndMP; itMP++)
        {
            MapPoint* pMP = *itMP; //recuperamos el punto
            if(!pMP) //si no esxite una estructura continuamos
                continue;
            if(pMP->mnTrackReferenceForFrame==mCurrentFrame.mnId) //si el referenceFrameTrack es el mismo continuamos
                continue;
            if(!pMP->isBad()) //el punto no es malo?
            {
                mvpLocalMapPoints.push_back(pMP); //insertamos en el mapa local
                pMP->mnTrackReferenceForFrame=mCurrentFrame.mnId; //asignamos como referenceFrameTrack
            }
        }
    }
}


void Tracking::UpdateLocalKeyFrames()
{
    // Each map point vote for the keyframes in which it has been observed
    map<KeyFrame*,int> keyframeCounter;
    for(int i=0; i<mCurrentFrame.N; i++) //para cada punto en el currentFrame
    {
        if(mCurrentFrame.mvpMapPoints[i]) //si existe un punto
        {
            MapPoint* pMP = mCurrentFrame.mvpMapPoints[i]; //recuperamos el punto
            if(!pMP->isBad()) //no es un punto malo?
            {
                const map<KeyFrame*,size_t> observations = pMP->GetObservations(); //lista de observaciones
                //para cada observacion
                for(map<KeyFrame*,size_t>::const_iterator it=observations.begin(), itend=observations.end(); it!=itend; it++)
                    keyframeCounter[it->first]++; //almacenamos cuantas veces se vieron en ese frame
            }
            else// el punto es malo?
            {
                mCurrentFrame.mvpMapPoints[i]=NULL;
            }
        }
    }

    if(keyframeCounter.empty()) //si no existe observacion
        return;

    int max=0; //maximo numero de puntos observados desde el keyframe
    KeyFrame* pKFmax= static_cast<KeyFrame*>(NULL); //en que keyframe se vio mas puntos

    mvpLocalKeyFrames.clear(); //limpiamos la lista de keyframes locales
    mvpLocalKeyFrames.reserve(3*keyframeCounter.size());

    // All keyframes that observe a map point are included in the local map. Also check which keyframe shares most points
    //para todos los keyframes encontrados donde existe una observacion
    for(map<KeyFrame*,int>::const_iterator it=keyframeCounter.begin(), itEnd=keyframeCounter.end(); it!=itEnd; it++)
    {
        KeyFrame* pKF = it->first;//recuperamos el keyframe

        if(pKF->isBad())//el keyframe es malo? continuamos
            continue;

        if(it->second>max)//el numero de veces que se vio un punto de ese keyframe es max
        {
            max=it->second;
            pKFmax=pKF;
        }

        mvpLocalKeyFrames.push_back(it->first); //adicionamos como un keyframe en el mapa local
        pKF->mnTrackReferenceForFrame = mCurrentFrame.mnId; //la referencia para hacer tracking es con el currentFrame
    }


    // Include also some not-already-included keyframes that are neighbors to already-included keyframes
    //para toda la lista de keyframes que tienen observaciones de los puntos
    for(vector<KeyFrame*>::const_iterator itKF=mvpLocalKeyFrames.begin(), itEndKF=mvpLocalKeyFrames.end(); itKF!=itEndKF; itKF++)
    {
        // Limit the number of keyframes
        if(mvpLocalKeyFrames.size()>80) //numero de keyframes limitados
            break;

        KeyFrame* pKF = *itKF; //recuperamos keyframe

        const vector<KeyFrame*> vNeighs = pKF->GetBestCovisibilityKeyFrames(10);//recuperamos los vecinos

        for(vector<KeyFrame*>::const_iterator itNeighKF=vNeighs.begin(), itEndNeighKF=vNeighs.end(); itNeighKF!=itEndNeighKF; itNeighKF++)
        {
            KeyFrame* pNeighKF = *itNeighKF; //para cada vecino
            if(!pNeighKF->isBad()) //es malo el vecino?
            {
                if(pNeighKF->mnTrackReferenceForFrame!=mCurrentFrame.mnId) //el vecino tiene un diferente frame trak que el current?
                {
                    mvpLocalKeyFrames.push_back(pNeighKF); //agregamos a la lista de keyframes locales
                    pNeighKF->mnTrackReferenceForFrame=mCurrentFrame.mnId; //y modificamos con cual frame se realizara tracking
                    break;
                }
            }
        }

        const set<KeyFrame*> spChilds = pKF->GetChilds(); //recuperamos los hijos del arbol de expansion un nivel mas alto
        for(set<KeyFrame*>::const_iterator sit=spChilds.begin(), send=spChilds.end(); sit!=send; sit++)
        {
            KeyFrame* pChildKF = *sit; //para cada hijo
            if(!pChildKF->isBad()) //es malo el hijo?
            {
                if(pChildKF->mnTrackReferenceForFrame!=mCurrentFrame.mnId) //el hijo tiene un diferente frame de referencia para el track?
                {
                    mvpLocalKeyFrames.push_back(pChildKF); //adicionamos a la lista de keyframes locales
                    pChildKF->mnTrackReferenceForFrame=mCurrentFrame.mnId; //actualizamos su frame de referencia para el track
                    break;
                }
            }
        }

        KeyFrame* pParent = pKF->GetParent(); //recuperamos el padre del arbol de expansion
        if(pParent) //si existe el padre?
        {
            if(pParent->mnTrackReferenceForFrame!=mCurrentFrame.mnId) //verificamos si tiene diferente el reference frame trrack?
            {
                mvpLocalKeyFrames.push_back(pParent); //agregamos a la lista de keyframes locales
                pParent->mnTrackReferenceForFrame=mCurrentFrame.mnId; //asignamos el reference frame track actual
                break;
            }
        }

    }

    if(pKFmax) //si el keyframe max existe
    {
        mpReferenceKF = pKFmax; //reference keyframe sera ese
        mCurrentFrame.mpReferenceKF = mpReferenceKF; //el tracking se realizara con el keyframe con mas puntos compartidos
    }
}

bool Tracking::Relocalization()
{
    // Compute Bag of Words Vector
    mCurrentFrame.ComputeBoW(); //computar la palabra del currentFrame

    // Relocalization is performed when tracking is lost
    // Track Lost: Query KeyFrame Database for keyframe candidates for relocalisation
    //se hace una busqueda del currentFrame en la base de datos de keyframes para hallar candidatos para la relocalizacion
    vector<KeyFrame*> vpCandidateKFs = mpKeyFrameDB->DetectRelocalizationCandidates(&mCurrentFrame);

    if(vpCandidateKFs.empty()) //si no hay candidatos retornamos estado FALSE
        return false;

    const int nKFs = vpCandidateKFs.size(); //contamos el numero de candidatos

    // We perform first an ORB matching with each candidate
    // If enough matches are found we setup a PnP solver
    //primero realizamos un matching con los puntos ORB para cada candidato y encontramos
    //su posicion con el algorithmo PNP aqui realizamos la configuracion de los pnp
    ORBmatcher matcher(0.75,true);

    vector<PnPsolver*> vpPnPsolvers; //soluciones PNP para cada candidato
    vpPnPsolvers.resize(nKFs);

    vector<vector<MapPoint*> > vvpMapPointMatches; //lista de matched para cada candidato
    vvpMapPointMatches.resize(nKFs);

    vector<bool> vbDiscarded; //vector para verificar que soluciones estan correctamente
    vbDiscarded.resize(nKFs);

    int nCandidates=0; //contador de candidatos

    for(int i=0; i<nKFs; i++) //para cada candidato
    {
        KeyFrame* pKF = vpCandidateKFs[i]; //recuperamos el candidato
        if(pKF->isBad()) //si es un keyframe malo(historico) entonces lo descartamos como candidato
            vbDiscarded[i] = true;
        else //caso contrario hacemos la busqueda de matched
        {
            int nmatches = matcher.SearchByBoW(pKF,mCurrentFrame,vvpMapPointMatches[i]); //numero de matched y lista de matched para el candidato
            if(nmatches<15) //si el numero de matched es menor a 15 entonces lo descartamos como candidato
            {
                vbDiscarded[i] = true;
                continue;
            }
            else //caso pase los filtros y el candidato sea bueno
            {
                PnPsolver* pSolver = new PnPsolver(mCurrentFrame,vvpMapPointMatches[i]); //creamos un pnp para el currentFrame con los matched con el candidato
                pSolver->SetRansacParameters(0.99,10,300,4,0.5,5.991); //asignamos parametros de ransac
                vpPnPsolvers[i] = pSolver; //asignamos la estructura que resolvera a la lista de soluciones
                nCandidates++; //contamos los candidatos
            }
        }
    }

    // Alternatively perform some iterations of P4P RANSAC
    // Until we found a camera pose supported by enough inliers
    bool bMatch = false; //se encontro un match?
    ORBmatcher matcher2(0.9,true); //matcher

    while(nCandidates>0 && !bMatch) //para los buenos candidatos y si no se encontro un match
    {
        for(int i=0; i<nKFs; i++) //para cada candidato
        {
            if(vbDiscarded[i]) //si esta descartado seguimos con el proximo candidato
                continue;

            // Perform 5 Ransac Iterations
            vector<bool> vbInliers; //inlier de la iteracion
            int nInliers; //numero de inlier
            bool bNoMore; //no mas?

            PnPsolver* pSolver = vpPnPsolvers[i]; //recuperamos el solucionador pnp del candidato
            cv::Mat Tcw = pSolver->iterate(5,bNoMore,vbInliers,nInliers); //realizamos 5 iteraciones

            // If Ransac reachs max. iterations discard keyframe
            if(bNoMore) //si el numero de correspondencias de ransac no son suficientes lo descartamos como candidato
            {
                vbDiscarded[i]=true;
                nCandidates--;
            }

            // If a Camera Pose is computed, optimize
            if(!Tcw.empty()) // si se hallo una pose para la camara?
            {
                Tcw.copyTo(mCurrentFrame.mTcw); //asignar esa posicion al currentFrame

                set<MapPoint*> sFound; //conjunto de puntos encontrados

                const int np = vbInliers.size(); //tamano de la lista para verificar inliers

                for(int j=0; j<np; j++) //para toda la lista
                {
                    if(vbInliers[j]) //si es inlier
                    {
                        //asignamos a la lista de puntos en el currentFrame el punto que sirvio para localizar
                        mCurrentFrame.mvpMapPoints[j]=vvpMapPointMatches[i][j];
                        sFound.insert(vvpMapPointMatches[i][j]); //agregamos el punto al conjunto de encontrados
                    }
                    else //si es outlier
                        mCurrentFrame.mvpMapPoints[j]=NULL; //marcamos como null en la lista de currentFrame
                }

                int nGood = Optimizer::PoseOptimization(&mCurrentFrame); //optimizacion de la pose del currentFrame luego de la relocalizacion

                if(nGood<10) //verificar inliers de la optimizacion
                    continue;

                for(int io =0; io<mCurrentFrame.N; io++)  //para cada punto del currentFrame
                    if(mCurrentFrame.mvbOutlier[io]) //si es outlier
                        mCurrentFrame.mvpMapPoints[io]=static_cast<MapPoint*>(NULL); //llenamos como null en el mapa de puntos del currentFrame

                // If few inliers, search by projection in a coarse window and optimize again
                //si tiene pocos inliers buscar po proyeccion en una ventana grande y volver a optimizar
                if(nGood<50)
                {
                    //numero de matched por proyeccion
                    int nadditional =matcher2.SearchByProjection(mCurrentFrame,vpCandidateKFs[i],sFound,10,100);

                    if(nadditional+nGood>=50)
                    {
                        //optimizar y ver los inlier de la optimizacion
                        nGood = Optimizer::PoseOptimization(&mCurrentFrame);

                        // If many inliers but still not enough, search by projection again in a narrower window
                        // the camera has been already optimized with many points
                        //si tiene inliers en el intervalo entonces buscamos los matched en una ventana mas pequena
                        //
                        if(nGood>30 && nGood<50)
                        {
                            sFound.clear(); //limpiamos el found
                            for(int ip =0; ip<mCurrentFrame.N; ip++) //para todos los puntos
                                if(mCurrentFrame.mvpMapPoints[ip]) // si tenemos un punto
                                    sFound.insert(mCurrentFrame.mvpMapPoints[ip]); //agregamos al found
                            nadditional =matcher2.SearchByProjection(mCurrentFrame,vpCandidateKFs[i],sFound,3,64); //buscamos los matched por proyeccion

                            // Final optimization
                            //si los inliers nos satisfacen , realizamos una ultima optimizacion
                            if(nGood+nadditional>=50)
                            {
                                //optimizacion
                                nGood = Optimizer::PoseOptimization(&mCurrentFrame);

                                for(int io =0; io<mCurrentFrame.N; io++) //para todos los puntos
                                    if(mCurrentFrame.mvbOutlier[io]) //verifica si es outlier
                                        mCurrentFrame.mvpMapPoints[io]=NULL; //asigna en el punto 3d como null
                            }
                        }
                    }
                }


                // If the pose is supported by enough inliers stop ransacs and continue
                //si tiene mas mas de 50 inliers encontramos el candidato que hace match
                if(nGood>=50)
                {
                    bMatch = true; //no buscamos mas
                    break;
                }
            }
        }
    }

    if(!bMatch) //si no encontramos retornamos false
    {
        return false;
    }
    else//caso encontremos actualizamos el id del ultimo frame donde nos relocalizamos
    {
        mnLastRelocFrameId = mCurrentFrame.mnId;
        return true;
    }

}

void Tracking::Reset()
{
    mpViewer->RequestStop();

    cout << "System Reseting" << endl;
    while(!mpViewer->isStopped())
        usleep(3000);

    // Reset Local Mapping
    cout << "Reseting Local Mapper...";
    mpLocalMapper->RequestReset();
    cout << " done" << endl;

    // Reset Loop Closing
    cout << "Reseting Loop Closing...";
    mpLoopClosing->RequestReset();
    cout << " done" << endl;

    // Clear BoW Database
    cout << "Reseting Database...";
    mpKeyFrameDB->clear();
    cout << " done" << endl;

    // Clear Map (this erase MapPoints and KeyFrames)
    mpMap->clear();

    KeyFrame::nNextId = 0;
    Frame::nNextId = 0;
    mState = NO_IMAGES_YET;

    if(mpInitializer)
    {
        delete mpInitializer;
        mpInitializer = static_cast<Initializer*>(NULL);
    }

    mlRelativeFramePoses.clear();
    mlpReferences.clear();
    mlFrameTimes.clear();
    mlbLost.clear();

    mpViewer->Release();
}

void Tracking::ChangeCalibration(const string &strSettingPath)
{
    cv::FileStorage fSettings(strSettingPath, cv::FileStorage::READ);
    float fx = fSettings["Camera.fx"];
    float fy = fSettings["Camera.fy"];
    float cx = fSettings["Camera.cx"];
    float cy = fSettings["Camera.cy"];

    cv::Mat K = cv::Mat::eye(3,3,CV_32F);
    K.at<float>(0,0) = fx;
    K.at<float>(1,1) = fy;
    K.at<float>(0,2) = cx;
    K.at<float>(1,2) = cy;
    K.copyTo(mK);

    cv::Mat DistCoef(4,1,CV_32F);
    DistCoef.at<float>(0) = fSettings["Camera.k1"];
    DistCoef.at<float>(1) = fSettings["Camera.k2"];
    DistCoef.at<float>(2) = fSettings["Camera.p1"];
    DistCoef.at<float>(3) = fSettings["Camera.p2"];
    const float k3 = fSettings["Camera.k3"];
    if(k3!=0)
    {
        DistCoef.resize(5);
        DistCoef.at<float>(4) = k3;
    }
    DistCoef.copyTo(mDistCoef);

    mbf = fSettings["Camera.bf"];

    Frame::mbInitialComputations = true;
}

void Tracking::InformOnlyTracking(const bool &flag)
{
    mbOnlyTracking = flag;
}



} //namespace ORB_SLAM
