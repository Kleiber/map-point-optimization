#ifndef FRAME_H
#define FRAME_H

#include<vector>

#include "MapPoint.h"
#include "Thirdparty/DBoW2/DBoW2/BowVector.h"
#include "Thirdparty/DBoW2/DBoW2/FeatureVector.h"
#include "ORBVocabulary.h"
#include "KeyFrame.h"
#include "ORBextractor.h"

#include <opencv2/opencv.hpp>

namespace ORB_SLAM2
{
#define FRAME_GRID_ROWS 48
#define FRAME_GRID_COLS 64

class MapPoint;
class KeyFrame;

class Frame
{
public:
    Frame();

    // Copy constructor.
    //crear un nuevo frame con la informacion de otro, realiza una copia con todos los parametros iguales
    Frame(const Frame &frame);

    // Constructor for stereo cameras.
    //crear un nuevo keyframe con dos imagenes, obtengo los keypoints y inicializo las variables
    Frame(const cv::Mat &imLeft, const cv::Mat &imRight, const double &timeStamp, ORBextractor* extractorLeft, ORBextractor* extractorRight, ORBVocabulary* voc, cv::Mat &K, cv::Mat &distCoef, const float &bf, const float &thDepth);

    // Constructor for RGB-D cameras.
    //crea un nuevo keyframe con una imagen y su respectiva profundidad , obtengo los keypoints y inicializo las variables
    Frame(const cv::Mat &imGray, const cv::Mat &imDepth, const double &timeStamp, ORBextractor* extractor,ORBVocabulary* voc, cv::Mat &K, cv::Mat &distCoef, const float &bf, const float &thDepth);

    // Constructor for Monocular cameras.
    //crea un nuevo keyframe con una sola imagen, obtengo los keypoints y inicializo las variables
    Frame(const cv::Mat &imGray, const double &timeStamp, ORBextractor* extractor,ORBVocabulary* voc, cv::Mat &K, cv::Mat &distCoef, const float &bf, const float &thDepth);

    // Extract ORB on the image. 0 for left image and 1 for right image.
    // extraer los puntos orb en la imagen (flag 0(izquierda) 1(derecha))
    void ExtractORB(int flag, const cv::Mat &im);

    // Compute Bag of Words representation.
    //computar la representacion del descriptor en un descriptor de palabras (BoW vector y Feature Vector ambos en palabras)
    void ComputeBoW();

    // Set the camera pose.
    //asignar pose de la camara y actualizar el centro de la camara en el mundo
    void SetPose(cv::Mat Tcw);

    // Computes rotation, translation and camera center matrices from the camera pose.
    //computar rotacion, traslacion  y centro de la camara para la posicion de la camamara
    void UpdatePoseMatrices();

    // Returns the camera center.
    //retornar el centro de la camara
    inline cv::Mat GetCameraCenter(){
        return mOw.clone();
    }

    // Returns inverse of rotation
    //retornar la inversa de la matriz de rotacion
    inline cv::Mat GetRotationInverse(){
        return mRwc.clone();
    }

    // Check if a MapPoint is in the frustum of the camera
    // and fill variables of the MapPoint to be used by the tracking
    // verificar si el coseno de un punto en el frame esta dentro del limite estrablecido para ser considerado en el tracking
    bool isInFrustum(MapPoint* pMP, float viewingCosLimit);

    // Compute the cell of a keypoint (return false if outside the grid)
    //verificar si el punto cae en el grid
    bool PosInGrid(const cv::KeyPoint &kp, int &posX, int &posY);

    //retornar los puntos que estan dentro de un area
    vector<size_t> GetFeaturesInArea(const float &x, const float  &y, const float  &r, const int minLevel=-1, const int maxLevel=-1) const;

    // Search a match for each keypoint in the left image to a keypoint in the right image.
    // If there is a match, depth is computed and the right coordinate associated to the left keypoint is stored.
    // solo para camaras esteros en la creacion del keyframe
    void ComputeStereoMatches();

    // Associate a "right" coordinate to a keypoint if there is valid depth in the depthmap.
    // solo para camaras RGBD
    void ComputeStereoFromRGBD(const cv::Mat &imDepth);

    // Backprojects a keypoint (if stereo/depth info available) into 3D world coordinates.
    //calcula el punto en el espacio
    cv::Mat UnprojectStereo(const int &i);


public:
    // Vocabulary used for relocalization.
    //vocabulario se carga en la inicializacion
    ORBVocabulary* mpORBvocabulary;

    // Feature extractor. The right is used only in the stereo case.
    // extractores de los puntos orb para cada imagen en caso sea estero y solo left en caso sea monocular
    ORBextractor* mpORBextractorLeft, *mpORBextractorRight;

    // Frame timestamp.
    //tiempo que fue capturada la imagen
    double mTimeStamp;

    // Calibration matrix and OpenCV distortion parameters.
    //matrix de parametros intrinsicos y extrinsicos y los coeficientes de distorcion (matriz de la calibracion)
    cv::Mat mK;
    static float fx;
    static float fy;
    static float cx;
    static float cy;
    static float invfx;
    static float invfy;
    cv::Mat mDistCoef;

    // Stereo baseline multiplied by fx.
    //separacion de camaras en caso sea estero en pixeles
    float mbf;

    // Stereo baseline in meters.
    //separacion de las camaras en metros
    float mb;

    // Threshold close/far points. Close points are inserted from 1 view.
    // Far points are inserted as in the monocular case from 2 views.
    float mThDepth;

    // Number of KeyPoints.
    //numero de puntos obtenidos en la extraccion de orb puntos
    int N;

    // Vector of keypoints (original for visualization) and undistorted (actually used by the system).
    // In the stereo case, mvKeysUn is redundant as images must be rectified.
    // In the RGB-D case, RGB images can be distorted.
    //en caso de monoculas se usa los Keys  y KeysUn donde se almacenan los puntos originales y los puntos con undistorcion
    std::vector<cv::KeyPoint> mvKeys, mvKeysRight;
    std::vector<cv::KeyPoint> mvKeysUn;

    // Corresponding stereo coordinate and depth for each keypoint.
    // "Monocular" keypoints have a negative value.
    // en caso de ser estereo se utiliza las correscpondencias Right y la profundidda
    // en caso de ser rgbd se utiliza la profundidad
    // en caso de monocular todos tienen un valor negativo
    std::vector<float> mvuRight;
    std::vector<float> mvDepth;

    // Bag of Words Vector structures.
    //descriptor del frame representado como una palabra
    DBoW2::BowVector mBowVec;
    DBoW2::FeatureVector mFeatVec;

    // ORB descriptor, each row associated to a keypoint.
    // descriptor del frame asociado a cada punto en el frame
    //si es monocular solo se usa el descriptor
    //si es estereo se usa los dos descriptores
    cv::Mat mDescriptors, mDescriptorsRight;

    // MapPoints associated to keypoints, NULL pointer if no association.
    //cada punto es asociado a un punto en 3d el cual es su projeccion en el espacio
    std::vector<MapPoint*> mvpMapPoints;

    // Flag to identify outlier associations.
    //representa cuales puntos son outliers y pueden afectar a los calculos
    std::vector<bool> mvbOutlier;

    // Keypoints are assigned to cells in a grid to reduce matching complexity when projecting MapPoints.
    static float mfGridElementWidthInv;
    static float mfGridElementHeightInv;
    std::vector<std::size_t> mGrid[FRAME_GRID_COLS][FRAME_GRID_ROWS];

    // Camera pose.
    //posicion de la camara
    cv::Mat mTcw;

    // Current and Next Frame id.
    static long unsigned int nNextId; //identificador del siguiente frame
    long unsigned int mnId; //identificador del frame

    // Reference Keyframe.
    KeyFrame* mpReferenceKF; //keyframe al cual esta siendo referencia

    // Scale pyramid info.
    int mnScaleLevels;
    float mfScaleFactor;
    float mfLogScaleFactor;
    vector<float> mvScaleFactors;
    vector<float> mvInvScaleFactors;
    vector<float> mvLevelSigma2;
    vector<float> mvInvLevelSigma2;

    // Undistorted Image Bounds (computed once).
    static float mnMinX; //limite inferior en x
    static float mnMaxX; //limite superior en x
    static float mnMinY; //limite inferior en y
    static float mnMaxY; //limite superior en y

    static bool mbInitialComputations; //si es la primera vez que se registra el frame


private:

    // Undistort keypoints given OpenCV distortion parameters.
    // Only for the RGB-D case. Stereo must be already rectified!
    // (called in the constructor).
    //si existe coeficientes de distorcion entonces arreglamos la imagen y colocamos los puntos nuevos en la lista de puntos claves caso no haya no los modificamos y solo los copiamos
    void UndistortKeyPoints();

    // Computes image bounds for the undistorted image (called in the constructor).
    //nuevos limites para la imagen distorcionada en caso no exista consideramos como limites toda la imagen
    void ComputeImageBounds(const cv::Mat &imLeft);

    // Assign keypoints to the grid for speed up feature matching (called in the constructor).
    //asignar los puntos en el grid
    void AssignFeaturesToGrid();

    // Rotation, translation and camera center
    cv::Mat mRcw; //rotacion de la camara
    cv::Mat mtcw; //traslacion de la camra
    cv::Mat mRwc; // rotacion inversa
    cv::Mat mOw; //==mtwc // traslacion inversa
};

}// namespace ORB_SLAM

#endif // FRAME_H
