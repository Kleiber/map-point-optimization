#ifndef INITIALIZER_H
#define INITIALIZER_H

#include<opencv2/opencv.hpp>
#include "Frame.h"


namespace ORB_SLAM2
{

// THIS IS THE INITIALIZER FOR MONOCULAR SLAM. NOT USED IN THE STEREO OR RGBD CASE.
class Initializer
{
    typedef pair<int,int> Match;

public:

    // Fix the reference frame
    //crear nueva inicializacion con el frame de referencia una desviacion estandar(parametro de error) y el numero de iteraciones de ransac
    Initializer(const Frame &ReferenceFrame, float sigma = 1.0, int iterations = 200);

    // Computes in parallel a fundamental matrix and a homography
    // Selects a model and tries to recover the motion and the structure from motion
    //computa dos formas de solucion para el movimiento de la camara(matriz fundamental y homografia) y escoje el mejor (score para cada uno)
    //retorna la transformacion del frame actual para el frame de referencia, una lista de puntos en el espacio y si fueron calculados correctamente
    bool Initialize(const Frame &CurrentFrame, const vector<int> &vMatches12,
                    cv::Mat &R21, cv::Mat &t21, vector<cv::Point3f> &vP3D, vector<bool> &vbTriangulated);


private:

    //calcula la matriz homografia entre los buenos matches, retorna un score y la matriz de homografia
    void FindHomography(vector<bool> &vbMatchesInliers, float &score, cv::Mat &H21);
    //calcula la matriz fundamental entre los buenos matches, retorn un score y la matriz fundamental
    void FindFundamental(vector<bool> &vbInliers, float &score, cv::Mat &F21);

    //computa la matriz de homografia SVD
    cv::Mat ComputeH21(const vector<cv::Point2f> &vP1, const vector<cv::Point2f> &vP2);
    //computa la matriz fundamental SVD
    cv::Mat ComputeF21(const vector<cv::Point2f> &vP1, const vector<cv::Point2f> &vP2);

    //verifica si la matriz de homografia es apropiada, cuantos inlier tiene dependiendo de la desviacion estandar
    float CheckHomography(const cv::Mat &H21, const cv::Mat &H12, vector<bool> &vbMatchesInliers, float sigma);
    //verifica si la matriz fundamental es apropiada, cuantos inlier tiene dependiendo de la desviacion estandar
    float CheckFundamental(const cv::Mat &F21, vector<bool> &vbMatchesInliers, float sigma);

    //computa la rotacion y traslacion a partir de la matriz fundamental
    bool ReconstructF(vector<bool> &vbMatchesInliers, cv::Mat &F21, cv::Mat &K,
                      cv::Mat &R21, cv::Mat &t21, vector<cv::Point3f> &vP3D, vector<bool> &vbTriangulated, float minParallax, int minTriangulated);
    //computa la rotacion y traslacion a partir de la matriz de homografia
    bool ReconstructH(vector<bool> &vbMatchesInliers, cv::Mat &H21, cv::Mat &K,
                      cv::Mat &R21, cv::Mat &t21, vector<cv::Point3f> &vP3D, vector<bool> &vbTriangulated, float minParallax, int minTriangulated);

    //computa la triagulizacion entre dos puntos obteniendo el punto en el espacio 3d
    void Triangulate(const cv::KeyPoint &kp1, const cv::KeyPoint &kp2, const cv::Mat &P1, const cv::Mat &P2, cv::Mat &x3D);

    //normalizar lista de puntos y retornar una matriz que lo codifica (para calcular la matriz fundamental)
    void Normalize(const vector<cv::KeyPoint> &vKeys, vector<cv::Point2f> &vNormalizedPoints, cv::Mat &T);

    //verifica si la rotacion y traslacion hallada son correctas
    int CheckRT(const cv::Mat &R, const cv::Mat &t, const vector<cv::KeyPoint> &vKeys1, const vector<cv::KeyPoint> &vKeys2,
                       const vector<Match> &vMatches12, vector<bool> &vbInliers,
                       const cv::Mat &K, vector<cv::Point3f> &vP3D, float th2, vector<bool> &vbGood, float &parallax);

    //descompone la matriz esencial en una traslacion y una rotacion
    void DecomposeE(const cv::Mat &E, cv::Mat &R1, cv::Mat &R2, cv::Mat &t);


    // Keypoints from Reference Frame (Frame 1)
    vector<cv::KeyPoint> mvKeys1; //puntos en el primer frame, frame inicial o de referencia

    // Keypoints from Current Frame (Frame 2)
    vector<cv::KeyPoint> mvKeys2; //puntos en el segundo frame, frame actual

    // Current Matches from Reference to Current
    vector<Match> mvMatches12; //matches entre los puntos del frame actual y el de referencia
    vector<bool> mvbMatched1; //matches que tienen por lo menos un match como par en la lista

    // Calibration
    cv::Mat mK; // matriz de calibracion(parametros instrinsicos y extrinsicos)

    // Standard Deviation and Variance
    float mSigma, mSigma2; // desviacion estandar(rango de error) y varianza(cuadrado de desviacion estandar)

    // Ransac max iterations
    int mMaxIterations; //numero maximo de iteraciones para ransac

    // Ransac sets
    vector<vector<size_t> > mvSets;  //conjunto de indices aleatorios de los puntos con que se realizara ransac

};

} //namespace ORB_SLAM

#endif // INITIALIZER_H
