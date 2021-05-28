#ifndef SIM3SOLVER_H
#define SIM3SOLVER_H

#include <opencv2/opencv.hpp>
#include <vector>

#include "KeyFrame.h"



namespace ORB_SLAM2
{

class Sim3Solver
{
public:

    Sim3Solver(KeyFrame* pKF1, KeyFrame* pKF2, const std::vector<MapPoint*> &vpMatched12, const bool bFixScale = true);

    //asigna los parametros para ransac
    void SetRansacParameters(double probability = 0.99, int minInliers = 6 , int maxIterations = 300);

    //encuentra la mejor correspondencia usando ransac
    cv::Mat find(std::vector<bool> &vbInliers12, int &nInliers);

    //itera con los parametros de ransac y encuentra la mejor solucion
    cv::Mat iterate(int nIterations, bool &bNoMore, std::vector<bool> &vbInliers, int &nInliers);

    //recupera la mejor rotacion estimada
    cv::Mat GetEstimatedRotation();
    //recupera la mejor traslacion estimada
    cv::Mat GetEstimatedTranslation();
    //recupera la mejor escala estimada
    float GetEstimatedScale();


protected:

    //calcular el centroide
    void ComputeCentroid(cv::Mat &P, cv::Mat &Pr, cv::Mat &C);

    //modelo para hallar la rotacion, traslacion y escala
    void ComputeSim3(cv::Mat &P1, cv::Mat &P2);

    //verifica si un pixel es inlier o no observando la distancia entre el pixel y su projeccion
    void CheckInliers();

    //transforma los puntos 3d para otra referencia(rotacion y traslacion) y luego los projecta en la imagen en pixeles
    void Project(const std::vector<cv::Mat> &vP3Dw, std::vector<cv::Mat> &vP2D, cv::Mat Tcw, cv::Mat K);
    //projecta los puntos 3d en la imagen en pixeles
    void FromCameraToImage(const std::vector<cv::Mat> &vP3Dc, std::vector<cv::Mat> &vP2D, cv::Mat K);


protected:

    // KeyFrames and matches
    KeyFrame* mpKF1; //keyframe1
    KeyFrame* mpKF2; //keyframe2

    std::vector<cv::Mat> mvX3Dc1; //puntos 3d del keyframe1
    std::vector<cv::Mat> mvX3Dc2; //puntos 3d del keyframe2
    std::vector<MapPoint*> mvpMapPoints1; //puntos del mapa del keyframe1
    std::vector<MapPoint*> mvpMapPoints2; //puntos del mapa del keyframe2
    std::vector<MapPoint*> mvpMatches12; //lista de matches enntre los puntos del keyframe1 y el keyframe2
    std::vector<size_t> mvnIndices1; //lista de indices para asociar el match
    std::vector<size_t> mvSigmaSquare1; // sigma asociado al punto en la piramide y un octave en el primer keyframe1
    std::vector<size_t> mvSigmaSquare2; // sigma asociado al punto en la piramide y un octave en el primer keyframe2
    std::vector<size_t> mvnMaxError1; //error de distancia del keyframe1
    std::vector<size_t> mvnMaxError2; //eror de distancia del keyframe2

    int N; //numero de puntos del keyframe1
    int mN1; //numero de matches entre el keyframe1 y el keyframe2

    // Current Estimation
    cv::Mat mR12i; //rotacion estimada en la iteracion i
    cv::Mat mt12i; //traslacion estimada en la iteracion i
    float ms12i; //escala estimada en la iteracion i
    cv::Mat mT12i; //transformacion estimada en la iteracion i
    cv::Mat mT21i; //trasnformacion inversa estiamada en la iteracion i
    std::vector<bool> mvbInliersi; // lista de inliers en la iteracion i
    int mnInliersi; // numero de inliers en la iteracion i

    // Current Ransac State
    int mnIterations; //numero de iteraciones que se realizo para llegar al resultado
    std::vector<bool> mvbBestInliers; //lista de los inliers
    int mnBestInliers; //numero mejor de inliers
    cv::Mat mBestT12; //mejor transformacion estimada
    cv::Mat mBestRotation; //mejor rotacion estimada
    cv::Mat mBestTranslation; //mejor traslacion estimada
    float mBestScale; //mejor escala estimada

    // Scale is fixed to 1 in the stereo/RGBD case
    bool mbFixScale; //si la escala es fija (1)(stereo/RGBD , no es fija(0)(Mono)

    // Indices for random selection
    std::vector<size_t> mvAllIndices;//lista aleatoria de indices de los puntos a usar en cada iteracion de ransac

    // Projections
    std::vector<cv::Mat> mvP1im1; //projecion de los puntos del keyframe1 en la imagen1
    std::vector<cv::Mat> mvP2im2;//projecion de los puntos del keyframe2 en la imagen2

    // RANSAC probability
    double mRansacProb;//varibale asignada sobre la probabilidad de ransac

    // RANSAC min inliers
    int mRansacMinInliers; //numero min de inlier que considera ransac para aceptar como una posible solucion

    // RANSAC max iterations
    int mRansacMaxIts; //numero de iteraciones de ransac

    // Threshold inlier/outlier. e = dist(Pi,T_ij*Pj)^2 < 5.991*mSigma2
    float mTh; //limite para considerar si es inlier o outlier
    float mSigma2;

    // Calibration
    cv::Mat mK1;//matriz de calibracion del keyframe1
    cv::Mat mK2;//matriz de calibracion del keyframe2

};

} //namespace ORB_SLAM

#endif // SIM3SOLVER_H
