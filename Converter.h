#ifndef CONVERTER_H
#define CONVERTER_H

#include<opencv2/core/core.hpp>

#include<Eigen/Dense>
#include"Thirdparty/g2o/g2o/types/types_six_dof_expmap.h"
#include"Thirdparty/g2o/g2o/types/types_seven_dof_expmap.h"

namespace ORB_SLAM2
{

class Converter
{
public:
    //convierte la matriz descriptor a un vector descriptor
    static std::vector<cv::Mat> toDescriptorVector(const cv::Mat &Descriptors);

    //convierte una matriz de transformacion(rotacion y traslacion) en un quaternion en el espacio euclidiano 3d
    static g2o::SE3Quat toSE3Quat(const cv::Mat &cvT);
    //convierte una transformacion similirar(rotacion, traslacion y escala) para un quaternion en el el espacio euclideano 3d
    static g2o::SE3Quat toSE3Quat(const g2o::Sim3 &gSim3);

    //convierte un quaternion en el espacio euclidiano 3d a una matriz de transformacion(rotacion y traslacion)
    static cv::Mat toCvMat(const g2o::SE3Quat &SE3);
    //convierte una transformacion similar(rotacion, traslacion y escala) a una matriz de transformacion(rotacion y traslacion)
    static cv::Mat toCvMat(const g2o::Sim3 &Sim3);
    //convierte una matriz de eigen a una matriz mat de opencv
    static cv::Mat toCvMat(const Eigen::Matrix<double,4,4> &m);
    //convierte una matriz de eigen a una matriz mat de opencv
    static cv::Mat toCvMat(const Eigen::Matrix3d &m);
    //convierte un vector de eigen a una matriz mat de opencv
    static cv::Mat toCvMat(const Eigen::Matrix<double,3,1> &m);
    //convierte una rotacion y traslacion de eigen a una matriz de transformacion en at de opencv
    static cv::Mat toCvSE3(const Eigen::Matrix<double,3,3> &R, const Eigen::Matrix<double,3,1> &t);

    //convierte un vector de opencv a un vector de eigen
    static Eigen::Matrix<double,3,1> toVector3d(const cv::Mat &cvVector);
    //convierte un punto de opencv a un vector de eigen
    static Eigen::Matrix<double,3,1> toVector3d(const cv::Point3f &cvPoint);
    //convierte una matriz mat de opencv a una matriz de eigen
    static Eigen::Matrix<double,3,3> toMatrix3d(const cv::Mat &cvMat3);

    //convertir una matriz mat de opencv a un vector quaternion de opencv
    static std::vector<float> toQuaternion(const cv::Mat &M);
};

}// namespace ORB_SLAM

#endif // CONVERTER_H
