#include "se3.h"

SE3 :: SE3(){
}

SE3 ::SE3(cv::Mat pT){
    R(0,0) = pT.at<float>(0,0); R(0,1) = pT.at<float>(0,1); R(0,2) = pT.at<float>(0,2); t(0) = pT.at<float>(0,3);
    R(1,0) = pT.at<float>(1,0); R(1,1) = pT.at<float>(1,1); R(1,2) = pT.at<float>(1,2); t(1) = pT.at<float>(1,3);
    R(2,0) = pT.at<float>(2,0); R(2,1) = pT.at<float>(2,1); R(2,2) = pT.at<float>(2,2); t(2) = pT.at<float>(2,3);
}

SE3 :: SE3(Eigen::Matrix3d pRotation, Eigen::Vector3d ptraslation){
    R = pRotation;
    t = ptraslation;
}

Eigen::Vector3d SE3 :: getTraslation(){
    return t;
}

Eigen::Matrix3d SE3 :: getRotation(){
    return R;
}

Matrix34d SE3 :: getTransformation(){
    Matrix34d T;

    T << R(0,0), R(0,1), R(0,2), t(0),
         R(1,0), R(1,1), R(1,2), t(1),
         R(2,0), R(2,1), R(2,2), t(2);
    return T;
}

Eigen::Matrix3d SE3 :: skewSO3(const Eigen::Vector3d &pw){
    Eigen::Matrix3d skew;
    skew << 0.0  ,-pw(2), pw(1),
            pw(2), 0.0  ,-pw(0),
           -pw(1), pw(0), 0.0 ;

    return skew;
}

Eigen::Matrix3d SE3 ::  expSO3(const Eigen::Vector3d &pw){
    Eigen::Matrix3d Rotation;
    Eigen::Matrix3d eye;
    eye << 1.0, 0.0, 0.0,
           0.0, 1.0, 0.0,
           0.0, 0.0, 1.0;

    Eigen::Matrix3d w = skewSO3(pw);

    double theta = sqrt(pw(0)*pw(0) + pw(1)*pw(1) + pw(2)*pw(2));

    if(theta < 0.00015) Rotation = eye + w + 0.5*(w*w);
    else Rotation = eye + (sin(theta)/theta)*w + (1 - cos(theta))/(theta*theta)*(w*w);

    return Rotation;
}

Eigen::Vector3d SE3 :: logSO3(const Eigen::Matrix3d &pRotation){
    Eigen::Vector3d w;
    Eigen::Matrix3d skew;
    double value = 0.5 * (pRotation.trace() - 1);

    if(fabs(value) > 0.9999 && fabs(value) < 1.00001) skew = 0.5 * (pRotation - pRotation.transpose());
    else skew = 0.5 * (acos(value)/(sqrt(1.0 - value*value))) * (pRotation - pRotation.transpose());

    w << skew(2,1), skew(0,2), skew(1,0);

    return w;

}

Matrix34d SE3 :: expSE3(const Vector6d &ptwist){
    Eigen::Vector3d w, v;
    w << ptwist(0), ptwist(1), ptwist(2);
    v << ptwist(3), ptwist(4), ptwist(5);

    Eigen::Matrix3d Rotation;
    Eigen::Vector3d traslation;
    Rotation = expSO3(w);

    Eigen::Matrix3d A, eye, skew;
    eye << 1.0, 0.0, 0.0,
           0.0, 1.0, 0.0,
           0.0, 0.0, 1.0;

    skew = skewSO3(w);

    double theta = sqrt(w(0)*w(0) + w(1)*w(1) + w(2)*w(2));

    if(theta < 0.000015) A = eye + 0.5*skew + 0.166666667*(skew*skew);
    else A = eye + ((1 - cos(theta))/(theta*theta))*(skew) + ((theta - sin(theta))/(theta*theta*theta))*(skew * skew);

    traslation = A*v;

    Matrix34d Transformation;

    Transformation << Rotation(0,0), Rotation(0,1), Rotation(0,2), traslation(0),
                      Rotation(1,0), Rotation(1,1), Rotation(1,2), traslation(1),
                      Rotation(2,0), Rotation(2,1), Rotation(2,2), traslation(2);

    return Transformation;
}

Vector6d SE3 :: logSE3(const Matrix34d &pT){
    Eigen::Matrix3d Rotation;
    Eigen::Vector3d traslation;

    Rotation << pT(0,0), pT(0,1), pT(0,2),
                pT(1,0), pT(1,1), pT(1,2),
                pT(2,0), pT(2,1), pT(2,2);

    traslation << pT(0,3), pT(1,3), pT(2,3);

    Eigen::Vector3d w, v;
    w = logSO3(Rotation);

    Eigen::Matrix3d A, eye, skew;
    eye << 1.0, 0.0, 0.0,
           0.0, 1.0, 0.0,
           0.0, 0.0, 1.0;

    skew = skewSO3(w);

    double theta = sqrt(w(0)*w(0) + w(1)*w(1) + w(2)*w(2));

    if(theta < 0.000015f) A = eye + 0.5*skew + 0.166666667*(skew*skew);
    else A = eye + ((1 - cos(theta))/(theta*theta))*skew + ((theta - sin(theta))/(theta*theta*theta))*(skew * skew);

    v = A.inverse() * traslation;

    Vector6d twist;
    twist << w(0), w(1), w(2), v(0), v(1), v(2);

    return twist;
}

void SE3 :: Euler(double prx, double pry, double prz, double ptx, double pty, double ptz){
    Eigen::Matrix3d Rx, Ry, Rz;
    Rx << 1, 0, 0, 0, cos(prx), -sin(prx), 0, sin(prx), cos(prx);
    Ry << cos(pry), 0, sin(pry), 0, 1, 0, -sin(pry), 0, cos(pry);
    Rz << cos(prz), -sin(prz), 0, sin(prz), cos(prz), 0, 0, 0, 1;

    R = Rz * Ry * Rx;
    t << ptx, pty, ptz;
}

void SE3 :: Quaternion(double pqw, double pqx, double pqy, double pqz, double ptx, double pty, double ptz){
    R = Eigen::Quaterniond(pqw, pqx, pqy, pqz).toRotationMatrix();
    t << ptx, pty, ptz;
}

void SE3 :: Lie(double pwx, double pwy, double pwz, double pvx, double pvy, double pvz){
    Vector6d twist;
    twist << pwx, pwy, pwz, pvx, pvy, pvz;

    Matrix34d T = expSE3(twist);

    R << T(0,0), T(0,1), T(0,2),
         T(1,0), T(1,1), T(1,2),
         T(2,0), T(2,1), T(2,2);

    t << T(0,3), T(1,3), T(2,3);
}

Eigen::Vector3d SE3 :: traslate(const Eigen::Vector3d &pPoint){
    Eigen::Vector3d newPoint = t + pPoint;
    return newPoint;
}

Eigen::Vector3d SE3 :: rotate(const Eigen::Vector3d &pPoint){
    Eigen::Vector3d newPoint = R * pPoint;
    return newPoint;
}

Eigen::Vector3d SE3 :: transform(const Eigen::Vector3d &pPoint){
    Eigen::Vector3d newPoint = R * pPoint + t;
    return newPoint;
}

SE3 SE3 :: inverse(){
    Eigen::Matrix3d Rotation = R.transpose();
    Eigen::Vector3d traslation = -R.transpose()*t;

    SE3 se3 = SE3(Rotation, traslation);

    return se3;
}

SE3 SE3 :: operator *(const SE3 &pT){

    Eigen::Matrix3d Rotation;
    Eigen::Vector3d traslation;

    Rotation(0,0) = R(0,0)* pT.R(0,0) + R(0,1)*pT.R(1,0) + R(0,2)*pT.R(2,0);
    Rotation(0,1) = R(0,0)* pT.R(0,1) + R(0,1)*pT.R(1,1) + R(0,2)*pT.R(2,1);
    Rotation(0,2) = R(0,0)* pT.R(0,2) + R(0,1)*pT.R(1,2) + R(0,2)*pT.R(2,2);
    Rotation(1,0) = R(1,0)* pT.R(0,0) + R(1,1)*pT.R(1,0) + R(1,2)*pT.R(2,0);
    Rotation(1,1) = R(1,0)* pT.R(0,1) + R(1,1)*pT.R(1,1) + R(1,2)*pT.R(2,1);
    Rotation(1,2) = R(1,0)* pT.R(0,2) + R(1,1)*pT.R(1,2) + R(1,2)*pT.R(2,2);
    Rotation(2,0) = R(2,0)* pT.R(0,0) + R(2,1)*pT.R(1,0) + R(2,2)*pT.R(2,0);
    Rotation(2,1) = R(2,0)* pT.R(0,1) + R(2,1)*pT.R(1,1) + R(2,2)*pT.R(2,1);
    Rotation(2,2) = R(2,0)* pT.R(0,2) + R(2,1)*pT.R(1,2) + R(2,2)*pT.R(2,2);

    traslation(0) = t(0) + R(0,0)*pT.t(0) + R(0,1)*pT.t(1) + R(0,2)*pT.t(2);
    traslation(1) = t(1) + R(1,0)*pT.t(0) + R(1,1)*pT.t(1) + R(1,2)*pT.t(2);
    traslation(2) = t(2) + R(2,0)*pT.t(0) + R(2,1)*pT.t(1) + R(2,2)*pT.t(2);

    SE3 se3 = SE3(Rotation, traslation);

    return se3;
}

Eigen::Vector3d SE3 ::operator *(const Eigen::Vector3d &pPoint){
    Eigen::Vector3d point = R*pPoint + t;

    return point;
}
