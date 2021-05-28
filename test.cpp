#include<iostream>
#include <Eigen/Dense>

#include <pinholecamera.h>
#include <se3.h>
#include <seed.h>
#include <depthestimation.h>

using namespace std;

void test_PinholeCamera(){

    double depth = 1.0;
    Eigen::Vector3d pixel;
    pixel(0) = 160.0f;
    pixel(1) = 120.0f;
    pixel(2) = 1.0f;

    Eigen::Vector3d point;
    point(0) = -160.0f;
    point(1) = -120.0f;
    point(2) = 1.0f;

    double fx = 1.0f;
    double fy = 1.0f;
    double cx = 320.0f;
    double cy = 240.0f;

    double width = 640.0f;
    double heigth = 480.0f;

    PinholeCamera pinhole = PinholeCamera(fx, fy, cx, cy, width, heigth);

    Eigen::Vector3d point1 = pinhole.CamToWorld(pixel);
    Eigen::Vector3d point2 = pinhole.CamToWorld(pixel, depth);
    Eigen::Vector3d pixel1 = pinhole.WorldToCam(point);
    double errorangle = pinhole.ErrorPixelAngle();

    cout<<endl;
    cout<<"fx: "<<pinhole.fx<<endl;
    cout<<"fy: "<<pinhole.fy<<endl;
    cout<<"cx: "<<pinhole.cx<<endl;
    cout<<"cy: "<<pinhole.cy<<endl;
    cout<<"width : "<<pinhole.width<<endl;
    cout<<"heigth: "<<pinhole.height<<endl;
    cout<<"error angle in pixels: "<<errorangle<<endl;
    cout<<"["<<pixel.transpose()<<"]  ["<<point1.transpose()<<"]"<<endl;
    cout<<"["<<pixel.transpose()<<"]  ["<<point2.transpose()<<"]"<<endl;
    cout<<"["<<point.transpose()<<"]  ["<<pixel1.transpose()<<"]"<<endl;
    cout<<endl;
}

void test_se3(){

    double rx = 37.0f * M_PI/180.0f;
    double ry = 0;
    double rz = 0;

    double tx = 7.0f;
    double ty = 1.0f;
    double tz = 9.0f;

    double qw = 0.948324f;
    double qx = 0.317305f;
    double qy = 0.0f;
    double qz = 0.0f;

    Eigen::Vector3d w, v;
    w << 0.645772, 0, 0;
    v << 7, 3.87098, 8.36215;

    Vector6d twist;
    twist << w, v;

    Eigen::Quaterniond q;
    Eigen::Vector3d vector;
    vector << qx, qy, qz;
    q.w() = qw;
    q.vec() = vector;

    SE3 se3 = SE3();

    se3.Euler(rx, ry, rz, tx, ty, tz);
    cout<<"euler: "<<rx<<" "<<ry<<" "<<rz<<" "<<tx<<" "<<ty<<" "<<tz<<endl;
    cout<<endl;
    cout<<se3.getTransformation()<<endl;
    cout<<endl;

    se3.Lie(w(0), w(1), w(2), v(0), v(1), v(2));
    cout<<"twist: "<<twist.transpose()<<endl;
    cout<<endl;
    cout<<se3.getTransformation()<<endl;
    cout<<endl;

    se3.Quaternion(qw, qx, qy, qz,tx, ty, tz);
    Eigen::Quaterniond q1(se3.R);
    cout<<"quaternion: "<<q.w()<<" "<<q.vec().transpose()<<" "<<tx<<" "<<ty<<" "<<tz<<endl;
    cout<<"quaternion: "<<q1.w()<<" "<<q1.vec().transpose()<<" "<<tx<<" "<<ty<<" "<<tz<<endl;
    cout<<endl;
    cout<<se3.getTransformation()<<endl;
    cout<<endl;

    Matrix34d T = se3.getTransformation();
    Vector6d twist1 = se3.logSE3(T);
    Matrix34d T1 = se3.expSE3(twist);
    cout<<endl;
    cout<<twist1<<endl;
    cout<<T<<endl;
    cout<<endl;
    cout<<twist1<<endl;
    cout<<T1<<endl;
    cout<<endl;

    cout<<"traslation vector:"<<se3.getTraslation().transpose()<<endl;
    cout<<"rotation matrix :"<<endl;
    cout<<se3.getRotation()<<endl;
    cout<<"transformation matrix :"<<endl;
    cout<<se3.getTransformation()<<endl;
    cout<<endl;

    Eigen::Vector3d point, newPoint;
    point<< 1, 1, 1;
    cout<<"point : "<<point.transpose()<<endl;
    newPoint = se3.traslate(point);
    cout<<"traslate : "<<newPoint.transpose()<<endl;
    newPoint = se3.rotate(point);
    cout<<"rotate : "<<newPoint.transpose()<<endl;
    newPoint = se3.transform(point);
    cout<<"transform : "<<newPoint.transpose()<<endl;
    cout<<endl;
    cout<<endl;

    Eigen::Vector3d vect;
    Eigen::Vector4d vect1;
    vect<<1,2,3;
    vect1<<1,2,3,1;

    Eigen::Vector3d e;
    Eigen::Matrix4d a,b;

    a<< 1 ,        0,         0,         1,
        0 , 0.707107, -0.707107,         1,
        0 , 0.707107,  0.707107,         1,
        0 ,        0,         0,         1;

    b<< 1 ,0 ,0 ,1,
        0 ,1 ,0 ,1,
        0 ,0 ,1 ,1,
        0, 0, 0, 1;


    SE3 se3a = SE3();
    se3a.Euler(M_PI/4,0,0,1,1,1);
    SE3 se3b = SE3();
    se3b.Euler(0,0,0,1,1,1);

    SE3 se3c = se3a*se3b;

    cout<<endl;
    cout<<se3c.getTransformation()<<endl;
    cout<<endl;
    cout<<a*b<<endl;

    cout<<endl;

    cout<<endl;
    cout<<se3b*vect<<endl;
    cout<<endl;
    cout<<b*vect1<<endl;

    SE3 se3d = se3b.inverse();

    cout<<endl;

    cout<<endl;
    cout<<b.inverse()<<endl;
    cout<<endl;
    cout<<se3d.getTransformation()<<endl;
}


void test_seed(){

    double depthMin = 24;
    double depthMean = 28;
    double sigma2 = 0.032;
    double mu = 32;
    Eigen::Vector3d pixel;
    pixel<<0,0,0;

    Seed seed = Seed();
    seed.initSeed(pixel, depthMean, depthMin);
    seed.updateSeed(mu, sigma2);
}

void test_depthestimation(){
    SE3 se3 = SE3();

    se3.Euler(M_PI/4, M_PI/8, M_PI/16, 1, 1, 1);

    PinholeCamera* pinhole = new PinholeCamera(1.0f, 1.0f, 320.f, 240.f, 640.0f, 480.f);

    DepthEstimation depthestimation = DepthEstimation(pinhole);

    Eigen::Vector3d ref, curr;
    ref << 0.3, 0.4, 0.2;
    curr<< 0.5, 0.5, 0.2;

    Eigen::Vector3d a = depthestimation.Triangulation(se3, ref, curr);
    double value = depthestimation.TriangulationUncertainty(se3, ref, a(2));
    cout<<"triangulation :"<<endl;
    cout<<a<<endl;
    cout<<"uncertainty triangulation : "<<value<<endl;
    cout<<endl;


}

int main(){
   //test_PinholeCamera();
   //test_se3();
   //test_seed();
   //test_depthestimation();

    Eigen::Matrix3d m;
    m << 1, 3, 1,
         1, 2, 2,
         1, 1, 3;

    Eigen::MatrixXd r,i,val,vec,vet;

    Eigen::EigenSolver<Eigen::Matrix3d> m_solve(m);
    r = m_solve.eigenvalues().real();
    i = m_solve.eigenvalues().imag();
    val = m_solve.pseudoEigenvalueMatrix().diagonal();
    //vec = m_solve.eigenvectors();
    vet = m_solve.pseudoEigenvectors();

    cout<<r.rows()<<" "<<r.cols()<<endl;
    cout<<i.rows()<<" "<<i.cols()<<endl;
    cout<<val.rows()<<" "<<val.cols()<<endl;
    //cout<<vec.rows()<<" "<<vec.cols()<<endl;
    cout<<vet.rows()<<" "<<vet.cols()<<endl;
    cout<<vet.col(0).transpose()<<endl;
    cout<<vet.col(1).transpose()<<endl;
    cout<<vet.col(2).transpose()<<endl;
    //A.inverse()
    //A.determinant()
    //a.transpose()
    Eigen::MatrixXd res = vet*(m_solve.pseudoEigenvalueMatrix())*vet.transpose();
    cout<<res<<endl;

   return 0;
}



