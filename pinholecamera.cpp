#include "pinholecamera.h"

PinholeCamera::PinholeCamera(){
    fx = 0.0;
    fy = 0.0;
    cx = 0.0;
    cy = 0.0;

    width = 0;
    height = 0;

    K << 1.0, 0.0, 0.0,
         0.0, 1.0, 0.0,
         0.0, 0.0, 1.0;
}

PinholeCamera::PinholeCamera(double pfx, double pfy, double pcx, double pcy, double pwidth, double pheight){
    fx = pfx;
    fy = pfy;
    cx = pcx;
    cy = pcy;

    width = pwidth;
    height = pheight;

    K << fx , 0.0,  cx ,
         0.0,  fy ,  cy ,
         0.0, 0.0, 1.0;
}

Eigen::Vector3d PinholeCamera :: CamToWorld(const Eigen::Vector3d &pPixel){
    Eigen::Vector3d point;
    point(0) = (pPixel(0) - cx)/fx;
    point(1) = (pPixel(1) - cy)/fy;
    point(2) = 1.0;

    return point;
}

Eigen::Vector3d PinholeCamera :: CamToWorld(const Eigen::Vector3d &pPixel, double pDepth){
    Eigen::Vector3d point;
    point(0) = (pPixel(0) - cx)*pDepth/fx;
    point(1) = (pPixel(1) - cy)*pDepth/fy;
    point(2) = pDepth;

    return point;
}

Eigen::Vector3d PinholeCamera :: WorldToCam(const Eigen::Vector3d &pPoint){
    Eigen::Vector3d pixel;
    pixel(0) = (fx*pPoint(0)/pPoint(2)) + cx;
    pixel(1) = (fy*pPoint(1)/pPoint(2)) + cy;
    pixel(2) = 1.0;

    return pixel;
}


Eigen::Vector3d PinholeCamera :: Normalize(const Eigen::Vector3d &pPoint){
    Eigen::Vector3d point;
    point = pPoint/pPoint.norm();

    return point;
}

bool PinholeCamera :: isInFrame(Eigen::Vector3d &pPixel){

    bool check = pPixel(0) >= IMAGE_PADDING && pPixel(0) < width - IMAGE_PADDING &&
                 pPixel(1) >= IMAGE_PADDING && pPixel(1) < height - IMAGE_PADDING;

    return check;
}

double PinholeCamera :: ErrorPixelAngle(){
    // law of chord (sehnensatz)
    double pixelNoise = 1.0;
    double errorAngle = atan2f(pixelNoise, 2.0f*fx)*2.0;

    return errorAngle;
}

