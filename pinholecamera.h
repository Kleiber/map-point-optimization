#ifndef PINHOLECAMERA_H
#define PINHOLECAMERA_H

#include <Eigen/Dense>
#include <se3.h>

#define IMAGE_PADDING 10

class PinholeCamera{
    public:

        PinholeCamera();
        PinholeCamera(double pfx, double pfy, double pcx, double pcy, double pwidth, double pheigth);

        Eigen::Vector3d CamToWorld(const Eigen::Vector3d &pPixel);
        Eigen::Vector3d CamToWorld(const Eigen::Vector3d &pPixel, double pDepth);
        Eigen::Vector3d WorldToCam(const Eigen::Vector3d &pPoint);
        Eigen::Vector3d Normalize(const Eigen::Vector3d &pPoint);
        bool isInFrame(Eigen::Vector3d &pPixel);
        double ErrorPixelAngle();

        double fx;
        double fy;
        double cx;
        double cy;

        Eigen::Matrix3d K;

        double width;
        double height;
};

#endif // PINHOLECAMERA_H
