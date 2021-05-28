#ifndef SEED_H
#define SEED_H

#include <Eigen/Dense>
#include <boost/math/distributions/normal.hpp>

#define SEED_CONVERGED 1
#define SEED_DIVERGED  0
#define SEED_UPDATE    2

#define SIZE_PATCH 5
#define AREA_PATCH SIZE_PATCH * SIZE_PATCH

class Seed{
    public:
        Seed();
        Seed(Eigen::Vector3d &pPixel, double pdepthMean, double pdepthMin);

        void initSeed(Eigen::Vector3d &pPixel, double pdepthMean, double pdepthMin);
        void updateSeed(double pmu, double psigma);
        void checkSeed();

        void initNCC();
        double sigma();

        Eigen::Vector3d pixel_match;
        Eigen::Vector3d pixel;
        double range;

        double a;
        double b;
        double mu;
        double sigma2;

        double sumTemplate;
        double sumTemplate2;
        double denTemplate;

        int state;

        double inlier;
        double outlier;
        double convergence_sigma2_thresh;
};

#endif // SEED_H
