#include "seed.h"
Seed :: Seed(){
}

Seed :: Seed(Eigen::Vector3d &pPixel, double pdepthMean, double pdepthMin){
    initSeed(pPixel, pdepthMean, pdepthMin);
    //initNCC();
}

void Seed :: initSeed(Eigen::Vector3d &pPixel, double pdepthMean, double pdepthMin){

    pixel = pPixel;
    range = 1.0/pdepthMin;

    mu = 1.0/pdepthMean;
    sigma2 = range*range/36.0;
    a = 10.0;
    b = 10.0;

    inlier = 0.7;
    outlier = 0.05;
    convergence_sigma2_thresh = range/200.0;
}

void Seed :: updateSeed(double pmu, double psigma2){

    if(std::isnan(sigma2 + psigma2)) return;

    boost::math::normal_distribution<double> gaussian(mu, sigma2 + psigma2);
    double pdf_gaussian = boost::math::pdf(gaussian, pmu);

    double fusion_mu = (mu*psigma2 + pmu*sigma2)/(sigma2 + psigma2);
    double fusion_sigma2 = (sigma2*psigma2)/(sigma2 + psigma2);

    double c1 = a/(a + b) * pdf_gaussian;
    double c2 = b/(a + b) * (1.0/range);

    double normalize = c1 + c2;
    c1 = c1/normalize;
    c2 = c2/normalize;

    double v1 = c1*(a+1.0)/(a+b+1.0) + c2*a/(a+b+1.0);
    double v2 = c1*(a+1.0)*(a+2.0)/((a+b+1.0)*(a+b+2.0)) + c2*a*(a+1.0)/((a+b+1.0)*(a+b+2.0));

    double new_mu = c1*fusion_mu + c2*mu;
    double new_sigma2 = c1 *(fusion_sigma2 + fusion_mu*fusion_mu) + c2*(sigma2 + mu*mu) - new_mu*new_mu;
    double new_a = (v2 - v1)/(v1 - v2/v1);
    double new_b = new_a*(1.0 - v1)/v1;

    mu = new_mu;
    sigma2 = new_sigma2;
    a = new_a;
    b = new_b;
}

void Seed :: checkSeed(){
    double mu_beta = a/(a + b);
    double mode_beta = (a - 1.0)/(a + b - 2.0);

    if(mu_beta > inlier && sigma() < convergence_sigma2_thresh){
        state = SEED_CONVERGED;
    }else{
        if(mode_beta < outlier){
            state = SEED_DIVERGED;
        }else{
            state = SEED_UPDATE;
        }
    }
}

void Seed ::initNCC(){

    for(int v = 0; v < SIZE_PATCH; v++){
        for(int u = 0; u < SIZE_PATCH; u++){
            double templ = 0.0; //img_ref(u,v) with center in pixel_ref
            sumTemplate  += templ;
            sumTemplate2 += templ*templ;
        }
    }
    denTemplate = (double)AREA_PATCH*sumTemplate2 - sumTemplate*sumTemplate;
}

double Seed :: sigma(){
    return  sqrtf(sigma2);
}
