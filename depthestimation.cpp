#include "depthestimation.h"

DepthEstimation :: DepthEstimation(PinholeCamera* pPinhole){
    Pinhole = pPinhole;
}

bool DepthEstimation :: EpipolarMatchNCC(SE3 &pT_curr_ref, Seed &pSeed, Eigen::Vector3d &pbestPixel){
    Eigen::Vector3d beam_ref = Pinhole->CamToWorld(pSeed.pixel);
    beam_ref = Pinhole->Normalize(beam_ref);

    Eigen::Vector3d beam_mu_curr  = pT_curr_ref  * (beam_ref * pSeed.mu);
    Eigen::Vector3d beam_min_curr = pT_curr_ref * (beam_ref * (pSeed.mu - std::fmax(3.0*pSeed.sigma(), 0.01)));
    Eigen::Vector3d beam_max_curr = pT_curr_ref * (beam_ref * (pSeed.mu + 3.0*pSeed.sigma()));

    Eigen::Vector3d pixel_mu  = Pinhole->WorldToCam(beam_mu_curr);
    Eigen::Vector3d pixel_min = Pinhole->WorldToCam(beam_min_curr);
    Eigen::Vector3d pixel_max = Pinhole->WorldToCam(beam_max_curr);


    Eigen::Vector3d epipolarLine = pixel_max - pixel_min;
    Eigen::Vector3d epipolarDirection = Pinhole->Normalize(epipolarLine);
    double epipolarNorm = epipolarLine.norm();
    double epipolarLenght = 0.5*std::fmin(epipolarNorm,MAX_LENGTH_EPIPOLAR_LINE);

    double bestNCC = -1.0;

    for(double i = -epipolarLenght; i <= epipolarLenght; i += STEP_EPIPOLAR_LINE){

        Eigen::Vector3d pixel_curr = pixel_mu + i*epipolarDirection;
        if(!Pinhole->isInFrame(pixel_curr)) continue;

        double sumImage  = 0.0;
        double sumImage2 = 0.0;
        double sumImageTemplate = 0.0;

        for(int v = 0; v < SIZE_PATCH; v++){
            for(int u = 0; u < SIZE_PATCH; u++){

                double templ = 0.0;//img_ref(u,v) with center in pixel_curr
                double img = 0.0;//img_curr(u,v) with center in pixel_ref
                sumImage  += img;
                sumImage2 += img*img;
                sumImageTemplate+= img*templ;
            }
        }

        double numeratorNCC = (double)AREA_PATCH*sumImageTemplate - sumImage*pSeed.sumTemplate;
        double denominatorNCC = ((double)AREA_PATCH*sumImage2 - sumImage*sumImage)*pSeed.denTemplate;
        double NCC = numeratorNCC/sqrtf(denominatorNCC);

        if(NCC > bestNCC){
            bestNCC = NCC;
            pbestPixel = pixel_curr;
        }
    }

    if(bestNCC > 0.5) return true;
    else return false;
}

Eigen::Vector3d DepthEstimation :: Triangulation(SE3 &pT_ref_curr, Eigen::Vector3d &pbeam_ref, Eigen::Vector3d &pbeam_curr){

    Eigen::Vector3d t = pT_ref_curr.getTraslation();
    Eigen::Vector3d beam_curr = pT_ref_curr.rotate(pbeam_curr);

    Eigen::Vector2d b;
    b(0) = t.dot(pbeam_ref);
    b(1) =-t.dot(beam_curr);

    Eigen::Matrix2d A;
    A(0,0) = pbeam_curr.dot(beam_curr);
    A(0,1) = pbeam_ref.dot(beam_curr);
    A(1,0) = pbeam_ref.dot(beam_curr);
    A(1,1) = pbeam_ref.dot(pbeam_ref);

    Eigen::Vector2d lambda = (A * b)/A.determinant();

    Eigen::Vector3d xm = lambda(0) * pbeam_ref;
    Eigen::Vector3d xn = t + lambda(1) * beam_curr;

    Eigen::Vector3d point = (xm + xn)/2.0;

    return point;
}

double DepthEstimation :: TriangulationUncertainty(SE3 &pT_ref_curr, Eigen::Vector3d &pbeam_ref, double pdepth){

    Eigen::Vector3d t = pT_ref_curr.getTraslation();
    Eigen::Vector3d beam_curr = pbeam_ref * pdepth - t;

    double alpha = acos( t.dot(pbeam_ref)/t.norm());
    double beta  = acos(-t.dot(beam_curr)/(t.norm()*beam_curr.norm()));

    double beta_plus  = beta + Pinhole->ErrorPixelAngle();
    double theta = M_PI - alpha - beta_plus;
    double depth = t.norm()*sin(beta_plus)/sin(theta);

    return depth - pdepth;
}


void DepthEstimation ::initKeyFrameSeeds(ORB_SLAM2::KeyFrame *Frame_ref){
    //-- inicilaizar la cantidad de seeds
    conta_Seed = 0;
    total_Seed = 10000;

    //-- recuperar puntos para inicializar

    vector<ORB_SLAM2::MapPoint*> &pointList = Frame_ref->mvpMapPoints;

    Eigen::Vector3d pixel;
    double depthMean = 0.0;;
    double depthMin = std::numeric_limits<double>::max();
    int cont = 0;

    for(size_t i = 0; i < pointList.size(); i++){
        cv::Mat pos = pointList[i]->GetWorldPos();

        depthMin = fmin(depthMin, pos.at<float>(2));
        depthMean += pos.at<float>(2);
        cont++;
    }

    //-- verificar si existe puntos antes de inicializar

    if(cont != 0){
        depthMean = depthMean/cont;

        for(size_t i = 0; i < pointList.size(); i++){
            cv::Mat pos = pointList[i]->GetWorldPos();

            pixel(0) = pos.at<float>(0);
            pixel(1) = pos.at<float>(1);
            pixel(2) = pos.at<float>(2);

            pixel = Pinhole->WorldToCam(pixel);

            SeedList.push_back(Seed(pixel, depthMean, depthMin));
            conta_Seed++;
        }
    }
}

void DepthEstimation :: updateKeyFrameSeeds(ORB_SLAM2::KeyFrame *Frame_curr){

/*
    PinholeCamera pPinhole;
    SE3 T_f_w;

    std::list<Seed> :: iterator it =  SeedList.begin();

    while(it != SeedList.end()){
        SE3 T_ref_curr;
        SE3 T_curr_ref;

        Eigen::Vector3d point =  T_curr_ref * (1.0/it->mu * it->pixel);
        if(point(2) < 0.0){
            it++;
            continue;
        }

        Eigen::Vector3d pixel = pPinhole.WorldToCam(point);
        if(!pPinhole.isInFrame(pixe)){
            it++;
            continue;
        }

        Eigen::Vector3d match;
        bool isMatch = EpipolarMatchNCC(T_curr_ref, pPinhole, it, match);

        if(!isMatch){
            it->b++;
            it++;
            continue;
        }

        Eigen::Vector3d a = Triangulation(T_ref_curr, it->pixel, match);
        double depth = a(2);
        double pixelErrorAngle = pPinhole.ErrorPixelAngle();
        double tau = TriangulationUncertainty(T_ref_curr, it->pixel, depth, pixelErrorAngle);
        double tau_inverse = 0.5*(1.0/fmax(0.0000001, depth - tau) - 1.0/(depth + tau));

        it->updateSeed(1.0/depth, tau_inverse * tau_inverse);
        it->checkSeed();

        if(it->state == SEED_CONVERGED){
            Eigen::Vector3d world = T_w_f * (1.0/it->mu * it->pixel); //add map
            it = SeedList.erase(it);
        }else{
            if(it->state == SEED_DIVERGED || std::isnan(it->mu)) it = SeedList.erase(it);
            else{
                if(it->state == SEED_UPDATE) it++;
            }
        }
    }*/
}
