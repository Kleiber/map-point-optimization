#include<iostream>
#include<vector>
#include<fstream>
#include<thread>
#include<Eigen/Dense>

#include<visualizer.h>

typedef Eigen::Matrix<double, 3, 4> Matrix34d;

using namespace std;

void load_Poses(vector<cv::Mat> &pPoses){

    double r00, r01, r02, r10, r11, r12, r20, r21, r22, t1, t2, t3;

    ifstream fin("/home/kleiber/Desktop/DataSet/dataset3/Poses.txt");

    while(fin>>r00>>r01>>r02>>t1>>r10>>r11>>r12>>t2>>r20>>r21>>r22>>t3){
        cv::Mat pose = cv::Mat::eye(4, 4, CV_32F);

        pose.at<float>(0,0) = r00;
        pose.at<float>(0,1) = r01;
        pose.at<float>(0,2) = r02;

        pose.at<float>(1,0) = r10;
        pose.at<float>(1,1) = r11;
        pose.at<float>(1,2) = r12;

        pose.at<float>(2,0) = r20;
        pose.at<float>(2,1) = r21;
        pose.at<float>(2,2) = r22;

        pose.at<float>(0,3) = t1;
        pose.at<float>(1,3) = t2;
        pose.at<float>(2,3) = t3;

        pPoses.push_back(pose);
    }
}

void load_Images(vector< pair< double, pair<string, string> > > &pImages){

    string img, depth;
    double time;

    ifstream fin("/home/kleiber/Desktop/DataSet/dataset3/images.txt");

    while(fin>>time>>img>>depth){
        pImages.push_back(make_pair(time, make_pair(depth, img)));
    }
}

int main(){

    vector< pair< double, pair<string, string> > > images;
    vector<cv::Mat>poses;

    //-- Images
    load_Images(images);
    //-- Poses
    load_Poses(poses);

    //Camera model
    PinholeCamera *Pinhole = new PinholeCamera(535.4, 539.2, 320.1, 247.6, 640, 480);

    //visualizer
    VisualizerFrame* FrameDrawer = new VisualizerFrame(images);
    VisualizerMap* MapDrawer = new VisualizerMap(poses, Pinhole);
    Visualizer* Viewer = new Visualizer(FrameDrawer, MapDrawer);

    //Thread Vizualizer
    std::thread* threadViewer = new std::thread(&Visualizer::Run, Viewer);
    threadViewer->join();


    return 0;
}
