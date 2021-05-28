#include<iostream>
#include<fstream>
#include<iomanip>
#include<algorithm>
#include<chrono>

#include<opencv2/opencv.hpp>

#include<System.h>
#include<planefiltering.h>
#include<pcl/visualization/cloud_viewer.h>
#include<pcl/point_types.h>

using namespace std;

bool test_Live(string Vocabulary, string Configuration){

    cv::VideoCapture capture(0);
    if (!capture.isOpened()) capture.open("0");
    if (!capture.isOpened()){
            cerr << "Failed to open the video device!\n" << endl;
            return 1;
    }

    ORB_SLAM2::System SLAM(Vocabulary, Configuration,ORB_SLAM2::System::MONOCULAR,true);

    cout << endl << "-------" << endl;
    cout << "Start processing..." << endl;

    cv::Mat im;
    while(1){
        capture >> im;
        double tframe = capture.get(CV_CAP_PROP_POS_MSEC);

        if(im.empty()){
            cerr << endl << "Failed to load image"<< endl;
            return 1;
        }
    
        std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
        SLAM.TrackMonocular(im,tframe);
        std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();

        double ttrack= std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();
        usleep(ttrack*1e6);
    }

    SLAM.Shutdown();
    SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");

    return 1;
}

bool test_Monocular(vector< pair< float, pair<string, string> > > datos,string address, string Vocabulary, string Configuration){

    ORB_SLAM2::System SLAM(Vocabulary, Configuration,ORB_SLAM2::System::MONOCULAR,true);

    cout << endl << "-------" << endl;
    cout << "Start processing..." << endl;

    cv::Mat im;

    for(size_t i = 0; i < datos.size(); i++){
        string name = address + datos[i].second.first;
        double tframe = datos[i].first;

        im = cv::imread(name ,CV_LOAD_IMAGE_UNCHANGED);

        if(im.empty()){
            cerr << endl << "Failed to load image"<< endl;
            return 0;
        }

        std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
        SLAM.TrackMonocular(im,tframe);
        std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();

        double ttrack= std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();
        usleep(ttrack*1e6);
    }

    SLAM.Shutdown();
    SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");
    SLAM.SaveVisualOdometry("VisualOdometry.txt");

    return 1;
}


bool test_Monocular(vector< pair<float, string> > datos,string address, string Vocabulary, string Configuration){

    ORB_SLAM2::System SLAM(Vocabulary, Configuration,ORB_SLAM2::System::MONOCULAR,true);

    cout << endl << "-------" << endl;
    cout << "Start processing..." << endl;

    cv::Mat im;

    for(size_t i = 0; i < datos.size(); i++){
        string name = address + datos[i].second;
        double tframe = datos[i].first;

        im = cv::imread(name ,CV_LOAD_IMAGE_UNCHANGED);

        if(im.empty()){
            cerr << endl << "Failed to load image"<< endl;
            return 0;
        }

        std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
        SLAM.TrackMonocular(im,tframe);
        std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();

        double ttrack= std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();
        usleep(ttrack*1e6);
    }

    SLAM.Shutdown();
    SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");

    return 1;
}


bool test_RGBD(vector< pair< float, pair<string, string> > > datos,string address, string Vocabulary, string Configuration){

    ORB_SLAM2::System SLAM(Vocabulary, Configuration, ORB_SLAM2::System::RGBD,true);

    cout << endl << "-------" << endl;
    cout << "Start processing..." << endl;

    cv::Mat imRGB, imD;

    for(size_t i = 0; i < datos.size(); i++){
        string name1 = address + datos[i].second.first;
        string name2 = address + datos[i].second.second;
        double tframe = datos[i].first;

        imRGB = cv::imread(name1, CV_LOAD_IMAGE_UNCHANGED);
        imD   = cv::imread(name2, CV_LOAD_IMAGE_UNCHANGED);

        if(imRGB.empty() || imD.empty()){
            cerr << endl << "Failed to load image at: "<<endl;
            return 0;
        }

        std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
        SLAM.TrackRGBD(imRGB,imD,tframe);
        std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();

        double ttrack= std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();
        usleep(ttrack*1e6);
    }


    SLAM.Shutdown();
    //SLAM.SaveTrajectoryTUM("/home/kleiber/Desktop/VisionProject/lib/KeyFrameTrajectory.txt");
    SLAM.SaveTrajectoryTUM("CameraTrajectory.txt");

    return 1;
}

void test_Filter(vector< pair< float, pair<string, string> > > datos, string address){

    /*
    int w = 640;
    int h = 340;
    float fx = 204.83454216;
    float fy = 194.29848866;
    float cx = 150.50804407;
    float cy = 127.24407125;
    float minDepth = 8000;
    float maxDepth = 20000;

    int maxPoints = 10000;
    int numSamples = 200000;
    int numLocalSamples = 50;
    int planeSize = 60;
    float WorldPlaneSize = 2.0;
    float maxError = 0.03;
    float minInlierFraction = 0.80;
    float maxDepthDiff = 1.8;
    int numRetries = 2;

    cout << endl << "-------" << endl;
    cout << "Start processing..." << endl;

    ORB_SLAM2::PlaneFiltering *Object = new ORB_SLAM2::PlaneFiltering(w, h, fx, fy, cx, cy, minDepth, maxDepth, maxPoints, numSamples,
                                               numLocalSamples, planeSize, WorldPlaneSize,maxError,minInlierFraction,
                                               maxDepthDiff, numRetries);


    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1 (new pcl::PointCloud<pcl::PointXYZ>);

    pcl::visualization::PCLVisualizer viewer ("test");

    viewer.setBackgroundColor(0, 0, 0);
    viewer.addCoordinateSystem (1.0);
    viewer.addPointCloud<pcl::PointXYZ> (cloud1);

    int ind = 0;

    cv::namedWindow("Current Frame");

    while(!viewer.wasStopped() && ind < (int)datos.size()){

        string name1 = address + datos[ind].second.first;
        string name2 = address + datos[ind].second.second;

        cv::Mat im    = cv::imread(name1, CV_LOAD_IMAGE_UNCHANGED);
        cv::Mat depth = cv::imread(name2, CV_LOAD_IMAGE_UNCHANGED);
        depth.convertTo(depth, CV_32F);

        cloud->clear();
        cloud1->clear();
        ind++;

        for(int i = 0; i < depth.rows; i++){
            for(int j = 0; j < depth.cols; j++){
                Point2i p = Point2i(j,i);
                Point3f proj = Object->ConvertTo3D(p, depth);

                pcl::PointXYZ point;
                point.x = proj.x;
                point.y = proj.y;
                point.z = proj.z;
                cloud->points.push_back(point);
            }
        }

        vector<Point3f> pointcloud, normals, outliers;
        vector<Point2i> pixels;


        Object->filteredPointCloud(depth, pointcloud, pixels, normals, outliers);

        for(size_t i = 0; i < pointcloud.size(); i++){
            pcl::PointXYZ point;
            point.x = pointcloud[i].x;
            //point.y = 0;
            point.y = pointcloud[i].y;
            point.z = pointcloud[i].z;
            //if(pointcloud[i].y < 0)
            cloud1->push_back(point);
        }


        viewer.updatePointCloud(cloud1);
        viewer.spinOnce(100);
        cv::imshow("Current Frame",im);
        cv::waitKey(50);
    }
    */
}


int main(){
    int option = 3;
    string Vocabulary = "/home/kleiber/Desktop/VisionProject/Vocabulary/ORBvoc.txt";
    string Configuration = "/home/kleiber/Desktop/VisionProject/Configuration/Camera.yaml";
    string Address = "/home/kleiber/Desktop/DataSet/dataset14/";
    vector< pair< float, pair<string, string> > > imageList1;
    vector< pair<float,string> >imageList2;

    ifstream fin("/home/kleiber/Desktop/DataSet/dataset14/images.txt");


    if(option == 1){
        float time;
        string image;
        while(fin>>time>>image){
            imageList2.push_back(make_pair(time, image));
        }
    }

    if(option == 2){
        float time;
        string depth, image;
        while(fin>>time>>depth>>image){
            imageList1.push_back(make_pair(time, make_pair(image, depth)));
        }
    }

    if(option == 3){
        float time1, time2;
        string depth, image;
        while(fin>>time1>>depth>>time2>>image){
            imageList1.push_back(make_pair(time1, make_pair(image, depth)));
        }
    }


    fclose(stdin);

    //test_Live(Vocabulary, Configuration);
    test_Monocular(imageList1, Address,Vocabulary, Configuration);
    //test_Monocular(imageList2, Address,Vocabulary, Configuration); //LSD
    //test_RGBD(imageList1, Address, Vocabulary, Configuration);
    //test_Filter(imageList1, Address);

    return 0;
}

//965
//1240
//880

//1 living_room_near
//2 living_room_far
//3 living_room_loop
//14 teddy

//7

//12
//13
//14



