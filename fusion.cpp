#include<iostream>
#include<fstream>
#include<iomanip>
#include<algorithm>
#include<chrono>

#include<opencv2/opencv.hpp>
#include<System.h>

using namespace std;
void saveHeadersTrajectory(vector<string> &trajectories, string filename){
    cout<<"           Saving trajectory headers ... ";

    ofstream f;
    f.open(filename.c_str());
    f << fixed;

    for(size_t i = 0; i < trajectories.size(); i++){
        f<<trajectories[i]<<endl;
    }

    f.close();
    cout<<"saved!" << endl;
}

void saveHeadersOdometry(vector<string> &odometries, string filename){
    cout<<"           Saving trajectory headers ... ";

    ofstream f;
    f.open(filename.c_str());
    f << fixed;

    for(size_t i = 0; i < odometries.size(); i++){
        f<<odometries[i]<<endl;
    }

    f.close();
    cout<<"saved!" << endl;
}

void saveHeadersMap(vector<string> &map, string filename){
    cout<<"           Saving Map headers ... ";

    ofstream f;
    f.open(filename.c_str());
    f << fixed;

    for(size_t i = 0; i < map.size(); i++){
        f<<map[i]<<endl;
    }

    f.close();
    cout<<"saved!" << endl;
}

void saveHeadersInfo(vector<string> &info, string filename){
    cout<<"           Saving Information headers ... ";

    ofstream f;
    f.open(filename.c_str());
    f << fixed;

    for(size_t i = 0; i < info.size(); i++){
        f<<info[i]<<endl;
    }

    f.close();
    cout<<"saved!" << endl;
}

void readDatasets(vector<string> &dataset, string addressExperiments){
    string datasetname;
    ifstream fin(addressExperiments);
    while(fin>>datasetname) dataset.push_back(datasetname);
    fclose(stdin);
}

void readAssociations(vector< pair<double,string> > &associations, string addressAssociations){
    double timergb;
    string rgb;
    ifstream fin(addressAssociations);
    while(fin>>timergb>>rgb) associations.push_back(make_pair(timergb, rgb));
    fclose(stdin);
}


bool Monocular(){
    string address = "/home/kleiber/Desktop/Experiments/";
    string experiments = "Datasets.txt";
    string addressVocabulary = "/home/kleiber/Desktop/VisionProject/Vocabulary/ORBvoc.txt";
    string addressConfiguration = "/home/kleiber/Desktop/VisionProject/Configuration/Camera.yaml";

    ORB_SLAM2::System SLAM(addressVocabulary, addressConfiguration, ORB_SLAM2::System::MONOCULAR, true);
    cout <<"Start processing..."<<endl<<endl;

    string addressExperiments = address + experiments;

    vector<string> dataset;
    readDatasets(dataset, addressExperiments);

    for(size_t i = 0; i < dataset.size(); i++){

        std::cout<<i<<" ----- TESTING DATASET : "<<dataset[i]<<" ..."<<endl;

        string addressDataset = address + dataset[i] + "/";
        string addressCameraConfiguration = addressDataset + "Camera.yaml";
        string addressAssociations = addressDataset + "rgb.txt";

        vector< pair<double,string> > associations;
        readAssociations(associations, addressAssociations);

        SLAM.ChangeCalibration(addressCameraConfiguration);

        vector<string> trajectory;
        vector<string> odometry;
        vector<string> map;
        vector<string> info;

        for(size_t test = 0; test < 10; test++){

            cv::Mat image;

            for(size_t j = 0; j < associations.size(); j++){

                string addressImage = addressDataset + associations[j].second;
                double timeframe = associations[j].first;

                image = cv::imread(addressImage ,CV_LOAD_IMAGE_UNCHANGED);

                if(image.empty()){
                    cerr<<endl<<"Failed to load image"<<endl;
                    return 0;
                }

                std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
                SLAM.TrackMonocular(image,timeframe);
                std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();

                double ttrack= std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();
                usleep(ttrack*1e6);
            }

            string headertrajectory = "traj_" + dataset[i] + "_" + to_string(test) + ".txt";
            string headerodometry = "odom_" + dataset[i] + "_" + to_string(test) + ".txt";
            string headermap = "pc_" + dataset[i] + "_" + to_string(test) + ".txt";
            string headerinfo = "inf_" + dataset[i] + "_" + to_string(test) + ".txt";

            trajectory.push_back(headertrajectory);
            odometry.push_back(headerodometry);
            map.push_back(headermap);
            info.push_back(headerinfo);

            string addressTrajectory = addressDataset + "test/FUS/Trajectory/" + headertrajectory;
            string addressOdometry = addressDataset + "test/FUS/Odometry/" + headerodometry;
            string addressPointCloud = addressDataset + "test/FUS/Map/" + headermap;
            string addressInformation = addressDataset + "test/FUS/Information/" + headerinfo;

            SLAM.SaveKeyFrameTrajectory(addressTrajectory);
            SLAM.SaveVisualOdometry(addressOdometry);
            SLAM.saveCloudPoint(addressPointCloud);
            SLAM.saveOtherInformations(addressInformation);

            SLAM.Reset();
            usleep(5000);
        }

        string addressHeadersTrajectory = addressDataset + "test/FUS/Header/traj_headers_" + dataset[i] + ".txt";
        string addressHeadersOdometry = addressDataset + "test/FUS/Header/odom_headers_" + dataset[i] + ".txt";
        string addressHeadersMap = addressDataset + "test/FUS/Header/map_headers_" + dataset[i] + ".txt";
        string addressHeadersInfo = addressDataset + "test/FUS/Header/info_headers_" + dataset[i] + ".txt";

        saveHeadersTrajectory(trajectory, addressHeadersTrajectory);
        saveHeadersOdometry(odometry, addressHeadersOdometry);
        saveHeadersMap(map, addressHeadersMap);
        saveHeadersInfo(info, addressHeadersInfo);
    }

    SLAM.Shutdown();
    return 1;
}

int main(){
    Monocular();
    return 0;
}
