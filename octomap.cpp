#include<iostream>
#include<vector>
#include<fstream>
#include<octomap/OcTree.h>
#include <octomap/ColorOcTree.h>

using namespace std;

void buildMap(float discretization, vector<octomap::point3d> &pointCloud, string addressMapPlot, string dataset, int id, string method){

    octomap::OcTree* octree = new octomap::OcTree(discretization);

    for(size_t i = 0; i < pointCloud.size(); i++){
        octree->updateNode(pointCloud[i], true);
    }

    string nameOctree = method + "_map_" + dataset + "_" + to_string(id) + ".bt";
    string addressOctree = addressMapPlot + nameOctree;

    octree->writeBinary(addressOctree);
}

void buildColorMap(float discretization, vector<octomap::point3d> &pointCloud1, vector<octomap::point3d> &pointCloud2, string addressMapPlot, string dataset, int id1, int id2){

    octomap::ColorOcTree* octreeColor = new octomap::ColorOcTree(discretization);

    for(size_t i = 0; i < pointCloud1.size(); i++){
        octomap::ColorOcTreeNode* nodeColor = octreeColor->updateNode(pointCloud1[i], true);
        nodeColor->setColor(255,0,0);
    }

    for(size_t i = 0; i < pointCloud2.size(); i++){
        octomap::ColorOcTreeNode* nodeColor = octreeColor->updateNode(pointCloud2[i], true);
        if(!nodeColor->isColorSet()) nodeColor->setColor(0,255,0);
        else nodeColor->setColor(0,0,255);
    }

    string nameColorOctree = "colormap_" + dataset + "_" + to_string(id1) + "_" + to_string(id2) + ".ot";
    string addressColorOctree = addressMapPlot + nameColorOctree;

    octreeColor->write(addressColorOctree);
}

void readDatasets(vector<string> &dataset, string addressExperiments){
    dataset.clear();

    string datasetname;
    ifstream fin(addressExperiments);
    while(fin>>datasetname) dataset.push_back(datasetname);
    fclose(stdin);
}

void readHeadersMap(vector<string> &map, string addressHeadersMap){
    map.clear();

    string mapname;
    ifstream fin(addressHeadersMap);
    while(fin>>mapname) map.push_back(mapname);
    fclose(stdin);
}


void readPointCloud( vector<octomap::point3d> &cloudpoint, string addressPointCloud){
    cloudpoint.clear();

    ifstream fin(addressPointCloud);
    double x,y,z;
    while(fin>>x>>y>>z) cloudpoint.push_back(octomap::point3d(x, y, z));
}


int main(){

    float discretization = 0.01;

    string address = "/home/kleiber/Desktop/Experiments/";
    string experiments = "Datasets.txt";

    string addressExperiments = address + experiments;

    vector<string> dataset;
    readDatasets(dataset, addressExperiments);

    for(size_t i = 0; i < dataset.size(); i++){
        cout<<"Run dataset: "<<dataset[i]<<" ... done!"<<endl;

        string addressDataset = address + dataset[i] + "/";

        vector<string> headerMap1;
        vector<string> headerMap2;

        string addressHeaderMap1 = addressDataset + "test/ORB/Header/map_headers_" + dataset[i] +".txt";
        string addressHeaderMap2 = addressDataset + "test/FUS/Header/map_headers_" + dataset[i] + ".txt";

        readHeadersMap(headerMap1, addressHeaderMap1);
        readHeadersMap(headerMap2, addressHeaderMap2);

        vector<bool> isDraw1(headerMap1.size(), false);
        vector<bool> isDraw2(headerMap2.size(), false);

        vector<octomap::point3d> pointCloud1;
        vector<octomap::point3d> pointCloud2;

        string addressMapPlot = addressDataset + "test/PlotMap/";

        for(size_t j = 0 ; j < headerMap1.size(); j++){

            pointCloud1.clear();
            string addressPointCloud1 = addressDataset + "test/ORB/Map/" + headerMap1[j];
            readPointCloud(pointCloud1, addressPointCloud1);

            if(!isDraw1[j]){
                buildMap(discretization, pointCloud1, addressMapPlot, dataset[i] ,j, "ORB");
                isDraw1[j] = true;
            }

            for(size_t k = 0; k < headerMap2.size(); k++){

                pointCloud2.clear();
                string addressPointCloud2 = addressDataset + "test/FUS/Map/" + headerMap2[k];
                readPointCloud(pointCloud2, addressPointCloud2);

                if(!isDraw2[k]){
                    buildMap(discretization, pointCloud2, addressMapPlot, dataset[i], k, "FUS");
                    isDraw2[k] = true;
                }

                buildColorMap(discretization, pointCloud1, pointCloud2, addressMapPlot, dataset[i], j, k);
            }
        }
    }

    cout<<"Finish!"<<endl;

    return 0;
}
