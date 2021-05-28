#include<iostream>
#include<fstream>
#include<iomanip>
#include<algorithm>
#include<chrono>

using namespace std;

class datos{
    public:
    double a1,a2,a3,a4,a5,a6,a7,a8;
    datos(double _a1, double _a2, double _a3, double _a4, double _a5, double _a6, double _a7, double _a8){
    a1 = _a1;
    a2 = _a2;
    a3 = _a3;
    a4 = _a4;
    a5 = _a5;
    a6 = _a6;
    a7 = _a7;
    a8 = _a8;
    }
};

void readData(vector<datos> &Data, string addressData){
    double a1,a2,a3,a4,a5,a6,a7,a8;
    ifstream fin(addressData);
    while(fin>>a1>>a2>>a3>>a4>>a5>>a6>>a7>>a8) Data.push_back(datos(a1,a2,a3,a4,a5,a6,a7,a8));
    fclose(stdin);
}

void readAssociations(vector< pair<double,string> > &associations, string addressAssociations){
    double timergb;
    string rgb;
    ifstream fin(addressAssociations);
    while(fin>>timergb>>rgb) associations.push_back(make_pair(timergb, rgb));
    fclose(stdin);
}

void CopyTimeStandTrajectory(){
    string address = "/home/kleiber/Desktop/Experiments/dataset_living_room_near/";
    string fileCopy = "groundtruth.txt";
    string fileOriginal = "groundtruth_.txt";
    string filename = "text1.txt";

    string addressFileCopy = address + fileCopy;
    string addressFileOriginal = address + fileOriginal;
    string addressFilename = address + filename;

    vector<datos> DataCopy;
    readData(DataCopy, addressFileCopy);

    vector<datos> DataOriginal;
    readData(DataOriginal, addressFileOriginal);

    ofstream f;
    f << fixed;
    f.open(addressFilename.c_str());

    for(size_t i = 0; i < DataOriginal.size(); i++){
        f<<setprecision(7)<<DataCopy[i].a1<<setprecision(8)<<" "<<DataOriginal[i].a2<<" "<<DataOriginal[i].a3<<" "<<DataOriginal[i].a4<<" "<<DataOriginal[i].a5<<" "<<DataOriginal[i].a6<<" "<<DataOriginal[i].a7<<" "<<DataOriginal[i].a8<<endl;
    }
    f.close();
}

void CopyTimeStandImage(){
    string address = "/home/kleiber/Desktop/Experiments/dataset_living_room_near/";
    string fileCopy = "rgb.txt";
    string fileOriginal = "rgb_.txt";
    string filename = "text2.txt";

    string addressFileCopy = address + fileCopy;
    string addressFileOriginal = address + fileOriginal;
    string addressFilename = address + filename;

    vector< pair<double,string> > AssociationsCopy;
    readAssociations(AssociationsCopy, addressFileCopy);

    vector< pair<double,string> > AssociationsOriginal;
    readAssociations(AssociationsOriginal, addressFileOriginal);

    ofstream f;
    f << fixed;
    f.open(addressFilename.c_str());
    for(size_t i = 0; i < AssociationsOriginal.size(); i++){
        f<<setprecision(7)<<AssociationsCopy[i].first<<" "<<AssociationsOriginal[i].second<<endl;
    }
    f.close();
}

int main(){
    CopyTimeStandImage();
    CopyTimeStandTrajectory();
    return 0;
}
