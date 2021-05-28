#include "visualizerframe.h"

VisualizerFrame :: VisualizerFrame(vector< pair< double, pair<string, string> > > &pImageList){
    image = cv::Mat(480,640,CV_8UC3, cv::Scalar(0,0,0));
    a = pImageList;
}

cv::Mat VisualizerFrame :: DrawFrame(){

    string address = "/home/kleiber/Desktop/DataSet/dataset3/";
    string pathImage = address + a[cont].second.first;
    image = cv::imread(pathImage, CV_LOAD_IMAGE_UNCHANGED);

    cv::Mat img = image;
    if(img.channels() < 3) cvtColor(img, img, CV_GRAY2BGR);

    cv::Mat imgInfo;
    DrawInfo(img, imgInfo);

    return imgInfo;
}

void VisualizerFrame :: DrawInfo(cv::Mat &pImg, cv::Mat &pImgText){
    stringstream text;

    text << "imagen puede estar mal";

    int baseLine = 0;
    cv::Size size = cv::getTextSize(text.str(), cv::FONT_HERSHEY_PLAIN, 1, 1, &baseLine);
    pImgText = cv::Mat(pImg.rows + size.height + 10, pImg.cols, pImg.type());
    pImg.copyTo(pImgText.rowRange(0, pImg.rows).colRange(0, pImg.cols));
    pImgText.rowRange(pImg.rows, pImgText.rows) = cv::Mat::zeros(size.height + 10, pImg.cols, pImg.type());
    cv::putText(pImgText, text.str(), cv::Point(5, pImgText.rows - 5), cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(255, 255, 255), 1, 8);
}
