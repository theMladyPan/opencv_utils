#include <iostream>
#include <opencv2/opencv.hpp>
#include <boost/filesystem.hpp>
#include <filesystem>

const int blobMinSize = 500;
const int blobMaxSize = 1000000;
const int blobThreshold = 4000;

using namespace std;

bool sizeBetweenMinMax(double size){
    if(size <= blobMaxSize && size >= blobMinSize){
        return true;
    }else{
        return false;
    }
}

cv::Point contourCenter(vector<cv::Point> contour){
    double x(0);
    double y(0);
    int n(0);
    for(auto point:contour){
        x += point.x;
        y += point.y;
        n++;
    }
    return cv::Point(x/n, y/n);
}

void drawColorContours(const cv::Mat &destArray, const vector<vector<cv::Point>> &contours, const vector<cv::Vec4i> &hierarchy, const cv::Scalar color)
{
    for( size_t i = 0; i< contours.size(); i++ ){
        drawContours(destArray, contours, (int)i, color, 2, cv::LINE_4, hierarchy, 0);
    }
}

bool inspectLightOff(const std::string filename){
    cv::Mat inpArr;
    cv::namedWindow("Main", cv::WINDOW_NORMAL);
    inpArr = cv::imread(filename, cv::IMREAD_GRAYSCALE);

    cv::Mat outArr = cv::Mat(inpArr.rows, inpArr.cols, CV_8UC3);
    inpArr.copyTo(outArr);

    cv::threshold(inpArr, outArr, 254, 255, cv::THRESH_BINARY_INV);

    vector<vector<cv::Point>> contours;
    vector<cv::Vec4i> hierarchy;
    cv::findContours(outArr, contours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);
    int blobSize;

    for(auto contour:contours){
        blobSize = cv::contourArea(contour);
        if(sizeBetweenMinMax(blobSize)){
            cv::putText(inpArr, /*std::to_string(blobSize)*/blobSize>blobThreshold?"OK":"NOK", contourCenter(contour), cv::FONT_HERSHEY_PLAIN, 2, cv::Scalar(127), 2, cv::LINE_AA,false);
        }
    }
    drawColorContours(inpArr, contours, hierarchy, cv::Scalar(127));


    std::stringstream outFileName;
    outFileName << filename << "_result.png";
    cv::imshow("Main", inpArr);
    cv::waitKey();
    //cv::imwrite(outFileName.str(), inpArr);
    return true;
}

int main(int argc, char ** argv)
{
    if(argc != 2){
        cerr << "Wrong number of arguments\n";
        cerr << "Usage: './DrillHoleInspector folder/with/pictures\n\n";
        return EXIT_FAILURE;
    }

    std::string path(argv[1]);
    boost::filesystem::path fsPath(path);
    boost::filesystem::directory_iterator start(fsPath);

    while(start != boost::filesystem::directory_iterator{}){
        auto file = *start;
        inspectLightOff(file.path().string());
        start++;
    }


    return EXIT_SUCCESS;
}
