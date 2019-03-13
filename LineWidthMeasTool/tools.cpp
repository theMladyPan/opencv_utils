#include "tools.h"


QStringList getQStringList(const std::vector<std::string> &strVector)
{
    QStringList out;
    for(auto str:strVector){
        out.append(QString::fromStdString(str));
    }
    return out;
}

cv::Mat toCvArray(Spinnaker::ImagePtr image)
{
  const unsigned int XPadding = image->GetXPadding();
  const unsigned int YPadding = image->GetYPadding();
  const unsigned int rowsize = image->GetWidth();
  const unsigned int colsize = image->GetHeight();

  //image data contains padding. When allocating Mat container size, you need to account for the X,Y image data padding.

 return cv::Mat(colsize + YPadding, rowsize + XPadding, CV_8UC1, image->GetData(), image->GetStride());

}

Color::Color()
{
    rng = RNG(rand()%10000);
}

Color Color::setup(const int min, const int max)
{
    this->min = min;
    this->max = max;
    return *this;
}

Scalar Color::randomColor()
{
    return Scalar(rng.uniform(min,max), rng.uniform(min,max), rng.uniform(min,max));
}

Scalar Color::randomMono()
{
    return Scalar(rng.uniform(min,max));
}

Scalar Color::getColor(const int color)
{
    return Scalar(color/(256*256), (color%(256*256))/256, color%256);
}

int showCvMat(string name, Mat &inpArr, int delay_ms){
    namedWindow(name, WINDOW_NORMAL);       // Create a window for display.
    imshow(name, inpArr);                   // Show our image inside it.
    return waitKey(delay_ms);
}

void drawColorContours(const Mat &destArray, const vector<contour> &contours, const vector<Vec4i> &hierarchy){
    for( size_t i = 0; i< contours.size(); i++ )
    {
        auto color = Color().randomColor();
        drawContours( destArray, contours, (int)i, color, 1, LINE_AA, hierarchy, 0);
    }
}

void drawColorContours(const Mat &destArray, const vector<contour> &contours, const vector<Vec4i> &hierarchy, const Scalar color)
{
    for( size_t i = 0; i< contours.size(); i++ ){
        drawContours( destArray, contours, (int)i, color, 1, LINE_AA, hierarchy, 0);
    }
}

void drawPointVector(Mat &image, const pointVector &points, const Scalar &color){
    for(auto point:points){
        image.at<Scalar>(point.x, point.y) = color;
    }
}


void filterContours(const vector<contour> &original, vector<contour> &filtered, const long unsigned int minSize, const long unsigned int maxSize){
    for(auto vec=original.begin(); vec!=original.end(); vec++){
        if(vec->size()>minSize && vec->size() < maxSize){
              filtered.push_back(*vec);
          }
    }
}

FileCamera::FileCamera(const string name)
{
    this->name = QString::fromStdString( name.substr(name.rfind("/")+1, name.size() - name.rfind("/")-1));
    this->directory = QString::fromStdString(name);
}

string FileCamera::getName()
{
    return name.toStdString();
}

string FileCamera::getNextFile()
{
    if(_imagesInDir.isEmpty()){
        _imagesInDir = QDir(directory).entryList(QStringList("*.png"));
    }
    if(_imagesInDir.isEmpty()){
        return "";
    }
    string toReturn = directory.toStdString() + "/" + _imagesInDir.last().toStdString();
    _imagesInDir.pop_back();
    return toReturn;
}
