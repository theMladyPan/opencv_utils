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
        auto color = Color().setup(150, 255).randomColor();
        drawContours( destArray, contours, (int)i, color, 1, LINE_4, hierarchy, 0);
    }
}

void drawColorContours(const Mat &destArray, const vector<contour> &contours, const vector<Vec4i> &hierarchy, const Scalar color)
{
    for( size_t i = 0; i< contours.size(); i++ ){
        drawContours( destArray, contours, (int)i, color, 1, LINE_4, hierarchy, 0);
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

Saver::Saver(const string &destination)
{
    _destination = destination;
}

void Saver::setDestination(const string &dest)
{
    _destination = dest;
}

void Saver::saveRaw(const Mat &Arr)
{
    stringstream msg;
    msg.str("");
    msg << _destination<< "/" << "raw_" << setfill('0') << setw(5)<< ++_counter << ".png";
    string text = msg.str();
    // TODO Check if directory exists
    imwrite(msg.str(), Arr);
}

void Saver::saveGui(const Mat &Arr)
{
    stringstream msg;
    msg.str("");
    msg << _destination<< "/" << "gui_" << setfill('0') << setw(5)<< _counter << ".png";

    imwrite(msg.str(), Arr);
}

string Saver::getDestination()
{
    return _destination;
}

void stampMat(Mat &inOutArr, double width, double blobs, double stdDev)
{
    stringstream text;
    int ratio = (stdDev*100 / width);
    width =  trunc(1000*width) / 1000;
    stdDev =  trunc(10000*stdDev) / 10000;
    text.str("");
    text << "Average width: " << width << "mm, s" << stdDev*1000 << "um (" << ratio << "%)";
    putText(inOutArr, text.str(), Point(22,inOutArr.rows - 8),FONT_HERSHEY_COMPLEX, 1.5, Scalar(127,127,127),2, LINE_AA);
    putText(inOutArr, text.str(), Point(20,inOutArr.rows - 10),FONT_HERSHEY_COMPLEX, 1.5, Scalar(0,0,0),2, LINE_AA);
    putText(inOutArr, text.str(), Point(19,inOutArr.rows - 11),FONT_HERSHEY_COMPLEX, 1.5, Scalar(255,255,255),2, LINE_AA);
    text.str("");
    text<< "Blobs found: " << blobs;
    putText(inOutArr, text.str(), Point(22,inOutArr.rows-68),FONT_HERSHEY_COMPLEX, 1.5, Scalar(127,127,127),2, LINE_AA);
    putText(inOutArr, text.str(), Point(20,inOutArr.rows-70),FONT_HERSHEY_COMPLEX, 1.5, Scalar(0,0,0),2, LINE_AA);
    putText(inOutArr, text.str(), Point(19,inOutArr.rows-71),FONT_HERSHEY_COMPLEX, 1.5, Scalar(255,255,255),2, LINE_AA);
}
