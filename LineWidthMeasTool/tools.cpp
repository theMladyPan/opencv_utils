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
