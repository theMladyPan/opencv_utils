#ifndef TOOLS_H
#define TOOLS_H

#include <vector>
#include <string>
#include <QStringList>
#include <Spinnaker.h>
#include <opencv2/opencv.hpp>

QStringList getQStringList(const std::vector<std::string> &strVector);
cv::Mat toCvArray(Spinnaker::ImagePtr image);

#endif // TOOLS_H
