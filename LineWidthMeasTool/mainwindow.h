#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <iostream>
#include "grabber.h"
#include <qstring.h>
#include <string>
#include <vector>
#include "tools.h"
#include <thread>
#include <Spinnaker.h>
#include <chrono>
#include <QTimer>

using namespace std;
using namespace cv;
using namespace Spinnaker;

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();

private slots:
    void on_actionExit_triggered();

    void on_buttonRefresh_clicked();

    void on_buttonConnect_clicked();

    void getAndUpdateImage();

    void on_comboBoxThresholdType_currentIndexChanged(int index);

    void on_comboBoxInverted_currentIndexChanged(int index);

    void on_actionRefresh_triggered();

private:
    Ui::MainWindow *ui;
    stringstream _message;
    int _camAvail = 0;
    bool _connected = false;
    Grabber _Grabber;
    ImagePtr _pLastImage = nullptr;
    cv::Mat *_pLastCvMat = new Mat();
    QImage *_pLastQImage = new QImage();

    int _threshMethod = 0;
    int _threshType = 0;

    void showMessage(const stringstream &message);
    void showMessage(const string &message);
    void updateSetupCameraImage();
    void updateSetupThresholdImage();
    void updateMeasureImage();
};



#endif // MAINWINDOW_H
