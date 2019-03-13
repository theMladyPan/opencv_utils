#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/opencv.hpp>
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
#include <QFileDialog>
#include <cmath>

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

    void on_buttonStart_clicked();

    void on_actionConnect_triggered();

    void on_actionDisconnect_triggered();

    void on_action_Zoom_in_triggered();

    void on_actionZoom_Out_triggered();

    void on_actionOpen_triggered();

    void on_actionStart_triggered();

private:
    Ui::MainWindow *ui;
    stringstream _message;
    int _camAvail = 0;
    bool _usingVirtual = false;
    bool _connected = false;
    bool _grab = false;
    bool _updateNeeded = false;
    Grabber _Grabber;
    ImagePtr _pLastImage = nullptr;
    cv::Mat *_pLastCvMat = new Mat();
    QImage *_pLastQImage = new QImage();
    double _scale = 1;
    Color *_pColor = new Color();
    vector<FileCamera> _FileCameras;
    FileCamera _FileCamera;
    float _avgWidth = 0;


    int _threshMethod = 0;
    int _threshType = 0;

    void showMessage(const stringstream &message);
    void showMessage(const string &message);
    void updateSetupCameraImage();
    void updateSetupThresholdImage();
    void updateMeasureImage();
    void ApplyThreshold(Mat &outArr);
    void ApplyGrid(Mat &outArr, const Scalar color=Scalar(255));
    void RefreshButtons();
};



#endif // MAINWINDOW_H
