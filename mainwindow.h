#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <opencv2/opencv.hpp>
#include "QFileDialog"
#include <iostream>
#include <sstream>
#include <string>

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

private slots:
    void on_actionOpen_triggered();

    void on_actionExit_triggered();

    void on_methodComboBox_currentIndexChanged(int index);

    void on_typeComboBox_currentIndexChanged(int index);

    void on_dialBlockSize_valueChanged(int value);

    void on_dialOffset_valueChanged(int value);

    void on_actionZoom_In_triggered();

    void on_actionZoom_Out_triggered();

    void on_action_Save_triggered();

    void on_caricatureCheckBox_toggled(bool checked);

    void on_caricatureLevelSlider_valueChanged(int value);

    void on_blurCheckBox_toggled(bool checked);

    void on_blurLevelSlider_valueChanged(int value);

private:
    // parameters
    cv::Mat *_srcImg = new cv::Mat();
    cv::Mat *_destImg = new cv::Mat();
    int _adaptiveMethod = cv::ADAPTIVE_THRESH_MEAN_C;
    int _thresholdType = cv::THRESH_BINARY;
    int _blocksize = 3;
    int _offset = 0;
    double _scale = 1;
    double _caricatureLevel = .5;
    int _blur_level = 1;

    //methods
    void updateImage();
    Ui::MainWindow *ui;
    void loadMat(const QString &fileName);
};

#endif // MAINWINDOW_H
