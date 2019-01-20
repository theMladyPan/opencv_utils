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

using namespace std;
using namespace cv;

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

private:
    Ui::MainWindow *ui;
    stringstream _message;
    int _camAvail = 0;

    void showMessage(const stringstream &message);
    void showMessage(const string &message);
};



#endif // MAINWINDOW_H
