#include "mainwindow.h"
#include "ui_mainwindow.h"

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    QTimer *timer = new QTimer(this);
    connect(timer, SIGNAL(timeout()), this, SLOT(getAndUpdateImage()));
    timer->start(50);

    ui->setupUi(this);
    ui->tabMeasure->setEnabled(false);
    ui->tabThreshSetup->setEnabled(false);
}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::on_actionExit_triggered()
{
    qApp->closeAllWindows();
}

void MainWindow::on_buttonRefresh_clicked()
{
    ui->tabCameraSetup->setEnabled(false);
    ui->progressBarSetupCamera->setValue(0);
    _message.str("");
    _message << "Reloading camera list...";
    showMessage(_message);
    _camAvail = nOfCamAvail();
    ui->progressBarSetupCamera->setValue(10);
    _message.str(string());
    _message << "Number of cameras found: " << _camAvail<<", updating list...";
    showMessage(_message);
    _message.str(string());
    _message << "Available cameras ("<<_camAvail<<")";
    ui->labelAvailableCameras->setText(QString::fromStdString(_message.str()));

    if(_camAvail>0){
        // some cameras were found
        vector<string> camList = camAvail();
        ui->progressBarSetupCamera->setValue(33);
        auto qCamList = getQStringList(camList);
        ui->comboBoxCameraList->clear();
        ui->comboBoxCameraList->insertItems(0, qCamList);
        ui->buttonConnect->setEnabled(true);
        ui->actionConnect->setEnabled(true);
    }else{
        ui->comboBoxCameraList->clear();
        ui->comboBoxCameraList->insertItem(0, QString::fromStdString(" - "));
        ui->buttonConnect->setEnabled(false);
        ui->actionConnect->setEnabled(false);
    }
    ui->progressBarSetupCamera->setValue(50);
    showMessage("List updated");
    ui->tabCameraSetup->setEnabled(true);
}

void MainWindow::showMessage(const stringstream &message)
{
    showMessage(message.str());
}

void MainWindow::showMessage(const string &message)
{
    ui->statusBar->clearMessage();
    ui->statusBar->showMessage(QString::fromStdString(message));
}

void MainWindow::updateSetupCameraImage()
{
    Mat outArr(*_pLastCvMat);
    *_pLastQImage = QImage((const unsigned char*)(outArr.data), outArr.cols, outArr.rows, QImage::Format_Grayscale8).scaledToWidth(outArr.cols*_scale);
    ui->labelImageSetupCamera->resize(outArr.cols, outArr.rows);
    ui->labelImageSetupCamera->setPixmap(QPixmap::fromImage(*_pLastQImage));
}

void MainWindow::updateSetupThresholdImage()
{
    //apply threshold according to parameters
    Mat outArr;
    ApplyThreshold(outArr);

    if(ui->checkBoxShowGrid->isChecked()){
        ApplyGrid(outArr, Scalar(150));
    }
    if(ui->checkBoxShowRoi->isChecked()){
        rectangle(outArr, Rect(outArr.cols/4, outArr.rows/4, outArr.cols/2, outArr.rows/2), Scalar(100), 3, LINE_AA);
    }

    *_pLastQImage = QImage((const unsigned char*)(outArr.data), outArr.cols, outArr.rows, QImage::Format_Grayscale8).scaledToWidth(outArr.cols*_scale);
    ui->labelImageSetupThreshold->resize(outArr.cols, outArr.rows);
    ui->labelImageSetupThreshold->setPixmap(QPixmap::fromImage(*_pLastQImage));
}

void MainWindow::updateMeasureImage()
{
    if(_updateNeeded){
        Mat outArr;
        _pLastCvMat->copyTo(outArr);
        Mat imatcolor = Mat(outArr.rows, outArr.cols, CV_8UC3);

        ApplyThreshold(outArr);
        ApplyGrid(outArr);
        rectangle(outArr, Rect(0,0, imatcolor.cols, imatcolor.rows), Scalar(255));

        rectangle(outArr, Rect(0,0, outArr.cols/4, outArr.rows), Scalar(255), FILLED);
        rectangle(outArr, Rect(0,0, outArr.cols, outArr.rows/4), Scalar(255), FILLED);
        rectangle(outArr, Rect(outArr.cols*3/4,0, outArr.cols, outArr.rows), Scalar(255), FILLED);
        rectangle(outArr, Rect(0,outArr.rows*3/4, outArr.cols, outArr.rows/4), Scalar(255), FILLED);

        vector<contour> contours;
        vector<Vec4i> hierarchy;
        findContours(outArr, contours, hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE);
        vector<vector<Point>> largeContours;

        for(auto vec=contours.begin(); vec!=contours.end(); vec++){
            if(vec->size() > static_cast<long unsigned int>(ui->spinBoxBlobSize->value())){ // replace this with tiddlitoddly
                largeContours.push_back(*vec);
            }
        }

        _message.str(string());
        _message << largeContours.size();
        ui->editBlobsFound->setText(QString::fromStdString(_message.str()));

        cvtColor(outArr, imatcolor,COLOR_GRAY2RGB);

        // get minimal size rectangles
        vector<float> widths;
        vector<RotatedRect> rectangles;
        for(auto contour: largeContours) {
            rectangles.push_back(minAreaRect(contour));
        }

        // TODO: Add weighed...

        drawColorContours(imatcolor, largeContours, hierarchy);
        // Draw rectangles
        for(auto rectangle:rectangles){
            Point2f rect_points[4]; rectangle.points( rect_points );
            auto randomColor = _pColor->randomColor();
            for( int j = 0; j < 4; j++ ){
              line( imatcolor, rect_points[j], rect_points[(j+1)%4], randomColor, 1, LINE_AA );
            }
            stringstream center;
            float lesser(rectangle.size.width<rectangle.size.height?rectangle.size.width:rectangle.size.height);
            widths.push_back(lesser);
            center<<lesser<<"pix";
            putText(imatcolor, center.str(), rectangle.center, FONT_HERSHEY_COMPLEX, 1, randomColor, 1, LINE_AA);
        }
        float avgWidth = 0;
        int n=0;
        for(auto width:widths){
            avgWidth += width;
            n++;
        }
        avgWidth = avgWidth / n;

        _message.str(string());
        _message << avgWidth;
        ui->editAvgWidth->setText(QString::fromStdString(_message.str()));

        auto lQImage = QImage((const unsigned char*)(imatcolor.data), imatcolor.cols, imatcolor.rows, QImage::Format_RGB888).scaledToWidth(imatcolor.cols*_scale);
        ui->labelMeasure->resize(imatcolor.cols, imatcolor.rows);
        ui->labelMeasure->setPixmap(QPixmap::fromImage(lQImage));
        _updateNeeded = false;
    }

}

void MainWindow::ApplyThreshold(Mat &outArr)
{
    if(_threshMethod == 0){
        // binary threshold
        threshold(*_pLastCvMat, outArr, ui->spinBoxOffset->value(), 255, _threshType);

    }else{
        // adaptive threshold
        int area = ui->spinBoxArea->value();
        if(area%2==0){
            area += 1;
        }
        if (area<3){
            area = 3;
        }
        adaptiveThreshold(*_pLastCvMat, outArr, 255, ADAPTIVE_THRESH_MEAN_C, _threshType, area, ui->spinBoxOffset->value());
    }
}

void MainWindow::ApplyGrid(Mat &outArr, const Scalar color)
{
    for(int row=0; row<outArr.rows; row+=ui->dialGridSize->value()){
        line(outArr, Point(0,row), Point(outArr.cols, row), color, 2);
    }
    for(int col=0; col<outArr.cols; col+=ui->dialGridSize->value()){
        line(outArr, Point(col, 0), Point(col, outArr.rows), color, 2);
    }
}

void MainWindow::getAndUpdateImage()
{
    if(_connected){
        if(_grab){
            _Grabber.start();
            _pLastImage = _Grabber.getResult();
            while(_pLastImage->IsIncomplete()){
                _pLastImage = _Grabber.getResult();
            }
            toCvArray(_pLastImage).copyTo(*_pLastCvMat);
            _updateNeeded = true;
        }else{
            _Grabber.stop();
        }

        if(ui->tabCameraSetup->isVisible() and _grab)
        {
            updateSetupCameraImage();
        }else if(ui->tabThreshSetup->isVisible()){
            updateSetupThresholdImage();
        }else if(ui->tabMeasure->isVisible()){
            updateMeasureImage();
        }
    }
}

void MainWindow::on_buttonConnect_clicked()
{
    if(!_connected){
        on_actionConnect_triggered();

    }else{
        on_actionDisconnect_triggered();
    }
}

void MainWindow::on_comboBoxThresholdType_currentIndexChanged(int index)
{
    _threshMethod = index;
    if(index){
        ui->spinBoxArea->setEnabled(true);
        ui->horizontalSliderArea->setEnabled(true);
    }else{
        ui->spinBoxArea->setEnabled(false);
        ui->horizontalSliderArea->setEnabled(false);
    }
}

void MainWindow::on_comboBoxInverted_currentIndexChanged(int index)
{
    _threshType = index;
}

void MainWindow::on_actionRefresh_triggered()
{
    on_buttonRefresh_clicked();
}

void MainWindow::on_buttonStart_clicked()
{
    if(!_grab){
        _grab = true;
        showMessage("Acquisition started");
        ui->buttonStart->setText("Stop");
        ui->progressBarSetupCamera->setValue(100);
    }else{
        _grab = false;
        showMessage("Acquisition stopped");
        ui->buttonStart->setText("Start");
        ui->progressBarSetupCamera->setValue(70);
    }
}

void MainWindow::on_actionConnect_triggered()
{
    ui->progressBarSetupCamera->setValue(70);
    string camSerial = ui->comboBoxCameraList->currentText().toStdString();
    _Grabber = Grabber(camSerial);
    _Grabber.init();
    _connected = true;

    showMessage("Connected");
    ui->actionDisconnect->setEnabled(true);
    ui->actionConnect->setEnabled(true);
    ui->buttonConnect->setText("Disconnect");
    ui->tabThreshSetup->setEnabled(true);
    ui->tabMeasure->setEnabled(true);
    ui->buttonStart->setEnabled(true);
    ui->progressBarSetupCamera->setValue(90);
}

void MainWindow::on_actionDisconnect_triggered()
{
    _Grabber.free();
    _connected = false;
    // on_buttonRefresh_clicked();

    showMessage("Disconnected");
    ui->buttonConnect->setText("Connect");
    ui->tabThreshSetup->setEnabled(false);
    ui->tabMeasure->setEnabled(false);
    ui->buttonStart->setEnabled(false);
}

void MainWindow::on_action_Zoom_in_triggered()
{
    _scale *= 1.2;
}

void MainWindow::on_actionZoom_Out_triggered()
{
    _scale /= 1.2;
}
