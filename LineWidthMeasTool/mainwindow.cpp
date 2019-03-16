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
    ui->editDestination->setText(QString::fromStdString(_Saver.getDestination()));
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
    int physicalCams = nOfCamAvail();
    _camAvail = physicalCams;
    _camAvail+=_FileCameras.size();
    ui->progressBarSetupCamera->setValue(10);
    _message.str(string());
    _message << "Number of cameras found: " << _camAvail<<", updating list...";
    showMessage(_message);
    _message.str(string());
    _message << "Available cameras ("<<_camAvail<<")";
    ui->labelAvailableCameras->setText(QString::fromStdString(_message.str()));

    ui->comboBoxCameraList->clear();
    if(_camAvail>0){
        // some cameras were found
        vector<string> camListAvailable;
        if(physicalCams>0){
            camListAvailable= camAvail();
        }
        vector<string> camList;
        for(auto FileCamera:_FileCameras){
            camList.push_back("VIRTUAL-"+FileCamera.getName());
        }
        for(auto cam:camListAvailable){
            camList.push_back(cam);
        }
        ui->progressBarSetupCamera->setValue(33);
        auto qCamList = getQStringList(camList);
        ui->comboBoxCameraList->insertItems(0, qCamList);
    }else{
        ui->comboBoxCameraList->insertItem(0, QString::fromStdString(" - "));
    }

    RefreshButtons();

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
        Mat originalColor = Mat(outArr.rows, outArr.cols, CV_8UC3);
        _pLastCvMat->copyTo(outArr);

        if(_record){
            _Saver.saveRaw(outArr);
        }

        cvtColor(*_pLastCvMat, originalColor, COLOR_GRAY2RGB);

        ApplyThreshold(outArr);
        ApplyGrid(outArr);

        rectangle(outArr, Rect(0,0, outArr.cols/4, outArr.rows), Scalar(255), FILLED);
        rectangle(outArr, Rect(0,0, outArr.cols, outArr.rows/4), Scalar(255), FILLED);
        rectangle(outArr, Rect(outArr.cols*3/4,0, outArr.cols, outArr.rows), Scalar(255), FILLED);
        rectangle(outArr, Rect(0,outArr.rows*3/4, outArr.cols, outArr.rows/4), Scalar(255), FILLED);

        //rectangle(originalColor, Rect(outArr.cols/4+2, outArr.rows/4+2, outArr.cols/2+2, outArr.rows/2+2), Scalar(127, 127, 127), 3, LINE_AA);
        rectangle(originalColor, Rect(outArr.cols/4+1, outArr.rows/4+1, outArr.cols/2+1, outArr.rows/2+1), Scalar(0, 0, 0), 3, LINE_AA);
        rectangle(originalColor, Rect(outArr.cols/4, outArr.rows/4, outArr.cols/2, outArr.rows/2), Scalar(100, 255, 100), 3, LINE_AA);

        vector<contour> contours;
        vector<Vec4i> hierarchy;
        findContours(outArr, contours, hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE);
        vector<vector<Point>> largeContours;

        for(auto vec=contours.begin(); vec!=contours.end(); vec++){
            double contourSize = contourArea(*vec);
            if(vec->size() > 4 && contourSize > static_cast<long unsigned int>(ui->spinBoxBlobSize->value())){ // replace this with tiddlitoddly
                largeContours.push_back(*vec);
            }
        }
        drawColorContours(originalColor, largeContours, hierarchy);

        _message.str(string());
        _message << largeContours.size();
        ui->editBlobsFound->setText(QString::fromStdString(_message.str()));

        // get minimal size rectangles
        vector<float> widths;
        for(auto contour: largeContours) {
            auto rectangle = minAreaRect(contour);
            Point2f rect_points[4]; rectangle.points( rect_points );
            auto randomColor = _pColor->randomColor();
            for( int j = 0; j < 4; j++ ){
                line( originalColor, Point(rect_points[j].x+1, rect_points[j].y+1), Point(rect_points[(j+1)%4].x+1, rect_points[(j+1)%4].y+1), Scalar(0,0,0), 2, LINE_AA);
                line( originalColor, rect_points[j], rect_points[(j+1)%4], randomColor, 2, LINE_AA);
            }
            stringstream center;
            float lesser(rectangle.size.width<rectangle.size.height?rectangle.size.width:rectangle.size.height);

            // get area of contour and rectangle
            double areaContour = contourArea(contour);
            double areaRectangle = rectangle.size.width * rectangle.size.height;
            double ratio = areaContour / areaRectangle;

            // calculate relative part of width acc. to ratio of areas
            lesser = lesser * ratio;
            // convert pixels into mm, keep 2 decimals
            lesser = trunc(100*lesser/(_calibrationConstant * _calibrationFine)) / 100;

            widths.push_back(lesser);
            center<<lesser<<"mm";
            putText(originalColor, center.str(), Point(rectangle.center.x+1, rectangle.center.y+1), FONT_HERSHEY_COMPLEX, 1.5, Scalar(0,0,0), 2, LINE_AA);
            putText(originalColor, center.str(), rectangle.center, FONT_HERSHEY_COMPLEX, 1.5, randomColor, 2, LINE_AA);

        }
        float avgWidth = 0;
        int n=0;
        for(auto width:widths){
            avgWidth += width;
            n++;
        }
        avgWidth = avgWidth / n;
        if(_avgWidth<0 or _avgWidth!= _avgWidth){
            _avgWidth = 0;
        }
        if(_avgWidth==0){
            _avgWidth = avgWidth;
        }else{
            _avgWidth = (_avgWidth * 0.9) + (avgWidth * 0.1);
        }
        _message.str("");
        _avgWidth = trunc(1000*_avgWidth)/1000;
        _message << _avgWidth<<"mm";
        ui->editAvgWidth->setText(QString::fromStdString(_message.str()));

        // calculate averge deviation
        /***
         * X=mean(xi), i=<1,n>
         *
         * sqrt( sum((xi-X)^2)/(n-1) )
         *
         */

        vector<double> deviations;
        for(auto width:widths){
            deviations.push_back(avgWidth - width);
        }

        n=0;
        double sumDev = 0;
        double meanDeviation;
        for(auto deviation:deviations){
            sumDev += deviation;
            n++;
        }
        meanDeviation = sumDev / n;

        double sumSquaredPwrs = 0;
        for(auto deviation:deviations){
            sumSquaredPwrs += pow(deviation - meanDeviation, 2);;
        }

        double standardDev;
        standardDev = sqrt( sumSquaredPwrs /n); // in mm
        int accuracy = 100 - (100*standardDev/_avgWidth);

        ui->barAccuracy->setValue(accuracy);

        // write out to user
        _message.str("");
        _message << trunc(standardDev*10000)/10 << "um";
        ui->editInaccuracy->setText(QString::fromStdString(_message.str()));

        // Put additional informations into picture
        stampMat(originalColor, avgWidth, static_cast<double>(largeContours.size()), standardDev);

        // if gui record enabled, save image with GUI.
        if(_record && _recordGui){
            _Saver.saveGui(originalColor);
        }

        //finally show the picture
        auto lQImage = QImage((const unsigned char*)(originalColor.data), originalColor.cols, originalColor.rows, QImage::Format_RGB888).scaledToWidth(originalColor.cols*_scale);

        ui->labelMeasure->resize(originalColor.cols, originalColor.rows);
        ui->labelMeasure->setPixmap(QPixmap::fromImage(lQImage));
        _updateNeeded = false;

    }else{
        // limit to 33 fps. sleep for 30ms.
        this_thread::sleep_for(chrono::milliseconds(30));
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
    /*for(int col=0; col<outArr.cols; col+=ui->dialGridSize->value()){
        line(outArr, Point(col, 0), Point(col, outArr.rows), color, 2);
    }*/
}

void MainWindow::RefreshButtons()
{
    if(_camAvail>0){
        ui->buttonConnect->setEnabled(true);
        ui->actionConnect->setEnabled(true);
    }else{
        ui->buttonConnect->setEnabled(false);
        ui->actionConnect->setEnabled(false);
    }
}

void MainWindow::getAndUpdateImage()
{
    if(_usingVirtual && _grab){
        // simulate grabbing an image
        Mat loaded = imread(_FileCamera.getNextFile());

        cvtColor(loaded, *_pLastCvMat, COLOR_RGB2GRAY);
        this_thread::sleep_for(chrono::milliseconds(30));
        _updateNeeded = true;
    }else{
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
        }
    }
    if(ui->tabCameraSetup->isVisible() and _grab){
        updateSetupCameraImage();
    }else if(ui->tabThreshSetup->isVisible()){
        updateSetupThresholdImage();
    }else if(ui->tabMeasure->isVisible()){
        updateMeasureImage();
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
        ui->buttonConnect->setEnabled(false);
        ui->progressBarSetupCamera->setValue(100);
    }else{
        _grab = false;
        showMessage("Acquisition stopped");
        ui->buttonStart->setText("Start");
        ui->buttonConnect->setEnabled(true);
        ui->progressBarSetupCamera->setValue(70);
    }
}

void MainWindow::on_actionConnect_triggered()
{

    ui->progressBarSetupCamera->setValue(70);
    string camSerial = ui->comboBoxCameraList->currentText().toStdString();
    // check if we are about to connect to virtual camera
    if(camSerial.find("VIRTUAL")==string::npos){
        _Grabber = Grabber(camSerial);
        _Grabber.init();
        _connected = true;
        _usingVirtual = false;
    }else{
        _usingVirtual = true;
        _FileCamera = _FileCameras.at(ui->comboBoxCameraList->currentIndex());
        _message.str("");
        _message<<"Using: " << _FileCamera.getName();
        showMessage(_message);
    }

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
    _scale *= 1.1;
}

void MainWindow::on_actionZoom_Out_triggered()
{
    _scale /= 1.1;
}

void MainWindow::on_actionOpen_triggered()
{
    // fake camera from folder
    QString dirWithPictures = QFileDialog::getExistingDirectory(this, tr("Select directory with pictures"));
    _FileCameras.push_back(FileCamera(dirWithPictures.toStdString()));

    ui->tabCameraSetup->setEnabled(false);

    _camAvail += 1;

    vector<string> camList;
    for(auto FileCamera:_FileCameras){
        camList.push_back("VIRTUAL-"+FileCamera.getName());
    }
    ui->progressBarSetupCamera->setValue(33);
    auto qCamList = getQStringList(camList);

    ui->comboBoxCameraList->insertItems(static_cast<int>(ui->comboBoxCameraList->size().height()), qCamList);

    RefreshButtons();

    ui->progressBarSetupCamera->setValue(50);
    showMessage("List updated");
    ui->tabCameraSetup->setEnabled(true);
}

void MainWindow::on_actionStart_triggered()
{
    on_buttonStart_clicked();
}

void MainWindow::on_actionWith_GUI_triggered()
{
    _recordGui = ui->actionWith_GUI->isChecked();
}

void MainWindow::on_actionDestination_triggered()
{
    QString dirToSave = QFileDialog::getExistingDirectory(this, tr("Select directory where pictures will be saved"));
    _Saver.setDestination(dirToSave.toStdString());
    ui->editDestination->setText(dirToSave);
}

void MainWindow::on_actionRecord_triggered()
{
    _record = ui->actionRecord->isChecked();
    auto text = QString();
    if(_record){
        text = QString("Measure 🔴");
    }else{
        text = QString("Measure");
    }
    ui->TabPane->setTabText(2, text);
}

void MainWindow::on_actionReset_triggered()
{
    _scale = 1;
}

void MainWindow::on_sliderFineCalibration_valueChanged(int value)
{
    _calibrationFine = value / 1000.0;
}
