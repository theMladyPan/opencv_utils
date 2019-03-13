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
        vector<string> camListAvailable = camAvail();
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
        cvtColor(*_pLastCvMat, originalColor, COLOR_GRAY2RGB);

        ApplyThreshold(outArr);
        ApplyGrid(outArr);

        rectangle(outArr, Rect(0,0, outArr.cols/4, outArr.rows), Scalar(255), FILLED);
        rectangle(outArr, Rect(0,0, outArr.cols, outArr.rows/4), Scalar(255), FILLED);
        rectangle(outArr, Rect(outArr.cols*3/4,0, outArr.cols, outArr.rows), Scalar(255), FILLED);
        rectangle(outArr, Rect(0,outArr.rows*3/4, outArr.cols, outArr.rows/4), Scalar(255), FILLED);
        rectangle(originalColor, Rect(outArr.cols/4, outArr.rows/4, outArr.cols/2, outArr.rows/2), Scalar(100, 200, 100), 3, LINE_AA);

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

        // get minimal size rectangles
        vector<float> widths;
        vector<RotatedRect> rectangles;
        for(auto contour: largeContours) {
            rectangles.push_back(minAreaRect(contour));
        }

        drawColorContours(originalColor, largeContours, hierarchy);
        // Draw rectangles
        _pColor->setup(0,50);
        for(auto rectangle:rectangles){
            Point2f rect_points[4]; rectangle.points( rect_points );
            auto randomColor = _pColor->randomColor();
            for( int j = 0; j < 4; j++ ){
                line( originalColor, rect_points[j], rect_points[(j+1)%4], randomColor, 2, LINE_AA );
            }
            stringstream center;
            float lesser(rectangle.size.width<rectangle.size.height?rectangle.size.width:rectangle.size.height);

            // convert pixels into mm, keep 2 decimals
            lesser = trunc(100*lesser/41.5) / 100;
            widths.push_back(lesser);
            center<<lesser<<"mm";
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
        _message << _avgWidth;
        ui->editAvgWidth->setText(QString::fromStdString(_message.str()));
        auto lQImage = QImage((const unsigned char*)(originalColor.data), originalColor.cols, originalColor.rows, QImage::Format_RGB888).scaledToWidth(originalColor.cols*_scale);

        ui->labelMeasure->resize(originalColor.cols, originalColor.rows);
        ui->labelMeasure->setPixmap(QPixmap::fromImage(lQImage));
        _updateNeeded = false;
    }else{
        // limit to 30 fps. sleep for 20ms.
        this_thread::sleep_for(chrono::milliseconds(20));
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
    _scale *= 1.2;
}

void MainWindow::on_actionZoom_Out_triggered()
{
    _scale /= 1.2;
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
