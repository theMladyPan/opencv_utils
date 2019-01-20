#include "mainwindow.h"
#include "ui_mainwindow.h"

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    QTimer *timer = new QTimer(this);
    connect(timer, SIGNAL(timeout()), this, SLOT(getAndUpdateImage()));
    timer->start(30);

    ui->setupUi(this);
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
    ui->progressBarSetupCamera->setValue(33);
    _message.str(string());
    _message << "Number of cameras found: " << _camAvail<<", updating list...";
    showMessage(_message);
    _message.str(string());
    _message << "Available cameras ("<<_camAvail<<")";
    ui->labelAvailableCameras->setText(QString::fromStdString(_message.str()));

    if(_camAvail>0){
        // some cameras were found
        vector<string> camList = camAvail();
        ui->progressBarSetupCamera->setValue(90);
        auto qCamList = getQStringList(camList);
        ui->comboBoxCameraList->clear();
        ui->comboBoxCameraList->insertItems(0, qCamList);
        ui->buttonConnect->setEnabled(true);
    }else{
        ui->comboBoxCameraList->clear();
        ui->comboBoxCameraList->insertItem(0, QString::fromStdString(" - "));
        ui->buttonConnect->setEnabled(false);
    }
    ui->progressBarSetupCamera->setValue(100);
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
    *_pLastQImage = QImage((const unsigned char*)(_pLastCvMat->data), _pLastCvMat->cols, _pLastCvMat->rows, QImage::Format_Grayscale8);
    ui->labelImageSetupCamera->resize(_pLastCvMat->cols, _pLastCvMat->rows);
    ui->labelImageSetupCamera->setPixmap(QPixmap::fromImage(*_pLastQImage));
}

void MainWindow::updateSetupThresholdImage()
{
    //apply threshold according to parameters
    Mat outArr;
    if(_threshMethod == 0){
        // binary threshold
        threshold(*_pLastCvMat, outArr, ui->spinBoxOffset->value(), 255, _threshType);

    }else{
        // adaptive threshold
        adaptiveThreshold(*_pLastCvMat, outArr, 255, ADAPTIVE_THRESH_MEAN_C, _threshType, ui->spinBoxArea->value(), ui->spinBoxOffset->value());
    }
    *_pLastQImage = QImage((const unsigned char*)(outArr.data), outArr.cols, outArr.rows, QImage::Format_Grayscale8);
    for(int row=0; row<outArr.rows; row+=ui->dialGridSize->value()){
        line(outArr, Point(0,row), Point(outArr.cols, row), Scalar(150), 2);
    }
    for(int col=0; col<outArr.cols; col+=ui->dialGridSize->value()){
        line(outArr, Point(col, 0), Point(col, outArr.rows), Scalar(150), 2);
    }
    rectangle(outArr, Rect(outArr.rows/4, outArr.cols/4, outArr.rows/2, outArr.cols/2), Scalar(100), 3, LINE_AA);
    ui->labelImageSetupThreshold->resize(_pLastCvMat->cols, _pLastCvMat->rows);
    ui->labelImageSetupThreshold->setPixmap(QPixmap::fromImage(*_pLastQImage));
}

void MainWindow::updateMeasureImage()
{
    // TODO
}

void MainWindow::getAndUpdateImage()
{
    if(_connected){
        _Grabber.start();
        _pLastImage = _Grabber.getResult();
        *_pLastCvMat = toCvArray(_pLastImage);
    }

    if(ui->tabCameraSetup->isVisible())
    {
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
        string camSerial = ui->comboBoxCameraList->currentText().toStdString();
        _Grabber = Grabber(camSerial);
        _connected = true;

        ui->buttonConnect->setText("Disconnect");
        ui->tabThreshSetup->setEnabled(true);
        ui->tabMeasure->setEnabled(true);
    }else{
        on_buttonRefresh_clicked();
        _Grabber.free();
        _connected = false;

        ui->buttonConnect->setText("Connect");
        ui->tabThreshSetup->setEnabled(false);
        ui->tabMeasure->setEnabled(false);
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
