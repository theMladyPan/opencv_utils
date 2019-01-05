#include "mainwindow.h"
#include "ui_mainwindow.h"

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);
}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::on_actionOpen_triggered()
{
    using namespace std;
    QString fileName = QFileDialog::getOpenFileName(this, tr("Open Image"), "", tr("JPEG(*.jpg *.jpeg *.JPG *.JPEG);;PNG(*.png *.PNG);;GIF animation(*.gif *.GIF);;All Files (*)"));
    stringstream message;
    if(fileName.size()>0){
        _scale=1;
        message << "File: " << fileName.toStdString() << " opened.";
        ui->srcImageInput->clear();
        ui->srcImageInput->insert(fileName);
        loadMat(fileName);
        updateImage();
        int diagonal = static_cast<int>(sqrt(pow(_srcImg->cols, 2)+ pow(_srcImg->rows, 2))/10);
        ui->dialBlockSize->setMaximum(diagonal);
    }else{
        message << "No file selected!";
    }
    ui->statusBar->clearMessage();
    ui->statusBar->showMessage(QString::fromStdString(message.str()));
}

void MainWindow::loadMat(const QString &fileName){
    assert(fileName.size()>0&&"File name must not be empty");

    *_srcImg = cv::imread(fileName.toStdString(), CV_8UC1);
}

void MainWindow::on_actionExit_triggered()
{
    qApp->closeAllWindows();
}

void MainWindow::updateImage()
{
    if(!_srcImg->empty()){

        cv::adaptiveThreshold(*_srcImg, *_destImg, 255, _adaptiveMethod, _thresholdType, _blocksize, _offset);

        if(ui->blurCheckBox->isChecked()){
            cv::Mat tempDst;
            // cv::bilateralFilter(*_destImg, tempDst, _blur_level, _blur_level*2, _blur_level/2);
            // tempDst.copyTo(*_destImg);
            cv::GaussianBlur(*_destImg, *_destImg, cv::Size(), _blur_level);
        }
        if(ui->caricatureCheckBox->isChecked()){
            cv::addWeighted(*_destImg, _caricatureLevel, *_srcImg, 1-_caricatureLevel, 0, *_destImg);
        }

        if(_scale!=1.0){
            cv::resize(*_destImg, *_destImg, cv::Size(), _scale, _scale, cv::INTER_LINEAR);
        }

        QImage result = QImage((const unsigned char*)(_destImg->data),
                               _destImg->cols, _destImg->rows, QImage::Format_Grayscale8);
        ui->label_image->setPixmap(QPixmap::fromImage(result));
        ui->label_image->resize(_destImg->cols, _destImg->rows);
    }
}



void MainWindow::on_methodComboBox_currentIndexChanged(int index)
{
    assert(index>=0 && index<2);
    _adaptiveMethod = index;
    updateImage();
}

void MainWindow::on_typeComboBox_currentIndexChanged(int index)
{
    assert(index>=0 && index < 2);

    _thresholdType = index;
    updateImage();
}

void MainWindow::on_dialBlockSize_valueChanged(int value)
{
    if(value%2==0){
        value++; // make value odd
    }
    if(value<3){
        value=3;
    }

    ui->label_block_size->clear();
    std::stringstream message;
    message << tr("Smoothness (block size): ").toStdString() << value << "\n";
    ui->label_block_size->setText(QString::fromStdString(message.str()));
    _blocksize = value;
    updateImage();
}

void MainWindow::on_dialOffset_valueChanged(int value)
{
    ui->label_offset->clear();
    std::stringstream message;
    message << tr("Offset: ").toStdString() << value << "\n";
    ui->label_offset->setText(QString::fromStdString(message.str()));
    _offset = value;
    updateImage();

}

void MainWindow::on_actionZoom_In_triggered()
{
    _scale = _scale * 2;
    updateImage();
}

void MainWindow::on_actionZoom_Out_triggered()
{
    _scale = _scale / 2;
    updateImage();
}

void MainWindow::on_action_Save_triggered()
{
    QString fileName = QFileDialog::getSaveFileName(this, tr("Save image as..."), QString("~/"));
    if(fileName.length()>0){
        cv::imwrite(fileName.toStdString(), *_destImg);
    }
    std::stringstream message;
    message << "File "<<fileName.toStdString() << " saved successfully";
    ui->statusBar->clearMessage();
    ui->statusBar->showMessage(QString::fromStdString(message.str()));
}

void MainWindow::on_caricatureCheckBox_toggled(bool checked)
{
    ui->caricatureLevelSlider->setEnabled(checked);
    updateImage();
}

void MainWindow::on_caricatureLevelSlider_valueChanged(int value)
{
    std::stringstream message;
    message << tr("Caricature (level): ").toStdString() << value << "%";
    ui->caricatureCheckBox->setText(QString::fromStdString(message.str()));

    _caricatureLevel = (value/100.0);
    updateImage();
}

void MainWindow::on_blurCheckBox_toggled(bool checked)
{
    ui->blurLevelSlider->setEnabled(checked);
    updateImage();
}

void MainWindow::on_blurLevelSlider_valueChanged(int value)
{
    std::stringstream message;
    message << tr("Blur (level): ").toStdString() << value << "\n";
    ui->blurCheckBox->setText(QString::fromStdString(message.str()));

    _blur_level = value;
    updateImage();
}
