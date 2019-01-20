#include "mainwindow.h"
#include "ui_mainwindow.h"

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    std::cout<<"Yes\n";

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
    _message.str("");
    _message << "Reloading camera list...";
    showMessage(_message);
    _camAvail = nOfCamAvail();
    _message.str("");
    _message << "Number of cameras found: " << _camAvail;
    showMessage(_message);
    if(_camAvail>0){
        // some cameras were found
        vector<string> camList = camAvail();
        auto qCamList = getQStringList(camAvail());
        ui->comboBoxCameraList->clear();
        ui->comboBoxCameraList->insertItems(0, qCamList);
        ui->buttonConnect->setEnabled(true);
    }else{
        ui->comboBoxCameraList->clear();
        ui->comboBoxCameraList->insertItem(0, QString::fromStdString(" - "));
        ui->buttonConnect->setEnabled(false);
    }
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
