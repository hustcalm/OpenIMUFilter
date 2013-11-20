#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <QTimer>
#include <QDateTime>

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    QTimer* timeUpdater = new QTimer(this);
    connect(timeUpdater, SIGNAL(timeout()), this, SLOT(updateCurrentTime()));
    timeUpdater->start(1000);
}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::updateCurrentTime()
{
    QDateTime currentTime = QDateTime::currentDateTime();
    QString currentTimeString = currentTime.toString("yyyy-MM-dd hh:mm:ss dddd");
    ui->label_currentTime->setText(currentTimeString);
}

void MainWindow::updateGyroAndAccelData(double gyroRateX, double gyroRateY, double gyroRateZ, double accelRateX, double accelRateY, double accelRateZ)
{
    ui->lineEdit_Gyro_X->setText(QString::number(gyroRateX));
    ui->lineEdit_Gyro_Y->setText(QString::number(gyroRateY));
    ui->lineEdit_Gyro_Z->setText(QString::number(gyroRateZ));

    ui->lineEdit_Accel_X->setText(QString::number(accelRateX));
    ui->lineEdit_Accel_Y->setText(QString::number(accelRateY));
    ui->lineEdit_Accel_Z->setText(QString::number(accelRateZ));
}

void MainWindow::updateFilterOrientationData(double pitch, double yaw, double roll)
{
    ui->lineEdit_Gyro_Orientation_X->setText(QString::number(pitch));
    ui->lineEdit_Gyro_Orientation_Y->setText(QString::number(yaw));
    ui->lineEdit_Gyro_Orientation_Z->setText(QString::number(roll));
}
