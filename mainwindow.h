#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();

private:
    Ui::MainWindow *ui;

private slots:
    void updateCurrentTime();

public slots:
    void updateGyroAndAccelData(double gyroRateX, double gyroRateY, double gyroRateZ, double accelRateX, double accelRateY, double accelRateZ);
    void updateFilterOrientationData(double pitch, double yaw, double roll);
};

#endif // MAINWINDOW_H
