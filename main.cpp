#include "mainwindow.h"

#include <QObject>
#include <QApplication>
#include <QDebug>
#include <QElapsedTimer>
#include <QTimer>
#include <fstream>
#include "devicefilter.h"

using namespace std;

// Variables for calculating IMU-F100A5 angles
qint64 totalReceivedFrames = 0;
qint64 totalReceivedID66 = 0;
qint64 totalReceivedID67 = 0;
qint64 totalReceivedID69 = 0;

int iterateTimes = 100;

double gyroXAccumulator = 0.0;
double gyroYAccumulator = 0.0;
double gyroZAccumulator = 0.0;

double gyroXBias = 0.0;
double gyroYBias = 0.0;
double gyroZBias = 0.0;

double accelXAccumulator = 0.0;
double accelYAccumulator = 0.0;
double accelZAccumulator = 0.0;

double accelXBias = 0.0;
double accelYBias = 0.0;
double accelZBias = 0.0;

signed long gyroXLong = 0;
signed long gyroYLong = 0;
signed long gyroZLong = 0;

signed long tempOfIMULong = 0;

signed short accelXLong = 0;
signed short accelYLong = 0;
signed short accelZLong = 0;
double gyroRateX = 0;
double gyroRateY = 0;
double gyroRateZ = 0;

double accelRateX = 0;
double accelRateY = 0;
double accelRateZ = 0;

double tempIMU = 0;

IMUfilter *imuFilter;
USBCAN* usbCANDevice;

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    MainWindow w;
    w.show();

    cout << "Get Pitch, Yaw, Roll using IMU Filter..." << endl;

    // Open file for log
    ofstream outfile("seq_data.txt");
    if(!outfile){
        cerr << "Open Log File Failed!" << endl;
    }
    else{
        cout << "Log File Ready!" << endl;
    }

    // Get USBCAN-I+ to work
    usbCANDevice = new USBCAN(USBCAN::_nDefaultDevType, USBCAN::_nDefaultDevIndex, USBCAN::_nDefaultReserved);

    // Open Device
    if(usbCANDevice->openDevice()){
        cout << "Device Opened Succesfully!" << endl;
    }
    else{
        usbCANDevice->showErrorInfo();
    }

    // Initialize Device
    PVCI_INIT_CONFIG pInitConfig = new VCI_INIT_CONFIG;
    pInitConfig->AccCode = 0;
    pInitConfig->AccMask = 0xFFFFFFFF;
    pInitConfig->Filter = 1;
    pInitConfig->Mode = 0;
    pInitConfig->Timing0 = 0x00;
    pInitConfig->Timing1 = 0x14;

    if(usbCANDevice->initCAN(pInitConfig)){
        cout << "Device Initialized Successfuly!" << endl;
    }
    else{
        cout<< "Device Initialized Failed!" << endl;
        usbCANDevice->showErrorInfo();
        usbCANDevice->closeDevice();
    }

    // Clear Device buffer first
    if(usbCANDevice->clearBuffer()){
        cout << "Device buffer cleard!" << endl;
    }
    else{
        usbCANDevice->showErrorInfo();
    }

    // Start Communication
    VCI_CAN_OBJ vco[1000];
    DWORD recNumber;
    DWORD receivedFrames;

    try
    {
        if(usbCANDevice->startCAN()){
            cout << "Begin to grab Frames..." << endl;

            QElapsedTimer timer;
            qint64 nanoSec;
            timer.start();

            while(TRUE && iterateTimes){
                if(usbCANDevice->getReceiveNumber()){
                    recNumber = usbCANDevice->getUnreadFramesNumber();
                }
                else{
                    usbCANDevice->showErrorInfo();
                }

                if(usbCANDevice->receiveData(vco, recNumber)){
                    receivedFrames = usbCANDevice->getReveivedFramsNumber();
                    totalReceivedFrames += receivedFrames;

                    for(unsigned int i = 0; i < receivedFrames; i++){
                        if(vco[i].ID == GYRO_X_Y){
                            totalReceivedID66++;

                            // Gyro X and Y Rate in Raw format
                            gyroXLong = vco[i].Data[3] << 24 | vco[i].Data[2] << 16 | vco[i].Data[1] << 8 | vco[i].Data[0];
                            gyroYLong = vco[i].Data[7] << 24 | vco[i].Data[6] << 16 | vco[i].Data[5] << 8 | vco[i].Data[4];

                            // Real Gyro X and Y Rate
                            gyroRateX = gyroXLong * GYRO_RATE_LSB;
                            gyroRateY = gyroYLong * GYRO_RATE_LSB;

                            gyroXAccumulator += gyroRateX;
                            gyroYAccumulator += gyroRateY;
                        }
                        else if(vco[i].ID == GYRO_Z_Temp){
                            totalReceivedID67++;

                            // Gyro Z Rate in Raw format
                            gyroZLong = vco[i].Data[3] << 24 | vco[i].Data[2] << 16 | vco[i].Data[1] << 8 | vco[i].Data[0];
                            tempOfIMULong = vco[i].Data[7];

                            // Real Gyro Z Rate
                            gyroRateZ = gyroZLong * GYRO_RATE_LSB;
                            tempIMU = tempOfIMULong * TEMP_IMU_LSB;

                            gyroZAccumulator += gyroRateZ;
                        }
                        else if(vco[i].ID == ACCEL_X_Y_Z){
                            totalReceivedID69++;

                            // Accel X, Y, Z Rate in Raw format
                            accelXLong = vco[i].Data[1] << 8 | vco[i].Data[0];
                            accelYLong = vco[i].Data[3] << 8 | vco[i].Data[2];
                            accelZLong = vco[i].Data[5] << 8 | vco[i].Data[4];

                            // Real Accel X, Y, Z Rate
                            accelRateX = accelXLong * ACCEL_RATE_LSB;
                            accelRateY = accelYLong * ACCEL_RATE_LSB;
                            accelRateZ = accelZLong * ACCEL_RATE_LSB;

                            accelXAccumulator += accelRateX;
                            accelYAccumulator += accelRateY;
                            accelZAccumulator += accelRateZ;
                        }
                        else{
                            cout << "No defined frame ID received!" << endl;
                        }
                    }

                    // Output Data Every Iteration
                    FALSE &&
                    cout << "Rates: " << gyroRateX << " " << gyroRateY << " " << gyroRateZ << " "
                                      << accelRateX << " " << accelRateY << " " << accelRateZ << " "
                                      << endl;
                }
                else{
                    usbCANDevice->showErrorInfo();
                }

                if(totalReceivedFrames >= SAMPLE_FRAMES_FOR_CALIBRATION){
                    break;
                }
                else{
                    // Go On Sampling
                }
            }

            nanoSec = timer.nsecsElapsed();
            cout << "Grabbed " << totalReceivedFrames << " in " << nanoSec << " nano seconds!" << endl;
            cout << "Grabbed " << totalReceivedID66 << " Frames with ID " <<  "0x66" << endl;
            cout << "Grabbed " << totalReceivedID67 << " Frames with ID " <<  "0x67" << endl;
            cout << "Grabbed " << totalReceivedID69 << " Frames with ID " <<  "0x69" << endl;
            cout << "Frame rates is " << totalReceivedFrames * 1000000000 / nanoSec << endl;



            gyroXBias = gyroXAccumulator/totalReceivedID66;
            gyroYBias = gyroYAccumulator/totalReceivedID66;
            gyroZBias = gyroYAccumulator/totalReceivedID67;

            accelXBias = accelXAccumulator/totalReceivedID69;
            accelYBias = accelYAccumulator/totalReceivedID69;
            accelZBias = accelYAccumulator/totalReceivedID69;

            std::cout << "Calibration Done and Zero Offset perspectively: " << endl;

            std::cout << "Gyro X Bias: " << gyroXBias << endl;
            std::cout << "Gyro Y Bias: " << gyroYBias << endl;
            std::cout << "Gyro Z Bias: " << gyroZBias << endl;

            std::cout << "Accel X Bias: " << accelXBias << endl;
            std::cout << "Accel Y Bias: " << accelYBias << endl;
            std::cout << "Accel Z Bias: " << accelZBias << endl;
        }
        else{
            usbCANDevice->showErrorInfo();
        }
    }
    catch(std::exception& e){
        qDebug()<<e.what();
    }

    // Using IMUFilter to output Euler Angles
    cout << "Staring IMUFilter..." << endl;
    imuFilter = new IMUfilter(FILTER_RATE, 0.3);

    QThread* imuDeviceFilterThread = new QThread;
    deviceFilter* imuDeviceFilter = new deviceFilter();
    imuDeviceFilter->attachUsbCANDevice(usbCANDevice);
    imuDeviceFilter->attachImuFilter(imuFilter);
    imuDeviceFilter->setBiasForGyroAndAccel(gyroXBias, gyroYBias, gyroZBias, accelXBias, accelYBias, accelZBias);

    QObject::connect(imuDeviceFilter, SIGNAL(postGyroAndAccelData(double,double,double,double,double,double)), &w, SLOT(updateGyroAndAccelData(double,double,double,double,double,double)));
    QObject::connect(imuDeviceFilter, SIGNAL(postFilterOrientationData(double,double,double)), &w, SLOT(updateFilterOrientationData(double,double,double)));

    imuDeviceFilter->moveToThread(imuDeviceFilterThread);
    imuDeviceFilterThread->start();

    // Check if we have unread frames
    if(usbCANDevice->getReceiveNumber()){
        cout << "Have " << usbCANDevice->getUnreadFramesNumber() << " unread frames left!" << endl;
    }
    else{
        usbCANDevice->showErrorInfo();
    }

    // Close Device
    /*
    if(usbCANDevice->closeDevice()){
        cout << "Device Closed!" << endl;
    }
    else{
        usbCANDevice->showErrorInfo();
    }
    */

    return a.exec();
}
