#ifndef DEVICEFILTER_H
#define DEVICEFILTER_H

#include <QThread>
#include <QTimer>
#include "usbcan.h"
#include "IMUfilter.h"
#include "macrodefs.h"

using namespace std;

class deviceFilter : public QObject
{
    Q_OBJECT
public:
    deviceFilter();
    ~deviceFilter();

signals:
    void postGyroAndAccelData(double gyroRateX, double gyroRateY, double gyroRateZ, double accelRateX, double accelRateY, double accelRateZ);
    void postFilterOrientationData(double pitch, double yaw, double roll);

public slots:
    void filterUpdateAndCalc();
    void sampleGyroAndAccel();

public slots:
    void testSampleTimer();
    void testFilterTimer();

    void naiveIntegrationFilter();

private:
    QTimer* _sampleTimer;
    QTimer* _filterTimer;

    USBCAN* _usbCANDevice;
    IMUfilter* _imuFilter;

public:
    void attachUsbCANDevice(USBCAN* usbCANDevice);
    void attachImuFilter(IMUfilter *imuFilter);
    void setBiasForGyroAndAccel(double gyroXBias, double gyroYBias, double gyroZBias, double accelXBias, double accelYBias, double accelZBias);

public:

    //Offsets for the gyroscope.
    //The readings we take when the gyroscope is stationary won't be 0, so we'll
    //average a set of readings we do get when the gyroscope is stationary and
    //take those away from subsequent readings to ensure the gyroscope is offset
    //or "biased" to 0.
    double w_xBias;
    double w_yBias;
    double w_zBias;

    //Offsets for the accelerometer.
    //Same as with the gyroscope.
    double a_xBias;
    double a_yBias;
    double a_zBias;

    //Accumulators used for oversampling and then averaging.
    volatile double a_xAccumulator;
    volatile double a_yAccumulator;
    volatile double a_zAccumulator;
    volatile double w_xAccumulator;
    volatile double w_yAccumulator;
    volatile double w_zAccumulator;

    //Accelerometer and gyroscope readings for x, y, z axes.
    volatile double a_x;
    volatile double a_y;
    volatile double a_z;
    volatile double w_x;
    volatile double w_y;
    volatile double w_z;

    // Last reading
    volatile double a_x_last;
    volatile double a_y_last;
    volatile double a_z_last;
    volatile double w_x_last;
    volatile double w_y_last;
    volatile double w_z_last;

    volatile double a_x_m_ss;
    volatile double a_y_m_ss;
    volatile double a_z_m_ss;
    volatile double w_x_rad_s;
    volatile double w_y_rad_s;
    volatile double w_z_rad_s;

    //Buffer for accelerometer readings.
    int readings[3];
    //Number of accelerometer samples we're on.
    int accelerometerSamples;
    //Number of gyroscope samples we're on.
    int gyroscopeSamples;

    // Orientation for X, Y, Z(Pitch, Yaw and Roll)
    double gyroOrientationX;
    double gyroOrientationY;
    double gyroOrientationZ;

    /**
     * Prototypes
     */
    //Set up the ADXL345 appropriately.
    void initializeAcceleromter(void);

    //Calculate the null bias.
    void calibrateAccelerometer(void);

    //Take a set of samples and average them.
    void sampleAccelerometer(void);

    //Set up the ITG3200 appropriately.
    void initializeGyroscope(void);

    //Calculate the null bias.
    void calibrateGyroscope(void);

    //Take a set of samples and average them.
    void sampleGyroscope(void);

    // Calculate the null bias for both Gyro and Accel
    void calibrateGyroAndAccel(void);

    // Take a set of samples both for Gyro and Accel
    //void sampleGyroAndAccel(void);

    //Update the filter and calculate the Euler angles.
    //void filterUpdateAndCalc(void);

public:

    // Variables for calculating IMU-F100A5 angles
    unsigned long totalReceivedFrames;
    unsigned long totalReceivedID66;
    unsigned long totalReceivedID67;
    unsigned long totalReceivedID69;

    int iterateTimes;

    signed long gyroXLong;
    signed long gyroYLong;
    signed long gyroZLong;

    signed long tempOfIMULong;

    signed short accelXLong;
    signed short accelYLong;
    signed short accelZLong;
    double gyroRateX;
    double gyroRateY;
    double gyroRateZ;

    double accelRateX;
    double accelRateY;
    double accelRateZ;

    double tempIMU;

public:

    VCI_CAN_OBJ vco[1000];
    DWORD recNumber;
    DWORD receivedFrames;

public:
    bool showVerboseInfo;
};

#endif // DEVICEFILTER_H
