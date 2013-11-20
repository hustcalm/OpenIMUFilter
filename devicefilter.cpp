#include "devicefilter.h"

deviceFilter::deviceFilter()
{
    // Initialization for variables
    showVerboseInfo = FALSE;

    totalReceivedFrames = 0;
    totalReceivedID66 = 0;
    totalReceivedID67 = 0;
    totalReceivedID69 = 0;

    // Initialization for Accel and Gyro
    std::cout << "Now Initializing Accel..." << std::endl;
    initializeAcceleromter();
    std::cout << "Done!" << endl;

    std::cout << "Now Initializing Gyro..." << std::endl;
    initializeGyroscope();
    std::cout << "Done!" << std::endl;

    std::cout << "Now calibrating Gyro and Accel for Zeor Offset.." << std::endl;
    //calibrateGyroAndAccel();
    std::cout << "Done!" << std::endl;

    _sampleTimer = new QTimer(this);
    connect(_sampleTimer, SIGNAL(timeout()), this, SLOT(sampleGyroAndAccel()));
    //connect(_sampleTimer, SIGNAL(timeout()), this, SLOT(testSampleTimer()));

    _filterTimer = new QTimer(this);
    //connect(_filterTimer, SIGNAL(timeout()), this, SLOT(naiveIntegrationFilter()));
    connect(_filterTimer, SIGNAL(timeout()), this, SLOT(filterUpdateAndCalc()));
    //connect(_filterTimer, SIGNAL(timeout()), this, SLOT(testFilterTimer()));

    _sampleTimer->start(TIMER_INTERVAL);
    _filterTimer->start(TIMER_INTERVAL);
}

deviceFilter::~deviceFilter()
{
    if(_sampleTimer){
        delete _sampleTimer;
    }

    if(_filterTimer){
        delete _filterTimer;
    }

    if(_usbCANDevice){
        if(_usbCANDevice->closeDevice()){
            std::cout << "Device Closed!" << endl;
        }
        else{
            _usbCANDevice->showErrorInfo();
        }

        delete _usbCANDevice;
    }
    if(_imuFilter){
        delete _imuFilter;
    }
}

void deviceFilter::sampleGyroAndAccel()
{
    w_xAccumulator = 0.0;
    w_yAccumulator = 0.0;
    w_zAccumulator = 0.0;

    a_xAccumulator = 0.0;
    a_yAccumulator = 0.0;
    a_zAccumulator = 0.0;

    totalReceivedID66 = 0;
    totalReceivedID67 = 0;
    totalReceivedID69 = 0;

    if(_usbCANDevice->getReceiveNumber()){
        recNumber = _usbCANDevice->getUnreadFramesNumber();
    }
    else{
        _usbCANDevice->showErrorInfo();
    }

    if(_usbCANDevice->receiveData(vco, recNumber)){
        receivedFrames = _usbCANDevice->getReveivedFramsNumber();
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

                // Accumulator
                w_xAccumulator += gyroRateX;
                w_yAccumulator += gyroRateY;

                //w_xAccumulator += gyroRateX - w_xBias;
                //w_yAccumulator += gyroRateY - w_yBias;
            }
            else if(vco[i].ID == GYRO_Z_Temp){
                totalReceivedID67++;

                // Gyro Z Rate in Raw format
                gyroZLong = vco[i].Data[3] << 24 | vco[i].Data[2] << 16 | vco[i].Data[1] << 8 | vco[i].Data[0];
                tempOfIMULong = vco[i].Data[7];

                // Real Gyro Z Rate
                gyroRateZ = gyroZLong * GYRO_RATE_LSB;
                tempIMU = tempOfIMULong * TEMP_IMU_LSB;

                // Accumulator
                w_zAccumulator += gyroRateZ;

                //w_zAccumulator += gyroRateZ - w_zBias;
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

                // Accumulator
                a_xAccumulator += accelRateX;
                a_yAccumulator += accelRateY;
                a_zAccumulator += accelRateZ;

                //a_xAccumulator += accelRateX - a_xBias;
                //a_yAccumulator += accelRateY - a_yBias;
                //a_zAccumulator += accelRateZ - a_zBias;
            }
            else{
                std::cout << "No defined frame ID received!" << std::endl;
            }
        }

        // Return the averaged rates

        // degree/s
        if(totalReceivedID66 != 0){
            w_x = w_xAccumulator/totalReceivedID66;
            w_y = w_yAccumulator/totalReceivedID66;

            w_x_last = w_x;
            w_y_last = w_y;
        }
        else{
            w_x = w_x_last;
            w_y = w_y_last;
        }

        if(totalReceivedID67 != 0){
            w_z = w_zAccumulator/totalReceivedID67;

            w_z_last = w_z;
        }
        else{
            w_z = w_z_last;
        }

        // rad/s
        w_x_rad_s = toRadians(w_x);
        w_y_rad_s = toRadians(w_y);
        w_z_rad_s = toRadians(w_z);

        // g
        if(totalReceivedID69 != 0){
            a_x = a_xAccumulator/totalReceivedID69;
            a_y = a_yAccumulator/totalReceivedID69;
            a_z = a_zAccumulator/totalReceivedID69;
        }
        else{
            a_x = a_x_last;
            a_y = a_y_last;
            a_z = a_z_last;
        }

        // m/s/s
        a_x_m_ss = a_x * g0;
        a_y_m_ss = a_y * g0;
        a_z_m_ss = a_z * g0;

        /*
        cout << w_x << " "
             << w_y << " "
             << w_z << " "
             << a_x << " "
             << a_y << " "
             << a_z << endl;
        */

        emit postGyroAndAccelData(w_x, w_y, w_z, a_x, a_y, a_z);

    }
    else{
        _usbCANDevice->showErrorInfo();
    }
}

void deviceFilter::filterUpdateAndCalc()
{
    //Sample Gyro and Accel Data
    //sampleGyroAndAccel();

    // Update the filter variables
    _imuFilter->updateFilter(w_x_rad_s, w_y_rad_s, w_z_rad_s, a_x_m_ss, a_y_m_ss, a_z_m_ss);
    _imuFilter->computeEuler();
    showVerboseInfo &&
    std::cout << "Pitch: " << _imuFilter->getPitch() << " "
              << "Yaw: "   << _imuFilter->getYaw() << " "
              << "Roll: "  << _imuFilter->getRoll() << std::endl;

    emit postFilterOrientationData(_imuFilter->getPitchInDegrees(), _imuFilter->getYawInDegrees(), _imuFilter->getRollInDegrees());
}

void deviceFilter::naiveIntegrationFilter()
{
    // Just do simple and stupid integration
    gyroOrientationX = gyroOrientationX + w_x * FILTER_RATE;
    gyroOrientationY = gyroOrientationY + w_y * FILTER_RATE;
    gyroOrientationZ = gyroOrientationZ + w_z * FILTER_RATE;

    // Debug for NAN
    //gyroOrientationX = gyroOrientationY = gyroOrientationZ = 90.0;

    cout << gyroOrientationX << " " << gyroOrientationY << " " << gyroOrientationZ << endl;
    emit postFilterOrientationData(gyroOrientationX, gyroOrientationY, gyroOrientationZ);

}

void deviceFilter::attachUsbCANDevice(USBCAN* usbCANDevice)
{
    this->_usbCANDevice = usbCANDevice;
}

void deviceFilter::attachImuFilter(IMUfilter* imuFilter)
{
    this->_imuFilter = imuFilter;
}

void deviceFilter::testSampleTimer()
{
    std::cout << "sampleTimer Triggered!"  << std::endl;
}

void deviceFilter::testFilterTimer()
{
    std::cout << "filterTimer Triggered!"  << std::endl;
}

void deviceFilter::initializeAcceleromter()
{
    // Initialization of Accel

    a_x = 0.0;
    a_y = 0.0;
    a_z = 0.0;

    a_x_last = 0.0;
    a_y_last = 0.0;
    a_z_last = 0.0;

    a_x_m_ss = 0.0;
    a_y_m_ss = 0.0;
    a_z_m_ss = 0.0;

    a_xBias = 0.0;
    a_yBias = 0.0;
    a_zBias = 0.0;

    a_xAccumulator = 0.0;
    a_yAccumulator = 0.0;
    a_zAccumulator = 0.0;

    accelXLong = 0;
    accelYLong = 0;
    accelZLong = 0;

    accelRateX = 0.0;
    accelRateY = 0.0;
    accelRateZ = 0.0;
}

void deviceFilter::calibrateAccelerometer()
{
    // Calibration of Accel to calculate the Bias
    std::cout << "Please use calibrateGyroAndAccel instead!" << std::endl;
}

void deviceFilter::initializeGyroscope()
{
    // Initialization of Gyro

    w_x = 0.0;
    w_y = 0.0;
    w_z = 0.0;


    w_x_last = 0.0;
    w_y_last = 0.0;
    w_z_last = 0.0;

    w_x_rad_s = 0.0;
    w_y_rad_s = 0.0;
    w_z_rad_s = 0.0;

    w_xBias = 0;
    w_yBias = 0;
    w_zBias = 0;

    w_xAccumulator = 0.0;
    w_yAccumulator = 0.0;
    w_zAccumulator = 0.0;

    gyroXLong = 0.0;
    gyroYLong = 0.0;
    gyroZLong = 0.0;

    gyroRateX = 0.0;
    gyroRateY = 0.0;
    gyroRateZ = 0.0;

    gyroOrientationX = 0.0;
    gyroOrientationY = 0.0;
    gyroOrientationZ = 0.0;
}

void deviceFilter::calibrateGyroscope()
{
    // Calibration of Gyro to calculate the Bias
    std::cout << "Please use calibrateGyroAndAccel instead!" << std::endl;
}

void deviceFilter::calibrateGyroAndAccel()
{

    w_xAccumulator = 0.0;
    w_yAccumulator = 0.0;
    w_zAccumulator = 0.0;

    a_xAccumulator = 0.0;
    a_yAccumulator = 0.0;
    a_zAccumulator = 0.0;

    w_xBias = 0.0;
    w_yBias = 0.0;
    w_zBias = 0.0;

    a_xBias = 0.0;
    a_yBias = 0.0;
    a_zBias = 0.0;

    totalReceivedID66 = 0;
    totalReceivedID67 = 0;
    totalReceivedID69 = 0;

    totalReceivedFrames = 0;

    bool calibrationDone = FALSE;

    while(!calibrationDone){


        if(_usbCANDevice->getReceiveNumber()){
            recNumber = _usbCANDevice->getUnreadFramesNumber();
        }
        else{
            _usbCANDevice->showErrorInfo();
        }

        if(_usbCANDevice->receiveData(vco, recNumber)){
            receivedFrames = _usbCANDevice->getReveivedFramsNumber();
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

                    // Accumulator
                    w_xAccumulator += gyroRateX;
                    w_yAccumulator += gyroRateY;
                }
                else if(vco[i].ID == GYRO_Z_Temp){
                    totalReceivedID67++;

                    // Gyro Z Rate in Raw format
                    gyroZLong = vco[i].Data[3] << 24 | vco[i].Data[2] << 16 | vco[i].Data[1] << 8 | vco[i].Data[0];
                    tempOfIMULong = vco[i].Data[7];

                    // Real Gyro Z Rate
                    gyroRateZ = gyroZLong * GYRO_RATE_LSB;
                    tempIMU = tempOfIMULong * TEMP_IMU_LSB;

                    // Accumulator
                    w_zAccumulator += gyroRateZ;
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

                    // Accumulator
                    a_xAccumulator += accelRateX;
                    a_yAccumulator += accelRateY;
                    a_zAccumulator += accelRateZ;
                }
                else{
                    std::cout << "No defined frame ID received!" << std::endl;
                }
            }

        }
        else{
            _usbCANDevice->showErrorInfo();
        }

        // Return the averaged rates
        if(totalReceivedFrames >= SAMPLE_FRAMES_FOR_CALIBRATION){
            // Jump out of While Loop
            calibrationDone = TRUE;
        }
        else{
            // Go on sampling
        }

    }

    // degree/s
    if(totalReceivedID66 != 0){
        w_xBias = w_xAccumulator/totalReceivedID66;
        w_yBias = w_yAccumulator/totalReceivedID66;
    }
    else{
        std::cerr << "receive Frames with ID 0x66 Error!" << std::endl;
    }

    if(totalReceivedID67 != 0){
        w_zBias = w_zAccumulator/totalReceivedID67;
    }
    else{
        std::cerr << "receive Frames with ID 0x67 Error!" << std::endl;
    }

    // g
    if(totalReceivedID69 != 0){
        a_xBias = a_xAccumulator/totalReceivedID69;
        a_yBias = a_yAccumulator/totalReceivedID69;
        a_zBias = a_zAccumulator/totalReceivedID69;
    }
    else{
        std::cerr << "receive Frames with ID 0x69 Error!" << std::endl;
    }

    std::cout << "Calibration Done and Zero Offset perspectively: " << endl;

    std::cout << "Gyro X Bias: " << w_xBias << endl;
    std::cout << "Gyro Y Bias: " << w_yBias << endl;
    std::cout << "Gyro Z Bias: " << w_zBias << endl;

    std::cout << "Accel X Bias: " << a_xBias << endl;
    std::cout << "Accel Y Bias: " << a_yBias << endl;
    std::cout << "Accel Z Bias: " << a_zBias << endl;
}

void deviceFilter::setBiasForGyroAndAccel(double gyroXBias, double gyroYBias, double gyroZBias, double accelXBias, double accelYBias, double accelZBias)
{
    w_xBias = gyroXBias;
    w_yBias = gyroYBias;
    w_zBias = gyroZBias;

    a_xBias = accelXBias;
    a_yBias = accelYBias;
    a_zBias = accelZBias;
}
