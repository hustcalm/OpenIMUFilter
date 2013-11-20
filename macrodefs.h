#ifndef MACRODEFS_H
#define MACRODEFS_H

// CAN Frame Micro Defines
#define GYRO_X_Y 0x00000066
#define GYRO_Z_Temp 0x00000067
#define ACCEL_X_Y_Z 0x00000069

#define GYRO_FULL_32_BIT	((unsigned int)(0xFFFFFFFF))
#define ACCEL_FULL_16_BIT   ((unsigned int)(0xFFFF))

#define GYRO_RATE_LSB 0.001
#define ACCEL_RATE_LSB 0.0001
#define TEMP_IMU_LSB 1

/* Blow macros are imported from http://mbed.org/cookbook/IMU and may not be used
 */

//Gravity at Earth's surface in m/s/s
#define g0 9.812865328

//Number of samples to average.
#define SAMPLES 4

//Number of samples to be averaged for a null bias calculation
//during calibration.
#define CALIBRATION_SAMPLES 128

//Convert from radians to degrees.
#define toDegrees(x) (x * 57.2957795)

//Convert from degrees to radians.
#define toRadians(x) (x * 0.01745329252)

//ITG-3200 sensitivity is 14.375 LSB/(degrees/sec).
#define GYROSCOPE_GAIN (1 / 14.375)

//Full scale resolution on the ADXL345 is 4mg/LSB.
#define ACCELEROMETER_GAIN (0.004 * g0)

//Sampling gyroscope at 200Hz.
#define GYRO_RATE   0.005

//Sampling accelerometer at 200Hz.
#define ACC_RATE    0.005

//Updating filter at 1/FILTER_RATE Hz.
#define FILTER_RATE 0.01

// Timer Interval, namely 1000*FILTER_RATE
#define TIMER_INTERVAL 10

// Sample Frames for Calibration
#define SAMPLE_FRAMES_FOR_CALIBRATION 10000

#endif // MACRODEFS_H
