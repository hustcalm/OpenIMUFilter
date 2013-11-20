/**
 * @author Aaron Berk
 *
 * @section LICENSE
 *
 * Copyright (c) 2010 ARM Limited
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 *
 * @section DESCRIPTION
 *
 * IMU orientation filter developed by Sebastian Madgwick.
 *
 * Find more details about his paper here:
 *
 * http://code.google.com/p/imumargalgorithm30042010sohm/
 */

#ifndef IMU_FILTER_H
#define IMU_FILTER_H

/**
 * Includes
 */

#include <math.h>
#include "macrodefs.h"

/**
 * Defines
 */
#define PI 3.1415926536

/**
 * IMU orientation filter.
 */
class IMUfilter {

public:

    /**
     * Constructor.
     *
     * Initializes filter variables.
     *
     * @param rate The rate at which the filter should be updated.
     * @param gyroscopeMeasurementError The error of the gyroscope in degrees
     *  per second. This used to calculate a tuning constant for the filter.
     *  Try changing this value if there are jittery readings, or they change
     *  too much or too fast when rotating the IMU.
     */
    IMUfilter(double rate, double gyroscopeMeasurementError);

    /**
     * Update the filter variables.
     *
     * @param w_x X-axis gyroscope reading in rad/s.
     * @param w_y Y-axis gyroscope reading in rad/s.
     * @param w_z Z-axis gyroscope reading in rad/s.
     * @param a_x X-axis accelerometer reading in m/s/s.
     * @param a_y Y-axis accelerometer reading in m/s/s.
     * @param a_z Z-axis accelerometer reading in m/s/s.
     */
    void updateFilter(double w_x, double w_y, double w_z,
                      double a_x, double a_y, double a_z);

    /**
     * Compute the Euler angles based on the current filter data.
     */
    void computeEuler(void);

    /**
     * Get the current roll.
     *
     * @return The current roll angle in radians.
     */
    double getRoll(void);

    /**
     * Get the current roll.
     *
     * @return The current roll angle in degrees.
     */
    double getRollInDegrees(void);

    /**
     * Get the current pitch.
     *
     * @return The current pitch angle in radians.
     */
    double getPitch(void);

    /**
     * Get the current pitch.
     *
     * @return The current pitch angle in degrees.
     */
    double getPitchInDegrees(void);

    /**
     * Get the current yaw.
     *
     * @return The current yaw angle in radians.
     */
    double getYaw(void);

    /**
     * Get the current yaw.
     *
     * @return The current yaw angle in degrees.
     */
    double getYawInDegrees(void);

    /**
     * Reset the filter.
     */
    void reset(void);

private:

    int firstUpdate;

    //Quaternion orientation of earth frame relative to auxiliary frame.
    double AEq_1;
    double AEq_2;
    double AEq_3;
    double AEq_4;

    //Estimated orientation quaternion elements with initial conditions.
    double SEq_1;
    double SEq_2;
    double SEq_3;
    double SEq_4;

    //Sampling period
    double deltat;

    //Gyroscope measurement error (in degrees per second).
    double gyroMeasError;

    //Compute beta (filter tuning constant..
    double beta;

    double phi;
    double theta;
    double psi;

};

#endif /* IMU_FILTER_H */
