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

/**
 * Includes
 */
#include "IMUfilter.h"

IMUfilter::IMUfilter(double rate, double gyroscopeMeasurementError){

    firstUpdate = 0;
    
    //Quaternion orientation of earth frame relative to auxiliary frame.
    AEq_1 = 1;
    AEq_2 = 0;
    AEq_3 = 0;
    AEq_4 = 0;
    
    //Estimated orientation quaternion elements with initial conditions.
    SEq_1 = 1;
    SEq_2 = 0;
    SEq_3 = 0;
    SEq_4 = 0;

    //Sampling period (typical value is ~0.1s).
    deltat = rate;
    
    //Gyroscope measurement error (in degrees per second).
    gyroMeasError = gyroscopeMeasurementError;
    
    //Compute beta.
    beta = sqrt(3.0 / 4.0) * (PI * (gyroMeasError / 180.0));

}

void IMUfilter::updateFilter(double w_x, double w_y, double w_z, double a_x, double a_y, double a_z) {

    //Local system variables.

    //Vector norm.
    double norm;
    //Quaternion rate from gyroscope elements.
    double SEqDot_omega_1;
    double SEqDot_omega_2;
    double SEqDot_omega_3;
    double SEqDot_omega_4;
    //Objective function elements.
    double f_1;
    double f_2;
    double f_3;
    //Objective function Jacobian elements.
    double J_11or24;
    double J_12or23;
    double J_13or22;
    double J_14or21;
    double J_32;
    double J_33;
    //Objective function gradient elements.
    double nablaf_1;
    double nablaf_2;
    double nablaf_3;
    double nablaf_4;

    //Auxiliary variables to avoid reapeated calcualtions.
    double halfSEq_1 = 0.5 * SEq_1;
    double halfSEq_2 = 0.5 * SEq_2;
    double halfSEq_3 = 0.5 * SEq_3;
    double halfSEq_4 = 0.5 * SEq_4;
    double twoSEq_1 = 2.0 * SEq_1;
    double twoSEq_2 = 2.0 * SEq_2;
    double twoSEq_3 = 2.0 * SEq_3;

    //Compute the quaternion rate measured by gyroscopes.
    SEqDot_omega_1 = -halfSEq_2 * w_x - halfSEq_3 * w_y - halfSEq_4 * w_z;
    SEqDot_omega_2 = halfSEq_1 * w_x + halfSEq_3 * w_z - halfSEq_4 * w_y;
    SEqDot_omega_3 = halfSEq_1 * w_y - halfSEq_2 * w_z + halfSEq_4 * w_x;
    SEqDot_omega_4 = halfSEq_1 * w_z + halfSEq_2 * w_y - halfSEq_3 * w_x;

    //Normalise the accelerometer measurement.
    norm = sqrt(a_x * a_x + a_y * a_y + a_z * a_z);
    a_x /= norm;
    a_y /= norm;
    a_z /= norm;

    //Compute the objective function and Jacobian.
    f_1 = twoSEq_2 * SEq_4 - twoSEq_1 * SEq_3 - a_x;
    f_2 = twoSEq_1 * SEq_2 + twoSEq_3 * SEq_4 - a_y;
    f_3 = 1.0 - twoSEq_2 * SEq_2 - twoSEq_3 * SEq_3 - a_z;
    //J_11 negated in matrix multiplication.
    J_11or24 = twoSEq_3;
    J_12or23 = 2 * SEq_4;
    //J_12 negated in matrix multiplication
    J_13or22 = twoSEq_1;
    J_14or21 = twoSEq_2;
    //Negated in matrix multiplication.
    J_32 = 2 * J_14or21;
    //Negated in matrix multiplication.
    J_33 = 2 * J_11or24;

    //Compute the gradient (matrix multiplication).
    nablaf_1 = J_14or21 * f_2 - J_11or24 * f_1;
    nablaf_2 = J_12or23 * f_1 + J_13or22 * f_2 - J_32 * f_3;
    nablaf_3 = J_12or23 * f_2 - J_33 * f_3 - J_13or22 * f_1;
    nablaf_4 = J_14or21 * f_1 + J_11or24 * f_2;

    //Normalise the gradient.
    norm = sqrt(nablaf_1 * nablaf_1 + nablaf_2 * nablaf_2 + nablaf_3 * nablaf_3 + nablaf_4 * nablaf_4);
    nablaf_1 /= norm;
    nablaf_2 /= norm;
    nablaf_3 /= norm;
    nablaf_4 /= norm;

    //Compute then integrate the estimated quaternion rate.
    SEq_1 += (SEqDot_omega_1 - (beta * nablaf_1)) * deltat;
    SEq_2 += (SEqDot_omega_2 - (beta * nablaf_2)) * deltat;
    SEq_3 += (SEqDot_omega_3 - (beta * nablaf_3)) * deltat;
    SEq_4 += (SEqDot_omega_4 - (beta * nablaf_4)) * deltat;

    //Normalise quaternion
    norm = sqrt(SEq_1 * SEq_1 + SEq_2 * SEq_2 + SEq_3 * SEq_3 + SEq_4 * SEq_4);
    SEq_1 /= norm;
    SEq_2 /= norm;
    SEq_3 /= norm;
    SEq_4 /= norm;

    if (firstUpdate == 0) {
        //Store orientation of auxiliary frame.
        AEq_1 = SEq_1;
        AEq_2 = SEq_2;
        AEq_3 = SEq_3;
        AEq_4 = SEq_4;
        firstUpdate = 1;
    }

}

void IMUfilter::computeEuler(void){

    //Quaternion describing orientation of sensor relative to earth.
    double ESq_1, ESq_2, ESq_3, ESq_4;
    //Quaternion describing orientation of sensor relative to auxiliary frame.
    double ASq_1, ASq_2, ASq_3, ASq_4;    
                              
    //Compute the quaternion conjugate.
    ESq_1 = SEq_1;
    ESq_2 = -SEq_2;
    ESq_3 = -SEq_3;
    ESq_4 = -SEq_4;

    //Compute the quaternion product.
    ASq_1 = ESq_1 * AEq_1 - ESq_2 * AEq_2 - ESq_3 * AEq_3 - ESq_4 * AEq_4;
    ASq_2 = ESq_1 * AEq_2 + ESq_2 * AEq_1 + ESq_3 * AEq_4 - ESq_4 * AEq_3;
    ASq_3 = ESq_1 * AEq_3 - ESq_2 * AEq_4 + ESq_3 * AEq_1 + ESq_4 * AEq_2;
    ASq_4 = ESq_1 * AEq_4 + ESq_2 * AEq_3 - ESq_3 * AEq_2 + ESq_4 * AEq_1;

    //Compute the Euler angles from the quaternion.
    phi = atan2(2 * ASq_3 * ASq_4 - 2 * ASq_1 * ASq_2, 2 * ASq_1 * ASq_1 + 2 * ASq_4 * ASq_4 - 1);
    theta = asin(2 * ASq_2 * ASq_3 - 2 * ASq_1 * ASq_3);
    psi = atan2(2 * ASq_2 * ASq_3 - 2 * ASq_1 * ASq_4, 2 * ASq_1 * ASq_1 + 2 * ASq_2 * ASq_2 - 1);

}

double IMUfilter::getRoll(void){

    return phi;

}

double IMUfilter::getPitch(void){

    return theta;

}

double IMUfilter::getYaw(void){

    return psi;

}

// Degrees
double IMUfilter::getRollInDegrees(void){

    return toDegrees(phi);

}

double IMUfilter::getPitchInDegrees(void){

    return toDegrees(theta);

}

double IMUfilter::getYawInDegrees(void){

    return toDegrees(psi);

}

void IMUfilter::reset(void) {

    firstUpdate = 0;

    //Quaternion orientation of earth frame relative to auxiliary frame.
    AEq_1 = 1;
    AEq_2 = 0;
    AEq_3 = 0;
    AEq_4 = 0;

    //Estimated orientation quaternion elements with initial conditions.
    SEq_1 = 1;
    SEq_2 = 0;
    SEq_3 = 0;
    SEq_4 = 0;

}
