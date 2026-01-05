//=====================================================================================================
// MadgwickAHRS.h
//=====================================================================================================
//
// Implementation of Madgwick's IMU and AHRS algorithms.
// See: http://www.x-io.co.uk/node/8#open_source_ahrs_and_imu_algorithms
//
// Date			Author          Notes
// 29/09/2011	SOH Madgwick    Initial release
// 02/10/2011	SOH Madgwick	Optimised for reduced CPU load
// 05/01/2026   G Eismann       Adapted for Robot Manipulator Project
//
//=====================================================================================================
#ifndef MADGWICK_AHRS_H
#define MADGWICK_AHRS_H

#include "mpu9250.h"

//---------------------------------------------------------------------------------------------------
// Function declarations

void MadgwickAHRSinit(float sampleFreqDef, float betaDef);
void MadgwickAHRSupdate(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz);
void MadgwickAHRSupdateIMU(float gx, float gy, float gz, float ax, float ay, float az);
void MadgwickGetEulerAnglesDegrees(float *heading, float *pitch, float *roll);
void RotateGravityAccelerationVector(float gx, float gy, float gz);
vector_t getRotatedGravityVector();

#endif
//=====================================================================================================
// End of file
//=====================================================================================================