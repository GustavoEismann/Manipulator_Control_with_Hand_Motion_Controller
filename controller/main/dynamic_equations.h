/**
 * @file dynamic_equations.h
 * @brief Dynamic motion equations for IMU-based motion tracking
 * 
 * This module provides functions for processing IMU sensor data to calculate
 * velocity and acceleration vectors. It includes coordinate transformations,
 * velocity integration with drift compensation, and acceleration filtering.
 * 
 * @author Gustavo Eismann
 * @date 2022
 */

#ifndef __DYNAMIC_EQUATIONS_H
#define __DYNAMIC_EQUATIONS_H

#include <math.h>
#include "mpu9250.h"

/** @brief Number of samples for moving average acceleration filtering */
#define IMU_SAMPLE_SIZE 10

/**
 * @brief Transform accelerometer and gyroscope readings to device coordinate frame
 * 
 * Applies coordinate transformation to align sensor readings with the device's
 * physical orientation:
 * - Rotates 180° around Z-axis
 * - Rotates -90° around X-axis
 * 
 * @param[in,out] v Pointer to vector to be transformed in-place
 */
void transform_accel_gyro(vector_t *v);

/**
 * @brief Transform magnetometer readings to device coordinate frame
 * 
 * Applies coordinate transformation to align magnetometer readings with the
 * device's physical orientation and other sensors.
 * 
 * @param[in,out] v Pointer to vector to be transformed in-place
 */
void transform_mag(vector_t *v);

/**
 * @brief Calculate velocity with scaled drift compensation
 * 
 * Integrates acceleration to velocity using v = v0 + a*dt.
 * Implements adaptive drift compensation by counting consecutive low-acceleration
 * samples and zeroing velocity when drift is detected.
 * 
 * @param[in] a Pointer to acceleration vector (cm/s²)
 * @param[in,out] v Pointer to velocity vector to update (cm/s)
 * @param[in] dt Time step in seconds
 */
void getVelocityScaled(vector_t *a, vector_t *v, float dt);

/**
 * @brief Calculate velocity with threshold-based limiting
 * 
 * Integrates acceleration to velocity with immediate zeroing when acceleration
 * falls below the specified threshold. Simpler than getVelocityScaled but more
 * aggressive in drift elimination.
 * 
 * @param[in] a Pointer to acceleration vector (cm/s²)
 * @param[in,out] v Pointer to velocity vector to update (cm/s)
 * @param[in] dt Time step in seconds
 * @param[in] stdby_tol Standby tolerance threshold (cm/s²)
 */
void getVelocityLimited(vector_t *a, vector_t *v, float dt, float stdby_tol);

/**
 * @brief Calculate velocity without drift compensation
 * 
 * Simple velocity integration using v = v0 + a*dt without any filtering
 * or drift compensation. Use only when drift is not a concern.
 * 
 * @param[in] a Pointer to acceleration vector (cm/s²)
 * @param[in,out] v Pointer to velocity vector to update (cm/s)
 * @param[in] dt Time step in seconds
 */
void getVelocityVector(vector_t *a, vector_t *v, float dt);

/**
 * @brief Calculate raw acceleration by removing gravity component
 * 
 * Computes linear acceleration by subtracting the rotated gravity vector
 * from measured acceleration and scaling to cm/s².
 * 
 * @param[in] ma Measured acceleration vector (in g's)
 * @param[in] ga Gravity acceleration vector in device frame (in g's)
 * @return Raw linear acceleration vector (cm/s²)
 */
vector_t getRawAcceleration(vector_t ma, vector_t ga);

/**
 * @brief Calculate filtered acceleration using moving average
 * 
 * Updates a circular buffer with the latest acceleration sample and returns
 * the moving average of the last IMU_SAMPLE_SIZE samples. Helps reduce
 * noise and transient spikes in acceleration data.
 * 
 * @param[in,out] sa Sample array buffer of size IMU_SAMPLE_SIZE
 * @param[in] ma Measured acceleration vector (in g's)
 * @param[in] ga Gravity acceleration vector in device frame (in g's)
 * @param[in] count Current position in circular buffer (0 to IMU_SAMPLE_SIZE-1)
 * @return Averaged raw linear acceleration vector (cm/s²)
 */
vector_t getSampledAcceleration(vector_t sa[IMU_SAMPLE_SIZE], vector_t ma, vector_t ga, uint8_t count);

/**
 * @brief Calculate the magnitude of an acceleration vector
 * 
 * Computes the Euclidean norm (magnitude) of a 3D acceleration vector.
 * 
 * @param[in] va Acceleration vector
 * @return Magnitude of the vector
 */
float getAccelerationModule(vector_t va);

#endif // __DYNAMIC_EQUATIONS_H