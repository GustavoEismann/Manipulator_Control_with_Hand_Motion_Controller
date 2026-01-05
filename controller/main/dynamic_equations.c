/**
 * @file dynamic_equations.c
 * @brief Implementation of dynamic motion equations for IMU data processing
 * 
 * This file implements coordinate transformations and motion calculations for
 * processing MPU9250 IMU data. It handles gravity compensation, velocity
 * integration with drift correction, and acceleration filtering.
 */

#include "dynamic_equations.h"

/** @brief Acceleration threshold to detect stationary state (cm/s²) */
#define STANDBY_TOLERANCE 10.0

/** @brief Number of consecutive low-acceleration samples before zeroing velocity */
#define ZERO_COUNT 5

/** @brief Standard gravity acceleration in cm/s² (used for unit conversion) */
#define GRAVITY_ACCEL 980.665   // cm/s² (alternative: 9.80665 m/s²)

/** @brief Counter array for tracking consecutive low-acceleration samples per axis */
static uint8_t accel_count[3] = {0, 0, 0};

/**
 * @brief Transform accelerometer and gyroscope vectors to device coordinate frame
 * 
 * This function applies a coordinate transformation to align the sensor's physical
 * orientation with the device's reference frame. The transformation consists of:
 * 1. 180° rotation around Z-axis: X' = -X, Y' = -Y
 * 2. -90° rotation around X-axis: Y'' = -Z', Z'' = -Y'
 * 
 * Combined transformation:
 *   X_device = -X_sensor
 *   Y_device = -Z_sensor  
 *   Z_device = -Y_sensor
 * 
 * @param[in,out] v Pointer to vector to transform in-place
 */
void transform_accel_gyro(vector_t *v)
{
    float x = v->x;
    float y = v->y;
   @brief Transform magnetometer vector to align with device coordinate frame
 * 
 * Applies coordinate transformation to align magnetometer readings with the
 * accelerometer and gyroscope coordinate frame.
 * 
 * Transformation:
 *   X_device = -Y_sensor
 *   Y_device = Z_sensor
 *   Z_device = -X_sensor
 * 
 * @param[in,out] v Pointer to magnetometer vector to transform in-place
    v->y = -z;
    v->z = -y;
}

/**
 * @brief Integrate acceleration to velocity without filtering
 * 
 * Performs simple numerical integration: v = v0 + a*dt
 * No drift compensation or filtering is applied.
 * 
 * @note Use only when sensor drift is negligible or handled externally
 * 
 * @param[in] a Pointer to acceleration vector (cm/s²)
 * @param[in,out] v Pointer to velocity vector to update (cm/s)
 * @param[in] dt Time step in seconds
 */
void getVelocityVector(vector_t *a, vector_t *v, float dt)
{
/**
 * @brief Integrate acceleration to velocity with adaptive drift compensation
 * 
 * This function integrates acceleration to velocity while implementing an adaptive
 * drift correction algorithm. For each axis:
 * 1. If |acceleration| < STANDBY_TOLERANCE for ZERO_COUNT consecutive samples,
 *    the velocity is reset to zero (assuming device is stationary)
 * 2. Otherwise, normal integration is performed: v = v0 + a*dt
 * 
 * This helps eliminate velocity drift caused by IMU bias and integration errors
 * while allowing proper motion detection.
 * 
 * @param[in] a Pointer to acceleration vector (cm/s²)
 * @param[in,out] v Pointer to velocity vector to update (cm/s)
 * @param[in] dt Time step in seconds
 */
void getVelocityScaled(vector_t *a, vector_t *v, float dt)
{
    // X-axis: integrated error treatment with consecutive sample countingor_t *v, float dt)
{
    /* Simplified:
            v = a*dt
       Complete:
            v = v0 + a*dt
    */
    // transform_accel_gyro(a);

    v->x = (v->x + a->x * dt);
    v->y = (v->y + a->y * dt);
    v->z = (v->z + a->z * dt);
}

void getVelocityScaled(vector_t *a, vector_t *v, float dt)
{
    /* Simplified:
            v = a*dt
       Complete:
            v = v0 + a*dt
    */

    // X-axis integrated error treatment
    if (abs(a->x) < STANDBY_TOLERANCE) {
        if (accel_count[0] > ZERO_COUNT) {
            v->x = 0.0;
            accel_count[0] = 0;
        }
        else {
            v->x = (v->x + a->x * dt);
            accel_count[0] += 1;
        }
    }
    else {
        accel_count[0] = 0;
        v->x = (v->x + a->x * dt);
    }

    // Y-axis integrated error treatment
    if (abs(a->y) < STANDBY_TOLERANCE) {
        if (accel_count[1] > ZERO_COUNT) {
            v->y = 0.0;
            accel_count[1] = 0;
        }
        else {
            v->y = (v->y + a->y * dt);
            accel_count[1] += 1;
/**
 * @brief Integrate acceleration to velocity with threshold-based drift elimination
 * 
 * Simpler drift compensation than getVelocityScaled. Velocity is immediately
 * zeroed when acceleration magnitude falls below the threshold, otherwise
 * integration proceeds normally.
 * 
 * @param[in] a Pointer to acceleration vector (cm/s²)
 * @param[in,out] v Pointer to velocity vector to update (cm/s)
 * @param[in] dt Time step in seconds
 * @param[in] stdby_tol Standby tolerance threshold (cm/s²)
 */
void getVelocityLimited(vector_t *a, vector_t *v, float dt, float stdby_tol)
{
    // X-axis: immediate threshold-based filtering
        accel_count[1] = 0;
        v->y = (v->y + a->y * dt);
    }

    // Z-axis integrated error treatment
    if (abs(a->z) < STANDBY_TOLERANCE) {
        if (accel_count[2] > ZERO_COUNT) {
            v->z = 0.0;
            accel_count[2] = 0;
        }
        else {
            v->z = (v->z + a->z * dt);
/**
 * @brief Compute linear acceleration by removing gravity component
 * 
 * Calculates raw (linear) acceleration by subtracting the gravity vector from
 * measured acceleration. The gravity vector should be pre-rotated to the device
 * frame (e.g., from Madgwick AHRS).
 * 
 * Note: This function also applies axis remapping (Y<->Z swap) during calculation.
 * 
 * @param[in] ma Measured acceleration from IMU (in g's)
 * @param[in] ga Rotated gravity vector in device frame (in g's)
 **
 * @brief Compute filtered linear acceleration using moving average
 * 
 * This function:
 * 1. Calculates raw acceleration (measured - gravity) for the current sample
 * 2. Stores it in a circular buffer at the specified position
 * 3. Computes and returns the moving average of all samples in the buffer
 * 
 * The moving average helps reduce noise and transient spikes in the acceleration data.
 * 
 * @param[in,out] sa Sample array (circular buffer) of size IMU_SAMPLE_SIZE
 * @param[in] ma Measured acceleration from IMU (in g's)
 * @param[in] ga Rotated gravity vector in device frame (in g's)
 * @param[in] count Current position in circular buffer (0 to IMU_SAMPLE_SIZE-1)
 * @return Moving average of linear acceleration (cm/s²)
 * 
 * @note Caller is responsible for maintaining the count index and wrapping it
 *       when it reaches IMU_SAMPLE_SIZE
 */
vector_t getSampledAcceleration(vector_t sa[IMU_SAMPLE_SIZE], vector_t ma, vector_t ga, uint8_t count)
{
    vector_t rawAcc;

    // Store current sample in circular buffer (with Y-Z axis swap)
    sa[count].x = (ma.x + ga.x) * GRAVITY_ACCEL;
    sa[count].y = (ma.z + ga.z) * GRAVITY_ACCEL;
    sa[count].z = (ma.y + ga.y) * GRAVITY_ACCEL;

    // Calculate moving average across all samples        accel_count[2] = 0;
        v->z = (v->z + a->z * dt);
    }
}
/**
 * @brief Calculate the magnitude (Euclidean norm) of an acceleration vector
 * 
 * Computes ||v|| = sqrt(x² + y² + z²)
 * 
 * @param[in] va Acceleration vector
 * @return Magnitude of the acceleration vector
 */

void getVelocityLimited(vector_t *a, vector_t *v, float dt, float stdby_tol)
{
    // X-axis integrated error treatment
    if (abs(a->x) < stdby_tol) {
        v->x = 0.0;
    }
    else {
        v->x = (v->x + a->x * dt);
    }

    // Y-axis integrated error treatment
    if (abs(a->y) < stdby_tol) {
        v->y = 0.0;
    }
    else {
        v->y = (v->y + a->y * dt);
    }

    // Z-axis integrated error treatment
    if (abs(a->z) < stdby_tol) {
        v->z = 0.0;
    }
    else {
        v->z = (v->z + a->z * dt);
    }
}

vector_t getRawAcceleration(vector_t ma, vector_t ga)
{
    vector_t rawAcc;

    rawAcc.x = (ma.x + ga.x) * GRAVITY_ACCEL;
    rawAcc.y = (ma.z + ga.z) * GRAVITY_ACCEL;
    rawAcc.z = (ma.y + ga.y) * GRAVITY_ACCEL;

    return rawAcc;
}

// sa = Sample Vector; ma = Measured Vector; ga = Rotated Gravity Vector; count = Sample Vector Position
vector_t getSampledAcceleration(vector_t sa[IMU_SAMPLE_SIZE], vector_t ma, vector_t ga, uint8_t count)
{
    vector_t rawAcc;

    sa[count].x = (ma.x + ga.x) * GRAVITY_ACCEL;
    sa[count].y = (ma.z + ga.z) * GRAVITY_ACCEL;
    sa[count].z = (ma.y + ga.y) * GRAVITY_ACCEL;

    float sum_ax = 0.0, sum_ay = 0.0, sum_az = 0.0;
    for (int i = 0; i < IMU_SAMPLE_SIZE; i++)
    {  
        sum_ax = sum_ax + sa[i].x;
        sum_ay = sum_ay + sa[i].y;
        sum_az = sum_az + sa[i].z;
    }

    rawAcc.x = sum_ax / IMU_SAMPLE_SIZE;
    rawAcc.y = sum_ay / IMU_SAMPLE_SIZE;
    rawAcc.z = sum_az / IMU_SAMPLE_SIZE;

    return rawAcc;
}

float getAccelerationModule(vector_t va)
{
    return sqrt(va.x*va.x + va.y*va.y + va.z*va.z);
}
