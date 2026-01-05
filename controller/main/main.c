/**
 * @file main.c
 * @brief ESP32 Robot Manipulator Motion Controller
 * 
 * This application implements a motion controller for a robot manipulator using:
 * - MPU9250 9-axis IMU for motion sensing
 * - Madgwick AHRS algorithm for orientation estimation
 * - Velocity and acceleration calculation with drift compensation
 * - GPIO buttons for gripper and brake control
 * 
 * The system reads IMU data at ~100Hz, processes it through sensor fusion,
 * removes gravity components, and outputs motion data via serial interface.
 * 
 * @author Gustavo Eismann
 * @date 2022
 */

#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "esp_log.h"
#include "esp_system.h"
#include "esp_err.h"
#include "esp_task_wdt.h"
#include "esp_timer.h"

#include "driver/i2c.h"
#include "driver/gpio.h"

#include "mpu9250.h"
#include "callibration.h"
#include "common.h"
#include "madgwick_ahrs.h"
#include "dynamic_equations.h"

/* ==================== Configuration Defines ==================== */

/** @brief I2C port number for MPU9250 communication */
#define I2C_MASTER_NUM I2C_NUM_0

/** @brief Uncomment to enter calibration mode for new sensor calibration values */
// #define CONFIG_CALIBRATION_MODE

/** @brief GPIO pin for brake/enable button (Trigger 2) */
#define BREAK_BUTTON 17

/** @brief GPIO pin for gripper control button (Trigger 1) */
#define GRIPP_BUTTON 16

/* ==================== Global Variables ==================== */

/** @brief Previous timestamp in microseconds */
uint64_t t1 = 0;

/** @brief Current timestamp in microseconds */
uint64_t t2 = 0;

/** @brief Delta time between iterations in seconds */
float dt = 0.1;

/** @brief Circular buffer for acceleration moving average filter */
vector_t sampled_va[IMU_SAMPLE_SIZE];

/** @brief Current position in acceleration sample buffer */
/* ==================== IMU Processing Functions ==================== */

/**
 * @brief Main IMU processing loop
 * 
 * This function implements the core motion sensing algorithm:
 * 1. Initialize MPU9250 sensor and Madgwick AHRS filter
 * 2. Continuously read accelerometer, gyroscope, and magnetometer data
 * 3. Transform sensor data to device coordinate frame
 * 4. Apply Madgwick sensor fusion for orientation estimation
 * 5. Compute gravity vector in device frame
 * 6. Calculate linear acceleration (measured - gravity)
 * 7. Integrate to velocity with drift compensation
 * 8. Output motion data via serial in JSON format
 * 
 * The loop runs at approximately 100Hz (determined by pause() function).
 * Motion data is disabled for the first 10 seconds and when brake button is released.
 */
void run_imu(void)
{
    // Initialize IMU with calibration data
    i2c_mpu9250_init(&cal);
    
    // Initialize Madgwick AHRS filter (beta=0.8 for good dynamic response)
    MadgwickAHRSinit(SAMPLE_FREQ_Hz, 0.8);

    // Initialize acceleration sample buffer to zero
/* ==================== Sensor Calibration Data ==================== */

/**
 * @brief Pre-computed calibration values for MPU9250 sensor
 * 
 * These values were obtained through the calibration procedure and compensate for:
 * - Magnetometer: Hard-iron offset and soft-iron scale factors
 * - Accelerometer: Bias offset and scale correction for each axis
 * - Gyroscope: Bias offset for drift compensation
 * 
 * @note To obtain new calibration values, enable CONFIG_CALIBRATION_MODE
 */
calibration_t cal = {
    .mag_offset = {.x = 23.232422, .y = 8.421875, .z = 2.296875},
    .mag_scale = {.x = 0.796865, .y = 1.719117, .z = 0.859559},

    .accel_offset = {.x = 0.029853, .y = 0.019898, .z = -0.136127},
    .accel_scale_lo = {.x = -0.994985, .y = -0.994987, .z = -1.080485},
    .accel_scale_hi = {.x = 1.005897, .y = 1.006174, .z = 0.937822},
// Initialize timestamp
    t1 = esp_timer_get_time();

    uint64_t i = 0;  // Iteration counter for periodic output
    while (true)
    {
        // Vector declarations
        vector_t va, vg, vm;    // Measured Accel, Gyro and Mag from IMU
        vector_t vv;            // Calculated velocity vector
        vector_t ga, ra;        // Gravity and Raw (linear) Acceleration

        // Read raw sensor data from MPU9250
        ESP_ERROR_CHECK(get_accel_gyro_mag(&va, &vg, &vm));

        // Transform sensor readings to device coordinate frame
        transform_accel_gyro(&va);
        transform_accel_gyro(&vg);
        transform_mag(&vm);

        // Update Madgwick AHRS filter with sensor data
        // Gyro values converted from degrees to radians
        MadgwickAHRSupdate(DEG2RAD(vg.x), DEG2RAD(vg.y), DEG2RAD(vg.z),
                            va.x, va.y, va.z,
                            vm.x, vm.y, vm.z);
        
        // Get gravity vector rotated to current device orientation
        ga = getRotatedGravityVector();
// Read IMU temperature for monitoring
            float temp;
            ESP_ERROR_CHECK(get_temperature_celsius(&temp));

            // Get orientation as Euler angles (heading, pitch, roll) in degrees
            float heading, pitch, roll;
            MadgwickGetEulerAnglesDegrees(&heading, &pitch, &roll);

            // Calculate linear acceleration by removing gravity component
            ra = getRawAcceleration(va, ga);
            // Alternative: Use moving average filter (currently disabled)
            // ra = getSampledAcceleration(sampled_va, va, ga, sCount);
            
            // Update sample buffer index for moving average (wraps at IMU_SAMPLE_SIZE)
            sCount = sCount + 1;
            if (sCount >= IMU_SAMPLE_SIZE) {
                sCount = 0;
            }

            // Calculate time elapsed since last iteration
            t2 = esp_timer_get_time();
            dt = (t2 - t1) / 1000000.0;  // Convert microseconds to seconds

            // Store current time for next iteration's dt calculation
            t1 = esp_timer_get_time();

            // Integrate acceleration to velocity with drift compensation
            getVelocityScaled(&ra, &vv, dt);

            // Zero motion data during initial 10-second warm-up period
            // This allows the AHRS filter to stabilize
            if (t1 < (10 * 1000000.0))  // 10 seconds in microsecondsees(&heading, &pitch, &roll);
            // ESP_LOGI(TAG, "heading: %2.3f°, pitch: %2.3f°, roll: %2.3f°, Temp %2.3f°C", heading, pitch, roll, temp);

            // Compute Raw Acceleration from Moving Average
            ra = getRawAcceleration(va, ga);
            // ra = getSampledAcceleration(sampled_va, va, ga, sCount);
            
            // Increments counter to index the moving average function
            sCount = sCount + 1;
            if (sCount < IMU_SAMPLE_SIZE) {
                sCount = 0;
            }

            // Compute this iteration time of execution (elapsed time between iterations)
            t2 = esp_timer_get_time();
            dt = (t2 - t1) / 1000000.0;

            // Get time before start new iteration
            t1 = esp_timer_get_time();

            // Integrate Acceleration to obtain Velocity
            getVelocityScaled(&ra, &vv, dt);

            if (t1 < (10 * 1000000.0))
            {
                ra.x = 0.0;
                ra.y = 0.0;
            // Zero motion data when brake button is not pressed (btn2 == 0)
            // This acts as an enable/disable switch for motion tracking
                ra.z = 0.0;
                
                vv.x = 0.0;
                vv.y = 0.0;
                vv.z = 0.0;
            }

            if (btn2 == 0)
            {
               Output motion data as JSON via serial interface
            // Format: {"dt":time_delta, "vx,vy,vz":velocity, "ax,ay,az":acceleration, 
            //          "ha":heading, "b1":gripper_btn, "b2":brake_btn}
            printf("{\"dt\":%.4f,\"vx\":%2.1f,\"ax\":%2.1f,\"vy\":%2.1f,\"ay\":%2.1f,\"vz\":%2.1f,\"az\":%2.1f,\"ha\":%1.0f,\"b1\":%d,\"b2\":%d}\n", 
                    dt, vv.x, ra.x, vv.y, ra.y, vv.z, ra.z, heading, btn1, btn2);

            // Reset watchdog timer to prevent system reset
            esp_task_wdt_reset();
        }

        // Wait for next sample period (~100Hz loop rate)
        pause();
    }
}

/* ==================== FreeRTOS Task Functions ==================== */

/**// Calibration mode: run sensor calibration procedures
    calibrate_gyro();
    calibrate_accel();
    calibrate_mag();
    #else
    // Normal operation mode: run IMU processing loop
    run_imu();
    #endif

    // Clean up (this code is never reached in normal operation)
    vTaskDelay(200 / portTICK_RATE_MS);
    i2c_driver_delete(I2C_MASTER_NUM);

    vTaskDelete(NULL);
}

/**
 * @brief FreeRTOS task for monitoring button inputs
 * 
 * Continuously polls two GPIO buttons:
 * - GRIPP_BUTTON (GPIO 16): Controls gripper state
 * - BREAK_BUTTON (GPIO 17): Enables/disables motion tracking
 * 
 * Button states are stored in global variables btn1 and btn2.
 * 
 * @param[in] arg Unused task parameter
 */
static void buttons_task(void *arg)
{
    // Configure brake button with internal pull-up resistor
    gpio_set_direction(BREAK_BUTTON, GPIO_MODE_INPUT);
    gpio_set_pull_mode(BREAK_BUTTON, GPIO_PULLUP_ONLY);

    // Configure gripper button with internal pull-up resistor
    gpio_set_direction(GRIPP_BUTTON, GPIO_MODE_INPUT);
    gpio_set_pull_mode(GRIPP_BUTTON, GPIO_PULLUP_ONLY);

    // Button polling loop
    while (1)
    {
        // Read brake button state (1=pressed, 0=released)
        if (gpio_get_level(BREAK_BUTTON) == 1)
        {
            btn2 = 1;  // Enable motion tracking
        }
        else
        {
            btn2 = 0;  // Disable motion tracking
        }

        // Read gripper button state (1=pressed, 0=released)
        if (gpio_get_level(GRIPP_BUTTON) == 1)
        {
            btn1 = 1;  // Gripper active
        }
        else
        {
            btn1 = 0;  // Gripper inactive
        }

        // Poll every 10ms for responsive button detection
        vTaskDelay(10 / portTICK_RATE_MS);
    }
}

/**
 * @brief Application entry point
 * 
 * Creates two FreeRTOS tasks:
 * - imu_task: High priority (9) for time-critical IMU processing
 * - buttons_task: Higher priority (10) for responsive button handling
 */
void app_main(void)
{
    // Create IMU processing task with 4KB stack
    xTaskCreate(imu_task, "imu_task", 4096, NULL, 9, NULL);
    
    // Create button monitoring task with 1KB stack
        if (gpio_get_level(GRIPP_BUTTON) == 1)
        {
            // Implement gripper enable
            btn1 = 1;
        }
        else
        {
            // Implement gripper disable
            btn1 = 0;
        }

        vTaskDelay(10 / portTICK_RATE_MS);
    }

}

void app_main(void)
{
    //start i2c task
    xTaskCreate(imu_task, "imu_task", 4096, NULL, 9, NULL);
    xTaskCreate(buttons_task, "buttons_task", 1024, NULL, 10, NULL);
}
