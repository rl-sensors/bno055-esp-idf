//
// Created by bohm on 1/31/24.
//

#include <stdio.h>
#include <freertos/portmacro.h>
#include "esp_log.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c.h"

#ifndef BNO055_TEST_BNO055_H
#define BNO055_TEST_BNO055_H

#include <esp_err.h>

#define BNO055_SENSOR_ADDR              0x29 // 0x29 on TAS fliers

// Page 0
#define BNO055_ID (0xA0)
#define BNO055_CHIP_ID 0x00        // value: 0xA0
#define BNO055_SW_REV_ID_LSB 0x04  // value: 0x08
#define BNO055_SW_REV_ID_MSB 0x05  // value: 0x03
#define BNO055_BL_REV_ID 0x06      // N/A
#define BNO055_PAGE_ID 0x07

#define BNO055_SYS_TRIGGER 0x3F

#define BNO055_TEMP 0x34
#define BNO055_CALIB_STAT 0x35
#define BNO055_ST_RESULT 0x36
#define BNO055_INT_STATUS 0x37
#define BNO055_SYS_CLK_STATUS 0x38
#define BNO055_SYS_STATUS 0x39
#define BNO055_SYS_ERR 0x3A
#define BNO055_UNIT_SEL 0x3B
#define BNO055_OPR_MODE 0x3D
#define BNO055_PWR_MODE 0x3E
#define BNO055_SYS_TRIGGER 0x3F
#define BNO055_TEMP_SOURCE 0x40
#define BNO055_AXIS_MAP_CONFIG 0x41
#define BNO055_AXIS_MAP_SIGN 0x42
#define BNO055_ACC_OFFSET_X_LSB 0x55


typedef enum {  // BNO-55 operation modes
  BNO055_OPERATION_MODE_CONFIG = 0x00,
  // Sensor Mode
  BNO055_OPERATION_MODE_ACCONLY,
  BNO055_OPERATION_MODE_MAGONLY,
  BNO055_OPERATION_MODE_GYRONLY,
  BNO055_OPERATION_MODE_ACCMAG,
  BNO055_OPERATION_MODE_ACCGYRO,
  BNO055_OPERATION_MODE_MAGGYRO,
  BNO055_OPERATION_MODE_AMG,  // 0x07
                              // Fusion Mode
  BNO055_OPERATION_MODE_IMU,
  BNO055_OPERATION_MODE_COMPASS,
  BNO055_OPERATION_MODE_M4G,
  BNO055_OPERATION_MODE_NDOF_FMC_OFF,
  BNO055_OPERATION_MODE_NDOF  // 0x0C
} bno055_opmode_t;

typedef enum {
    BNO055_VECTOR_ACCELEROMETER = 0x08,  // Default: m/s²
    BNO055_VECTOR_MAGNETOMETER = 0x0E,   // Default: uT
    BNO055_VECTOR_GYROSCOPE = 0x14,      // Default: rad/s
    BNO055_VECTOR_EULER = 0x1A,          // Default: degrees
    BNO055_VECTOR_QUATERNION = 0x20,     // No units
    BNO055_VECTOR_LINEARACCEL = 0x28,    // Default: m/s²
    BNO055_VECTOR_GRAVITY = 0x2E         // Default: m/s²
} bno055_vector_type_t;

static esp_err_t register_read(uint8_t reg_addr, uint8_t *data, size_t len);
static esp_err_t register_write_byte(uint8_t reg_addr, uint8_t data);

void bno055_setPage(uint8_t page);
void bno055_delay(int time);
void bno055_setOperationMode(bno055_opmode_t mode);
void bno055_setOperationModeConfig();
void bno055_setOperationModeNDOF();
void bno055_task(void *pvParams);

#endif //BNO055_TEST_BNO055_H
