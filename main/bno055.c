//
// Created by bohm on 1/31/24.
//


#include <math.h>
#include "bno055.h"

extern char *TAG;

#define I2C_MASTER_TIMEOUT_MS       1000

void bno055_task(void *pvParams) {
    // reset
    ESP_LOGI(TAG, "Going to reset the BNO");
    register_write_byte(0x00, 0x00);
//    register_write_byte(BNO055_SYS_TRIGGER, 0x20);
    bno055_delay(700);

    uint8_t id = 0;
    register_read(BNO055_CHIP_ID, &id, 1);
    if (id != BNO055_ID) {
        ESP_LOGE(TAG, "Can't find BNO055, id: 0x%02x. Please check your wiring.\r\n", id);
    } else {
        ESP_LOGI(TAG, "BNO055 Found and reset");

        bno055_setPage(0);
        register_write_byte(BNO055_SYS_TRIGGER, 0x0);

        // Select BNO055 config mode
        bno055_setOperationModeConfig();
        bno055_delay(10);
        // the units don't work - only the orientation Android vs Windows works
        uint8_t unitsel = (1 << 7) | // Orientation = Android
                          (0 << 4) | // Temperature = Celsius
                          (1 << 2) | // Euler = Rads
                          (1 << 1) | // Gyro = Rads per sec
                          (0 << 0);  // Accelerometer = m/s^2
        register_write_byte(BNO055_UNIT_SEL, unitsel);
//        register_write_byte(BNO055_UNIT_SEL, 0b10000110);
        bno055_delay(10);

        bno055_setOperationModeNDOF();

        uint8_t frequency = 10;
        TickType_t xLastWakeTime;
        const TickType_t xFrequency = frequency / portTICK_PERIOD_MS;

        uint8_t bno_euler[8];
        uint8_t bno_gyro[6];

        while (1) {
            xLastWakeTime = xTaskGetTickCount();
            register_read(BNO055_VECTOR_EULER, bno_euler, 6);
            //ESP_LOGI(TAG, "[%d] Yaw: %d, Roll: %d, Pitch: %d", xTaskGetTickCount() / portTICK_PERIOD_MS, bno_euler[0], bno_euler[1], bno_euler[2]);

            register_read(BNO055_VECTOR_GYROSCOPE, bno_gyro, 6);
            //ESP_LOGI(TAG, "[%d] Yaw Dot: %d, Roll Dot: %d, Pitch Dot: %d", xTaskGetTickCount() / portTICK_PERIOD_MS, bno_gyro[2], bno_gyro[1], bno_gyro[0]);
            uint32_t end = xTaskGetTickCount();
            ESP_LOGI(TAG, "[%lu]\tY: %d\tR: %d\tP: %d\tYD: %d\tRD: %d\tPD: %d",
                    end,
                     bno_euler[0], bno_euler[1], bno_euler[2],
                     bno_gyro[2], bno_gyro[1], bno_gyro[0]);

            vTaskDelayUntil(&xLastWakeTime, xFrequency);
        }
    }

    ESP_ERROR_CHECK(i2c_driver_delete(CONFIG_I2C_MASTER_NUM));
    ESP_LOGI(TAG, "I2C unitialized successfully");
    vTaskDelete(NULL);
}

static esp_err_t register_read(uint8_t reg_addr, uint8_t *data, size_t len)
{
    return i2c_master_write_read_device(CONFIG_I2C_MASTER_NUM, BNO055_SENSOR_ADDR, &reg_addr, 1, data, len, pdMS_TO_TICKS(I2C_MASTER_TIMEOUT_MS));
}

static esp_err_t register_write_byte(uint8_t reg_addr, uint8_t data)
{
    int ret;
    uint8_t write_buf[2] = {reg_addr, data};

    ret = i2c_master_write_to_device(CONFIG_I2C_MASTER_NUM, BNO055_SENSOR_ADDR, write_buf, sizeof(write_buf), pdMS_TO_TICKS(I2C_MASTER_TIMEOUT_MS));

    return ret;
}

void bno055_setPage(uint8_t page) {
    register_write_byte(BNO055_PAGE_ID, page);
}

void bno055_delay(int time) {
    vTaskDelay(pdMS_TO_TICKS(time));;
}

void bno055_setOperationMode(bno055_opmode_t mode) {
    register_write_byte(BNO055_OPR_MODE, mode);
    if (mode == BNO055_OPERATION_MODE_CONFIG) {
        bno055_delay(19);
    } else {
        bno055_delay(7);
    }
}

void bno055_setOperationModeConfig() {
    bno055_setOperationMode(BNO055_OPERATION_MODE_CONFIG);
}

void bno055_setOperationModeNDOF() {
    bno055_setOperationMode(BNO055_OPERATION_MODE_NDOF);
}