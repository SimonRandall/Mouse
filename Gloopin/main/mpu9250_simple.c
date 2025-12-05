#include "mpu9250_simple.h"
#include "esp_log.h"

#define REG_PWR_MGMT_1      0x6B
#define REG_ACCEL_XOUT_H    0x3B
#define REG_GYRO_XOUT_H     0x43
#define REG_WHO_AM_I        0x75

// sensitivity divisors for default ±2g and ±250 dps
#define ACCEL_SCALE 16384.0f
#define GYRO_SCALE  131.0f

static const char *TAG = "MPU_SIMPLE";

// small helper for reading a block of registers
static esp_err_t read_bytes(i2c_port_t port, uint8_t reg, uint8_t *data, int len)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();

    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (MPU9250_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg, true);

    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (MPU9250_ADDR << 1) | I2C_MASTER_READ, true);

    for (int i = 0; i < len - 1; i++)
        i2c_master_read_byte(cmd, &data[i], I2C_MASTER_ACK);

    i2c_master_read_byte(cmd, &data[len - 1], I2C_MASTER_NACK);

    i2c_master_stop(cmd);

    esp_err_t ret = i2c_master_cmd_begin(port, cmd, 50 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}

// write one register
static esp_err_t write_reg(i2c_port_t port, uint8_t reg, uint8_t val)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();

    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (MPU9250_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg, true);
    i2c_master_write_byte(cmd, val, true);
    i2c_master_stop(cmd);

    esp_err_t ret = i2c_master_cmd_begin(port, cmd, 50 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}

// ---------------------------------------------------------------
// INIT
// ---------------------------------------------------------------
esp_err_t mpu9250_init(i2c_port_t port)
{
    uint8_t who_am_i = 0;
    esp_err_t ret = read_bytes(port, REG_WHO_AM_I, &who_am_i, 1);

    if (ret != ESP_OK) return ret;

    if (who_am_i != 0x71)
        ESP_LOGW(TAG, "WHO_AM_I mismatch: 0x%02X (expected 0x71)", who_am_i);

    // Wake up chip
    return write_reg(port, REG_PWR_MGMT_1, 0x00);
}

// ---------------------------------------------------------------
// ACCEL READ
// ---------------------------------------------------------------
esp_err_t mpu9250_read_accel(i2c_port_t port,
                             float *ax, float *ay, float *az)
{
    uint8_t buf[6];
    esp_err_t ret = read_bytes(port, REG_ACCEL_XOUT_H, buf, 6);
    if (ret != ESP_OK) return ret;

    int16_t ax_raw = (buf[0] << 8) | buf[1];
    int16_t ay_raw = (buf[2] << 8) | buf[3];
    int16_t az_raw = (buf[4] << 8) | buf[5];

    *ax = ax_raw / ACCEL_SCALE;
    *ay = ay_raw / ACCEL_SCALE;
    *az = az_raw / ACCEL_SCALE;

    return ESP_OK;
}

// ---------------------------------------------------------------
// GYRO READ
// ---------------------------------------------------------------
esp_err_t mpu9250_read_gyro(i2c_port_t port,
                            float *gx, float *gy, float *gz)
{
    uint8_t buf[6];
    esp_err_t ret = read_bytes(port, REG_GYRO_XOUT_H, buf, 6);
    if (ret != ESP_OK) return ret;

    int16_t gx_raw = (buf[0] << 8) | buf[1];
    int16_t gy_raw = (buf[2] << 8) | buf[3];
    int16_t gz_raw = (buf[4] << 8) | buf[5];

    *gx = gx_raw / GYRO_SCALE;
    *gy = gy_raw / GYRO_SCALE;
    *gz = gz_raw / GYRO_SCALE;

    return ESP_OK;
}
