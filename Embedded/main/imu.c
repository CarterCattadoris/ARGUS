#include "imu.h"

#include <string.h>
#include <math.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c_master.h"
#include "esp_log.h"
#include "cJSON.h"

#include "lwip/sockets.h"
#include "lwip/netdb.h"

static const char *TAG = "imu";

/* ── I2C handles ────────────────────────────────────── */
static i2c_master_bus_handle_t  s_bus_handle;
static i2c_master_dev_handle_t  s_dev_handle;

/* ── Sensitivity divisors (default ±250°/s, ±2g) ───── */
#define GYRO_SENSITIVITY   131.0f   /* LSB per °/s  */
#define ACCEL_SENSITIVITY  16384.0f /* LSB per g    */

/* ── Gyro bias calibration ──────────────────────────── */
#define CALIBRATION_SAMPLES 100     /* ~2 s at 50 Hz */
static float s_gx_bias = 0.0f;
static float s_gy_bias = 0.0f;
static float s_gz_bias = 0.0f;

/* ── Low-level helpers ──────────────────────────────── */

static esp_err_t mpu6050_write_reg(uint8_t reg, uint8_t val)
{
    uint8_t buf[2] = { reg, val };
    return i2c_master_transmit(s_dev_handle, buf, sizeof(buf), -1);
}

static esp_err_t mpu6050_read_regs(uint8_t start_reg, uint8_t *out, size_t len)
{
    return i2c_master_transmit_receive(s_dev_handle,
                                       &start_reg, 1,
                                       out, len, -1);
}

/* ── Bus + device init ──────────────────────────────── */

static esp_err_t i2c_bus_init(void)
{
    i2c_master_bus_config_t bus_cfg = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port   = I2C_NUM_0,
        .sda_io_num = IMU_SDA_GPIO,
        .scl_io_num = IMU_SCL_GPIO,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    esp_err_t ret = i2c_new_master_bus(&bus_cfg, &s_bus_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "failed to create I2C bus: %s", esp_err_to_name(ret));
        return ret;
    }

    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address  = MPU6050_ADDR,
        .scl_speed_hz    = IMU_I2C_FREQ_HZ,
    };
    ret = i2c_master_bus_add_device(s_bus_handle, &dev_cfg, &s_dev_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "failed to add MPU6050 device: %s", esp_err_to_name(ret));
    }
    return ret;
}

/* ── MPU6050 configuration ──────────────────────────── */

static esp_err_t mpu6050_init(void)
{
    /* WHO_AM_I check — should return 0x68 */
    uint8_t who = 0;
    esp_err_t ret = mpu6050_read_regs(MPU6050_REG_WHO_AM_I, &who, 1);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "WHO_AM_I read failed");
        return ret;
    }
    ESP_LOGI(TAG, "WHO_AM_I = 0x%02X", who);
    if (who != 0x68) {
        ESP_LOGW(TAG, "unexpected WHO_AM_I value (expected 0x68)");
    }

    /* Wake up — clear sleep bit */
    ret = mpu6050_write_reg(MPU6050_REG_PWR_MGMT_1, 0x00);
    if (ret != ESP_OK) return ret;

    /* Small delay for oscillator startup */
    vTaskDelay(pdMS_TO_TICKS(50));

    /* Sample rate divider: sample_rate = 1kHz / (1+div)
       div=19 → 50 Hz internal sample rate, matches our send rate */
    ret = mpu6050_write_reg(MPU6050_REG_SMPLRT_DIV, 19);
    if (ret != ESP_OK) return ret;

    /* DLPF config: bandwidth ~21 Hz (register value 4) */
    ret = mpu6050_write_reg(MPU6050_REG_CONFIG, 4);
    if (ret != ESP_OK) return ret;

    /* Gyro: ±250 °/s (FS_SEL = 0) */
    ret = mpu6050_write_reg(MPU6050_REG_GYRO_CFG, 0x00);
    if (ret != ESP_OK) return ret;

    /* Accel: ±2g (AFS_SEL = 0) */
    ret = mpu6050_write_reg(MPU6050_REG_ACCEL_CFG, 0x00);
    if (ret != ESP_OK) return ret;

    ESP_LOGI(TAG, "MPU6050 configured (±250°/s, ±2g, 50 Hz, DLPF ~21 Hz)");

    /* ── Gyro bias calibration ── */
    ESP_LOGI(TAG, "calibrating gyro bias (%d samples, keep car still)...",
             CALIBRATION_SAMPLES);

    float sum_gx = 0.0f, sum_gy = 0.0f, sum_gz = 0.0f;
    uint8_t raw[MPU6050_RAW_DATA_LEN];

    for (int i = 0; i < CALIBRATION_SAMPLES; i++) {
        ret = mpu6050_read_regs(MPU6050_REG_ACCEL_XOUT_H,
                                raw, MPU6050_RAW_DATA_LEN);
        if (ret != ESP_OK) {
            ESP_LOGW(TAG, "cal read failed at sample %d, retrying", i);
            i--;
            vTaskDelay(pdMS_TO_TICKS(IMU_SEND_INTERVAL_MS));
            continue;
        }

        sum_gx += (int16_t)((raw[8]  << 8) | raw[9])  / GYRO_SENSITIVITY;
        sum_gy += (int16_t)((raw[10] << 8) | raw[11]) / GYRO_SENSITIVITY;
        sum_gz += (int16_t)((raw[12] << 8) | raw[13]) / GYRO_SENSITIVITY;

        vTaskDelay(pdMS_TO_TICKS(IMU_SEND_INTERVAL_MS));
    }

    s_gx_bias = sum_gx / CALIBRATION_SAMPLES;
    s_gy_bias = sum_gy / CALIBRATION_SAMPLES;
    s_gz_bias = sum_gz / CALIBRATION_SAMPLES;

    ESP_LOGI(TAG, "gyro bias: gx=%.2f gy=%.2f gz=%.2f",
             s_gx_bias, s_gy_bias, s_gz_bias);

    return ESP_OK;
}

/* ── Telemetry task ─────────────────────────────────── */

static void imu_task(void *pv)
{
    /* Create UDP socket */
    int sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (sock < 0) {
        ESP_LOGE(TAG, "failed to create UDP socket");
        vTaskDelete(NULL);
        return;
    }

    struct sockaddr_in dest = {
        .sin_family = AF_INET,
        .sin_port   = htons(IMU_TARGET_PORT),
    };
    inet_pton(AF_INET, IMU_TARGET_IP, &dest.sin_addr);

    uint8_t raw[MPU6050_RAW_DATA_LEN];

    for (;;) {
        /* Burst-read accel (6) + temp (2) + gyro (6) = 14 bytes */
        esp_err_t ret = mpu6050_read_regs(MPU6050_REG_ACCEL_XOUT_H,
                                           raw, MPU6050_RAW_DATA_LEN);
        if (ret != ESP_OK) {
            ESP_LOGW(TAG, "I2C read failed, skipping");
            vTaskDelay(pdMS_TO_TICKS(IMU_SEND_INTERVAL_MS));
            continue;
        }

        /* Combine high/low bytes (big-endian from MPU6050) */
        int16_t ax_raw = (int16_t)((raw[0]  << 8) | raw[1]);
        int16_t ay_raw = (int16_t)((raw[2]  << 8) | raw[3]);
        int16_t az_raw = (int16_t)((raw[4]  << 8) | raw[5]);
        /* raw[6..7] = temperature, skip */
        int16_t gx_raw = (int16_t)((raw[8]  << 8) | raw[9]);
        int16_t gy_raw = (int16_t)((raw[10] << 8) | raw[11]);
        int16_t gz_raw = (int16_t)((raw[12] << 8) | raw[13]);

        /* Scale to physical units */
        float ax = ax_raw / ACCEL_SENSITIVITY;
        float ay = ay_raw / ACCEL_SENSITIVITY;
        float az = az_raw / ACCEL_SENSITIVITY;
        float gx = gx_raw / GYRO_SENSITIVITY - s_gx_bias;
        float gy = gy_raw / GYRO_SENSITIVITY - s_gy_bias;
        float gz = gz_raw / GYRO_SENSITIVITY - s_gz_bias;
        // ESP_LOGI(TAG, "ax=%.2f ay=%.2f az=%.2f gx=%.2f gy=%.2f gz=%.2f", ax, ay, az, gx, gy, gz);

        /* Build JSON payload */
        cJSON *root = cJSON_CreateObject();
        cJSON_AddNumberToObject(root, "ax", ax);
        cJSON_AddNumberToObject(root, "ay", ay);
        cJSON_AddNumberToObject(root, "az", az);
        cJSON_AddNumberToObject(root, "gx", gx);
        cJSON_AddNumberToObject(root, "gy", gy);
        cJSON_AddNumberToObject(root, "gz", gz);

        char *json_str = cJSON_PrintUnformatted(root);
        if (json_str) {
            sendto(sock, json_str, strlen(json_str), 0,
                   (struct sockaddr *)&dest, sizeof(dest));
            cJSON_free(json_str);
        }
        cJSON_Delete(root);

        vTaskDelay(pdMS_TO_TICKS(IMU_SEND_INTERVAL_MS));
    }
}

/* ── Public API ─────────────────────────────────────── */

esp_err_t imu_init(void)
{
    esp_err_t ret = i2c_bus_init();
    if (ret != ESP_OK) return ret;

    ret = mpu6050_init();
    if (ret != ESP_OK) return ret;

    xTaskCreate(imu_task, "imu_task", IMU_TASK_STACK, NULL,
                IMU_TASK_PRIORITY, NULL);
    ESP_LOGI(TAG, "IMU task started (streaming to %s:%d at ~%d Hz)",
             IMU_TARGET_IP, IMU_TARGET_PORT,
             1000 / IMU_SEND_INTERVAL_MS);
    return ESP_OK;
}