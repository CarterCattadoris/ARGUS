#include "motor.h"
#include "command.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <math.h>

static const char *TAG = "MOTOR";

esp_err_t motor_init(void) {
    ledc_timer_config_t motor_timer = {
        .speed_mode      = MOTOR_LEDC_MODE,
        .duty_resolution = MOTOR_PWM_RESOLUTION,
        .timer_num       = MOTOR_LEDC_TIMER,
        .freq_hz         = MOTOR_PWM_FREQ_HZ,
        .clk_cfg         = LEDC_AUTO_CLK,
    };
    ESP_ERROR_CHECK(ledc_timer_config(&motor_timer));

    ledc_channel_config_t left_ch = {
        .speed_mode = MOTOR_LEDC_MODE,
        .channel    = MOTOR_LEFT_CHANNEL,
        .timer_sel  = MOTOR_LEDC_TIMER,
        .gpio_num   = MOTOR_LEFT_IN1,
        .duty       = 0,
        .hpoint     = 0,
    };
    ESP_ERROR_CHECK(ledc_channel_config(&left_ch));

    ledc_channel_config_t right_ch = {
        .speed_mode = MOTOR_LEDC_MODE,
        .channel    = MOTOR_RIGHT_CHANNEL,
        .timer_sel  = MOTOR_LEDC_TIMER,
        .gpio_num   = MOTOR_RIGHT_IN3,
        .duty       = 0,
        .hpoint     = 0,
    };
    ESP_ERROR_CHECK(ledc_channel_config(&right_ch));

    gpio_config_t dir_pins = {
        .pin_bit_mask = (1ULL << MOTOR_LEFT_IN2) |
                        (1ULL << MOTOR_RIGHT_IN4),
        .mode         = GPIO_MODE_OUTPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&dir_pins));

    ESP_LOGI(TAG, "Motor init complete");
    return ESP_OK;
}

static void set_single_motor(float val, ledc_channel_t channel, gpio_num_t dir_pin) {
    // Deadband
    if (fabsf(val) < 0.05f) {
        gpio_set_level(dir_pin, 0);
        ESP_ERROR_CHECK(ledc_set_duty(MOTOR_LEDC_MODE, channel, 0));
        ESP_ERROR_CHECK(ledc_update_duty(MOTOR_LEDC_MODE, channel));
        return;
    }

    // Clamp
    if (val < -1.0f) val = -1.0f;
    if (val > 1.0f) val = 1.0f;

    uint32_t duty = (uint32_t)(fabsf(val) * MOTOR_PWM_MAX_DUTY);

    if (val > 0.0f) {
        // Forward: IN2/IN4 HIGH (motors wired reversed), PWM on IN1/IN3
        gpio_set_level(dir_pin, 1);
        ESP_ERROR_CHECK(ledc_set_duty(MOTOR_LEDC_MODE, channel, duty));
    } else {
        // Reverse: IN2/IN4 LOW, PWM inverted on IN1/IN3
        gpio_set_level(dir_pin, 0);
        ESP_ERROR_CHECK(ledc_set_duty(MOTOR_LEDC_MODE, channel, MOTOR_PWM_MAX_DUTY - duty));
    }

    ESP_ERROR_CHECK(ledc_update_duty(MOTOR_LEDC_MODE, channel));
}

void motor_set(float left, float right) {
    set_single_motor(left, MOTOR_LEFT_CHANNEL, MOTOR_LEFT_IN2);
    set_single_motor(right, MOTOR_RIGHT_CHANNEL, MOTOR_RIGHT_IN4);
}

void motor_stop(void) {
    gpio_set_level(MOTOR_LEFT_IN2, 0);
    gpio_set_level(MOTOR_RIGHT_IN4, 0);
    ESP_ERROR_CHECK(ledc_set_duty(MOTOR_LEDC_MODE, MOTOR_LEFT_CHANNEL, 0));
    ESP_ERROR_CHECK(ledc_set_duty(MOTOR_LEDC_MODE, MOTOR_RIGHT_CHANNEL, 0));
    ESP_ERROR_CHECK(ledc_update_duty(MOTOR_LEDC_MODE, MOTOR_LEFT_CHANNEL));
    ESP_ERROR_CHECK(ledc_update_duty(MOTOR_LEDC_MODE, MOTOR_RIGHT_CHANNEL));
}

void motor_task(void *pvParameters) {
    motor_cmd_t cmd;

    while (1) {
        command_get(&cmd);
        // ESP_LOGI("MTASK", "L=%.2f R=%.2f", cmd.left, cmd.right);
        motor_set(cmd.left, cmd.right);
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}