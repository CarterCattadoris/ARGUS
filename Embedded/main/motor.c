#include "motor.h"
#include "command.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "MOTOR";

esp_err_t motor_init(void) {
    // Timer shared by all motor LEDC channels
    ledc_timer_config_t motor_timer = {
        .speed_mode      = MOTOR_LEDC_MODE,
        .duty_resolution = MOTOR_PWM_RESOLUTION,
        .timer_num       = MOTOR_LEDC_TIMER,
        .freq_hz         = MOTOR_PWM_FREQ_HZ,
        .clk_cfg         = LEDC_AUTO_CLK,
    };
    ESP_ERROR_CHECK(ledc_timer_config(&motor_timer));

    // IN1 — left motor PWM (forward)
    ledc_channel_config_t left_ch = {
        .speed_mode = MOTOR_LEDC_MODE,
        .channel    = MOTOR_LEFT_CHANNEL,
        .timer_sel  = MOTOR_LEDC_TIMER,
        .gpio_num   = MOTOR_LEFT_IN1,
        .duty       = 0,
        .hpoint     = 0,
    };
    ESP_ERROR_CHECK(ledc_channel_config(&left_ch));

    // IN3 — right motor PWM (forward)
    ledc_channel_config_t right_ch = {
        .speed_mode = MOTOR_LEDC_MODE,
        .channel    = MOTOR_RIGHT_CHANNEL,
        .timer_sel  = MOTOR_LEDC_TIMER,
        .gpio_num   = MOTOR_RIGHT_IN3,
        .duty       = 0,
        .hpoint     = 0,
    };
    ESP_ERROR_CHECK(ledc_channel_config(&right_ch));

    // IN2 and IN4 as plain GPIO for direction
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

void motor_set(float left, float right, int dir) {
    if (left < 0.0f) left = 0.0f;
    if (left > 1.0f) left = 1.0f;
    if (right < 0.0f) right = 0.0f;
    if (right > 1.0f) right = 1.0f;

    uint32_t left_duty = (uint32_t)(left * MOTOR_PWM_MAX_DUTY);
    uint32_t right_duty = (uint32_t)(right * MOTOR_PWM_MAX_DUTY);

    if (dir == 1) {
        // Forward: PWM on IN1/IN3, IN2/IN4 LOW
        gpio_set_level(MOTOR_LEFT_IN2, 0);
        gpio_set_level(MOTOR_RIGHT_IN4, 0);

        ESP_ERROR_CHECK(ledc_set_duty(MOTOR_LEDC_MODE, MOTOR_LEFT_CHANNEL, left_duty));
        ESP_ERROR_CHECK(ledc_set_duty(MOTOR_LEDC_MODE, MOTOR_RIGHT_CHANNEL, right_duty));
    } else if (dir == -1) {
        // Reverse: IN1/IN3 LOW, PWM on IN2/IN4
        // Stop PWM channels first
        ESP_ERROR_CHECK(ledc_set_duty(MOTOR_LEDC_MODE, MOTOR_LEFT_CHANNEL, 0));
        ESP_ERROR_CHECK(ledc_set_duty(MOTOR_LEDC_MODE, MOTOR_RIGHT_CHANNEL, 0));
        ESP_ERROR_CHECK(ledc_update_duty(MOTOR_LEDC_MODE, MOTOR_LEFT_CHANNEL));
        ESP_ERROR_CHECK(ledc_update_duty(MOTOR_LEDC_MODE, MOTOR_RIGHT_CHANNEL));

        // Set IN2/IN4 HIGH, IN1/IN3 held LOW by 0 duty
        gpio_set_level(MOTOR_LEFT_IN2, 1);
        gpio_set_level(MOTOR_RIGHT_IN4, 1);

        // PWM on IN1/IN3 inverted: high duty = more braking, so we invert
        ESP_ERROR_CHECK(ledc_set_duty(MOTOR_LEDC_MODE, MOTOR_LEFT_CHANNEL, MOTOR_PWM_MAX_DUTY - left_duty));
        ESP_ERROR_CHECK(ledc_set_duty(MOTOR_LEDC_MODE, MOTOR_RIGHT_CHANNEL, MOTOR_PWM_MAX_DUTY - right_duty));
    }

    ESP_ERROR_CHECK(ledc_update_duty(MOTOR_LEDC_MODE, MOTOR_LEFT_CHANNEL));
    ESP_ERROR_CHECK(ledc_update_duty(MOTOR_LEDC_MODE, MOTOR_RIGHT_CHANNEL));

    ESP_LOGI(TAG, "dir=%d left_duty=%lu right_duty=%lu", dir, left_duty, right_duty);
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
        motor_set(cmd.left, cmd.right, cmd.dir);
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}