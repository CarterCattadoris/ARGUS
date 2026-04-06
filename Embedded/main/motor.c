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

    /* Left motor — IN1 (forward PWM) */
    ledc_channel_config_t left_fwd = {
        .speed_mode = MOTOR_LEDC_MODE,
        .channel    = MOTOR_LEFT_CHANNEL,
        .timer_sel  = MOTOR_LEDC_TIMER,
        .gpio_num   = MOTOR_LEFT_IN1,
        .duty       = 0,
        .hpoint     = 0,
    };
    ESP_ERROR_CHECK(ledc_channel_config(&left_fwd));

    /* Left motor — IN2 (reverse PWM) */
    ledc_channel_config_t left_rev = {
        .speed_mode = MOTOR_LEDC_MODE,
        .channel    = MOTOR_LEFT_REV_CHANNEL,
        .timer_sel  = MOTOR_LEDC_TIMER,
        .gpio_num   = MOTOR_LEFT_IN2,
        .duty       = 0,
        .hpoint     = 0,
    };
    ESP_ERROR_CHECK(ledc_channel_config(&left_rev));

    /* Right motor — IN3 (forward PWM) */
    ledc_channel_config_t right_fwd = {
        .speed_mode = MOTOR_LEDC_MODE,
        .channel    = MOTOR_RIGHT_CHANNEL,
        .timer_sel  = MOTOR_LEDC_TIMER,
        .gpio_num   = MOTOR_RIGHT_IN3,
        .duty       = 0,
        .hpoint     = 0,
    };
    ESP_ERROR_CHECK(ledc_channel_config(&right_fwd));

    /* Right motor — IN4 (reverse PWM) */
    ledc_channel_config_t right_rev = {
        .speed_mode = MOTOR_LEDC_MODE,
        .channel    = MOTOR_RIGHT_REV_CHANNEL,
        .timer_sel  = MOTOR_LEDC_TIMER,
        .gpio_num   = MOTOR_RIGHT_IN4,
        .duty       = 0,
        .hpoint     = 0,
    };
    ESP_ERROR_CHECK(ledc_channel_config(&right_rev));

    ESP_LOGI(TAG, "Motor init complete (4-channel LEDC)");
    return ESP_OK;
}

static void set_single_motor(float val, ledc_channel_t fwd_ch, ledc_channel_t rev_ch) {
    /* Deadband */
    if (fabsf(val) < 0.05f) {
        ledc_set_duty(MOTOR_LEDC_MODE, fwd_ch, 0);
        ledc_update_duty(MOTOR_LEDC_MODE, fwd_ch);
        ledc_set_duty(MOTOR_LEDC_MODE, rev_ch, 0);
        ledc_update_duty(MOTOR_LEDC_MODE, rev_ch);
        return;
    }

    /* Clamp */
    if (val < -1.0f) val = -1.0f;
    if (val >  1.0f) val =  1.0f;

    uint32_t duty = (uint32_t)(fabsf(val) * MOTOR_PWM_MAX_DUTY);

    if (val > 0.0f) {
        /* Forward: PWM on IN1/IN3, IN2/IN4 = 0 */
        ledc_set_duty(MOTOR_LEDC_MODE, fwd_ch, duty);
        ledc_update_duty(MOTOR_LEDC_MODE, fwd_ch);
        ledc_set_duty(MOTOR_LEDC_MODE, rev_ch, 0);
        ledc_update_duty(MOTOR_LEDC_MODE, rev_ch);
    } else {
        /* Reverse: IN1/IN3 = 0, PWM on IN2/IN4 */
        ledc_set_duty(MOTOR_LEDC_MODE, fwd_ch, 0);
        ledc_update_duty(MOTOR_LEDC_MODE, fwd_ch);
        ledc_set_duty(MOTOR_LEDC_MODE, rev_ch, duty);
        ledc_update_duty(MOTOR_LEDC_MODE, rev_ch);
    }
}

void motor_set(float left, float right) {
    set_single_motor(left,  MOTOR_LEFT_CHANNEL,  MOTOR_LEFT_REV_CHANNEL);
    set_single_motor(right, MOTOR_RIGHT_CHANNEL, MOTOR_RIGHT_REV_CHANNEL);
}

void motor_stop(void) {
    ledc_set_duty(MOTOR_LEDC_MODE, MOTOR_LEFT_CHANNEL, 0);
    ledc_update_duty(MOTOR_LEDC_MODE, MOTOR_LEFT_CHANNEL);
    ledc_set_duty(MOTOR_LEDC_MODE, MOTOR_LEFT_REV_CHANNEL, 0);
    ledc_update_duty(MOTOR_LEDC_MODE, MOTOR_LEFT_REV_CHANNEL);
    ledc_set_duty(MOTOR_LEDC_MODE, MOTOR_RIGHT_CHANNEL, 0);
    ledc_update_duty(MOTOR_LEDC_MODE, MOTOR_RIGHT_CHANNEL);
    ledc_set_duty(MOTOR_LEDC_MODE, MOTOR_RIGHT_REV_CHANNEL, 0);
    ledc_update_duty(MOTOR_LEDC_MODE, MOTOR_RIGHT_REV_CHANNEL);
}

void motor_task(void *pvParameters) {
    motor_cmd_t cmd;

    while (1) {
        command_get(&cmd);
        motor_set(cmd.left, cmd.right);
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}