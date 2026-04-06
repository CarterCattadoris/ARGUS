#ifndef MOTOR_H
#define MOTOR_H

#include "driver/ledc.h"
#include "driver/gpio.h"
#include "esp_err.h"

// ── Pin assignments ──
#define MOTOR_LEFT_IN1    GPIO_NUM_17
#define MOTOR_LEFT_IN2    GPIO_NUM_16
#define MOTOR_RIGHT_IN3   GPIO_NUM_3
#define MOTOR_RIGHT_IN4   GPIO_NUM_8

// ── LEDC config ──
#define MOTOR_PWM_FREQ_HZ       1000
#define MOTOR_PWM_RESOLUTION    LEDC_TIMER_10_BIT
#define MOTOR_PWM_MAX_DUTY      1023

#define MOTOR_LEFT_CHANNEL      LEDC_CHANNEL_0   // IN1
#define MOTOR_LEFT_REV_CHANNEL  LEDC_CHANNEL_2   // IN2
#define MOTOR_RIGHT_CHANNEL     LEDC_CHANNEL_1   // IN3
#define MOTOR_RIGHT_REV_CHANNEL LEDC_CHANNEL_3   // IN4
#define MOTOR_LEDC_TIMER        LEDC_TIMER_0
#define MOTOR_LEDC_MODE         LEDC_LOW_SPEED_MODE

// ── Public API ──
esp_err_t motor_init(void);
void motor_set(float left, float right);
void motor_stop(void);
void motor_task(void *pvParameters);

#endif // MOTOR_H