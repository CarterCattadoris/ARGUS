#include "wifi.h"
#include "udp.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "command.h"
#include "motor.h"
#include "esp_log.h"


void app_main(void) {
    wifi_init_sta(); // init wifi station to connect to wifi
    command_init(); // init mutex for data security
    motor_init(); // init motor control

    xTaskCreate(
        udp_listener_task,
        "udp_rx",
        4096,
        NULL,
        5,
        NULL
    ); // create a task so it runs in the background and doesn't block main
    // while creating a socket to listen on

    xTaskCreate(
        motor_task,
        "motor_task",
        2048,
        NULL,
        5,
        NULL
    ); // create a task to update the motors continuously (while(1) loop) from udp commands

    while (1) {
    ESP_LOGI("TEST", "Forward 100%%");
    motor_set(1.0, 1.0, 1);
    vTaskDelay(pdMS_TO_TICKS(3000));

    ESP_LOGI("TEST", "Stop");
    motor_stop();
    vTaskDelay(pdMS_TO_TICKS(2000));

    ESP_LOGI("TEST", "Reverse 50%%");
    motor_set(0.5, 0.5, -1);
    vTaskDelay(pdMS_TO_TICKS(3000));

    ESP_LOGI("TEST", "Stop");
    motor_stop();
    vTaskDelay(pdMS_TO_TICKS(2000));

    ESP_LOGI("TEST", "Turn left (right motor only)");
    motor_set(0.0, 0.7, 1);
    vTaskDelay(pdMS_TO_TICKS(3000));

    ESP_LOGI("TEST", "Turn right (left motor only)");
    motor_set(0.7, 0.0, 1);
    vTaskDelay(pdMS_TO_TICKS(3000));

    ESP_LOGI("TEST", "Stop — restarting loop");
    motor_stop();
    vTaskDelay(pdMS_TO_TICKS(3000));
    }
}