#include "wifi.h"
#include "udp.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "command.h"
#include "motor.h"
#include "esp_log.h"
#include "imu.h"
#include "lcd.h"


void app_main(void) {
    wifi_init_sta(); // init wifi station to connect to wifi
    command_init(); // init mutex for data security
    motor_init(); // init motor control
    imu_init(); // init imu sending

    // give WiFi a moment to connect before LCD reads IP
    vTaskDelay(pdMS_TO_TICKS(2000));
    lcd_init(); // init LCD display (shows IP + status)

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
}