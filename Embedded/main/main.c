#include "wifi.h"
#include "udp.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "command.h"


void app_main(void) {
    wifi_init_sta(); // init wifi station to connect to wifi
    command_init();

    xTaskCreate(
        udp_listener_task,
        "udp_rx",
        4096,
        NULL,
        5,
        NULL
    ); // create a task so it runs in the background and doesn't block main
    // while creating a socket to listen on

}