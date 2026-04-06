#include <stdio.h>
#include <string.h>
#include "lwip/sockets.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "cJSON.h"
#include "command.h"
#include "esp_timer.h"

#define PORT 9876
#define BUF_SIZE 1024
static const char *TAG = "udp_listener";

void udp_listener_task(void *arg)
{
    int fd = socket(AF_INET, SOCK_DGRAM, 0);
    if (fd < 0) { ESP_LOGE(TAG, "Failed to create socket"); vTaskDelete(NULL); }

    struct sockaddr_in addr = {
        .sin_family = AF_INET,
        .sin_port = htons(PORT),
        .sin_addr.s_addr = INADDR_ANY,
    };

    if (bind(fd, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        ESP_LOGE(TAG, "Failed to bind");
        close(fd);
        vTaskDelete(NULL);
    }

    ESP_LOGI(TAG, "Listening on UDP :%d", PORT);

    char buf[BUF_SIZE];
    struct sockaddr_in client;
    socklen_t client_len = sizeof(client);

    for (;;) {
        ssize_t n = recvfrom(
            fd, 
            buf, 
            BUF_SIZE - 1, 
            0,
            (struct sockaddr *)&client, 
            &client_len
        );
        if (n < 0) { ESP_LOGW(TAG, "recvfrom error"); continue; }

        buf[n] = '\0';
        ESP_LOGI(TAG, "[%s:%d] %s", 
            inet_ntoa(client.sin_addr),
            ntohs(client.sin_port), buf
        );

        cJSON *root = cJSON_Parse(buf);
        if (!root) {
            ESP_LOGW(TAG, "Invalid JSON");
            continue;
        }

        cJSON *j_left  = cJSON_GetObjectItem(root, "left");
        cJSON *j_right = cJSON_GetObjectItem(root, "right");

        if (cJSON_IsNumber(j_left) && cJSON_IsNumber(j_right)) {
            float left  = (float)j_left->valuedouble;
            float right = (float)j_right->valuedouble;

            command_update(left, right, esp_timer_get_time());
            ESP_LOGI(TAG, "CMD: L=%.2f R=%.2f", left, right);
            // Echo back (only for testing, delete later)
            char resp[128];
            int len = snprintf(resp, sizeof(resp),
                "{\"left\":%.2f,\"right\":%.2f}",
                left, right);
            sendto(fd, resp, len, 0, (struct sockaddr *)&client, client_len);
        }

        cJSON_Delete(root);
    }

    close(fd);
}