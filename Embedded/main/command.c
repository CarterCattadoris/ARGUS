#include "command.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

static motor_cmd_t g_cmd = {0};
static SemaphoreHandle_t g_mutex;

void command_init(void) {
    g_mutex = xSemaphoreCreateMutex();
}

void command_update(float left, float right, int64_t timestamp) {
    xSemaphoreTake(g_mutex, portMAX_DELAY);
    g_cmd.left = left;
    g_cmd.right = right;
    g_cmd.timestamp = timestamp;
    xSemaphoreGive(g_mutex);
}

void command_get(motor_cmd_t *out) {
    xSemaphoreTake(g_mutex, portMAX_DELAY);
    *out = g_cmd;
    xSemaphoreGive(g_mutex);
}

int64_t command_get_timestamp(void) {
    xSemaphoreTake(g_mutex, portMAX_DELAY);
    int64_t copy = g_cmd.timestamp;
    xSemaphoreGive(g_mutex);
    return copy;
}