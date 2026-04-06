#ifndef COMMAND_H
#define COMMAND_H

#include <stdint.h>

typedef struct {
    float left;
    float right;
    int64_t timestamp;
} motor_cmd_t;

void command_init(void);
void command_update(float left, float right, int64_t timestamp);
void command_get(motor_cmd_t *out);
int64_t command_get_timestamp(void);

#endif