#ifndef SIMPLE_OS_H
#define SIMPLE_OS_H

#include <stdint.h>

typedef struct {
    const char *name;
    void (*task_func)(void);
    uint32_t max_time_ms;
} SimpleTask;

void OS_Init(const SimpleTask *tasks, uint8_t num_tasks, void (*init_task)(void), uint32_t init_max_time_ms);
void OS_Run(void);

#endif 