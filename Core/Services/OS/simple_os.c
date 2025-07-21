#include "simple_os.h"
#include "stm32f0xx_hal.h"

static const SimpleTask *os_tasks = 0;
static uint8_t os_num_tasks = 0;

void OS_Init(const SimpleTask *tasks, uint8_t num_tasks) {
    os_tasks = tasks;
    os_num_tasks = num_tasks;
}

void OS_Run(void) {
    uint8_t i = 0;
    while (1) {
        uint32_t start = HAL_GetTick();
        os_tasks[i].task_func();
        uint32_t elapsed = HAL_GetTick() - start;
        if (elapsed < os_tasks[i].max_time_ms) {
            uint32_t wait_until = start + os_tasks[i].max_time_ms;
            while (HAL_GetTick() < wait_until) {}
        }
        i++;
        if (i >= os_num_tasks) i = 0;
    }
} 