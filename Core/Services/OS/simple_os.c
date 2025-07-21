#include "simple_os.h"
#include "stm32f0xx_hal.h"
#include "Rte.h"

static const SimpleTask *os_tasks = 0;
static uint8_t os_num_tasks = 0;
static void (*os_init_task)(void) = 0;
static uint32_t os_init_max_time_ms = 0;

void OS_Init(const SimpleTask *tasks, uint8_t num_tasks, void (*init_task)(void), uint32_t init_max_time_ms) {
    os_tasks = tasks;
    os_num_tasks = num_tasks;
    os_init_task = init_task;
    os_init_max_time_ms = init_max_time_ms;
}

void OS_Run(void) {
    if (os_init_task) {
        uint32_t start = HAL_GetTick();
        os_init_task();
        uint32_t elapsed = HAL_GetTick() - start;
        if (elapsed > os_init_max_time_ms) {
            os_task_overrun_count[0]++;
            os_task_overrun_time=elapsed;
        } else {
            while ((HAL_GetTick() - start) < os_init_max_time_ms) {}
        }
    }
    uint8_t i = 0;
    while (1) {
        uint32_t start = HAL_GetTick();
        os_tasks[i].task_func();
        uint32_t elapsed = HAL_GetTick() - start;
        if (elapsed > os_tasks[i].max_time_ms) {
            os_task_overrun_count[i+1]++;
            os_task_overrun_time=elapsed;
        } else {
            while ((HAL_GetTick() - start) < os_tasks[i].max_time_ms) {}//bucla de asteptare pana la urmatorul task
        }
        i++;
        if (i >= os_num_tasks) i = 0;
    }
} 
