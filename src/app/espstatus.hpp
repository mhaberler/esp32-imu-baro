
#pragma once

#include <Fmt.h>
#include <task.h>
#include <freertos/task.h>

// report task either by name or by handle
void task_details(bool bitch, const char *taskName = NULL, TaskHandle_t handle = NULL);
// NULL-terminated list of task names
void task_report(const char *fn, const int line, ...);
void heap_report(const char *fn = NULL, const int line = 0);
void psram_report(const char *fn, const int line);
void platform_report(void);
void build_setup_report(void);
