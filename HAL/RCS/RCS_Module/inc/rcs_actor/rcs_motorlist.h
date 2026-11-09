#pragma once

#include "rcs_private_config.h"
#include "rcs_types.h"
#include "sync_pid.h"
#include "rcs_can.h"

#include <FreeRTOS.h>
#include <task.h>
#include <event_groups.h>

#include <unordered_map>
#include <vector>

#define RCS_MOTORLIST_MALLOC pvPortMalloc
#define RCS_MOTORLIST_FREE vPortFree

#define RCS_MOTORLIST_EXCUTE_TASK_PRIORITY 5
#define RCS_MOTORLIST_EXCUTE_TASK_STACK_SIZE 512
#define RCS_MOTORLIST_EXCUTE_TASK_DELAY pdMS_TO_TICKS(5)


