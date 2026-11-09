#ifndef RC_EVENT_DISPATCH_H
#define RC_EVENT_DISPATCH_H

#include <stdbool.h>
#include "remote_control_sbus.h"

typedef struct {
    bool shutdown;
    bool emergency_stop;
    bool folded;
    bool gait_active;
    bool wheeled_mode;
} RemoteEventState_t;

void rc_event_dispatch(RemoteEventState_t *state, const RemoteCommand_t *command, float now);

#endif
