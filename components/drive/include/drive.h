#pragma once

// Drive motors from the T8L: side scroll wheel (CH10) sets forward speed through the ESCs
// (all the way left = stopped, forward only for now). CH5 is the kill switch: the motors only
// run while it is in the RUN position, and only become enabled while the wheel is at stop. Stops if the switch goes to KILL or the
// radio drops out. Speed is capped at DRIVE_LIMIT_PCT of full throttle.
//
// Needs rc_start() to be running. Owns the ESC: don't run alongside another ESC user.

#include <stdbool.h>
#include <stdint.h>

typedef struct {
    uint32_t pulse_us;      // what is being sent to the ESC right now
    bool motor_enabled;     // kill switch in RUN and enabled (stick was centred)
} drive_state_t;

void drive_start(void);
void drive_get(drive_state_t *out);
