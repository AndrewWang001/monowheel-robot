#pragma once

// Legs from the T8L: left stick up/down (CH3, no spring) blends both legs (servo IDs 1-6) from
// vertical (stick up, each joint's zero in legs.c) to the standing pose (stick down, STAND_POS
// in leg_control.c). Every joint moves in a straight line between its two raw positions (no IK).
//
// Kill switch (CH5, see rc.h): on KILL or radio loss the legs freeze where they are. On RUN they
// move slowly to the stick position, then follow it. Servos stay limp until the first RUN.
// Missing servos are skipped and join in when they show up.
//
// Needs rc_start() to be running. Owns the servo bus: don't run alongside another servo user.

#include <stdbool.h>

typedef struct {
    float pose_pct;     // pose last sent: 0 = vertical, 100 = standing
    bool following;     // kill switch in RUN and legs following the stick
    int servos;         // leg servos currently answering (of 6)
} leg_control_state_t;

void leg_control_start(void);
void leg_control_get(leg_control_state_t *out);
