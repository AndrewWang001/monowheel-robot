#pragma once

// Legs from the T8L, both legs (servo IDs 1-6) together:
//   CH7 left middle 3-position switch: up = vertical (each joint's zero in legs.c), middle =
//       halfway, down = ground pose (GROUND_POS in leg_control.c)
//   CH6 left top button: press toggles the button pose (BUTTON_POS); moving CH7 cancels it
//   In the button pose only, stand-up in 3 stages (STAND_POS keyframes):
//       CH8 right middle switch picks the stage (down 1, middle 2, up 3), CH3 left stick moves
//       through it (bottom = stage start, top = stage end). A stage change is only accepted at the
//       matching end of the current stage, and CH3 must then come to the legs' position before it
//       takes over. Works in reverse the same way.
// Each change is one smooth move; every joint moves in a straight line between raw positions (no IK).
//
// Kill switch (CH5, see rc.h): on KILL or radio loss the legs freeze where they are. On RUN they
// move slowly to the stick position, then follow it. Servos stay limp until the first RUN.
// Missing servos are skipped and join in when they show up.
//
// Needs rc_start() to be running. Owns the servo bus: don't run alongside another servo user.

#include <stdbool.h>

typedef struct {
    const char *pose;   // pose last sent: "vertical", "half", "ground" or "button"
    int stand_stage;    // stand-up stage 1..3, 0 = not in the button pose
    float stand_pct;    // progress through the current stage, 0..100
    bool stand_armed;   // CH3 is in control (false = waiting for CH3 / CH8)
    bool following;     // kill switch in RUN and legs following the stick
    int servos;         // leg servos currently answering (of 6)
} leg_control_state_t;

void leg_control_start(void);
void leg_control_get(leg_control_state_t *out);
