#pragma once

// Legs from the T8L, both legs (servo IDs 1-6) together:
//   CH7 left middle 3-position switch: up = vertical (each joint's zero in legs.c), middle =
//       halfway, down = ground pose, wait, then the final standing pose
//   CH6 left top button: one press runs the whole stand-up (STAND_STEPS in leg_control.c):
//       vertical, button pose, mid push up, crawl in, push up straight, back vertical, grounded, final.
//       Press again to pause (stops where it is), again to continue. Once done, presses are ignored.
//       Moving CH7 cancels it; the next press starts over from vertical.
//   CH9 right top button: offset calibration. Press: servos go limp, readings print. Hold every leg
//       straight up, press again: new offsets (current + reading - 500) are saved in the servos and
//       the legs go to 500. Moving CH7 or KILL before the second press cancels, nothing saved.
//       Ignored while the stand-up is moving.
// Each step is one smooth move; every joint moves in a straight line between raw positions (no IK).
//
// Kill switch (CH5, see rc.h): on KILL or radio loss the legs stop where they are (servos hold).
// On RUN: a paused stand-up waits for CH6; anything else carries on or moves to the CH7 pose.
// Servos stay limp until the first RUN. Missing servos are skipped and join in when they show up.
//
// Needs rc_start() to be running. Owns the servo bus: don't run alongside another servo user.

#include <stdbool.h>

typedef struct {
    const char *seq;    // what's running: "stand-up", "vertical", "half", "ground", "CALIBRATE" or "-"
    const char *step;   // current step name
    int step_num;       // 1..steps
    int steps;
    const char *status; // "move", "wait", "PAUSED", "done" or "-"
    bool following;     // kill switch in RUN and legs active
    int servos;         // leg servos currently answering (of 6)
} leg_control_state_t;

void leg_control_start(void);
void leg_control_get(leg_control_state_t *out);
