#pragma once

// Every firmware mode. Exactly one runs, chosen by APP_MODE in main/mode_select.h
// (written by tools/pick_mode.py; gitignored). With no mode_select.h, MODE_DRIVE runs.
//
// Each test exercises one part of the robot on its own, so it can be checked step by step.

#define MODE_DRIVE              1   // normal: IMU + controller, drive motors (CH2) + legs (CH7/CH6), kill switch CH5
#define MODE_TEST_IMU           2   // IMU only: prints roll / pitch / yaw / gyro
#define MODE_TEST_RC            3   // controller only: prints all channels
#define MODE_TEST_LEG_SWEEP     4   // leg servos IDs 1-6 swing +/-45 deg around their zero
#define MODE_TEST_MOTOR_SWEEP   5   // drive motor scripted: slow fwd, slow rev, up to 80% (no controller)
#define MODE_TEST_MOTOR_RC      6   // drive motor from the controller with kill switch, nothing else
#define MODE_TOOL_SERVO_CAL     7   // leg servo calibration tool (step set by SERVO_CAL_STEP)
#define MODE_TOOL_SERVO_SET_ID  99  // DO NOT USE: renumbers a servo. See tool_servo_set_id.c

// Steps for MODE_TOOL_SERVO_CAL (full instructions in tool_servo_cal.c)
#define CAL_CENTER  0   // horns off: servos go to mid travel (500) and hold, fit horns in the zero pose
#define CAL_READ    1   // torque off: prints raw position + angle, copy zeros/signs into legs.c
#define CAL_CHECK   2   // torque on: each joint goes to +20 deg and back to confirm directions
#define CAL_SLIDER  3   // torque on: drive each servo live from sim/servo_sliders.py

void mode_drive_run(void);
void test_imu_run(void);
void test_rc_run(void);
void test_leg_sweep_run(void);
void test_motor_sweep_run(void);
void test_motor_rc_run(void);
void tool_servo_cal_run(int step);
void tool_servo_set_id_run(void);

// Shared status printing (status_print.c). Each prints one segment of a line, no newline.
void print_imu(void);
void print_rc(int channels);
void print_drive(void);
void print_legs(void);
