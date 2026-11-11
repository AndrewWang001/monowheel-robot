// Picks one firmware mode and runs it. Modes are listed in modes/modes.h.
//
// The mode comes from main/mode_select.h, written by tools/pick_mode.py (gitignored, so
// switching modes never shows up as a code change). CMake creates it with MODE_DRIVE if missing.
//
// Every mode runs on its own at power-up: no laptop or serial commands needed.

#include "esp_log.h"
#include "modes.h"
#if __has_include("mode_select.h")
#include "mode_select.h"
#else
// Deleted after the last CMake run. Don't silently fall back: the build could then miss the
// file when it comes back and keep running the wrong mode.
#error "main/mode_select.h is missing. Pick a mode with tools/pick_mode.py (or run: idf.py reconfigure)"
#endif

#ifndef APP_MODE
#define APP_MODE MODE_DRIVE
#endif

#ifndef SERVO_CAL_STEP
#define SERVO_CAL_STEP CAL_SLIDER
#endif

static const char *TAG = "MAIN";

void app_main(void)
{
#if APP_MODE == MODE_DRIVE
    ESP_LOGI(TAG, "Mode: drive (IMU + controller: CH2 motors, CH7/CH6 legs, CH5 kill switch)");
    mode_drive_run();
#elif APP_MODE == MODE_TEST_IMU
    ESP_LOGI(TAG, "Mode: test IMU");
    test_imu_run();
#elif APP_MODE == MODE_TEST_RC
    ESP_LOGI(TAG, "Mode: test controller");
    test_rc_run();
#elif APP_MODE == MODE_TEST_LEG_SWEEP
    ESP_LOGI(TAG, "Mode: test leg sweep (IDs 1-6, +/-45 deg)");
    test_leg_sweep_run();
#elif APP_MODE == MODE_TEST_MOTOR_SWEEP
    ESP_LOGI(TAG, "Mode: test motor sweep (scripted, no controller)");
    test_motor_sweep_run();
#elif APP_MODE == MODE_TEST_MOTOR_RC
    ESP_LOGI(TAG, "Mode: test motor from controller");
    test_motor_rc_run();
#elif APP_MODE == MODE_TOOL_SERVO_CAL
    ESP_LOGI(TAG, "Mode: servo calibration tool, step %d", SERVO_CAL_STEP);
    tool_servo_cal_run(SERVO_CAL_STEP);
#elif APP_MODE == MODE_TEST_BALANCE
    ESP_LOGI(TAG, "Mode: test balance (legs, CH8 off/manual/PID, CH10 u / gain, CH5 kill switch)");
    test_balance_run();
#elif APP_MODE == MODE_TEST_IK
    ESP_LOGI(TAG, "Mode: test IK (CH8 joystick/hold/circle, CH1/CH4/CH2 foot x/y/z, CH5 kill switch)");
    test_ik_run();
#elif APP_MODE == MODE_TOOL_SERVO_SET_ID
    ESP_LOGW(TAG, "Mode: SERVO SET ID TOOL (renumbers a servo)");
    tool_servo_set_id_run();
#else
#error "Unknown APP_MODE in main/mode_select.h. See main/modes/modes.h for the list."
#endif
}
