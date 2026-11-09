#include "drive.h"

#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esc.h"
#include "rc.h"

#define THROTTLE_CH       1       // CH2 (index 1): right stick up/down, springs to centre
                                  // 174 (down) / 992 (centre) / 1811 (up)
                                  // kill switch: RC_KILL_SWITCH_CH in rc.h (CH5)
#define STICK_DEADBAND    50      // stick must be this far from RC_MID before the motors move

#define DRIVE_LIMIT_PCT   95      // cap at 95% of full throttle both ways (1025 .. 1975 us)

// Fine control: the first FINE_STICK_PCT of stick travel (past the deadband) only reaches
// FINE_THROTTLE_PCT of full throttle, for going really slow. The rest of the travel goes on to the cap.
#define FINE_STICK_PCT    20
#define FINE_THROTTLE_PCT 8

#define LOOP_MS           40      // 25 Hz. Keep >= 2 ESC PWM periods (40 ms), see esc_write_us()

#define DRIVE_TASK_STACK  3072
#define DRIVE_TASK_PRIO   5
#define DRIVE_TASK_CORE   0

static const char *TAG = "DRIVE";

static drive_state_t s_state = {.pulse_us = ESC_US_NEUTRAL};
static portMUX_TYPE s_lock = portMUX_INITIALIZER_UNLOCKED;

// Stick position -> ESC pulse. Centre (+- deadband) = 1500 (stopped), up = forward to
// 1500 + 500 * DRIVE_LIMIT_PCT%, down = reverse to 1500 - 500 * DRIVE_LIMIT_PCT%.
// Two straight lines: gentle for the first FINE_STICK_PCT of travel, steeper after.
static uint32_t stick_to_us(uint16_t ch)
{
    int off = (int)ch - RC_MID;
    if (abs(off) <= STICK_DEADBAND) return ESC_US_NEUTRAL;
    int span = (off > 0 ? RC_MAX - RC_MID : RC_MID - RC_MIN) - STICK_DEADBAND;
    int mag = abs(off) - STICK_DEADBAND;
    if (mag > span) mag = span;

    int stick_pct10 = mag * 1000 / span;            // stick travel, 0..1000
    int thr_pct10;                                  // throttle, 0..10 * DRIVE_LIMIT_PCT
    if (stick_pct10 <= FINE_STICK_PCT * 10) {
        thr_pct10 = stick_pct10 * FINE_THROTTLE_PCT / FINE_STICK_PCT;
    } else {
        thr_pct10 = FINE_THROTTLE_PCT * 10 + (stick_pct10 - FINE_STICK_PCT * 10) *
                    (DRIVE_LIMIT_PCT - FINE_THROTTLE_PCT) / (100 - FINE_STICK_PCT);
    }
    int us_off = thr_pct10 * 500 / 1000;
    return off > 0 ? ESC_US_NEUTRAL + us_off : ESC_US_NEUTRAL - us_off;
}

static void drive_task(void *arg)
{
    esc_init();   // outputs neutral from the start so the ESC arms
    ESP_LOGI(TAG, "Kill switch CH%d, speed CH%d (right stick: up forward, down reverse), limit %d%%",
             RC_KILL_SWITCH_CH + 1, THROTTLE_CH + 1, DRIVE_LIMIT_PCT);

    bool enabled = false;
    TickType_t wake = xTaskGetTickCount();
    while (1) {
        vTaskDelayUntil(&wake, pdMS_TO_TICKS(LOOP_MS));

        rc_state_t rc;
        rc_get(&rc);
        bool run = rc_run_allowed(&rc);
        uint32_t stick_us = rc.connected ? stick_to_us(rc.ch[THROTTLE_CH]) : ESC_US_NEUTRAL;

        if (!run) {
            if (enabled) ESP_LOGW(TAG, "%s", rc.connected ? "Kill switch: motor off" : "Radio lost: motor off");
            enabled = false;
        } else if (!enabled) {
            // Only enable with the stick centred, so flipping the switch never makes it jump
            if (stick_us == ESC_US_NEUTRAL) {
                enabled = true;
                ESP_LOGI(TAG, "Motor enabled");
            }
        }

        uint32_t us = enabled ? stick_us : ESC_US_NEUTRAL;
        esc_write_us(us);

        portENTER_CRITICAL(&s_lock);
        s_state.pulse_us = us;
        s_state.motor_enabled = enabled;
        portEXIT_CRITICAL(&s_lock);
    }
}

void drive_start(void)
{
    xTaskCreatePinnedToCore(drive_task, "drive", DRIVE_TASK_STACK, NULL, DRIVE_TASK_PRIO, NULL, DRIVE_TASK_CORE);
}

void drive_get(drive_state_t *out)
{
    portENTER_CRITICAL(&s_lock);
    *out = s_state;
    portEXIT_CRITICAL(&s_lock);
}
