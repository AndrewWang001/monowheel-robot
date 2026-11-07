#include "leg_control.h"

#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "servo.h"
#include "legs.h"
#include "rc.h"

#define LEG_CH            2       // CH3 (index 1 = CH2, 2 = CH3): left stick up/down, no spring

// Raw servo positions (0..1000) of the standing pose, measured with the calibration tool.
// Stick up = vertical (each joint's zero in legs.c), stick down = this pose, linear in between.
// Back q3 (ID 2) was measured at -213, outside the servo's range: it stops at 0 (~41 of 92 deg).
static const int16_t STAND_POS[NUM_LEGS][JOINTS_PER_LEG] = {
    [LEG_BACK]  = {719, 218, -213},     // IDs 1, 3, 2
    [LEG_FRONT] = {499, 817, 768},      // IDs 4, 5, 6
};

#define LOOP_MS           50      // 20 Hz
#define FOLLOW_MOVE_MS    100     // each small step is spread over this, so motion looks smooth
#define FIRST_MOVE_MS     1500    // slow move to the stick position when switching to RUN
#define MIN_STEP_PCT      1.0f    // ignore stick jitter smaller than this
#define FIND_EVERY_MS     2000    // how often to look for servos that haven't answered

#define LEG_TASK_STACK    4096
#define LEG_TASK_PRIO     4       // below drive (5) and IMU (6)
#define LEG_TASK_CORE     0

static const char *TAG = "LEG_CTRL";

static bool s_active[NUM_LEGS][JOINTS_PER_LEG];   // servos that have answered
static bool s_warned[NUM_LEGS][JOINTS_PER_LEG];
static leg_control_state_t s_state;
static portMUX_TYPE s_lock = portMUX_INITIALIZER_UNLOCKED;

// Stick -> pose, 0 % (up, vertical) .. 100 % (down, standing)
static float stick_to_pct(uint16_t ch)
{
    float x = ((float)RC_MAX - ch) / (RC_MAX - RC_MIN);
    if (x > 1.0f) x = 1.0f;
    if (x < 0.0f) x = 0.0f;
    return x * 100.0f;
}

// Blend each joint's raw position from vertical (zero) to standing
static void move_one(int leg, int j, float pct, uint16_t time_ms)
{
    float zero = joint_cal[leg][j].zero;
    float pos = zero + (STAND_POS[leg][j] - zero) * pct / 100.0f;
    if (pos < 0.0f) pos = 0.0f;
    if (pos > SERVO_POS_MAX) pos = SERVO_POS_MAX;
    servo_move(joint_cal[leg][j].id, (uint16_t)lroundf(pos), time_ms);
}

static void move_all(float pct, uint16_t time_ms)
{
    for (int leg = 0; leg < NUM_LEGS; leg++)
        for (int j = 0; j < JOINTS_PER_LEG; j++)
            if (s_active[leg][j]) move_one(leg, j, pct, time_ms);
}

// Ping servos that haven't answered yet. A newly found servo goes to position mode, limp
// unless the legs are already following, in which case it powers up and moves to the current angle.
static int find_servos(bool following, float pct)
{
    int n = 0;
    for (int leg = 0; leg < NUM_LEGS; leg++) {
        for (int j = 0; j < JOINTS_PER_LEG; j++) {
            uint8_t id = joint_cal[leg][j].id, got;
            if (!s_active[leg][j]) {
                if (servo_read_id(id, &got) == ESP_OK) {
                    s_active[leg][j] = true;
                    servo_set_servo_mode(id);
                    ESP_LOGI(TAG, "Servo ID %u (%s) found", id, joint_name(leg, j));
                    if (following) {
                        servo_set_torque(id, true);
                        move_one(leg, j, pct, FIRST_MOVE_MS);
                    }
                } else if (!s_warned[leg][j]) {
                    ESP_LOGW(TAG, "Servo ID %u (%s) not replying, will keep checking", id, joint_name(leg, j));
                    s_warned[leg][j] = true;
                }
            }
            n += s_active[leg][j];
        }
    }
    return n;
}

static void set_torque_all(bool on)
{
    for (int leg = 0; leg < NUM_LEGS; leg++)
        for (int j = 0; j < JOINTS_PER_LEG; j++)
            if (s_active[leg][j]) servo_set_torque(joint_cal[leg][j].id, on);
}

static void leg_task(void *arg)
{
    ESP_ERROR_CHECK(servo_bus_init(SERVO_UART, SERVO_TX_GPIO, SERVO_RX_GPIO));
    ESP_LOGI(TAG, "Legs follow CH%d (up = vertical, down = standing) while kill switch CH%d is RUN",
             LEG_CH + 1, RC_KILL_SWITCH_CH + 1);

    bool following = false, powered = false;
    float sent = 0.0f;
    int servos = find_servos(false, 0.0f);
    TickType_t last_find = xTaskGetTickCount();
    TickType_t wake = xTaskGetTickCount();

    while (1) {
        vTaskDelayUntil(&wake, pdMS_TO_TICKS(LOOP_MS));

        if (xTaskGetTickCount() - last_find >= pdMS_TO_TICKS(FIND_EVERY_MS)) {
            servos = find_servos(following, sent);
            last_find = xTaskGetTickCount();
        }

        rc_state_t rc;
        rc_get(&rc);
        bool run = rc_run_allowed(&rc);

        if (!run) {
            // Freeze: send nothing, servos hold their last position (or stay limp before first RUN)
            if (following) ESP_LOGW(TAG, "%s", rc.connected ? "Kill switch: legs frozen" : "Radio lost: legs frozen");
            following = false;
        } else {
            float target = stick_to_pct(rc.ch[LEG_CH]);
            if (!following) {
                if (!powered) {
                    set_torque_all(true);
                    powered = true;
                }
                ESP_LOGI(TAG, "Legs following stick, moving to %.0f %% standing", target);
                move_all(target, FIRST_MOVE_MS);
                vTaskDelay(pdMS_TO_TICKS(FIRST_MOVE_MS));
                wake = xTaskGetTickCount();
                sent = target;
                following = true;
            } else if (fabsf(target - sent) >= MIN_STEP_PCT) {
                move_all(target, FOLLOW_MOVE_MS);
                sent = target;
            }
        }

        portENTER_CRITICAL(&s_lock);
        s_state.pose_pct = sent;
        s_state.following = following;
        s_state.servos = servos;
        portEXIT_CRITICAL(&s_lock);
    }
}

void leg_control_start(void)
{
    xTaskCreatePinnedToCore(leg_task, "leg_ctrl", LEG_TASK_STACK, NULL, LEG_TASK_PRIO, NULL, LEG_TASK_CORE);
}

void leg_control_get(leg_control_state_t *out)
{
    portENTER_CRITICAL(&s_lock);
    *out = s_state;
    portEXIT_CRITICAL(&s_lock);
}
