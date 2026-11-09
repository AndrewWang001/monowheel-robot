// MODE_TEST_BALANCE: stationary side-to-side balance with the legs (IMU + controller + leg servos).
// IDs 3, 2, 5, 6 move; bases IDs 1 and 4 hold at 500.
//
// One number u (-1..+1) blends the legs through seven measured poses (KEYS below):
//   u = 0 balanced, +1 max lean left (fixes a lean to the right), -1 max lean right.
//   Straight lines between neighbouring poses.
// PID: u = gain * (KP * lean + KD * lean_rate), lean from the IMU (0 = upright, + = right).
//
// Controller:
//   CH5  kill switch: KILL = legs stop sending (servos hold), RUN = legs active
//   CH8  down = OFF (hold upright), middle = MANUAL (CH10 wheel sets u directly: check the poses
//        and that they're symmetric), up = PID balance
//   CH10 MANUAL: u from -1 (left end) to +1 (right end). PID: P gain 0 (left end) .. 2x (right end)
//   CH3  PID: D gain 0 (stick bottom) .. 2x (top)
//
// First test with the robot held: MANUAL, roll the wheel both ways, check the legs push the right
// way and look like mirror images. Then PID with the wheel low, raise it until it holds itself up.

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "modes.h"
#include "imu.h"
#include "rc.h"
#include "legs.h"
#include "servo.h"

#define MODE_CH        7       // CH8: right middle 3-position switch
#define WHEEL_CH       9       // CH10: side scroll wheel
#define KD_STICK_CH    2       // CH3: left stick up/down (no spring): D gain 0 (bottom) .. 2x (top)
#define WHEEL_MIN      191
#define WHEEL_MAX      1792

// Raw poses from read-mode snapshots, columns back IDs 1, 3, 2 / front IDs 4, 5, 6.
// Bases (IDs 1, 4) fixed at 500. Evenly spaced in u from -1 (first) to +1 (last).
#define NUM_KEYS 7
static const int16_t KEYS[NUM_KEYS][NUM_LEGS][JOINTS_PER_LEG] = {
    {[LEG_BACK] = {500, 476,  82}, [LEG_FRONT] = {500,  84, 478}},   // u = -1    max lean right
    {[LEG_BACK] = {500, 399, 633}, [LEG_FRONT] = {500, 125, 478}},   // u = -2/3  (old max right)
    {[LEG_BACK] = {500, 488, 672}, [LEG_FRONT] = {500, 219, 478}},   // u = -1/3  moving right
    {[LEG_BACK] = {500, 660, 711}, [LEG_FRONT] = {500, 307, 335}},   // u =  0    balanced
    {[LEG_BACK] = {500, 509, 801}, [LEG_FRONT] = {500, 322, 489}},   // u = +1/3  moving left
    {[LEG_BACK] = {500, 509, 888}, [LEG_FRONT] = {500, 376, 572}},   // u = +2/3  (old max left)
    {[LEG_BACK] = {500, 485, 936}, [LEG_FRONT] = {500, 932, 489}},   // u = +1    max lean left
};

// PID, all per degree / per deg/s of lean. u = 1 means the full lean pose.
#define KP             (1.0f / 15.0f)   // at gain 1: moving pose at 5 deg, old max at 10, max at 15
#define KD             0.010f           // 30 deg/s of falling adds 0.3
#define LEAN_DEADBAND  1.0f             // ignore leans smaller than this (deg), so it doesn't buzz
#define U_MAX          1.0f             // PID cap: 1 = can use the full max lean poses
#define RATE_FILTER    0.3f             // gyro low-pass for D: 0..1, lower = smoother (shake from the
                                        // legs moving is fast, a real fall is slow)
#define U_SLEW         0.06f            // max change of u per loop (20 ms): full swing 0 -> 0.6 in 0.2 s
#define FALLEN_DEG     30.0f            // past this: stop balancing, go upright
#define RESUME_DEG     5.0f             // balance again once back within this

#define LOOP_MS        20               // 50 Hz; each update is a LOOP_MS servo move
#define FIRST_MOVE_MS  1500
#define PRINT_EVERY    5                // print every 5th loop (10 Hz)

static const char *TAG = "BALANCE";

static bool s_active[NUM_LEGS][JOINTS_PER_LEG];

// u (-1..+1) -> raw position, straight lines between neighbouring keys
static int pose_at(float u, int leg, int j)
{
    if (u > 1.0f) u = 1.0f;
    if (u < -1.0f) u = -1.0f;
    float x = (u + 1.0f) * (NUM_KEYS - 1) / 2.0f;     // 0 .. 4
    int k = (int)x;
    if (k >= NUM_KEYS - 1) k = NUM_KEYS - 2;
    float t = x - k;
    int a = KEYS[k][leg][j], b = KEYS[k + 1][leg][j];
    return a + (int)lroundf((b - a) * t);
}

static void send_pose(float u, uint16_t ms)
{
    for (int leg = 0; leg < NUM_LEGS; leg++)
        for (int j = 0; j < JOINTS_PER_LEG; j++)
            if (s_active[leg][j]) servo_move(joint_cal[leg][j].id, pose_at(u, leg, j), ms);
}

static float wheel_frac(uint16_t ch)   // 0 .. 1
{
    float f = ((float)ch - WHEEL_MIN) / (WHEEL_MAX - WHEEL_MIN);
    return f < 0 ? 0 : f > 1 ? 1 : f;
}

static float clampf(float x, float lim)
{
    return x > lim ? lim : x < -lim ? -lim : x;
}

typedef enum { BAL_OFF, BAL_MANUAL, BAL_PID } bal_mode_t;
static const char *MODE_NAMES[] = {"OFF", "MANUAL", "PID"};

void test_balance_run(void)
{
    imu_start();
    rc_start();
    ESP_ERROR_CHECK(servo_bus_init(SERVO_UART, SERVO_TX_GPIO, SERVO_RX_GPIO));

    int n = 0;
    for (int leg = 0; leg < NUM_LEGS; leg++) {
        for (int j = 0; j < JOINTS_PER_LEG; j++) {
            uint8_t got;
            s_active[leg][j] = servo_read_id(joint_cal[leg][j].id, &got) == ESP_OK;
            if (s_active[leg][j]) servo_set_servo_mode(joint_cal[leg][j].id);
            else ESP_LOGW(TAG, "Servo ID %u (%s) not replying, skipped", joint_cal[leg][j].id, joint_name(leg, j));
            n += s_active[leg][j];
        }
    }
    ESP_LOGI(TAG, "%d/6 servos. CH5 kill switch, CH8 down OFF / middle MANUAL (CH10 = u) / up PID (CH10 = gain)", n);

    bool following = false, powered = false, fallen = false;
    int loop = 0;
    float rate_f = 0;       // filtered lean rate
    float u_sent = 0;       // u after the slew limit
    TickType_t wake = xTaskGetTickCount();

    while (1) {
        vTaskDelayUntil(&wake, pdMS_TO_TICKS(LOOP_MS));

        rc_state_t rc;
        rc_get(&rc);
        imu_state_t imu;
        imu_get(&imu);
        bool run = rc_run_allowed(&rc);

        uint16_t sw = rc.ch[MODE_CH];
        bal_mode_t mode = sw > 1400 ? BAL_PID : sw > 600 ? BAL_MANUAL : BAL_OFF;
        float wheel = wheel_frac(rc.ch[WHEEL_CH]);

        rate_f += RATE_FILTER * (imu.lean_rate - rate_f);
        float kd_gain = rc.connected ? ((float)rc.ch[KD_STICK_CH] - RC_MIN) / (RC_MAX - RC_MIN) * 2.0f : 0;
        if (kd_gain < 0) kd_gain = 0;

        float u = 0, p = 0, d = 0, gain = 0;
        if (mode == BAL_MANUAL) {
            u = wheel * 2.0f - 1.0f;                // left end -1 .. right end +1, full range
        } else if (mode == BAL_PID && imu.valid) {
            if (fabsf(imu.lean) > FALLEN_DEG) fallen = true;
            else if (fabsf(imu.lean) < RESUME_DEG) fallen = false;
            if (!fallen) {
                float lean = fabsf(imu.lean) <= LEAN_DEADBAND ? 0
                             : imu.lean - copysignf(LEAN_DEADBAND, imu.lean);
                gain = wheel * 2.0f;
                p = gain * KP * lean;
                d = kd_gain * KD * rate_f;
                u = clampf(p + d, U_MAX);
            }
        }
        // Slew limit in PID mode (manual follows the wheel directly)
        if (mode == BAL_PID) {
            if (u > u_sent + U_SLEW) u = u_sent + U_SLEW;
            else if (u < u_sent - U_SLEW) u = u_sent - U_SLEW;
        }
        u_sent = u;

        if (!run) {
            if (following) ESP_LOGW(TAG, "%s", rc.connected ? "Kill switch: legs stopped" : "Radio lost: legs stopped");
            following = false;
        } else if (!following) {
            if (!powered) {
                for (int leg = 0; leg < NUM_LEGS; leg++)
                    for (int j = 0; j < JOINTS_PER_LEG; j++)
                        if (s_active[leg][j]) servo_set_torque(joint_cal[leg][j].id, true);
                powered = true;
            }
            ESP_LOGI(TAG, "Legs active: moving to upright");
            send_pose(0, FIRST_MOVE_MS);
            vTaskDelay(pdMS_TO_TICKS(FIRST_MOVE_MS));
            wake = xTaskGetTickCount();
            following = true;
            u = u_sent = 0;
        } else {
            send_pose(u, LOOP_MS);
        }

        if (++loop >= PRINT_EVERY) {
            loop = 0;
            printf("BAL %-6s %s| lean %+6.1f rate %+7.1f (filt %+6.1f) | u %+5.2f (P %+5.2f D %+5.2f) "
                   "Pgain %.2f Dgain %.2f | %s\n",
                   MODE_NAMES[mode], fallen && mode == BAL_PID ? "FALLEN " : "", imu.lean, imu.lean_rate,
                   rate_f, u, p, d, gain, kd_gain, !rc.connected ? "NO RADIO" : following ? "RUN" : "KILL");
        }
    }
}
