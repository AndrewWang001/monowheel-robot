// MODE_TEST_IK: inverse kinematics on the real legs. A foot target (mm, leg frame, see
// leg_kinematics.h) goes through leg_ik() every loop and the joint angles go to the servos
// through the calibration in legs.c. The back leg gets the same target mirrored (x -> -x).
//
// Controller (sticks spring back to the home point when you let go):
//   CH5  kill switch: RUN = legs active (move slowly to home first), KILL = stop sending
//   CH8  down = JOYSTICK, middle = HOLD home, up = CIRCLE (foot traces a circle, for a demo)
//   JOYSTICK: CH1 right stick left/right = x, CH2 right stick up/down = z (height),
//             CH4 left stick left/right = y
//
// At boot it first runs a math self-test (random FK -> IK -> FK round trips) and logs PASSED/FAILED.
//
// Needs correct joint signs in legs.c (check with Servo calibration: check first). Run it with the
// robot held or on a stand, legs in the air. Every printed line shows the target, the IK angles,
// the FK round-trip error and what was sent, so a joint going the wrong way is easy to spot.

#include <math.h>
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_random.h"
#include "esp_timer.h"
#include "modes.h"
#include "rc.h"
#include "legs.h"
#include "servo.h"
#include "leg_kinematics.h"

#define MODE_CH        7       // CH8: right middle 3-position switch
#define X_CH           0       // CH1: right stick left/right
#define Z_CH           1       // CH2: right stick up/down
#define Y_CH           3       // CH4: left stick left/right

// Home: a bent pose in the air (not straight up, which is a singular, fully stretched point)
static const leg_angles_t HOME_Q = {.q1 = 0.0f, .q2 = 20.0f, .q3 = 40.0f};
#define STICK_RANGE_MM 60.0f   // full stick = this far from home along each axis
#define CIRCLE_R_MM    40.0f   // circle demo: radius, in the x-z plane around home
#define CIRCLE_S       3.0f    // seconds per lap
#define MAX_STEP_MM    3.0f    // target moves at most this far per loop (150 mm/s)

#define LOOP_MS        20      // 50 Hz
#define MOVE_MS        40      // each update is spread over 2 loops so the servos move smoothly
#define FIRST_MOVE_MS  1500
#define PRINT_EVERY    5       // 10 Hz

static const char *TAG = "IK_TEST";
static bool s_active[NUM_LEGS][JOINTS_PER_LEG];

static float stick(uint16_t ch)    // -1 .. +1, small deadband around centre
{
    float v = ((float)ch - RC_MID) / (RC_MAX - RC_MID);
    if (fabsf(v) < 0.05f) return 0;
    return v > 1 ? 1 : v < -1 ? -1 : v;
}

static float step_toward(float cur, float want)
{
    float d = want - cur;
    if (d > MAX_STEP_MM) d = MAX_STEP_MM;
    if (d < -MAX_STEP_MM) d = -MAX_STEP_MM;
    return cur + d;
}

static void move_leg(leg_id_t leg, const leg_angles_t *q, uint16_t ms)
{
    const float deg[JOINTS_PER_LEG] = {q->q1, q->q2, q->q3};
    for (int j = 0; j < JOINTS_PER_LEG; j++)
        if (s_active[leg][j]) servo_move(joint_cal[leg][j].id, joint_deg_to_pos(leg, j, deg[j]), ms);
}

// Math check on the ESP itself, before anything moves: pick random joint angles inside the limits,
// get the foot position with FK, solve it back with IK, run FK on the answer, and measure how far
// the foot lands from where it started. Every target is reachable, so every one should solve.
#define SELFTEST_N       5000
#define SELFTEST_TOL_MM  0.05f

static float rand_deg(void)
{
    return LEG_Q_MIN_DEG + (LEG_Q_MAX_DEG - LEG_Q_MIN_DEG) * (esp_random() / 4294967295.0f);
}

static void ik_self_test(void)
{
    int pass = 0, unsolved = 0;
    float worst = 0;
    int64_t t0 = esp_timer_get_time();
    for (int i = 0; i < SELFTEST_N; i++) {
        leg_angles_t q = {rand_deg(), rand_deg(), rand_deg()}, sol;
        leg_point_t p, back;
        leg_fk(&q, &p);
        if (leg_ik(&p, true, &sol) != LEG_OK) {
            unsolved++;
            continue;
        }
        leg_fk(&sol, &back);
        float e = sqrtf((back.x - p.x) * (back.x - p.x) + (back.y - p.y) * (back.y - p.y) +
                        (back.z - p.z) * (back.z - p.z));
        if (e > worst) worst = e;
        if (e <= SELFTEST_TOL_MM && leg_within_limits(&sol)) pass++;
    }
    float us_each = (float)(esp_timer_get_time() - t0) / SELFTEST_N;
    if (pass == SELFTEST_N) {
        ESP_LOGI(TAG, "IK self-test PASSED: %d/%d random poses round-trip, worst error %.4f mm, %.0f us per FK+IK+FK",
                 pass, SELFTEST_N, worst, us_each);
    } else {
        ESP_LOGE(TAG, "IK self-test FAILED: %d/%d passed, %d unsolved, worst error %.4f mm",
                 pass, SELFTEST_N, unsolved, worst);
    }
}

typedef enum { IK_JOY, IK_HOLD, IK_CIRCLE } ik_mode_t;
static const char *MODE_NAMES[] = {"JOY", "HOLD", "CIRCLE"};

void test_ik_run(void)
{
    ik_self_test();
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

    leg_point_t home;
    leg_fk(&HOME_Q, &home);
    ESP_LOGI(TAG, "%d/6 servos. Home foot = (%.0f, %.0f, %.0f) mm. CH5 kill, CH8 down JOY / mid HOLD / up CIRCLE",
             n, home.x, home.y, home.z);

    leg_point_t target = home;      // front leg's target; back leg uses x mirrored
    leg_angles_t q = HOME_Q, q_back = HOME_Q;
    leg_err_t err = LEG_OK;
    bool following = false, powered = false;
    float circle_t = 0;
    int loop = 0;
    TickType_t wake = xTaskGetTickCount();

    while (1) {
        vTaskDelayUntil(&wake, pdMS_TO_TICKS(LOOP_MS));

        rc_state_t rc;
        rc_get(&rc);
        bool run = rc_run_allowed(&rc);
        uint16_t sw = rc.ch[MODE_CH];
        ik_mode_t mode = !rc.connected ? IK_HOLD : sw > 1400 ? IK_CIRCLE : sw > 600 ? IK_HOLD : IK_JOY;

        // Where the foot should go
        leg_point_t want = home;
        if (mode == IK_JOY) {
            want.x += stick(rc.ch[X_CH]) * STICK_RANGE_MM;
            want.y += stick(rc.ch[Y_CH]) * STICK_RANGE_MM;
            want.z += stick(rc.ch[Z_CH]) * STICK_RANGE_MM;
        } else if (mode == IK_CIRCLE) {
            circle_t += LOOP_MS / 1000.0f;
            float a = 2.0f * 3.14159265f * circle_t / CIRCLE_S;
            want.x += CIRCLE_R_MM * cosf(a);
            want.z += CIRCLE_R_MM * sinf(a);
        }
        target.x = step_toward(target.x, want.x);
        target.y = step_toward(target.y, want.y);
        target.z = step_toward(target.z, want.z);

        // IK for both legs. If either fails, both hold their last good angles.
        leg_angles_t q_new, q_back_new;
        leg_point_t mirrored = {-target.x, target.y, target.z};
        err = leg_ik(&target, true, &q_new);
        if (err == LEG_OK) err = leg_ik(&mirrored, true, &q_back_new);
        if (err == LEG_OK) {
            q = q_new;
            q_back = q_back_new;
        }

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
            ESP_LOGI(TAG, "Legs active: moving to home");
            target = home;
            q = q_back = HOME_Q;
            move_leg(LEG_FRONT, &q, FIRST_MOVE_MS);
            move_leg(LEG_BACK, &q_back, FIRST_MOVE_MS);
            vTaskDelay(pdMS_TO_TICKS(FIRST_MOVE_MS));
            wake = xTaskGetTickCount();
            following = true;
        } else {
            move_leg(LEG_FRONT, &q, MOVE_MS);
            move_leg(LEG_BACK, &q_back, MOVE_MS);
        }

        if (++loop >= PRINT_EVERY) {
            loop = 0;
            leg_point_t check;
            leg_fk(&q, &check);     // round trip: should land back on the target
            float fk_err = sqrtf((check.x - target.x) * (check.x - target.x) +
                                 (check.y - target.y) * (check.y - target.y) +
                                 (check.z - target.z) * (check.z - target.z));
            printf("IK %-6s | foot %+5.0f %+5.0f %+5.0f mm | front q %+6.1f %+6.1f %+6.1f -> %3u %3u %3u | "
                   "back q %+6.1f %+6.1f %+6.1f -> %3u %3u %3u | FK err %.1f mm | %s | %s\n",
                   MODE_NAMES[mode], target.x, target.y, target.z,
                   q.q1, q.q2, q.q3, joint_deg_to_pos(LEG_FRONT, 0, q.q1), joint_deg_to_pos(LEG_FRONT, 1, q.q2),
                   joint_deg_to_pos(LEG_FRONT, 2, q.q3),
                   q_back.q1, q_back.q2, q_back.q3, joint_deg_to_pos(LEG_BACK, 0, q_back.q1),
                   joint_deg_to_pos(LEG_BACK, 1, q_back.q2), joint_deg_to_pos(LEG_BACK, 2, q_back.q3),
                   fk_err, err == LEG_OK ? "ok" : leg_err_str(err),
                   !rc.connected ? "NO RADIO" : following ? "RUN" : "KILL");
        }
    }
}
