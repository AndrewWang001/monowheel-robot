#include "leg_control.h"

#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "servo.h"
#include "legs.h"
#include "rc.h"

#define POSE_SWITCH_CH    6       // CH7 (index 6): left middle 3-position switch
#define STAND_BUTTON_CH   5       // CH6 (index 5): left top button, momentary (191 / 1792 while held)
#define CAL_BUTTON_CH     8       // CH9 (index 8): right top button, momentary: offset calibration

// Raw servo positions (0..1000), measured with the calibration tool after refitting horns and saving
// servo offsets (straight = 500 on every joint). Columns: back IDs 1, 3, 2 / front IDs 4, 5, 6.
// Front q2 (ID 5) ground/button values: recorded with servo offset -54, +13 for the new offset -67.
// P_VERT, P_HALF and P_PUSH_UP are filled in at startup (see init_positions).
enum {
    P_VERT,         // each joint's zero in legs.c (straight up, 500)
    P_HALF,         // midpoint of vertical and ground
    P_GROUND,
    P_BUTTON,
    P_MID_PUSH,     // stand-up keyframes from read-mode snapshots
    P_CRAWL,
    P_PUSH_UP,      // SLOW_END_FRAC of the way from crawl to straight: the slow part starts here
    P_STRAIGHT,
    P_BACK_VERT,
    P_GROUNDED,
    P_FINAL,
    NUM_POS
};
static int16_t s_pos[NUM_POS][NUM_LEGS][JOINTS_PER_LEG] = {
    [P_GROUND]    = {[LEG_BACK] = {490, 832, 928}, [LEG_FRONT] = {485, 118, 160}},
    [P_BUTTON]    = {[LEG_BACK] = {502,  51, 666}, [LEG_FRONT] = {501, 687,  53}},
    [P_MID_PUSH]  = {[LEG_BACK] = {492, 335, 227}, [LEG_FRONT] = {508, 237, 321}},
    [P_CRAWL]     = {[LEG_BACK] = {491, 129, 274}, [LEG_FRONT] = {503, 269, 120}},
    [P_STRAIGHT]  = {[LEG_BACK] = {497, 167, 104}, [LEG_FRONT] = {498, 105, 162}},
    [P_BACK_VERT] = {[LEG_BACK] = {479, 516, 493}, [LEG_FRONT] = {498, 105, 162}},
    [P_GROUNDED]  = {[LEG_BACK] = {444, 809, 908}, [LEG_FRONT] = {447,  99, 197}},
    [P_FINAL]     = {[LEG_BACK] = {445, 685, 927}, [LEG_FRONT] = {456,  84, 294}},
};
#define SLOW_END_FRAC     0.7f    // crawl -> straight: first 70% medium-slow, last 30% slow
// Speeds: raw steps per second for the joint that moves furthest; the others finish with it
#define SPEED_FASTEST     1000    // ~240 deg/s, close to the servo's max (~300 deg/s)
#define SPEED_MED_FAST    300
#define MOVE_SPEED        250     // CH7 poses
#define SPEED_MED         180     // ~43 deg/s
#define SPEED_MED_SLOW    120
#define SPEED_SLOW        60

// One step: move to a pose at a speed, then wait before the next step
typedef struct {
    uint8_t pos;
    uint16_t speed;
    uint16_t wait_ms;
    const char *name;
} step_t;

typedef struct {
    const char *name;
    const step_t *steps;
    int n;
} seq_t;

#define SEQ(name, steps) {name, steps, sizeof(steps) / sizeof(steps[0])}

// CH6: the whole stand-up from one press. Always starts by lining up at vertical.
static const step_t STAND_STEPS[] = {
    {P_VERT,      SPEED_MED,      1500, "vertical"},
    {P_BUTTON,    SPEED_MED,      1000, "button pose"},
    {P_MID_PUSH,  SPEED_MED_FAST, 0,    "mid push up"},
    {P_CRAWL,     SPEED_MED_FAST, 0,    "crawl in both"},
    {P_PUSH_UP,   SPEED_MED_SLOW, 0,    "push up"},
    {P_STRAIGHT,  SPEED_SLOW,     0,    "straight"},        // slow so it doesn't tip over
    {P_BACK_VERT, SPEED_FASTEST,  0,    "back vertical"},   // these two are one fast motion
    {P_GROUNDED,  SPEED_FASTEST,  2500, "grounded"},
    {P_FINAL,     SPEED_MED,      0,    "final"},
};
// CH7 poses
static const step_t VERT_STEPS[]   = {{P_VERT, MOVE_SPEED, 0, "vertical"}};
static const step_t HALF_STEPS[]   = {{P_HALF, MOVE_SPEED, 0, "half"}};
static const step_t GROUND_STEPS[] = {
    {P_GROUND, MOVE_SPEED, 1500, "ground"},
    {P_FINAL,  SPEED_MED,  0,    "final"},
};
static const seq_t SEQ_STAND  = SEQ("stand-up", STAND_STEPS);
static const seq_t SEQ_VERT   = SEQ("vertical", VERT_STEPS);
static const seq_t SEQ_HALF   = SEQ("half", HALF_STEPS);
static const seq_t SEQ_GROUND = SEQ("ground", GROUND_STEPS);
#define LOOP_MS           50      // 20 Hz
#define SEQ_MIN_MS        100     // shortest move (e.g. lining up when already in place)
#define FIRST_MOVE_MS     1500    // first move after switching to RUN is at least this slow
#define BUTTON_DEBOUNCE   2       // loops the button must read pressed before it counts (100 ms)
#define FIND_EVERY_MS     2000    // how often to look for servos that haven't answered

#define LEG_TASK_STACK    4096
#define LEG_TASK_PRIO     4       // below drive (5) and IMU (6)
#define LEG_TASK_CORE     0

static const char *TAG = "LEG_CTRL";

static bool s_active[NUM_LEGS][JOINTS_PER_LEG];   // servos that have answered
static bool s_warned[NUM_LEGS][JOINTS_PER_LEG];
static leg_control_state_t s_state;
static portMUX_TYPE s_lock = portMUX_INITIALIZER_UNLOCKED;

// Where each joint is: the target of the last move (measured after a stop), and the move in progress
static int16_t s_est[NUM_LEGS][JOINTS_PER_LEG];
static int16_t s_from[NUM_LEGS][JOINTS_PER_LEG];
static TickType_t s_move_start;
static uint32_t s_move_ms;

// Sequence runner
typedef enum { PH_MOVING, PH_WAITING, PH_DONE } phase_t;
static const seq_t *s_seq;          // NULL = nothing yet (or cancelled while killed)
static int s_step;
static phase_t s_phase;
static bool s_paused;
static TickType_t s_phase_end;      // end of the current move or wait
static TickType_t s_wait_left;      // remaining wait when paused during a wait

static int clamp_pos(int pos)
{
    if (pos < 0) return 0;
    if (pos > SERVO_POS_MAX) return SERVO_POS_MAX;
    return pos;
}

static void init_positions(void)
{
    for (int leg = 0; leg < NUM_LEGS; leg++) {
        for (int j = 0; j < JOINTS_PER_LEG; j++) {
            int zero = joint_cal[leg][j].zero;
            int crawl = s_pos[P_CRAWL][leg][j], straight = s_pos[P_STRAIGHT][leg][j];
            s_pos[P_VERT][leg][j] = clamp_pos(zero);
            s_pos[P_HALF][leg][j] = clamp_pos((zero + s_pos[P_GROUND][leg][j]) / 2);
            s_pos[P_PUSH_UP][leg][j] = clamp_pos(crawl + (int)((straight - crawl) * SLOW_END_FRAC));            s_est[leg][j] = s_pos[P_VERT][leg][j];
        }
    }
}

// 3-position switch (191 / 997 / 1792): up vertical, middle half, down ground (then final)
static const seq_t *switch_seq(uint16_t ch)
{
    if (ch > 1400) return &SEQ_VERT;
    if (ch > 600) return &SEQ_HALF;
    return &SEQ_GROUND;
}

static bool tick_reached(TickType_t now, TickType_t t)
{
    return (int32_t)(now - t) >= 0;
}

static void stop_all(void)
{
    for (int leg = 0; leg < NUM_LEGS; leg++)
        for (int j = 0; j < JOINTS_PER_LEG; j++)
            if (s_active[leg][j]) servo_stop(joint_cal[leg][j].id);
}

// Measure where the joints are; any servo that doesn't answer keeps its estimate
static void read_positions(void)
{
    for (int leg = 0; leg < NUM_LEGS; leg++) {
        for (int j = 0; j < JOINTS_PER_LEG; j++) {
            int16_t p;
            if (s_active[leg][j] && servo_read_pos(joint_cal[leg][j].id, &p) == ESP_OK) s_est[leg][j] = clamp_pos(p);
        }
    }
}

// Estimate where a move in progress has got to (in case a servo doesn't answer a position read)
static void estimate_now(TickType_t now)
{
    uint32_t done_ms = (now - s_move_start) * portTICK_PERIOD_MS;
    float f = s_move_ms && done_ms < s_move_ms ? (float)done_ms / s_move_ms : 1.0f;
    for (int leg = 0; leg < NUM_LEGS; leg++)
        for (int j = 0; j < JOINTS_PER_LEG; j++)
            s_est[leg][j] = s_from[leg][j] + (int)((s_est[leg][j] - s_from[leg][j]) * f);
}

// Send the current step's move, timed from where the joints are now
static void start_move(uint32_t min_ms)
{
    const step_t *st = &s_seq->steps[s_step];
    int max_d = 0;
    for (int leg = 0; leg < NUM_LEGS; leg++) {
        for (int j = 0; j < JOINTS_PER_LEG; j++) {
            int d = abs(s_pos[st->pos][leg][j] - s_est[leg][j]);
            if (s_active[leg][j] && d > max_d) max_d = d;
        }
    }
    uint32_t ms = (uint32_t)max_d * 1000 / st->speed;
    if (ms < SEQ_MIN_MS) ms = SEQ_MIN_MS;
    if (ms < min_ms) ms = min_ms;

    for (int leg = 0; leg < NUM_LEGS; leg++) {
        for (int j = 0; j < JOINTS_PER_LEG; j++) {
            s_from[leg][j] = s_est[leg][j];
            s_est[leg][j] = s_pos[st->pos][leg][j];
            if (s_active[leg][j]) servo_move(joint_cal[leg][j].id, s_est[leg][j], ms);
        }
    }
    s_move_start = xTaskGetTickCount();
    s_move_ms = ms;
    s_phase = PH_MOVING;
    s_phase_end = s_move_start + pdMS_TO_TICKS(ms);
    ESP_LOGI(TAG, "%s %d/%d: %s over %lu ms", s_seq->name, s_step + 1, s_seq->n, st->name, (unsigned long)ms);
}

static void seq_start(const seq_t *seq, uint32_t min_ms)
{
    s_seq = seq;
    s_step = 0;
    s_paused = false;
    start_move(min_ms);
}

// Stop where it is. A move is stopped mid-way; a wait remembers how long it had left.
static void seq_pause(void)
{
    if (!s_seq || s_paused || s_phase == PH_DONE) return;
    TickType_t now = xTaskGetTickCount();
    if (s_phase == PH_MOVING) {
        stop_all();
        estimate_now(now);
        read_positions();
    } else {
        s_wait_left = tick_reached(now, s_phase_end) ? 0 : s_phase_end - now;
    }
    s_paused = true;
    ESP_LOGW(TAG, "%s paused at %d/%d (%s)", s_seq->name, s_step + 1, s_seq->n, s_seq->steps[s_step].name);
}

// Carry on from where it paused: redo the interrupted move from here, or finish the wait
static void seq_resume(void)
{
    s_paused = false;
    ESP_LOGI(TAG, "%s resumed", s_seq->name);
    if (s_phase == PH_WAITING) s_phase_end = xTaskGetTickCount() + s_wait_left;
    else start_move(0);
}

static void seq_tick(void)
{
    if (!s_seq || s_paused || s_phase == PH_DONE) return;
    TickType_t now = xTaskGetTickCount();
    if (!tick_reached(now, s_phase_end)) return;

    const step_t *st = &s_seq->steps[s_step];
    if (s_phase == PH_MOVING && st->wait_ms > 0) {
        s_phase = PH_WAITING;
        s_phase_end = now + pdMS_TO_TICKS(st->wait_ms);
        return;
    }
    if (++s_step >= s_seq->n) {
        s_step = s_seq->n - 1;
        s_phase = PH_DONE;
        ESP_LOGI(TAG, "%s done", s_seq->name);
        return;
    }
    start_move(0);
}

// Ping servos that haven't answered yet. A newly found servo goes to position mode, limp
// unless the legs are already following, in which case it powers up and moves to where it should be.
static int find_servos(bool following)
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
                        servo_move(id, s_est[leg][j], FIRST_MOVE_MS);
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

// Momentary button: a press counts once it has read pressed for BUTTON_DEBOUNCE loops, then it
// waits for release. Presses while killed (allowed = false) are ignored.
typedef struct {
    int pressed_loops;
    bool latched;
} button_t;

static bool button_pressed(button_t *b, const rc_state_t *rc, int ch, bool allowed)
{
    if (rc->connected && rc->ch[ch] > 1400) {
        if (++b->pressed_loops >= BUTTON_DEBOUNCE && !b->latched) {
            b->latched = true;
            return allowed;
        }
    } else {
        b->pressed_loops = 0;
        b->latched = false;
    }
    return false;
}

static void set_torque_all(bool on)
{
    for (int leg = 0; leg < NUM_LEGS; leg++)
        for (int j = 0; j < JOINTS_PER_LEG; j++)
            if (s_active[leg][j]) servo_set_torque(joint_cal[leg][j].id, on);
}

// ---- CH9 offset calibration ----
// Press CH9: servos go limp and the readings print. Hold every joint straight up, press CH9
// again: each servo's offset becomes (current offset + reading - 500) and is saved in the servo,
// then the legs drive to 500 (vertical) to check. Moving CH7 or the kill switch before the
// second press cancels without saving anything.
#define CAL_SAMPLES       5       // readings averaged per servo for the snapshot
#define CAL_PRINT_MS      1000

static void cal_print_readings(void)
{
    char line[200];
    int len = snprintf(line, sizeof(line), "CAL: hold legs straight up, press CH%d to save (CH%d = cancel) |",
                       CAL_BUTTON_CH + 1, POSE_SWITCH_CH + 1);
    for (int leg = 0; leg < NUM_LEGS; leg++) {
        for (int j = 0; j < JOINTS_PER_LEG; j++) {
            int16_t p;
            if (s_active[leg][j] && servo_read_pos(joint_cal[leg][j].id, &p) == ESP_OK) {
                len += snprintf(line + len, sizeof(line) - len, " ID%u=%d", joint_cal[leg][j].id, p);
            } else {
                len += snprintf(line + len, sizeof(line) - len, " ID%u=--", joint_cal[leg][j].id);
            }
        }
    }
    ESP_LOGI(TAG, "%s", line);
}

// Take the snapshot and save the new offsets. Saves nothing unless every servo can be done.
static bool cal_snapshot(void)
{
    int8_t old_off[NUM_LEGS][JOINTS_PER_LEG], new_off[NUM_LEGS][JOINTS_PER_LEG];
    int reading[NUM_LEGS][JOINTS_PER_LEG];
    bool ok = true;

    for (int leg = 0; leg < NUM_LEGS; leg++) {
        for (int j = 0; j < JOINTS_PER_LEG; j++) {
            uint8_t id = joint_cal[leg][j].id;
            if (!s_active[leg][j]) {
                ESP_LOGE(TAG, "CAL: ID %u (%s) not replying, can't calibrate", id, joint_name(leg, j));
                ok = false;
                continue;
            }
            int sum = 0, n = 0;
            for (int s = 0; s < CAL_SAMPLES; s++) {
                int16_t p;
                if (servo_read_pos(id, &p) == ESP_OK) {
                    sum += p;
                    n++;
                }
            }
            if (n < CAL_SAMPLES / 2 + 1 || servo_read_offset(id, &old_off[leg][j]) != ESP_OK) {
                ESP_LOGE(TAG, "CAL: ID %u (%s) didn't answer the reads", id, joint_name(leg, j));
                ok = false;
                continue;
            }
            reading[leg][j] = (sum + n / 2) / n;
            int want = old_off[leg][j] + (reading[leg][j] - 500);
            if (want > SERVO_OFFSET_MAX || want < -SERVO_OFFSET_MAX) {
                ESP_LOGE(TAG, "CAL: ID %u (%s) reads %d, would need offset %d (max +-%d). "
                              "Refit that horn closer to straight", id, joint_name(leg, j),
                         reading[leg][j], want, SERVO_OFFSET_MAX);
                ok = false;
                continue;
            }
            new_off[leg][j] = (int8_t)want;
        }
    }
    if (!ok) {
        ESP_LOGE(TAG, "CAL: nothing saved. Fix the above and press CH%d again, or move CH%d to cancel",
                 CAL_BUTTON_CH + 1, POSE_SWITCH_CH + 1);
        return false;
    }

    for (int leg = 0; leg < NUM_LEGS; leg++) {
        for (int j = 0; j < JOINTS_PER_LEG; j++) {
            uint8_t id = joint_cal[leg][j].id;
            if (new_off[leg][j] != old_off[leg][j]) {
                servo_set_offset(id, new_off[leg][j]);
                servo_save_offset(id);
                vTaskDelay(pdMS_TO_TICKS(20));
            }
            ESP_LOGW(TAG, "CAL: ID %u (%-8s) read %4d: offset %+4d -> %+4d%s", id, joint_name(leg, j),
                     reading[leg][j], old_off[leg][j], new_off[leg][j],
                     new_off[leg][j] != old_off[leg][j] ? " (saved)" : " (unchanged)");
        }
    }
    // For SERVO_OFFSETS in main/modes/tool_servo_cal.c, which would otherwise put back its own values
    ESP_LOGW(TAG, "CAL: SERVO_OFFSETS = {1,%d} {3,%d} {2,%d} {4,%d} {5,%d} {6,%d}",
             new_off[LEG_BACK][0], new_off[LEG_BACK][1], new_off[LEG_BACK][2],
             new_off[LEG_FRONT][0], new_off[LEG_FRONT][1], new_off[LEG_FRONT][2]);
    return true;
}

static void leg_task(void *arg)
{
    ESP_ERROR_CHECK(servo_bus_init(SERVO_UART, SERVO_TX_GPIO, SERVO_RX_GPIO));
    init_positions();
    ESP_LOGI(TAG, "Legs: CH%d up/mid/down = vertical/half/ground->final, CH%d = stand-up (press again "
                  "to pause/continue), CH%d = offset calibration. Only while kill switch CH%d is RUN",
             POSE_SWITCH_CH + 1, STAND_BUTTON_CH + 1, CAL_BUTTON_CH + 1, RC_KILL_SWITCH_CH + 1);

    bool following = false, powered = false;
    const seq_t *last_switch = &SEQ_VERT;
    button_t stand_btn = {0}, cal_btn = {0};
    bool calibrating = false;       // CH9 calibration: servos limp, waiting for the snapshot press
    TickType_t cal_next_print = 0;

    int servos = find_servos(false);
    TickType_t last_find = xTaskGetTickCount();
    TickType_t wake = xTaskGetTickCount();

    while (1) {
        vTaskDelayUntil(&wake, pdMS_TO_TICKS(LOOP_MS));

        if (xTaskGetTickCount() - last_find >= pdMS_TO_TICKS(FIND_EVERY_MS)) {
            servos = find_servos(following && !calibrating);    // calibrating: late servos stay limp
            last_find = xTaskGetTickCount();
        }

        rc_state_t rc;
        rc_get(&rc);
        bool run = rc_run_allowed(&rc);

        bool pressed = button_pressed(&stand_btn, &rc, STAND_BUTTON_CH, run);
        bool cal_pressed = button_pressed(&cal_btn, &rc, CAL_BUTTON_CH, run);

        // CH7 moved: it always wins (cancels the stand-up, so the next CH6 press starts over)
        bool switch_moved = false;
        const seq_t *sw = switch_seq(rc.ch[POSE_SWITCH_CH]);
        if (rc.connected && sw != last_switch) {
            last_switch = sw;
            switch_moved = true;
            if (!following || !run) s_seq = NULL;   // killed: go to the CH7 pose on RUN instead of resuming
        }

        if (!run) {
            if (calibrating) {
                // Cancel: nothing saved. Servos stay limp; on RUN they power up and go to the CH7 pose.
                ESP_LOGW(TAG, "CAL: cancelled by %s, nothing saved", rc.connected ? "kill switch" : "radio loss");
                calibrating = false;
                powered = false;
                s_seq = NULL;
            }
            // Freeze: stop any move where it is, servos hold (or stay limp before the first RUN)
            if (following) {
                ESP_LOGW(TAG, "%s", rc.connected ? "Kill switch: legs stopped" : "Radio lost: legs stopped");
                seq_pause();
            }
            following = false;
        } else {
            if (!following) {
                if (!powered) {
                    set_torque_all(true);
                    powered = true;
                }
                read_positions();
                following = true;
                if (!s_seq) {
                    ESP_LOGI(TAG, "Legs active, moving to %s", last_switch->name);
                    seq_start(last_switch, FIRST_MOVE_MS);
                } else if (s_phase == PH_DONE) {
                    ESP_LOGI(TAG, "Legs active, holding %s", s_seq->name);
                } else if (s_seq == &SEQ_STAND) {
                    ESP_LOGI(TAG, "Legs active, stand-up still paused: press CH%d to continue", STAND_BUTTON_CH + 1);
                } else {
                    seq_resume();
                }
            } else if (calibrating) {
                if (switch_moved) {
                    ESP_LOGW(TAG, "CAL: cancelled by CH%d, nothing saved", POSE_SWITCH_CH + 1);
                    calibrating = false;
                    set_torque_all(true);
                    read_positions();
                    seq_start(last_switch, FIRST_MOVE_MS);
                } else if (cal_pressed) {
                    if (cal_snapshot()) {
                        // Offsets changed what the servos report, so measure again, then go to 500
                        calibrating = false;
                        set_torque_all(true);
                        read_positions();
                        ESP_LOGW(TAG, "CAL: done. Moving to 500: check every leg is straight up");
                        seq_start(&SEQ_VERT, FIRST_MOVE_MS);
                    }
                } else if (pressed) {
                    ESP_LOGI(TAG, "CAL: CH%d ignored while calibrating", STAND_BUTTON_CH + 1);
                } else if (tick_reached(xTaskGetTickCount(), cal_next_print)) {
                    cal_print_readings();
                    cal_next_print = xTaskGetTickCount() + pdMS_TO_TICKS(CAL_PRINT_MS);
                }
            } else if (switch_moved) {
                seq_start(last_switch, 0);
            } else if (cal_pressed) {
                if (s_seq == &SEQ_STAND && !s_paused && s_phase != PH_DONE) {
                    ESP_LOGW(TAG, "CAL: not while the stand-up is moving (press CH%d to pause it first)",
                             STAND_BUTTON_CH + 1);
                } else {
                    seq_pause();
                    s_seq = NULL;
                    set_torque_all(false);
                    calibrating = true;
                    cal_next_print = xTaskGetTickCount();
                    ESP_LOGW(TAG, "CAL: servos limp. Hold every leg straight up, then press CH%d. "
                                  "Move CH%d to cancel", CAL_BUTTON_CH + 1, POSE_SWITCH_CH + 1);
                }
            } else if (pressed) {
                if (s_seq != &SEQ_STAND) seq_start(&SEQ_STAND, 0);
                else if (s_phase == PH_DONE) ESP_LOGI(TAG, "Stand-up already done: use CH%d to reset", POSE_SWITCH_CH + 1);
                else if (s_paused) seq_resume();
                else seq_pause();
            }
            if (!calibrating) seq_tick();
        }

        portENTER_CRITICAL(&s_lock);
        if (calibrating) {
            s_state.seq = "CALIBRATE";
            s_state.step = "hold straight";
            s_state.step_num = s_state.steps = 0;
            s_state.status = "CH9=save";
        } else if (s_seq) {
            s_state.seq = s_seq->name;
            s_state.step = s_seq->steps[s_step].name;
            s_state.step_num = s_step + 1;
            s_state.steps = s_seq->n;
            s_state.status = s_phase == PH_DONE ? "done" : s_paused ? "PAUSED" : s_phase == PH_WAITING ? "wait" : "move";
        } else {
            s_state.seq = "-";
            s_state.step = "-";
            s_state.step_num = s_state.steps = 0;
            s_state.status = "-";
        }
        s_state.following = following;
        s_state.servos = servos;
        portEXIT_CRITICAL(&s_lock);
    }
}

void leg_control_start(void)
{
    s_state.seq = s_state.step = s_state.status = "-";
    xTaskCreatePinnedToCore(leg_task, "leg_ctrl", LEG_TASK_STACK, NULL, LEG_TASK_PRIO, NULL, LEG_TASK_CORE);
}

void leg_control_get(leg_control_state_t *out)
{
    portENTER_CRITICAL(&s_lock);
    *out = s_state;
    portEXIT_CRITICAL(&s_lock);
}
