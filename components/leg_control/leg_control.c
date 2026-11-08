#include "leg_control.h"

#include <math.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "servo.h"
#include "legs.h"
#include "rc.h"

#define POSE_SWITCH_CH    6       // CH7 (index 6): left middle 3-position switch
#define POSE_BUTTON_CH    5       // CH6 (index 5): left top button, momentary (191 / 1792 while held)
#define STAND_STAGE_CH    7       // CH8 (index 7): right middle 3-position switch, stand-up stage
#define STAND_CH          2       // CH3 (index 2): left stick up/down, no spring, stand-up progress

// Raw servo positions (0..1000) of each pose, measured with the calibration tool after refitting
// horns and saving servo offsets (straight = 500 on every joint). Vertical = each joint's zero in
// legs.c; halfway = midpoint of vertical and ground. Columns: back IDs 1, 3, 2 / front IDs 4, 5, 6.
// Front q2 (ID 5) values: recorded with servo offset -54, +13 for the new offset -67.
static const int16_t GROUND_POS[NUM_LEGS][JOINTS_PER_LEG] = {
    [LEG_BACK]  = {490, 832, 928},
    [LEG_FRONT] = {485, 118, 160},
};
static const int16_t BUTTON_POS[NUM_LEGS][JOINTS_PER_LEG] = {
    [LEG_BACK]  = {502, 51, 666},
    [LEG_FRONT] = {501, 687, 53},
};

// Stand-up sequence, only from the button pose. Keyframes from read-mode snapshots; where only one
// leg moved in a step, the other leg repeats the previous key. Split into 3 stages:
//   CH8 selects the stage (down = 1, middle = 2, up = 3). Within a stage, CH3 bottom = the stage's
//   first key, top = its last key, straight lines between keys.
//   Going up: finish a stage (CH3 at the top), set CH8 to the next stage, bring CH3 back to the
//   bottom, then raise it again. Going down is the mirror: at the bottom of a stage, set CH8 to the
//   previous stage, bring CH3 to the top, then lower it.
//   Whenever the stage changes (or after a kill), CH3 must first come to where the legs are
//   before it takes control again, so the legs never jump.
#define STAND_KEYS        7
static const int16_t STAND_POS[STAND_KEYS][NUM_LEGS][JOINTS_PER_LEG] = {
    {[LEG_BACK] = {502,  51, 666}, [LEG_FRONT] = {501, 687,  53}},    //  0 button pose (= BUTTON_POS)
    {[LEG_BACK] = {492, 335, 227}, [LEG_FRONT] = {508, 237, 321}},    //  1 mid push up
    {[LEG_BACK] = {491, 129, 274}, [LEG_FRONT] = {503, 269, 120}},    //  2 crawl in both      (end stage 1)
    {[LEG_BACK] = {497, 167, 104}, [LEG_FRONT] = {498, 105, 162}},    //  3 push up + straight (end stage 2)
    {[LEG_BACK] = {479, 516, 493}, [LEG_FRONT] = {498, 105, 162}},    //  4 back only: vertical
    {[LEG_BACK] = {444, 809, 908}, [LEG_FRONT] = {447,  99, 197}},    //  5 grounded
    {[LEG_BACK] = {445, 685, 927}, [LEG_FRONT] = {456,  84, 294}},    //  6 final              (end stage 3)
};
#define STAND_STAGES      3
static const int STAGE_FIRST_KEY[STAND_STAGES + 1] = {0, 2, 3, 6};  // stage s runs key [s-1] .. [s]

#define STAND_LOW_PCT     5.0f    // CH3 below this = bottom of the stage
#define STAND_HIGH_PCT    95.0f   // CH3 above this = top of the stage
#define STAND_PICKUP      0.05f   // CH3 must be this close (fraction of a stage) to the legs to take over
#define STAND_STEP        0.005f  // ignore CH3 changes smaller than this (fraction of a stage)
#define STAND_MOVE_MS     100     // each CH3 step is spread over this

typedef enum { POSE_VERTICAL, POSE_HALF, POSE_GROUND, POSE_BUTTON } pose_t;
static const char *POSE_NAMES[] = {"vertical", "half", "ground", "button"};

#define LOOP_MS           50      // 20 Hz
#define MOVE_SPEED        250     // raw steps per second for the joint that moves furthest (~60 deg/s)
#define BUTTON_MOVE_SPEED 180     // slower for moves into or out of the button pose (~43 deg/s)
#define MIN_MOVE_MS       300
#define FIRST_MOVE_MS     1500    // slow move to the selected pose when switching to RUN
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

// 3-position switch (191 / 997 / 1792): up vertical, middle half, down ground
static pose_t switch_pose(uint16_t ch)
{
    if (ch > 1400) return POSE_VERTICAL;
    if (ch > 600) return POSE_HALF;
    return POSE_GROUND;
}

// 3-position switch -> stand-up stage: down 1, middle 2, up 3
static int switch_stage(uint16_t ch)
{
    if (ch > 1400) return 3;
    if (ch > 600) return 2;
    return 1;
}

static int clamp_pos(int pos)
{
    if (pos < 0) return 0;
    if (pos > SERVO_POS_MAX) return SERVO_POS_MAX;
    return pos;
}

static int pose_pos(pose_t pose, int leg, int j)
{
    int zero = joint_cal[leg][j].zero;
    switch (pose) {
    case POSE_HALF:   return clamp_pos((zero + GROUND_POS[leg][j]) / 2);
    case POSE_GROUND: return clamp_pos(GROUND_POS[leg][j]);
    case POSE_BUTTON: return clamp_pos(BUTTON_POS[leg][j]);
    default:          return clamp_pos(zero);
    }
}

// Time for a pose change: the joint that moves furthest goes at the move speed, the rest finish with it
static uint16_t move_time(pose_t from, pose_t to)
{
    int max_d = 0;
    for (int leg = 0; leg < NUM_LEGS; leg++) {
        for (int j = 0; j < JOINTS_PER_LEG; j++) {
            int d = abs(pose_pos(to, leg, j) - pose_pos(from, leg, j));
            if (d > max_d) max_d = d;
        }
    }
    int speed = (from == POSE_BUTTON || to == POSE_BUTTON) ? BUTTON_MOVE_SPEED : MOVE_SPEED;
    int ms = max_d * 1000 / speed;
    return ms < MIN_MOVE_MS ? MIN_MOVE_MS : ms;
}

// CH3 -> fraction of the current stage: 0 at the bottom (below STAND_LOW_PCT) .. 1 at the top
static float stick_to_frac(uint16_t ch)
{
    float pct = ((float)ch - RC_MIN) / (RC_MAX - RC_MIN) * 100.0f;
    float f = (pct - STAND_LOW_PCT) / (STAND_HIGH_PCT - STAND_LOW_PCT);
    if (f > 1.0f) f = 1.0f;
    if (f < 0.0f) f = 0.0f;
    return f;
}

// Move every joint to frac (0..1) of the way through stage (1..3) of the stand-up keyframes
static void move_stand(int stage, float frac, uint16_t time_ms)
{
    int first = STAGE_FIRST_KEY[stage - 1], last = STAGE_FIRST_KEY[stage];
    float x = first + frac * (last - first);
    int k = (int)x;
    if (k >= last) k = last - 1;
    float t = x - k;                            // 0..1 between key k and k+1

    for (int leg = 0; leg < NUM_LEGS; leg++) {
        for (int j = 0; j < JOINTS_PER_LEG; j++) {
            if (!s_active[leg][j]) continue;
            int a = STAND_POS[k][leg][j], b = STAND_POS[k + 1][leg][j];
            servo_move(joint_cal[leg][j].id, clamp_pos(a + (int)lroundf((b - a) * t)), time_ms);
        }
    }
}

static void move_one(int leg, int j, pose_t pose, uint16_t time_ms)
{
    servo_move(joint_cal[leg][j].id, pose_pos(pose, leg, j), time_ms);
}

static void move_all(pose_t pose, uint16_t time_ms)
{
    for (int leg = 0; leg < NUM_LEGS; leg++)
        for (int j = 0; j < JOINTS_PER_LEG; j++)
            if (s_active[leg][j]) move_one(leg, j, pose, time_ms);
}

// Ping servos that haven't answered yet. A newly found servo goes to position mode, limp
// unless the legs are already following, in which case it powers up and moves to the current pose.
static int find_servos(bool following, pose_t pose)
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
                        move_one(leg, j, pose, FIRST_MOVE_MS);
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
    ESP_LOGI(TAG, "Legs: CH%d up/mid/down = vertical/half/ground, CH%d toggles the button pose, "
                  "then CH%d stage + CH%d stick = stand up. Only while kill switch CH%d is RUN",
             POSE_SWITCH_CH + 1, POSE_BUTTON_CH + 1, STAND_STAGE_CH + 1, STAND_CH + 1, RC_KILL_SWITCH_CH + 1);

    bool following = false, powered = false;
    pose_t sent = POSE_VERTICAL;
    pose_t last_switch = POSE_VERTICAL;
    bool button_pose = false;       // toggled by the CH6 button, cleared when CH7 moves
    int pressed_loops = 0;          // debounce counter for the button
    bool button_latched = false;    // press already counted, wait for release

    int stand_stage = 0;            // 0 = not in the button pose, else 1..3
    float stand_sent = 0.0f;        // fraction of the stage last sent
    bool stand_armed = false;       // CH3 controls the legs
    float stand_hold = 0.0f;        // where CH3 must come to before it takes control

    int servos = find_servos(false, sent);
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

        // Button: count a press once it has read pressed for BUTTON_DEBOUNCE loops, then wait for release
        if (rc.connected && rc.ch[POSE_BUTTON_CH] > 1400) {
            if (++pressed_loops >= BUTTON_DEBOUNCE && !button_latched) {
                button_latched = true;
                if (run) button_pose = !button_pose;    // presses while killed are ignored
            }
        } else {
            pressed_loops = 0;
            button_latched = false;
        }

        // Moving the CH7 switch always wins over the button pose
        pose_t sw = switch_pose(rc.ch[POSE_SWITCH_CH]);
        if (rc.connected && sw != last_switch) {
            last_switch = sw;
            button_pose = false;
        }
        pose_t target = button_pose ? POSE_BUTTON : last_switch;

        if (!run) {
            // Freeze: send nothing, servos hold their last position (or stay limp before first RUN)
            if (following) ESP_LOGW(TAG, "%s", rc.connected ? "Kill switch: legs frozen" : "Radio lost: legs frozen");
            following = false;
        } else if (!following) {
            if (!powered) {
                set_torque_all(true);
                powered = true;
            }
            if (target == POSE_BUTTON && sent == POSE_BUTTON && stand_stage > 0) {
                // Back from a kill mid stand-up: stay put, CH3 must come back to where the legs are
                stand_armed = false;
                stand_hold = stand_sent;
                ESP_LOGI(TAG, "Legs active, stand-up stage %d held: move CH%d to %.0f %% to continue",
                         stand_stage, STAND_CH + 1, stand_sent * 100.0f);
            } else {
                ESP_LOGI(TAG, "Legs active, moving to %s", POSE_NAMES[target]);
                move_all(target, FIRST_MOVE_MS);
                vTaskDelay(pdMS_TO_TICKS(FIRST_MOVE_MS));
                wake = xTaskGetTickCount();
                sent = target;
                stand_stage = 0;
            }
            following = true;
        } else if (target != sent) {
            // One smooth move; the servos interpolate it themselves
            uint16_t ms = move_time(sent, target);
            ESP_LOGI(TAG, "Legs %s -> %s over %u ms", POSE_NAMES[sent], POSE_NAMES[target], ms);
            move_all(target, ms);
            sent = target;
            stand_stage = 0;
        }

        // Stand-up, only in the button pose
        if (following && sent == POSE_BUTTON) {
            if (stand_stage == 0) {         // just arrived in the button pose: stage 1, at its start
                stand_stage = 1;
                stand_sent = 0.0f;
                stand_armed = false;
                stand_hold = 0.0f;
                ESP_LOGI(TAG, "Stand-up: set CH%d to stage 1 (down) and CH%d to the bottom",
                         STAND_STAGE_CH + 1, STAND_CH + 1);
            }
            float frac = stick_to_frac(rc.ch[STAND_CH]);
            int sel = rc.connected ? switch_stage(rc.ch[STAND_STAGE_CH]) : stand_stage;

            if (!stand_armed) {
                // CH3 takes over once it is where the legs are and CH8 matches the stage
                if (sel == stand_stage && fabsf(frac - stand_hold) <= STAND_PICKUP) {
                    stand_armed = true;
                    ESP_LOGI(TAG, "Stand-up stage %d: CH%d in control", stand_stage, STAND_CH + 1);
                }
            } else if (stand_sent == 1.0f && sel == stand_stage + 1 && stand_stage < STAND_STAGES) {
                // Top of a stage and CH8 moved up: next stage starts where this one ended
                stand_stage++;
                stand_sent = 0.0f;
                stand_armed = false;
                stand_hold = 0.0f;
                ESP_LOGI(TAG, "Stand-up stage %d: bring CH%d to the bottom, then raise it", stand_stage, STAND_CH + 1);
            } else if (stand_sent == 0.0f && sel == stand_stage - 1 && stand_stage > 1) {
                // Bottom of a stage and CH8 moved down: previous stage, starting from its end
                stand_stage--;
                stand_sent = 1.0f;
                stand_armed = false;
                stand_hold = 1.0f;
                ESP_LOGI(TAG, "Stand-up back to stage %d: bring CH%d to the top, then lower it", stand_stage, STAND_CH + 1);
            } else if (fabsf(frac - stand_sent) >= STAND_STEP ||
                       ((frac == 0.0f || frac == 1.0f) && frac != stand_sent)) {
                move_stand(stand_stage, frac, STAND_MOVE_MS);
                stand_sent = frac;
            }
        } else if (sent != POSE_BUTTON) {
            stand_stage = 0;
        }

        portENTER_CRITICAL(&s_lock);
        s_state.pose = POSE_NAMES[sent];
        s_state.stand_stage = stand_stage;
        s_state.stand_pct = stand_sent * 100.0f;
        s_state.stand_armed = stand_armed;
        s_state.following = following;
        s_state.servos = servos;
        portEXIT_CRITICAL(&s_lock);
    }
}

void leg_control_start(void)
{
    s_state.pose = POSE_NAMES[POSE_VERTICAL];
    xTaskCreatePinnedToCore(leg_task, "leg_ctrl", LEG_TASK_STACK, NULL, LEG_TASK_PRIO, NULL, LEG_TASK_CORE);
}

void leg_control_get(leg_control_state_t *out)
{
    portENTER_CRITICAL(&s_lock);
    *out = s_state;
    portEXIT_CRITICAL(&s_lock);
}
