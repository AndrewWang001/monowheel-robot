#include "legs.h"

#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "servo.h"

static const char *TAG = "LEGS";

// Calibration, one row per leg, columns q1 q2 q3. Measure with the servo calibration tool (CAL_READ):
//   zero: put the leg in the zero pose by hand (straight up, base facing +y) and copy the
//         raw positions it prints.
//   sign: turn each joint by hand in its positive direction (see main/modes/tool_servo_cal.c). If the
//         printed angle goes down instead of up, flip the sign.
// Then confirm with CAL_CHECK.
const joint_cal_t joint_cal[NUM_LEGS][JOINTS_PER_LEG] = {
// Horns refitted near 500, then each servo's angle offset (SERVO_OFFSETS in tool_servo_cal.c)
// makes straight up read exactly 500. Signs not measured yet.
    [LEG_BACK]  = {{.id = 1, .sign = +1, .zero = 500},
                   {.id = 3, .sign = +1, .zero = 500},
                   {.id = 2, .sign = +1, .zero = 500}},
    [LEG_FRONT] = {{.id = 4, .sign = +1, .zero = 500},
                   {.id = 5, .sign = +1, .zero = 500},
                   {.id = 6, .sign = +1, .zero = 500}},
};

const char *joint_name(leg_id_t leg, int joint)
{
    static const char *names[NUM_LEGS][JOINTS_PER_LEG] = {
        [LEG_BACK] = {"back q1", "back q2", "back q3"},
        [LEG_FRONT] = {"front q1", "front q2", "front q3"},
    };
    return names[leg][joint];
}

static float raw_pos(leg_id_t leg, int joint, float deg)
{
    const joint_cal_t *c = &joint_cal[leg][joint];
    return c->zero + c->sign * deg * JOINT_POS_PER_DEG;
}

uint16_t joint_deg_to_pos(leg_id_t leg, int joint, float deg)
{
    float p = roundf(raw_pos(leg, joint, deg));
    if (p < 0.0f) p = 0.0f;
    if (p > SERVO_POS_MAX) p = SERVO_POS_MAX;
    return (uint16_t)p;
}

float joint_pos_to_deg(leg_id_t leg, int joint, int16_t pos)
{
    const joint_cal_t *c = &joint_cal[leg][joint];
    return c->sign * (pos - c->zero) / JOINT_POS_PER_DEG;
}

// Saved to the servo's non-volatile memory, so only write when it changes
static void set_angle_limit(leg_id_t leg, int joint)
{
    uint8_t id = joint_cal[leg][joint].id;
    float a = raw_pos(leg, joint, LEG_Q_MIN_DEG), b = raw_pos(leg, joint, LEG_Q_MAX_DEG);
    float lo = fminf(a, b), hi = fmaxf(a, b);
    if (lo < 0.0f || hi > SERVO_POS_MAX) {
        ESP_LOGW(TAG, "%s (ID %u): +/-%.0f deg needs positions %.0f..%.0f, outside 0..%d. "
                      "Re-seat the horn closer to centre (zero should be near 500)",
                 joint_name(leg, joint), id, LEG_Q_MAX_DEG, lo, hi, SERVO_POS_MAX);
    }
    uint16_t want_lo = (uint16_t)fmaxf(0.0f, roundf(lo));
    uint16_t want_hi = (uint16_t)fminf(SERVO_POS_MAX, roundf(hi));

    uint16_t cur_lo, cur_hi;
    if (servo_read_angle_limit(id, &cur_lo, &cur_hi) == ESP_OK && cur_lo == want_lo && cur_hi == want_hi) return;
    ESP_LOGI(TAG, "%s (ID %u): setting servo angle limit %u..%u", joint_name(leg, joint), id, want_lo, want_hi);
    servo_write_angle_limit(id, want_lo, want_hi);
    vTaskDelay(pdMS_TO_TICKS(20));  // let it save
}

esp_err_t legs_init(void)
{
    for (int leg = 0; leg < NUM_LEGS; leg++) {
        for (int j = 0; j < JOINTS_PER_LEG; j++) {
            uint8_t id = joint_cal[leg][j].id, got;
            while (servo_read_id(id, &got) != ESP_OK) {
                ESP_LOGW(TAG, "%s (ID %u) not replying", joint_name(leg, j), id);
                vTaskDelay(pdMS_TO_TICKS(2000));
            }
            servo_set_servo_mode(id);
            set_angle_limit(leg, j);
        }
    }
    return ESP_OK;
}

esp_err_t legs_move(leg_id_t leg, const leg_angles_t *q, uint16_t time_ms)
{
    const float deg[JOINTS_PER_LEG] = {q->q1, q->q2, q->q3};
    for (int j = 0; j < JOINTS_PER_LEG; j++) {
        esp_err_t err = servo_move(joint_cal[leg][j].id, joint_deg_to_pos(leg, j, deg[j]), time_ms);
        if (err != ESP_OK) return err;
    }
    return ESP_OK;
}

esp_err_t legs_read(leg_id_t leg, leg_angles_t *q)
{
    float deg[JOINTS_PER_LEG];
    for (int j = 0; j < JOINTS_PER_LEG; j++) {
        int16_t pos;
        esp_err_t err = servo_read_pos(joint_cal[leg][j].id, &pos);
        if (err != ESP_OK) return err;
        deg[j] = joint_pos_to_deg(leg, j, pos);
    }
    q->q1 = deg[0];
    q->q2 = deg[1];
    q->q3 = deg[2];
    return ESP_OK;
}
