// MODE_TOOL_SERVO_CAL: one-time tool for installing the leg servos so the angles in
// leg_kinematics.h line up with the real joints. Works with however many of the servos in
// legs.c are connected, so you can do one leg (or one joint) at a time. Pick the step in
// tools/pick_mode.py (it sets SERVO_CAL_STEP).
//
// Joint directions (leg_kinematics.h), zero pose = leg pointing straight up, base facing +y:
//   q1 base      positive turns counterclockwise seen from above (FRONT leg).
//                The back leg is mirrored, so for it positive is CLOCKWISE seen from above.
//   q2 shoulder  positive leans the leg forward (toward +y, the direction of travel)
//   q3 elbow     positive bends the lower link further forward
//
// Steps:
//   1. CAL_CENTER  before fitting horns/links. Drives every servo to position 500 (mid travel)
//                  and holds it. Fit each horn so its joint is as close to the zero pose as the
//                  spline allows (the spline only gives you steps of ~14 deg; that's fine).
//   2. CAL_READ    torque off. Prints raw position and calibrated angle of every joint.
//                  Hold the leg in the zero pose, copy the raw positions into joint_cal[].zero
//                  (components/legs/legs.c), then turn each joint positive and flip
//                  joint_cal[].sign where the angle drops. Reflash and repeat until the zero
//                  pose reads ~0 and directions are right.
//   3. CAL_CHECK   torque on. Puts the legs in the zero pose, then moves one joint at a time to
//                  +CHECK_DEG and back, logging which joint should move which way.
//   CAL_SLIDER     torque on, servos hold where they are. Move them live from your computer:
//                  close idf.py monitor, then run  python sim/servo_sliders.py
//                  Serial protocol (USB console, 115200, one command per line):
//                    L                 -> one "J <id> <pos> <zero> <name>" line per connected
//                                         servo (<zero> = its 0 deg position from legs.c)
//                    M <id> <pos> [ms] -> move servo <id> to <pos> (0..1000) over ms (default 100)
//                  and it prints "P <id>=<pos> ..." with the read-back positions 4x a second.

#include "modes.h"

#include <stdio.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "legs.h"
#include "servo.h"

#define CENTER_POS     500
#define MOVE_MS        1500
#define CHECK_DEG      20.0f
#define READ_PERIOD_MS 250
#define SLIDER_MOVE_MS 100   // short ramp so slider drags look smooth instead of jerky

static const char *TAG = "CALIBRATE";

static bool s_present[NUM_LEGS][JOINTS_PER_LEG];

#define FOR_EACH_PRESENT(leg, j)                          \
    for (int leg = 0; leg < NUM_LEGS; leg++)         \
        for (int j = 0; j < JOINTS_PER_LEG; j++)          \
            if (s_present[leg][j])

static uint8_t id_of(int leg, int j)
{
    return joint_cal[leg][j].id;
}

// Find which of the servos in joint_cal are connected. Blocks until at least one answers.
static void find_servos(void)
{
    while (1) {
        int n = 0;
        for (int leg = 0; leg < NUM_LEGS; leg++) {
            for (int j = 0; j < JOINTS_PER_LEG; j++) {
                uint8_t got;
                s_present[leg][j] = servo_read_id(id_of(leg, j), &got) == ESP_OK;
                ESP_LOGI(TAG, "%-8s ID %u: %s", joint_name(leg, j), id_of(leg, j),
                         s_present[leg][j] ? "found" : "not connected");
                n += s_present[leg][j];
            }
        }
        if (n) return;
        ESP_LOGW(TAG, "No servos answering. Check power, wiring and IDs (scan with the leg sweep test)");
        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}

// Look again for servos that didn't answer at startup (late power-up, loose plug). Newly found
// servos get their angle limits opened and are left limp. Returns how many were found.
static int find_late_servos(void)
{
    int found = 0;
    for (int leg = 0; leg < NUM_LEGS; leg++) {
        for (int j = 0; j < JOINTS_PER_LEG; j++) {
            uint8_t got;
            if (s_present[leg][j] || servo_read_id(id_of(leg, j), &got) != ESP_OK) continue;
            s_present[leg][j] = true;
            servo_write_angle_limit(id_of(leg, j), 0, SERVO_POS_MAX);
            vTaskDelay(pdMS_TO_TICKS(20));
            servo_set_torque(id_of(leg, j), false);
            ESP_LOGI(TAG, "%-8s ID %u: found (late)", joint_name(leg, j), id_of(leg, j));
            found++;
        }
    }
    return found;
}

// A limit left over from an earlier calibration would stop the servo short, so open it up
// for calibrating. legs_init() sets the real limits in the normal app.
static void open_angle_limits(void)
{
    FOR_EACH_PRESENT(leg, j) {
        uint16_t lo, hi;
        if (servo_read_angle_limit(id_of(leg, j), &lo, &hi) == ESP_OK && lo == 0 && hi == SERVO_POS_MAX) continue;
        servo_write_angle_limit(id_of(leg, j), 0, SERVO_POS_MAX);
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

static void enable_position_mode(void)
{
    FOR_EACH_PRESENT(leg, j) {
        servo_set_servo_mode(id_of(leg, j));
        servo_set_torque(id_of(leg, j), true);
    }
}

static void move_joint(int leg, int j, float deg, uint16_t time_ms)
{
    servo_move(id_of(leg, j), joint_deg_to_pos(leg, j, deg), time_ms);
}

static void run_center(void)
{
    enable_position_mode();
    ESP_LOGW(TAG, "Driving all servos to %d. Keep links OFF until they stop", CENTER_POS);
    FOR_EACH_PRESENT(leg, j) servo_move(id_of(leg, j), CENTER_POS, MOVE_MS);
    vTaskDelay(pdMS_TO_TICKS(MOVE_MS + 500));
    ESP_LOGI(TAG, "Holding at %d. Now fit each horn/link with its joint in the zero pose "
                  "(leg straight up, base facing +y)", CENTER_POS);

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(3000));
        FOR_EACH_PRESENT(leg, j) {
            servo_move(id_of(leg, j), CENTER_POS, 0); // re-send in case a servo browned out
            int16_t pos;
            if (servo_read_pos(id_of(leg, j), &pos) == ESP_OK) {
                ESP_LOGI(TAG, "%-8s pos=%4d", joint_name(leg, j), pos);
            }
        }
    }
}

static void run_read(void)
{
    FOR_EACH_PRESENT(leg, j) servo_set_torque(id_of(leg, j), false);
    ESP_LOGI(TAG, "Torque off. Pose the leg at zero and copy 'raw' into joint_cal[].zero; turn each "
                  "joint positive and flip joint_cal[].sign where 'deg' goes down");

    char line[160];
    int64_t next_find = 0;
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(READ_PERIOD_MS));
        int64_t now = esp_timer_get_time() / 1000;
        if (now >= next_find) {
            next_find = now + 2000;     // missing servos join within ~2 s
            find_late_servos();
        }
        for (int leg = 0; leg < NUM_LEGS; leg++) {
            int len = 0;
            bool any = false;
            for (int j = 0; j < JOINTS_PER_LEG; j++) {
                if (!s_present[leg][j]) continue;
                any = true;
                int16_t pos;
                if (servo_read_pos(id_of(leg, j), &pos) == ESP_OK) {
                    len += snprintf(line + len, sizeof(line) - len, "  %s raw=%4d deg=%+6.1f",
                                    joint_name(leg, j), pos, joint_pos_to_deg(leg, j, pos));
                } else {
                    len += snprintf(line + len, sizeof(line) - len, "  %s (no reply)", joint_name(leg, j));
                }
            }
            if (any) ESP_LOGI(TAG, "%s", line);
        }
    }
}

static void run_check(void)
{
    static const char *expect[JOINTS_PER_LEG] = {
        "base turns CCW seen from above (back leg: CW)",
        "leg leans forward (+y)",
        "lower link bends forward",
    };

    enable_position_mode();
    while (1) {
        ESP_LOGI(TAG, "All joints to 0 deg: legs should point straight up, base facing +y");
        FOR_EACH_PRESENT(leg, j) move_joint(leg, j, 0.0f, MOVE_MS);
        vTaskDelay(pdMS_TO_TICKS(MOVE_MS + 1500));

        FOR_EACH_PRESENT(leg, j) {
            ESP_LOGI(TAG, "%s -> %+.0f deg: %s", joint_name(leg, j), CHECK_DEG, expect[j]);
            move_joint(leg, j, CHECK_DEG, MOVE_MS);
            vTaskDelay(pdMS_TO_TICKS(MOVE_MS + 300));
            int16_t pos;
            if (servo_read_pos(id_of(leg, j), &pos) == ESP_OK) {
                ESP_LOGI(TAG, "%s reads %+.1f deg", joint_name(leg, j), joint_pos_to_deg(leg, j, pos));
            }
            vTaskDelay(pdMS_TO_TICKS(1000));
            move_joint(leg, j, 0.0f, MOVE_MS);
            vTaskDelay(pdMS_TO_TICKS(MOVE_MS + 500));
        }
        vTaskDelay(pdMS_TO_TICKS(3000));
    }
}

static void slider_list(void)
{
    FOR_EACH_PRESENT(leg, j) {
        int16_t pos = -1;
        servo_read_pos(id_of(leg, j), &pos);
        printf("J %u %d %u %s\n", id_of(leg, j), pos, joint_deg_to_pos(leg, j, 0.0f), joint_name(leg, j));
    }
}

static void slider_command(char *line)
{
    if (line[0] == 'L') {
        slider_list();
    } else if (line[0] == 'M') {
        char *end;
        long id = strtol(line + 1, &end, 10);
        long pos = strtol(end, &end, 10);
        long ms = strtol(end, NULL, 10); // optional, 0 if missing
        if (id < 0 || id > 253 || pos < 0 || pos > SERVO_POS_MAX) return;
        if (ms <= 0 || ms > 30000) ms = SLIDER_MOVE_MS;
        servo_move((uint8_t)id, (uint16_t)pos, (uint16_t)ms);
    }
}

static void run_slider(void)
{
    // Hold each servo where it already is, so turning torque on doesn't make it jump
    FOR_EACH_PRESENT(leg, j) {
        int16_t pos;
        servo_set_servo_mode(id_of(leg, j));
        if (servo_read_pos(id_of(leg, j), &pos) == ESP_OK && pos >= 0 && pos <= SERVO_POS_MAX) {
            servo_move(id_of(leg, j), pos, 0);
        }
        servo_set_torque(id_of(leg, j), true);
    }

    // Read commands straight off the console UART (logging keeps working alongside)
    const uart_port_t con = CONFIG_ESP_CONSOLE_UART_NUM;
    if (!uart_is_driver_installed(con)) ESP_ERROR_CHECK(uart_driver_install(con, 256, 0, 0, NULL, 0));
    ESP_LOGI(TAG, "Slider mode. Close idf.py monitor and run sim/servo_sliders.py");
    slider_list();

    char line[32];
    int len = 0;
    int64_t next_report = 0;
    while (1) {
        uint8_t c;
        while (uart_read_bytes(con, &c, 1, pdMS_TO_TICKS(20)) == 1) {
            if (c == '\n' || c == '\r') {
                line[len] = 0;
                if (len) slider_command(line);
                len = 0;
            } else if (len < (int)sizeof(line) - 1) {
                line[len++] = c;
            }
        }

        int64_t now = esp_timer_get_time() / 1000;
        if (now >= next_report) {
            next_report = now + READ_PERIOD_MS;
            printf("P");
            FOR_EACH_PRESENT(leg, j) {
                int16_t pos;
                if (servo_read_pos(id_of(leg, j), &pos) == ESP_OK) printf(" %u=%d", id_of(leg, j), pos);
            }
            printf("\n");
        }
    }
}

void tool_servo_cal_run(int step)
{
    ESP_ERROR_CHECK(servo_bus_init(SERVO_UART, SERVO_TX_GPIO, SERVO_RX_GPIO));
    find_servos();
    open_angle_limits();

    switch (step) {
    case CAL_CENTER: run_center(); break;
    case CAL_READ:   run_read();   break;
    case CAL_CHECK:  run_check();  break;
    case CAL_SLIDER: run_slider(); break;
    }
}
