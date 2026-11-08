#include "servo.h"

#include <string.h>
#include "esp_timer.h"

#define SERVO_BAUD       115200
#define REPLY_TIMEOUT_US 50000 // servo answers within a few ms; this bounds a missing reply

// Command values (Hiwonder Bus Servo Communication Protocol, section 4)
#define CMD_MOVE_TIME_WRITE      1
#define CMD_MOVE_STOP            12
#define CMD_ID_WRITE             13
#define CMD_ID_READ              14
#define CMD_ANGLE_OFFSET_ADJUST  17
#define CMD_ANGLE_OFFSET_WRITE   18
#define CMD_ANGLE_OFFSET_READ    19
#define CMD_ANGLE_LIMIT_WRITE    20
#define CMD_ANGLE_LIMIT_READ     21
#define CMD_VIN_LIMIT_WRITE      22
#define CMD_VIN_LIMIT_READ       23
#define CMD_TEMP_READ            26
#define CMD_VIN_READ             27
#define CMD_POS_READ             28
#define CMD_OR_MOTOR_MODE_WRITE  29
#define CMD_LOAD_OR_UNLOAD_WRITE 31
#define CMD_LED_CTRL_WRITE       33

static uart_port_t s_port;

// ID, LEN, CMD and params are exactly LEN bytes starting at the ID byte
static uint8_t checksum(const uint8_t *from_id, uint8_t len)
{
    uint8_t sum = 0;
    for (int i = 0; i < len; i++) sum += from_id[i];
    return ~sum;
}

esp_err_t servo_bus_init(uart_port_t port, int tx_gpio, int rx_gpio)
{
    const uart_config_t cfg = {
        .baud_rate = SERVO_BAUD,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    s_port = port;
    esp_err_t err = uart_driver_install(port, 256, 0, 0, NULL, 0);
    if (err != ESP_OK) return err;
    err = uart_param_config(port, &cfg);
    if (err != ESP_OK) return err;
    return uart_set_pin(port, tx_gpio, rx_gpio, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
}

static esp_err_t send(uint8_t id, uint8_t cmd, const uint8_t *params, uint8_t n)
{
    uint8_t f[16];
    f[0] = 0x55;
    f[1] = 0x55;
    f[2] = id;
    f[3] = n + 3;
    f[4] = cmd;
    if (n) memcpy(&f[5], params, n);
    f[5 + n] = checksum(&f[2], n + 3);

    uart_flush_input(s_port); // drop stale bytes so the next frame we parse is the reply
    int len = n + 6;
    if (uart_write_bytes(s_port, f, len) != len) return ESP_FAIL;
    return uart_wait_tx_done(s_port, pdMS_TO_TICKS(20));
}

// Send a read command and wait for the reply carrying n data bytes. Frames that don't
// match (e.g. our own request echoed back on the half-duplex line) are skipped.
static esp_err_t request(uint8_t id, uint8_t cmd, uint8_t *data, uint8_t n)
{
    esp_err_t err = send(id, cmd, NULL, 0);
    if (err != ESP_OK) return err;

    uint8_t f[16];
    int i = 0;
    int64_t deadline = esp_timer_get_time() + REPLY_TIMEOUT_US;
    while (esp_timer_get_time() < deadline) {
        uint8_t b;
        if (uart_read_bytes(s_port, &b, 1, 1) != 1) continue;

        if (i < 2) { // hunt for the 0x55 0x55 header
            i = (b == 0x55) ? i + 1 : 0;
            continue;
        }
        f[i++] = b;
        if (i == 4 && (f[3] < 3 || f[3] > 10)) { i = 0; continue; } // implausible LEN
        if (i < 4 || i < 3 + f[3]) continue;                          // frame not complete yet

        // Full frame: f[2]=ID f[3]=LEN f[4]=CMD f[5..]=data, checksum last
        uint8_t len = f[3];
        i = 0;
        if (f[2 + len] != checksum(&f[2], len)) continue;
        if (f[4] != cmd || len != n + 3) continue;
        if (id != SERVO_BROADCAST_ID && f[2] != id) continue;
        memcpy(data, &f[5], n);
        return ESP_OK;
    }
    return ESP_ERR_TIMEOUT;
}

esp_err_t servo_move(uint8_t id, uint16_t pos, uint16_t time_ms)
{
    if (pos > SERVO_POS_MAX) pos = SERVO_POS_MAX;
    if (time_ms > 30000) time_ms = 30000;
    uint8_t p[4] = {pos & 0xFF, pos >> 8, time_ms & 0xFF, time_ms >> 8};
    return send(id, CMD_MOVE_TIME_WRITE, p, sizeof(p));
}

esp_err_t servo_stop(uint8_t id)
{
    return send(id, CMD_MOVE_STOP, NULL, 0);
}

esp_err_t servo_set_motor_mode(uint8_t id, int16_t speed)
{
    if (speed > SERVO_SPEED_MAX) speed = SERVO_SPEED_MAX;
    if (speed < -SERVO_SPEED_MAX) speed = -SERVO_SPEED_MAX;
    uint16_t s = (uint16_t)speed; // signed short sent as unsigned (two's complement)
    uint8_t p[4] = {1, 0, s & 0xFF, s >> 8};
    return send(id, CMD_OR_MOTOR_MODE_WRITE, p, sizeof(p));
}

esp_err_t servo_set_servo_mode(uint8_t id)
{
    uint8_t p[4] = {0, 0, 0, 0};
    return send(id, CMD_OR_MOTOR_MODE_WRITE, p, sizeof(p));
}

esp_err_t servo_set_torque(uint8_t id, bool on)
{
    uint8_t p[1] = {on ? 1 : 0};
    return send(id, CMD_LOAD_OR_UNLOAD_WRITE, p, sizeof(p));
}

esp_err_t servo_set_led(uint8_t id, bool on)
{
    uint8_t p[1] = {on ? 0 : 1}; // protocol: 0 = LED on, 1 = LED off
    return send(id, CMD_LED_CTRL_WRITE, p, sizeof(p));
}

esp_err_t servo_write_id(uint8_t id, uint8_t new_id)
{
    uint8_t p[1] = {new_id};
    return send(id, CMD_ID_WRITE, p, sizeof(p));
}

esp_err_t servo_read_id(uint8_t id, uint8_t *out)
{
    return request(id, CMD_ID_READ, out, 1);
}

esp_err_t servo_read_pos(uint8_t id, int16_t *pos)
{
    uint8_t d[2];
    esp_err_t err = request(id, CMD_POS_READ, d, 2);
    if (err == ESP_OK) *pos = (int16_t)(d[0] | (d[1] << 8)); // signed: may read slightly negative
    return err;
}

esp_err_t servo_set_offset(uint8_t id, int8_t offset)
{
    if (offset > SERVO_OFFSET_MAX) offset = SERVO_OFFSET_MAX;
    if (offset < -SERVO_OFFSET_MAX) offset = -SERVO_OFFSET_MAX;
    uint8_t p[1] = {(uint8_t)offset};
    return send(id, CMD_ANGLE_OFFSET_ADJUST, p, sizeof(p));
}

esp_err_t servo_save_offset(uint8_t id)
{
    return send(id, CMD_ANGLE_OFFSET_WRITE, NULL, 0);
}

esp_err_t servo_read_offset(uint8_t id, int8_t *offset)
{
    return request(id, CMD_ANGLE_OFFSET_READ, (uint8_t *)offset, 1);
}

esp_err_t servo_read_vin(uint8_t id, uint16_t *mv)
{
    uint8_t d[2];
    esp_err_t err = request(id, CMD_VIN_READ, d, 2);
    if (err == ESP_OK) *mv = d[0] | (d[1] << 8);
    return err;
}

esp_err_t servo_read_temp(uint8_t id, uint8_t *deg_c)
{
    return request(id, CMD_TEMP_READ, deg_c, 1);
}

esp_err_t servo_read_vin_limit(uint8_t id, uint16_t *min_mv, uint16_t *max_mv)
{
    uint8_t d[4];
    esp_err_t err = request(id, CMD_VIN_LIMIT_READ, d, 4);
    if (err == ESP_OK) {
        *min_mv = d[0] | (d[1] << 8);
        *max_mv = d[2] | (d[3] << 8);
    }
    return err;
}

esp_err_t servo_write_vin_limit(uint8_t id, uint16_t min_mv, uint16_t max_mv)
{
    uint8_t p[4] = {min_mv & 0xFF, min_mv >> 8, max_mv & 0xFF, max_mv >> 8};
    return send(id, CMD_VIN_LIMIT_WRITE, p, sizeof(p));
}

esp_err_t servo_read_angle_limit(uint8_t id, uint16_t *min_pos, uint16_t *max_pos)
{
    uint8_t d[4];
    esp_err_t err = request(id, CMD_ANGLE_LIMIT_READ, d, 4);
    if (err == ESP_OK) {
        *min_pos = d[0] | (d[1] << 8);
        *max_pos = d[2] | (d[3] << 8);
    }
    return err;
}

esp_err_t servo_write_angle_limit(uint8_t id, uint16_t min_pos, uint16_t max_pos)
{
    if (max_pos > SERVO_POS_MAX) max_pos = SERVO_POS_MAX;
    if (min_pos >= max_pos) return ESP_ERR_INVALID_ARG;
    uint8_t p[4] = {min_pos & 0xFF, min_pos >> 8, max_pos & 0xFF, max_pos >> 8};
    return send(id, CMD_ANGLE_LIMIT_WRITE, p, sizeof(p));
}
