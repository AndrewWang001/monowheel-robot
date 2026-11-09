// MODE_TEST_IMU: BNO085 only. Prints the robot's lean / tilt (0 = upright) plus the raw values,
// 20x a second (the IMU itself runs at 300 Hz; the serial port can't print that fast).

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "modes.h"
#include "imu.h"

#define PRINT_PERIOD_MS 50

// Side-to-side state from lean (deg, + = right) and lean rate (deg/s). Thresholds are first guesses.
#define UPRIGHT_DEG     7.0f    // within this = upright (+5 still counts, +10.7 is a real lean)
#define FALLEN_DEG      45.0f   // past this = fallen over
#define FALLING_DPS     30.0f   // moving further over faster than this = falling

static const char *lean_state(float lean, float rate)
{
    if (lean >= FALLEN_DEG) return "FALLEN RIGHT";
    if (lean <= -FALLEN_DEG) return "FALLEN LEFT";
    if (rate >= FALLING_DPS && lean > 0) return "FALLING RIGHT";
    if (rate <= -FALLING_DPS && lean < 0) return "FALLING LEFT";
    if (lean > UPRIGHT_DEG) return "leaning right";
    if (lean < -UPRIGHT_DEG) return "leaning left";
    return "upright";
}

void test_imu_run(void)
{
    imu_start();
    while (1) {
        imu_state_t s;
        imu_get(&s);
        if (s.valid) {
            printf("%-13s | LEAN %+6.1f (%+6.1f dps)  TILT %+6.1f (%+6.1f dps) | raw r %7.2f p %7.2f y %7.2f "
                   "gyro %6.1f %6.1f %6.1f | acc %u\n",
                   lean_state(s.lean, s.lean_rate), s.lean, s.lean_rate, s.tilt, s.tilt_rate, s.roll, s.pitch, s.yaw,
                   s.gyro[0], s.gyro[1], s.gyro[2], s.accuracy);
        } else {
            printf("IMU waiting for first reading\n");
        }
        vTaskDelay(pdMS_TO_TICKS(PRINT_PERIOD_MS));
    }
}
