// MODE_TEST_MOTOR_RC: drive motor from the controller, nothing else (no IMU, no legs).
// CH5 kill switch, CH10 scroll wheel = forward speed. See components/drive.

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "modes.h"
#include "rc.h"
#include "drive.h"

#define PRINT_PERIOD_MS 200

void test_motor_rc_run(void)
{
    drive_start();      // first, so the ESC sees neutral as early as possible
    rc_start();
    while (1) {
        rc_state_t rc;
        rc_get(&rc);
        print_drive();
        if (rc.connected) {
            printf("CH10 (speed) %4u  CH5 (kill switch) %4u\n", rc.ch[9], rc.ch[4]);
        } else {
            printf("RC no signal\n");
        }
        vTaskDelay(pdMS_TO_TICKS(PRINT_PERIOD_MS));
    }
}
