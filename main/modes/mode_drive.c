// MODE_DRIVE: the normal firmware. IMU + controller, with the CH5 kill switch over everything:
//   CH10 (side scroll wheel)  -> both drive motors, forward only (components/drive)
//   CH7 (left middle switch)  -> legs: up vertical, middle half, down ground (components/leg_control)
//   CH6 (left top button)     -> legs: press toggles the button pose
//   CH8 (right middle switch) + CH3 (left stick) -> stand-up in 3 stages from the button pose

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "modes.h"
#include "imu.h"
#include "rc.h"
#include "drive.h"
#include "leg_control.h"

#define PRINT_PERIOD_MS 200
#define PRINT_CHANNELS  10      // CH1-10 (CH10 = scroll wheel speed)

void mode_drive_run(void)
{
    drive_start();      // first, so the ESCs see neutral as early as possible
    imu_start();
    rc_start();
    leg_control_start();

    while (1) {
        print_imu();
        print_drive();
        print_legs();
        print_rc(PRINT_CHANNELS);
        printf("\n");
        vTaskDelay(pdMS_TO_TICKS(PRINT_PERIOD_MS));
    }
}
