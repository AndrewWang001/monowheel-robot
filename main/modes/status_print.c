#include <stdio.h>
#include "modes.h"
#include "imu.h"
#include "rc.h"
#include "drive.h"
#include "leg_control.h"

void print_imu(void)
{
    imu_state_t s;
    imu_get(&s);
    if (s.valid) {
        printf("R%7.1f P%7.1f Y%7.1f | ", s.roll, s.pitch, s.yaw);
    } else {
        printf("IMU waiting           | ");
    }
}

void print_rc(int channels)
{
    rc_state_t rc;
    rc_get(&rc);
    if (!rc.connected) {
        printf("RC no signal | ");
        return;
    }
    for (int i = 0; i < channels && i < RC_NUM_CHANNELS; i++) printf("CH%d %4u ", i + 1, rc.ch[i]);
    printf("| ");
}

void print_drive(void)
{
    drive_state_t d;
    drive_get(&d);
    printf("ESC %4lu %s | ", (unsigned long)d.pulse_us, d.motor_enabled ? "RUN " : "KILL");
}

void print_legs(void)
{
    leg_control_state_t l;
    leg_control_get(&l);
    printf("LEGS %-8s ", l.pose ? l.pose : "-");
    if (l.stand_stage) printf("stand S%d %3.0f%% %s ", l.stand_stage, l.stand_pct, l.stand_armed ? "go  " : "wait");
    printf("%s %d/6 | ", l.following ? "RUN " : "HOLD", l.servos);
}
