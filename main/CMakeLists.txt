idf_component_register(SRCS "main.c"
                            "modes/status_print.c"
                            "modes/mode_drive.c"
                            "modes/test_imu.c"
                            "modes/test_rc.c"
                            "modes/test_leg_sweep.c"
                            "modes/test_motor_sweep.c"
                            "modes/test_motor_rc.c"
                            "modes/tool_servo_cal.c"
                            "modes/test_balance.c"
                            "modes/tool_servo_set_id.c"
                    INCLUDE_DIRS "." "modes"
                    PRIV_REQUIRES imu rc esc drive leg_control servo_bus legs leg_kinematics gait pins
                                  esp_driver_uart esp_timer)

# Firmware mode file (written by tools/pick_mode.py, gitignored). It must exist before the first
# compile: the build only notices changes to headers it has already seen, so a file that appears
# later would be ignored and the old mode kept. Create it with the default (Drive) when CMake runs.
# After that the compiler tracks it, so picking a new mode just recompiles main.c.
set(MODE_FILE "${CMAKE_CURRENT_LIST_DIR}/mode_select.h")
if(NOT EXISTS "${MODE_FILE}")
    file(WRITE "${MODE_FILE}" "// Default mode (no mode picked yet). Change with tools/pick_mode.py\n#define APP_MODE MODE_DRIVE\n")
endif()
