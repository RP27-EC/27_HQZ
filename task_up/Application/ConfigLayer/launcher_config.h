#ifndef __LAUNCHER_CONFIG_H
#define __LAUNCHER_CONFIG_H

/* Friction wheel speed control. */
#define LAUNCHER_DIAL_ENABLE              1u
#define LAUNCHER_REPEAT_ENABLE            1u
#define LAUNCHER_FRIC_TARGET_RPM          2000.0f
#define LAUNCHER_FRIC_RAMP_RPM_PER_MS     20.0f
#define LAUNCHER_FRIC_STOP_RAMP_RPM_PER_MS 20.0f
#define LAUNCHER_FRIC_STOP_SPEED_RPM      100.0f
#define LAUNCHER_FRIC_STOP_CONFIRM_MS     1000u
#define LAUNCHER_FRIC_READY_TOL_RPM       500.0f
#define LAUNCHER_FRIC_READY_TIME_MS       100u
#define LAUNCHER_FRIC_KP                  6.0f
#define LAUNCHER_FRIC_KI                  0.02f
#define LAUNCHER_FRIC_KD                  0.0f
#define LAUNCHER_FRIC_INTEGRAL_MAX        7000.0f
#define LAUNCHER_FRIC_KFF                 0.3f
#define LAUNCHER_FRIC_OUT_MAX             10000.0f
#define LAUNCHER_FRIC_L_DIRECTION         1.0f
#define LAUNCHER_FRIC_R_DIRECTION         -1.0f

/* LK4005 dial control, feedback unit is 0.01 degree. */
#define LAUNCHER_DIAL_AUTO_RESET_ENABLE  0u
#define LAUNCHER_DIAL_READY_HOLD_ENABLE  0u
#define LAUNCHER_DIAL_ANGLE_KP            0.3f
#define LAUNCHER_DIAL_ANGLE_SIGN          1.0f
#define LAUNCHER_DIAL_SPEED_SIGN          1.0f
#define LAUNCHER_DIAL_OUTPUT_SIGN         1.0f
#define LAUNCHER_DIAL_DIRECTION           1.0f
#define LAUNCHER_DIAL_ANGLE_KI            0.0f
#define LAUNCHER_DIAL_ANGLE_KD            0.0f
#define LAUNCHER_DIAL_ANGLE_INTEGRAL_MAX  0.0f
#define LAUNCHER_DIAL_SPEED_KP             0.3f
#define LAUNCHER_DIAL_SPEED_KI             0.0f
#define LAUNCHER_DIAL_SPEED_KD             0.0f
#define LAUNCHER_DIAL_SPEED_INTEGRAL_MAX   0.0f
#define LAUNCHER_DIAL_CURRENT_LIMIT        2000.0f
#define LAUNCHER_DIAL_RESET_ANGLE          31259.0f
#define LAUNCHER_DIAL_RESET_TIMEOUT_MS     1000u
#define LAUNCHER_DIAL_ONE_SHOT_ANGLE      65536.0f
#define LAUNCHER_DIAL_REVERSE_ANGLE       65536.0f
#define LAUNCHER_DIAL_STOP_ERROR          500.0f
#define LAUNCHER_DIAL_MAX_SPEED_DPS       7000u
#define LAUNCHER_DIAL_REPEAT_SPEED_DPS    3600u
#define LAUNCHER_DIAL_SINGLE_TIMEOUT_MS   500u
#define LAUNCHER_DIAL_REVERSE_TIMEOUT_MS  200u
#define LAUNCHER_DIAL_RELOAD_TIMEOUT_MS   200u

/* Repeat speed loop. */
#define LAUNCHER_DIAL_REPEAT_KP            0.5f
#define LAUNCHER_DIAL_REPEAT_KI            0.01f
#define LAUNCHER_DIAL_REPEAT_KD            0.0f
#define LAUNCHER_DIAL_REPEAT_INTEGRAL_MAX  0.0f

/* Jam detection. */
#define LAUNCHER_DIAL_JAM_CURRENT_RAW     1200
#define LAUNCHER_DIAL_JAM_SPEED_DPS       2
#define LAUNCHER_DIAL_JAM_TIME_MS         500u
#define LAUNCHER_DIAL_JAM_MAX_RETRY       8u

#endif
