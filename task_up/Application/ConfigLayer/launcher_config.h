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

/* LK4005 dial control.
 *
 * Feedback unit is ENCODER COUNT, not degree:
 *   dial_angle / dial_target_angle are int32_t encoder counts,
 *   65536 counts = one full turn of the output shaft.
 *   deg -> count : count = deg * 65536 / 360
 *      36 deg = 6554 counts (one pellet slot)
 *      18 deg = 3277 counts
 *
 * Do NOT read this as 0.01 deg/LSB -- that is the unit of the motor's own
 * motorAngle field, which this project does not use.
 *
 * Angle loop : error[count]  -> speed target[dps]
 * Speed loop : error[dps]    -> iqControl raw value (-2000..2000)
 * Control period is 1 ms. */
#define LAUNCHER_DIAL_AUTO_RESET_ENABLE   0u
/* READY cuts torque to zero (the dial self-locks, so nothing holds it).
 * Turning this on makes the angle loop hold position instead -- but the
 * deadband edge is a step discontinuity, so the motor buzzes around the
 * target. Only enable it if the mechanism actually springs back when free. */
#define LAUNCHER_DIAL_READY_HOLD_ENABLE   0u

/* Angle loop: v_cmd[dps] = KP*err[count] + KD*(err - last_err).
 * With the 1 ms loop the P term alone gives tau = 1000/(182.04*KP) ms,
 * so KP=0.45 -> about 12 ms. err falls by speed[dps]*0.182 counts per ms,
 * so at full speed (1200 dps) the D term contributes
 *   1.5 * 0.182 * 1200 = 328 dps
 * of braking. That is the term that kills the overshoot -- P alone always
 * arrives at the target still carrying speed. Raise KD if it still overshoots,
 * lower KP if it oscillates at low frequency. */
#define LAUNCHER_DIAL_ANGLE_KP            0.45f
#define LAUNCHER_DIAL_ANGLE_KI            0.0f
#define LAUNCHER_DIAL_ANGLE_KD            1.5f
#define LAUNCHER_DIAL_ANGLE_INTEGRAL_MAX  0.0f
/* Below this error the angle loop outputs zero, so the READY hold stops
 * hunting around the target instead of buzzing. Keep it under STOP_ERROR
 * so the "arrived" test still fires first. Costs 1.4 deg of static accuracy. */
#define LAUNCHER_DIAL_ANGLE_DEADBAND      250.0f

/* Speed loop. err=1200 dps -> 720 iq, still under the 1200 jam threshold.
 * Keep this noticeably faster than the angle loop above, otherwise the two
 * loops have comparable bandwidth and the cascade rings. */
#define LAUNCHER_DIAL_SPEED_KP            0.6f
#define LAUNCHER_DIAL_SPEED_KI            0.0f
#define LAUNCHER_DIAL_SPEED_KD            0.0f
#define LAUNCHER_DIAL_SPEED_INTEGRAL_MAX  0.0f

/* Direction signs. If the speed feedback sign disagrees with the encoder_sum
 * direction, the speed loop drives the wrong way and the dial runs away.
 * Verify by hand-turning the dial before trusting these. */
#define LAUNCHER_DIAL_ANGLE_SIGN          1.0f  /* <0 flips the angle sign */
#define LAUNCHER_DIAL_SPEED_SIGN          1.0f  /* <0 flips the speed sign */
#define LAUNCHER_DIAL_OUTPUT_SIGN         1.0f  /* <0 flips the output current */
#define LAUNCHER_DIAL_DIRECTION           1.0f  /* <0 flips the one-shot direction */

#define LAUNCHER_DIAL_CURRENT_LIMIT       2000.0f
/* 1200 dps = 3.3 turns/s, so one 36 deg shot takes about 30 ms of travel.
 * Lower it to calm the approach down further, raise it for a snappier shot. */
#define LAUNCHER_DIAL_MAX_SPEED_DPS       1200u

#define LAUNCHER_DIAL_RESET_ANGLE         31259.0f
#define LAUNCHER_DIAL_RESET_TIMEOUT_MS    1000u

#define LAUNCHER_DIAL_ONE_SHOT_ANGLE      6554.0f  /* 36 deg */
#define LAUNCHER_DIAL_REVERSE_ANGLE       3277.0f  /* 18 deg, back off half a slot on jam */
#define LAUNCHER_DIAL_STOP_ERROR          400.0f   /* 2.2 deg */
#define LAUNCHER_DIAL_SINGLE_TIMEOUT_MS   800u
#define LAUNCHER_DIAL_REVERSE_TIMEOUT_MS  200u
#define LAUNCHER_DIAL_RELOAD_TIMEOUT_MS   200u

/* Repeat speed loop. */
#define LAUNCHER_DIAL_REPEAT_SPEED_DPS    3600u
#define LAUNCHER_DIAL_REPEAT_KP           0.5f
#define LAUNCHER_DIAL_REPEAT_KI           0.01f
#define LAUNCHER_DIAL_REPEAT_KD           0.0f
#define LAUNCHER_DIAL_REPEAT_INTEGRAL_MAX 0.0f

/* Jam detection. Without JAM_ENABLE the REVERSE/RELOAD states are compiled out. */
#define LAUNCHER_DIAL_JAM_ENABLE          1u
#define LAUNCHER_DIAL_JAM_CURRENT_RAW     1200
#define LAUNCHER_DIAL_JAM_SPEED_DPS       2
#define LAUNCHER_DIAL_JAM_TIME_MS         500u
#define LAUNCHER_DIAL_JAM_MAX_RETRY       8u

#endif
