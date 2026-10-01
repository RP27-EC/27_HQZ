/* gimbal_init_config.h - dedicated homing configuration
 *
 * Values ported from commit cfabc79 ("云台归中参数调整，现在能快速归中").
 * At that commit G_INIT fell through to the mechanical cascade, so homing used
 * the mechanical PID set with no brake-curve speed limit and no D filtering.
 * Everything here stays local to G_INIT and never touches G_MEC / G_GYRO / G_RATE.
 */

#ifndef __GIMBAL_INIT_CONFIG_H
#define __GIMBAL_INIT_CONFIG_H

/* Homing targets, mechanical coordinate (deg). */
#define GIMBAL_INIT_YAW_HOME_DEG              0.0f
#define GIMBAL_INIT_PITCH_HOME_DEG            0.0f

/* Homing motion limits.
 * GIMBAL_INIT_SPEED_LIMIT_ENABLE 0 = clamp the outer loop by out_max only,
 * which is what the legacy fast-homing path did. 1 = enable the sqrt brake
 * curve below. */
#define GIMBAL_INIT_SPEED_LIMIT_ENABLE        0u
#define GIMBAL_INIT_YAW_MAX_RATE_DEG_S        200.0f
#define GIMBAL_INIT_PITCH_MAX_RATE_DEG_S      45.0f
#define GIMBAL_INIT_YAW_DECEL_RAD_S2          4.0f
#define GIMBAL_INIT_PITCH_DECEL_RAD_S2        3.0f
#define GIMBAL_INIT_YAW_RAMP_DEG_PER_MS       0.25f
#define GIMBAL_INIT_PITCH_RAMP_DEG_PER_MS     0.25f

/* Homing completion. */
#define GIMBAL_INIT_TIMEOUT_MS                6000u
#define GIMBAL_INIT_STABLE_MS                 30u
#define GIMBAL_INIT_YAW_TOL_DEG               2.0f
#define GIMBAL_INIT_PITCH_TOL_DEG             2.0f
#define GIMBAL_INIT_YAW_SPEED_TOL_RAD_S       0.5f
#define GIMBAL_INIT_PITCH_SPEED_TOL_RAD_S     0.5f

/* Homing torque limits, independent from other modes. */
#define GIMBAL_INIT_YAW_TORQUE_LIMIT_NM       6.0f
#define GIMBAL_INIT_PITCH_TORQUE_LIMIT_NM     6.0f

/* Homing D-term filter. 0 = raw derivative, as in the legacy path. */
#define GIMBAL_INIT_D_FILTER_ALPHA            0.0f

/* Yaw homing PID. ki = 0 on purpose: the final torque clamp sits outside the
 * PID, so a non-zero integral would wind up and overshoot on arrival. */
#define GIMBAL_INIT_YAW_OUTER_KP              0.1f
#define GIMBAL_INIT_YAW_OUTER_KI              0.0f
#define GIMBAL_INIT_YAW_OUTER_KD              1.0f
#define GIMBAL_INIT_YAW_OUTER_INTEGRAL_MAX    0.0f
#define GIMBAL_INIT_YAW_OUTER_OUT_MAX         500.0f
#define GIMBAL_INIT_YAW_INNER_KP              1.5f
#define GIMBAL_INIT_YAW_INNER_KI              0.0f
#define GIMBAL_INIT_YAW_INNER_KD              0.2f
#define GIMBAL_INIT_YAW_INNER_INTEGRAL_MAX    0.0f
#define GIMBAL_INIT_YAW_INNER_OUT_MAX         100.0f

/* Pitch homing PID. */
#define GIMBAL_INIT_PITCH_OUTER_KP            1.6f
#define GIMBAL_INIT_PITCH_OUTER_KI            0.0f
#define GIMBAL_INIT_PITCH_OUTER_KD            0.0f
#define GIMBAL_INIT_PITCH_OUTER_INTEGRAL_MAX  500.0f
#define GIMBAL_INIT_PITCH_OUTER_OUT_MAX       10.0f
#define GIMBAL_INIT_PITCH_INNER_KP            1.2f
#define GIMBAL_INIT_PITCH_INNER_KI            0.0f
#define GIMBAL_INIT_PITCH_INNER_KD            0.0f
#define GIMBAL_INIT_PITCH_INNER_INTEGRAL_MAX  0.0f
#define GIMBAL_INIT_PITCH_INNER_OUT_MAX       10.0f

/* Pitch homing gravity feed-forward, independent from other modes. */
#define GIMBAL_INIT_PITCH_GRAVITY_ENABLE      1u
#define GIMBAL_INIT_PITCH_GRAVITY_K_NM        1.1f
#define GIMBAL_INIT_PITCH_GRAVITY_B_NM        0.0f
#define GIMBAL_INIT_PITCH_GRAVITY_SIGN        1.0f
#define GIMBAL_INIT_PITCH_GRAVITY_MIDDLE_DEG  0.0f

#endif
