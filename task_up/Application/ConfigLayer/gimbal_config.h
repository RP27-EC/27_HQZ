/* gimbal_config.h - 云台归中配置 */

#ifndef __GIMBAL_CONFIG_H
#define __GIMBAL_CONFIG_H

/* 机械零点：电机原始角转机械坐标 */
#define GIMBAL_YAW_MIDDLE_DEG          (-22.224138f)
#define GIMBAL_PITCH_MIDDLE_DEG        148.573157f

/* 上电归中目标：机械坐标，0 为机械中值 */
#define GIMBAL_HOME_YAW_DEG            0.0f
#define GIMBAL_HOME_PITCH_DEG          0.0f

/* 上电归中过程 */
#define GIMBAL_INIT_TIMEOUT_MS         6000u
#define GIMBAL_INIT_YAW_TOL_DEG        2.0f
#define GIMBAL_INIT_PITCH_TOL_DEG      2.0f
#define GIMBAL_INIT_YAW_RAMP_DEG_PER_MS 0.25f
#define GIMBAL_INIT_PITCH_RAMP_DEG_PER_MS 0.25f
#define GIMBAL_INIT_YAW_SPEED_TOL_RAD_S 0.5f
#define GIMBAL_INIT_PITCH_SPEED_TOL_RAD_S 0.5f
#define GIMBAL_INIT_STABLE_MS          30u

/* 模式切换目标斜坡，区别于上电归中 */
#define GIMBAL_MODE_YAW_RAMP_DEG_PER_MS 0.1f
#define GIMBAL_MODE_PITCH_RAMP_DEG_PER_MS 0.1f

#endif
