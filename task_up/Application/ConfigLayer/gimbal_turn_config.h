/* gimbal_turn_config.h - dedicated mechanical-mode yaw turn (掉头) configuration
 *
 * 掉头与归中用同一套「位置->速度串级」做法（同一个 gimbal_init_pid_calc），
 * 所以参数只有在同一个串级里才有意义：不能把数值单独搬到机械 yaw 自己的串级上，
 * 那边外环输出被制动曲线钳成 rad/s，量纲差 57 倍，搬过去只会让掉头更软。
 *
 * 默认全部是 GIMBAL_INIT_* 的【别名】—— 一个真值来源，调归中掉头自动跟随，
 * 结构上不可能再出现"两边静默分叉"。
 *
 * 如果哪天确实需要单独调掉头某一项：把那一行改回具体数值即可，其余仍保持别名，
 * 是「默认同步、需要时可拆」，不是永久绑定。
 *
 * 曾经的事故（别再犯）：本文件原先是一份手工拷贝。归中外环 kp 从 1.0 调到 0.1
 * 之后没有同步，掉头仍是 1.0。位置刚度 k = INNER_KP * OUTER_KP * 57.3 高了 10 倍，
 * 阻尼比 zeta = c / (2*sqrt(k*J)) 从约 0.57 掉到约 0.18，追 0.25 deg/ms 斜坡时
 * 速度冲过斜坡速度好几倍（4.4 -> 13 rad/s），表现为巨量超调 + 来回晃。
 * 别名化之后这类分叉不会再发生。
 */

#ifndef __GIMBAL_TURN_CONFIG_H
#define __GIMBAL_TURN_CONFIG_H

/* 别名依赖归中的定义，必须可见 */
#include "gimbal_init_config.h"

/* 掉头段与保持段的切换阈值：误差大于该值走掉头串级，否则交回机械 yaw 保持环。
 * 归中没有对应项（归中不存在保持段），所以这一项是掉头独有的具体数值。 */
#define GIMBAL_TURN_ENTER_ERR_DEG             1.0f

/* 掉头目标角斜坡步长 deg/ms。归中的"从任何位置都能稳稳到位"有一半来自这道斜坡：
 * 没有它，180° 是直接阶跃砸进环里；有了它，环始终只追一个移动的目标。 */
#define GIMBAL_TURN_YAW_RAMP_DEG_PER_MS       GIMBAL_INIT_YAW_RAMP_DEG_PER_MS

/* 掉头运动限幅。SPEED_LIMIT_ENABLE 0 = 外环只受 out_max 限幅 */
#define GIMBAL_TURN_SPEED_LIMIT_ENABLE        GIMBAL_INIT_SPEED_LIMIT_ENABLE
#define GIMBAL_TURN_YAW_MAX_RATE_DEG_S        GIMBAL_INIT_YAW_MAX_RATE_DEG_S
#define GIMBAL_TURN_YAW_DECEL_RAD_S2          GIMBAL_INIT_YAW_DECEL_RAD_S2

/* 掉头力矩限幅 */
#define GIMBAL_TURN_YAW_TORQUE_LIMIT_NM       GIMBAL_INIT_YAW_TORQUE_LIMIT_NM

/* 微分滤波，0 = 不过滤 */
#define GIMBAL_TURN_D_FILTER_ALPHA            GIMBAL_INIT_D_FILTER_ALPHA

/* 掉头位置外环。这两个值决定阻尼比，别名化之后不会再和归中分叉。 */
#define GIMBAL_TURN_YAW_OUTER_KP              GIMBAL_INIT_YAW_OUTER_KP
#define GIMBAL_TURN_YAW_OUTER_KI              GIMBAL_INIT_YAW_OUTER_KI
#define GIMBAL_TURN_YAW_OUTER_KD              GIMBAL_INIT_YAW_OUTER_KD
#define GIMBAL_TURN_YAW_OUTER_INTEGRAL_MAX    GIMBAL_INIT_YAW_OUTER_INTEGRAL_MAX
#define GIMBAL_TURN_YAW_OUTER_OUT_MAX         GIMBAL_INIT_YAW_OUTER_OUT_MAX

/* 掉头速度内环 */
#define GIMBAL_TURN_YAW_INNER_KP              GIMBAL_INIT_YAW_INNER_KP
#define GIMBAL_TURN_YAW_INNER_KI              GIMBAL_INIT_YAW_INNER_KI
#define GIMBAL_TURN_YAW_INNER_KD              GIMBAL_INIT_YAW_INNER_KD
#define GIMBAL_TURN_YAW_INNER_INTEGRAL_MAX    GIMBAL_INIT_YAW_INNER_INTEGRAL_MAX
#define GIMBAL_TURN_YAW_INNER_OUT_MAX         GIMBAL_INIT_YAW_INNER_OUT_MAX

#endif
