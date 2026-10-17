/* board_remote_config.h - 上下板遥控输入源开关 */

#ifndef __BOARD_REMOTE_CONFIG_H
#define __BOARD_REMOTE_CONFIG_H

/* Select the upper-board manual input source. */
#define GIMBAL_DOWN_RC_ENABLE    1u
#define GIMBAL_LOCAL_RC_ENABLE   0u

/* 板间反馈降频，不改变控制周期 */
#define BOARD_FEEDBACK_PERIOD_MS 5u // 每类反馈周期，ms，至少2

#define BOARD_RESET_TIMEOUT_MS 500u // 阶段确认上限，ms
#define BOARD_RESET_HOLD_MS 1000u // 组合键触发时长，ms
#define BOARD_RESET_REBOOT_DELAY_MS 50u // 提交后复位延时，ms

#endif

