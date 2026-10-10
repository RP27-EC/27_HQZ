/* rc_sensor.h - 遥控器设备抽象 */

#ifndef __RC_SENSOR_H
#define __RC_SENSOR_H
#include "rp_config.h"
#define RC_ONLINE	 (rc_dev.work_state==DEV_ONLINE)
#define RC_OFFLINE	 (rc_dev.work_state==DEV_OFFLINE)
/* ----------------------- RC Channel Definition------------------------------*/

#define    RC_CH_VALUE_MIN       ((uint16_t)364 )
#define    RC_CH_VALUE_OFFSET    ((uint16_t)1024)
#define	   RC_CH_VALUE_MAX       ((uint16_t)1684)
#define	   RC_CH_VALUE_SIDE_WIDTH	((RC_CH_VALUE_MAX-RC_CH_VALUE_MIN)/2)

/* ----------------------- RC Switch Definition-------------------------------*/

#define    RC_SW_UP              ((uint16_t)1)
#define    RC_SW_MID             ((uint16_t)3)
#define    RC_SW_DOWN            ((uint16_t)2)

/* ----------------------- RC Thumbwheel Definition-------------------------------*/

#define    RC_TB_UP              ((uint16_t)0)
#define    RC_TB_MU              ((uint16_t)1)
#define    RC_TB_MD              ((uint16_t)3)
#define    RC_TB_DN              ((uint16_t)2)


/* ----------------------- PC Key Definition-------------------------------- */

#define    KEY_PRESSED_OFFSET_W        ((uint16_t)0x01<<0)
#define    KEY_PRESSED_OFFSET_S        ((uint16_t)0x01<<1)
#define    KEY_PRESSED_OFFSET_A        ((uint16_t)0x01<<2)
#define    KEY_PRESSED_OFFSET_D        ((uint16_t)0x01<<3)
#define    KEY_PRESSED_OFFSET_SHIFT    ((uint16_t)0x01<<4)
#define    KEY_PRESSED_OFFSET_CTRL     ((uint16_t)0x01<<5)
#define    KEY_PRESSED_OFFSET_Q        ((uint16_t)0x01<<6)
#define    KEY_PRESSED_OFFSET_E        ((uint16_t)0x01<<7)
#define    KEY_PRESSED_OFFSET_R        ((uint16_t)0x01<<8)
#define    KEY_PRESSED_OFFSET_F        ((uint16_t)0x01<<9)
#define    KEY_PRESSED_OFFSET_G        ((uint16_t)0x01<<10)
#define    KEY_PRESSED_OFFSET_Z        ((uint16_t)0x01<<11)
#define    KEY_PRESSED_OFFSET_X        ((uint16_t)0x01<<12)
#define    KEY_PRESSED_OFFSET_C        ((uint16_t)0x01<<13)
#define    KEY_PRESSED_OFFSET_V        ((uint16_t)0x01<<14)
#define    KEY_PRESSED_OFFSET_B        ((uint16_t)0x01<<15)

/* 按键长按判定时间(ms) */
#define MOUSE_BTN_L_CNT_MAX     833    // 左键连点判定时间(ms)
#define MOUSE_BTN_R_CNT_MAX     500    // 右键长按判定(ms)
#define KEY_Q_CNT_MAX           500    // Q 长按判定(ms)
#define KEY_W_CNT_MAX           400    // W 长按判定(ms)
#define KEY_E_CNT_MAX           500    // E 长按判定(ms)
#define KEY_R_CNT_MAX           500    // R 长按判定(ms)
#define KEY_A_CNT_MAX           400    // A 长按判定(ms)
#define KEY_S_CNT_MAX           400    // S 长按判定(ms)
#define KEY_D_CNT_MAX           400    // D 长按判定(ms)
#define KEY_F_CNT_MAX           500    // F 长按判定(ms)
#define KEY_G_CNT_MAX           500    // G 长按判定(ms)
#define KEY_Z_CNT_MAX           500    // Z 长按判定(ms)
#define KEY_X_CNT_MAX           500    // X 长按判定(ms)
#define KEY_C_CNT_MAX           500    // C 长按判定(ms)
#define KEY_V_CNT_MAX           500    // V 长按判定(ms)
#define KEY_B_CNT_MAX           500    // B 长按判定(ms)
#define KEY_SHIFT_CNT_MAX       500    // Shift 长按判定(ms)
#define KEY_CTRL_CNT_MAX        2500    // Ctrl 长按判定(ms)

/* 鼠标移动平均滤波 */
#define REMOTE_SMOOTH_TIMES     10    // 滤波窗口长度

/* ----------------------- Function Definition-------------------------------- */
/* 遥控器通道取值宏 */
#define		RC_SW1_VALUE				(rc_data.s1)
#define		RC_SW2_VALUE				(rc_data.s2)
#define		RC_LEFT_CH_LR_VALUE			(rc_data.ch2)
#define		RC_LEFT_CH_UD_VALUE			(rc_data.ch3)
#define		RC_RIGH_CH_LR_VALUE			(rc_data.ch0)
#define		RC_RIGH_CH_UD_VALUE			(rc_data.ch1)
#define		RC_THUMB_WHEEL_VALUE		(rc_data.thumbwheel)

/* 读拨杆状态 */
#define    IF_RC_SW1_UP      (rc_data.s1.value == RC_SW_UP)
#define    IF_RC_SW1_MID     (rc_data.s1.value == RC_SW_MID)
#define    IF_RC_SW1_DOWN    (rc_data.s1.value == RC_SW_DOWN)
#define    IF_RC_SW2_UP      (rc_data.s2.value == RC_SW_UP)
#define    IF_RC_SW2_MID     (rc_data.s2.value == RC_SW_MID)
#define    IF_RC_SW2_DOWN    (rc_data.s2.value == RC_SW_DOWN)

/* 鼠标移动速度 */
#define    MOUSE_X_MOVE_SPEED    (rc_data.mouse_vx)
#define    MOUSE_Y_MOVE_SPEED    (rc_data.mouse_vy)
#define    MOUSE_Z_MOVE_SPEED    (rc_data.mouse_vz)

/* 鼠标按键: 1=按下, 0=松开 */
#define    MOUSE_PRESSED_LEFT    (rc_data.mouse_btn_l==1)
#define    MOUSE_PRESSED_RIGH    (rc_data.mouse_btn_r==1)


/* 键盘按键: 对应位为 1 表示按下 */
#define    KEY_PRESSED         (  rc_data.key_v  )
#define    KEY_PRESSED_W       ( (rc_data.key_v & KEY_PRESSED_OFFSET_W)    != 0 )
#define    KEY_PRESSED_S       ( (rc_data.key_v & KEY_PRESSED_OFFSET_S)    != 0 )
#define    KEY_PRESSED_A       ( (rc_data.key_v & KEY_PRESSED_OFFSET_A)    != 0 )
#define    KEY_PRESSED_D       ( (rc_data.key_v & KEY_PRESSED_OFFSET_D)    != 0 )
#define    KEY_PRESSED_Q       ( (rc_data.key_v & KEY_PRESSED_OFFSET_Q)    != 0 )
#define    KEY_PRESSED_E       ( (rc_data.key_v & KEY_PRESSED_OFFSET_E)    != 0 )
#define    KEY_PRESSED_G       ( (rc_data.key_v & KEY_PRESSED_OFFSET_G)    != 0 )
#define    KEY_PRESSED_X       ( (rc_data.key_v & KEY_PRESSED_OFFSET_X)    != 0 )
#define    KEY_PRESSED_Z       ( (rc_data.key_v & KEY_PRESSED_OFFSET_Z)    != 0 )
#define    KEY_PRESSED_C       ( (rc_data.key_v & KEY_PRESSED_OFFSET_C)    != 0 )
#define    KEY_PRESSED_B       ( (rc_data.key_v & KEY_PRESSED_OFFSET_B)    != 0 )
#define    KEY_PRESSED_V       ( (rc_data.key_v & KEY_PRESSED_OFFSET_V)    != 0 )
#define    KEY_PRESSED_F       ( (rc_data.key_v & KEY_PRESSED_OFFSET_F)    != 0 )
#define    KEY_PRESSED_R       ( (rc_data.key_v & KEY_PRESSED_OFFSET_R)    != 0 )
#define    KEY_PRESSED_CTRL    ( (rc_data.key_v & KEY_PRESSED_OFFSET_CTRL) != 0 )
#define    KEY_PRESSED_SHIFT   ( (rc_data.key_v & KEY_PRESSED_OFFSET_SHIFT) != 0 )
/* 按键状态 */
typedef enum
{
  release,  // 松开
  release_to_press,  // 按下沿
  short_press,  // 短按
  long_press,  // 长按
  press_to_release,  // 松开
}key_board_status_e;

/* 按键信息 */
typedef struct key_board_info_struct {
  uint8_t			  value;  // 当前值
  key_board_status_e status;  // 状态
  key_board_status_e last_status;       // 上次状态
	
  int16_t cnt;  // 按下计数
  int16_t cnt_max;  // 长按阈值
}kb_key_t;

/* 拨杆信息 */
typedef struct
{
  uint8_t value_last;  // 上次值
  uint8_t value;  // 当前值
  uint8_t status;  // 状态
}sw_state_t;

/* 拨轮信息 */
typedef struct
{
  int16_t value_last;  // 上次值
  int16_t value;  // 当前值
  uint8_t step[4];  // 四个档位翻转标志
  uint8_t step_rising_trigger[4];  // 档位跳变标志
	
}dial_t;

/* 拨杆状态 */
typedef enum 
{
  keep_R,  // 保持
  up_R,  // 上拨
  mid_R,  // 中位
  down_R,  // 下拨
}remote_status_e;

typedef struct rc_sensor_info_struct {
/* 拨轮档位阈值 */
	int16_t    tw_step_value[4];
	
/* 遥控器通道 */
	int16_t 	ch0;
	int16_t 	ch1;
	int16_t 	ch2;
	int16_t 	ch3;
	sw_state_t s1;
	sw_state_t s2;
	dial_t 			thumbwheel;  // 拨轮
/* 鼠标 */
  int16_t                 mouse_vx;  // 鼠标 x 速度
  int16_t                 mouse_vy;  // 鼠标 y 速度
  int16_t                 mouse_vz;  // 鼠标 z 速度
  float                   mouse_x;  // 滤波后 x 速度
  float                   mouse_y;  // 滤波后 y 速度
  float                   mouse_z;  // 滤波后 z 速度
  kb_key_t        mouse_btn_l;  // 鼠标左键
  kb_key_t        mouse_btn_r;  // 鼠标右键
  kb_key_t        Q;  // Q 键
  kb_key_t        W;  // W 键
  kb_key_t        E;  // E 键
  kb_key_t        R;  // R 键
  kb_key_t        A;  // A 键
  kb_key_t        S;  // S 键
  kb_key_t        D;  // D 键
  kb_key_t        F;  // F 键
  kb_key_t        G;  // G 键
  kb_key_t        Z;  // Z 键
  kb_key_t        X;  // X 键
  kb_key_t        C;  // C 键
  kb_key_t        V;  // V 键
  kb_key_t        B;  // B 键
  kb_key_t        Shift;  // Shift 键
  kb_key_t        Ctrl;  // Ctrl 键
	uint16_t								key_v;
	
	int16_t		offline_cnt;
	int16_t		offline_max_cnt;
} rc_data_t;

typedef struct rc_dev_struct {
	rc_data_t	*info;
	drv_uart_t		  	*driver;
	void				(*init)(struct rc_dev_struct *self);
	void				(*update)(struct rc_dev_struct *self, uint8_t *rxBuf);
	void				(*check)(struct rc_dev_struct *self);	
	void				(*heart_beat)(struct rc_dev_struct *self);
	dev_work_state_t	work_state;
	dev_errno_t			errno;
	dev_id_t			id;
} rc_dev_t;

extern rc_data_t rc_data;
extern rc_dev_t 		rc_dev;
bool rc_channel_reset(void);
void rc_reset_data(rc_dev_t *rc);
	
#endif

