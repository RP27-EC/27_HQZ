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


#define MOUSE_BTN_L_CNT_MAX     500  // MOUSEBTNL计数最大
#define MOUSE_BTN_R_CNT_MAX     500  // MOUSEBTN半径计数最大
#define KEY_Q_CNT_MAX           500  // KEYQ计数最大
#define KEY_W_CNT_MAX           400  // KEYW计数最大
#define KEY_E_CNT_MAX           500  // KEYE计数最大
#define KEY_R_CNT_MAX           500  // KEY半径计数最大
#define KEY_A_CNT_MAX           400  // KEYA计数最大
#define KEY_S_CNT_MAX           400  // KEYS计数最大
#define KEY_D_CNT_MAX           400  // KEYD计数最大
#define KEY_F_CNT_MAX           500  // KEYF计数最大
#define KEY_G_CNT_MAX           500  // KEY绿计数最大
#define KEY_Z_CNT_MAX           500  // KEYz计数最大
#define KEY_X_CNT_MAX           500  // KEYx计数最大
#define KEY_C_CNT_MAX           500  // KEYc计数最大
#define KEY_V_CNT_MAX           500  // KEYV计数最大
#define KEY_B_CNT_MAX           500  // KEY蓝计数最大
#define KEY_SHIFT_CNT_MAX       500  // KEYSHIFT计数最大
#define KEY_CTRL_CNT_MAX        500  // KEYCTRL计数最大


#define REMOTE_SMOOTH_TIMES     10

/* ----------------------- Function Definition-------------------------------- */

#define		RC_SW1_VALUE				(rc_data.s1)
#define		RC_SW2_VALUE				(rc_data.s2)
#define		RC_LEFT_CH_LR_VALUE			(rc_data.ch2)
#define		RC_LEFT_CH_UD_VALUE			(rc_data.ch3)
#define		RC_RIGH_CH_LR_VALUE			(rc_data.ch0)
#define		RC_RIGH_CH_UD_VALUE			(rc_data.ch1)
#define		RC_THUMB_WHEEL_VALUE		(rc_data.thumbwheel)


#define    IF_RC_SW1_UP      (rc_data.s1.value == RC_SW_UP)
#define    IF_RC_SW1_MID     (rc_data.s1.value == RC_SW_MID)
#define    IF_RC_SW1_DOWN    (rc_data.s1.value == RC_SW_DOWN)
#define    IF_RC_SW2_UP      (rc_data.s2.value == RC_SW_UP)
#define    IF_RC_SW2_MID     (rc_data.s2.value == RC_SW_MID)
#define    IF_RC_SW2_DOWN    (rc_data.s2.value == RC_SW_DOWN)


#define    MOUSE_X_MOVE_SPEED    (rc_data.mouse_vx)
#define    MOUSE_Y_MOVE_SPEED    (rc_data.mouse_vy)
#define    MOUSE_Z_MOVE_SPEED    (rc_data.mouse_vz)



#define    MOUSE_PRESSED_LEFT    (rc_data.mouse_btn_l==1)
#define    MOUSE_PRESSED_RIGH    (rc_data.mouse_btn_r==1)




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

typedef enum
{
  release,
  release_to_press,
  short_press,
  long_press,
  press_to_release,
}key_board_status_e;


typedef struct key_board_info_struct {
  uint8_t			  value;  // 值
  key_board_status_e status;  // 状态
  key_board_status_e last_status;  // last状态
	
  int16_t cnt;  // 计数
  int16_t cnt_max;  // 计数最大
}kb_key_t;



typedef enum 
{
  keep_R,  // keep半径
  up_R,  // up半径
  mid_R,  // 中半径
  down_R,  // down半径
}remote_status_e;



typedef struct
{
  uint8_t value_last;  // 值last
  uint8_t value;  // 值
  remote_status_e status;  // 状态
}sw_state_t;


typedef struct
{
  int16_t value_last;  // 值last
  int16_t value;  // 值
  uint8_t step[4];
  uint8_t step_rising_trigger[4];
	
}dial_t;



typedef struct rc_sensor_info_struct {

	int16_t    tw_step_value[4];
	

	int16_t 	ch0;
	int16_t 	ch1;
	int16_t 	ch2;
	int16_t 	ch3;
	sw_state_t s1;
	sw_state_t s2;
	dial_t 			thumbwheel;

  int16_t                 mouse_vx;
  int16_t                 mouse_vy;
  int16_t                 mouse_vz;
  float                   mouse_x;  // mousex
  float                   mouse_y;  // mousey
  float                   mouse_z;  // mousez
  kb_key_t        mouse_btn_l;
  kb_key_t        mouse_btn_r;  // mousebtn半径
  kb_key_t        Q;
  kb_key_t        W;
  kb_key_t        E;
  kb_key_t        R;  // 半径
  kb_key_t        A;
  kb_key_t        S;
  kb_key_t        D;
  kb_key_t        F;
  kb_key_t        G;  // 绿
  kb_key_t        Z;  // z
  kb_key_t        X;  // x
  kb_key_t        C;  // c
  kb_key_t        V;
  kb_key_t        B;  // 蓝
  kb_key_t        Shift;
  kb_key_t        Ctrl;
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

