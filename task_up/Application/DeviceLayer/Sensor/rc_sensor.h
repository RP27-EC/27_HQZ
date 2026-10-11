/* rc_sensor.h - 遥控器设备抽象 */

#ifndef __RC_SENSOR_H
#define __RC_SENSOR_H
#include "rp_config.h"
#define RC_ONLINE	 (rc_dev.work_state==DEV_ONLINE)
#define RC_OFFLINE	 (rc_dev.work_state==DEV_OFFLINE)
/* 遥控通道编码 */

#define    RC_CH_VALUE_MIN       ((uint16_t)364 ) // 通道编码下限，364
#define    RC_CH_VALUE_OFFSET    ((uint16_t)1024) // 通道中位编码，1024
#define	   RC_CH_VALUE_MAX       ((uint16_t)1684) // 通道编码上限，1684
#define	   RC_CH_VALUE_SIDE_WIDTH	((RC_CH_VALUE_MAX-RC_CH_VALUE_MIN)/2) // 中位单侧量程，660

/* 拨杆编码 */

#define    RC_SW_UP              ((uint16_t)1) // 拨杆上位编码，1
#define    RC_SW_MID             ((uint16_t)3) // 拨杆中位编码，3
#define    RC_SW_DOWN            ((uint16_t)2) // 拨杆下位编码，2

/* 拨轮档位索引 */

#define    RC_TB_UP              ((uint16_t)0) // 拨轮上极限档，索引0
#define    RC_TB_MU              ((uint16_t)1) // 拨轮上中间档，索引1
#define    RC_TB_MD              ((uint16_t)3) // 拨轮下中间档，索引3
#define    RC_TB_DN              ((uint16_t)2) // 拨轮下极限档，索引2


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

/* 长按按接收帧计数 */
#define MOUSE_BTN_L_CNT_MAX     833 // 长按门限，接收帧数
#define MOUSE_BTN_R_CNT_MAX     500 // 长按门限，接收帧数
#define KEY_Q_CNT_MAX           500 // 长按门限，接收帧数
#define KEY_W_CNT_MAX           400 // 长按门限，接收帧数
#define KEY_E_CNT_MAX           500 // 长按门限，接收帧数
#define KEY_R_CNT_MAX           500 // 长按门限，接收帧数
#define KEY_A_CNT_MAX           400 // 长按门限，接收帧数
#define KEY_S_CNT_MAX           400 // 长按门限，接收帧数
#define KEY_D_CNT_MAX           400 // 长按门限，接收帧数
#define KEY_F_CNT_MAX           500 // 长按门限，接收帧数
#define KEY_G_CNT_MAX           500 // 长按门限，接收帧数
#define KEY_Z_CNT_MAX           500 // 长按门限，接收帧数
#define KEY_X_CNT_MAX           500 // 长按门限，接收帧数
#define KEY_C_CNT_MAX           500 // 长按门限，接收帧数
#define KEY_V_CNT_MAX           500 // 长按门限，接收帧数
#define KEY_B_CNT_MAX           500 // 长按门限，接收帧数
#define KEY_SHIFT_CNT_MAX       500 // 长按门限，接收帧数
#define KEY_CTRL_CNT_MAX        2500 // 长按门限，接收帧数

/* 鼠标移动平均滤波 */
#define REMOTE_SMOOTH_TIMES     10 // 鼠标滤波窗口，样本数

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
  release, // 释放保持，状态0
  release_to_press, // 按下边沿，状态1
  short_press, // 短按保持，状态2
  long_press, // 长按保持，状态3
  press_to_release, // 释放边沿，状态4
}key_board_status_e;

/* 按键信息 */
typedef struct key_board_info_struct {
  uint8_t			  value; // 按键电平，0/1
  key_board_status_e status; // 本次按键状态，0~4
  key_board_status_e last_status; // 上次按键状态，0~4
	
  int16_t cnt; // 连续按下帧数，0~cnt_max
  int16_t cnt_max; // 长按门限，接收帧数
}kb_key_t;

/* 拨杆信息 */
typedef struct
{
  uint8_t value_last; // 上次拨杆编码，1/2/3
  uint8_t value; // 当前拨杆编码，1/2/3
  uint8_t status; // 拨杆变化状态，0~3
}sw_state_t;

/* 拨轮信息 */
typedef struct
{
  int16_t value_last; // 上次拨轮原始量，±660
  int16_t value; // 当前拨轮原始量，±660
  uint8_t step[4]; // 四档翻转电平，每项0/1
  uint8_t step_rising_trigger[4]; // 四档变化事件，每项0/1
	
}dial_t;

/* 拨杆状态 */
typedef enum 
{
  keep_R, // 拨杆保持，状态0
  up_R, // 进入上位，状态1
  mid_R, // 进入中位，状态2
  down_R, // 进入下位，状态3
}remote_status_e;

typedef struct rc_sensor_info_struct {
/* 拨轮档位阈值 */
	int16_t    tw_step_value[4]; // 拨轮四档门限，原始量
	
/* 遥控器通道 */
	int16_t 	ch0; // 右杆水平原始量，±660
	int16_t 	ch1; // 右杆垂直原始量，±660
	int16_t 	ch2; // 左杆水平原始量，±660
	int16_t 	ch3; // 左杆垂直原始量，±660
	sw_state_t s1; // 右拨杆，原始编码1/2/3
	sw_state_t s2; // 左拨杆，原始编码1/2/3
	dial_t 			thumbwheel; // 拨轮原始量与四档事件
/* 鼠标 */
  int16_t                 mouse_vx; // 鼠标X帧增量，原始计数
  int16_t                 mouse_vy; // 鼠标Y帧增量，原始计数
  int16_t                 mouse_vz; // 鼠标Z帧增量，原始计数
  float                   mouse_x; // 滤波X帧增量，原始计数
  float                   mouse_y; // 滤波Y帧增量，原始计数
  float                   mouse_z; // 滤波Z帧增量，原始计数
  kb_key_t        mouse_btn_l; // 鼠标左键，状态0~4
  kb_key_t        mouse_btn_r; // 鼠标右键，状态0~4
  kb_key_t        Q; // Q键，状态0~4
  kb_key_t        W; // W键，状态0~4
  kb_key_t        E; // E键，状态0~4
  kb_key_t        R; // R键，状态0~4
  kb_key_t        A; // A键，状态0~4
  kb_key_t        S; // S键，状态0~4
  kb_key_t        D; // D键，状态0~4
  kb_key_t        F; // F键，状态0~4
  kb_key_t        G; // G键，状态0~4
  kb_key_t        Z; // Z键，状态0~4
  kb_key_t        X; // X键，状态0~4
  kb_key_t        C; // C键，状态0~4
  kb_key_t        V; // V键，状态0~4
  kb_key_t        B; // B键，状态0~4
  kb_key_t        Shift; // Shift键，状态0~4
  kb_key_t        Ctrl; // Ctrl键，状态0~4
	uint16_t								key_v; // 键盘位图，16位各表示0/1
	
	int16_t		offline_cnt; // 未收帧心跳累计，调用次数
	int16_t		offline_max_cnt; // 离线门限，心跳调用次数
} rc_data_t;

typedef struct rc_dev_struct {
	rc_data_t	*info; // 遥控数据对象指针
	drv_uart_t		  	*driver; // 遥控串口驱动指针
	void				(*init)(struct rc_dev_struct *self); // 设备初始化接口
	void				(*update)(struct rc_dev_struct *self, uint8_t *rxBuf); // 单帧解码接口，载荷为字节
	void				(*check)(struct rc_dev_struct *self); // 通道与事件校验接口
	void				(*heart_beat)(struct rc_dev_struct *self); // 离线计数接口，按调用累加
	dev_work_state_t	work_state; // 设备在线状态，见枚举
	dev_errno_t			errno; // 设备错误码，见枚举
	dev_id_t			id; // 遥控设备编号，见枚举
} rc_dev_t;

extern rc_data_t rc_data;
extern rc_dev_t 		rc_dev;
bool rc_channel_reset(void);
void rc_reset_data(rc_dev_t *rc);
	
#endif

