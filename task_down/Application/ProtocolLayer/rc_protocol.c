#include "rc_protocol.h"
#include "rp_math.h"
#include "rc_sensor.h"
void keyboard_cnt_max_set(rc_dev_t *rc_sen);
void keyboard_status_update(kb_key_t *key);

extern uint32_t micros(void);
uint32_t tt1, tt2, ttp1; /* 遥控帧间隔计时 */

/* 初始化遥控设备状态，默认离线 */
void rc_init(rc_dev_t *rc_sen)
{
	// 初始化为离线状态
	rc_sen->info->offline_cnt = rc_sen->info->offline_max_cnt + 1; /* 强制离线 */
	rc_sen->work_state = DEV_OFFLINE;

	rc_reset_data(rc_sen);
	keyboard_cnt_max_set(rc_sen);

	if(rc_sen->id == DEV_ID_RC)
		rc_sen->errno = NONE_ERR;
	else
		rc_sen->errno = DEV_ID_ERR;
}

/* 设置按键长按阈值 */
/* 设置各按键的长按判定阈值 */
void keyboard_cnt_max_set(rc_dev_t *rc_sen)
{
	rc_data_t *info = rc_sen->info;

  info->mouse_btn_l.cnt_max = MOUSE_BTN_L_CNT_MAX;
  info->mouse_btn_r.cnt_max = MOUSE_BTN_R_CNT_MAX;
  info->Q.cnt_max = KEY_Q_CNT_MAX;
  info->W.cnt_max = KEY_W_CNT_MAX;
  info->E.cnt_max = KEY_E_CNT_MAX;
  info->R.cnt_max = KEY_R_CNT_MAX;
  info->A.cnt_max = KEY_A_CNT_MAX;
  info->S.cnt_max = KEY_S_CNT_MAX;
  info->D.cnt_max = KEY_D_CNT_MAX;
  info->F.cnt_max = KEY_F_CNT_MAX;
  info->G.cnt_max = KEY_G_CNT_MAX;
  info->Z.cnt_max = KEY_Z_CNT_MAX;
  info->X.cnt_max = KEY_X_CNT_MAX;
  info->C.cnt_max = KEY_C_CNT_MAX;
  info->V.cnt_max = KEY_V_CNT_MAX;
  info->B.cnt_max = KEY_B_CNT_MAX;
  info->Shift.cnt_max = KEY_SHIFT_CNT_MAX;
  info->Ctrl.cnt_max = KEY_CTRL_CNT_MAX;
}

/* 鼠标速度滤波 */
/* 对鼠标速度做滑动平均，降低抖动 */
void rc_interrupt_update(rc_dev_t *rc_sen)
{
	/* 鼠标速度均值滤波 */
	static int16_t mouse_x[REMOTE_SMOOTH_TIMES], mouse_y[REMOTE_SMOOTH_TIMES]; /* 历史缓存 */
	static int16_t index = 0; /* 环形索引 */
	if(index == REMOTE_SMOOTH_TIMES)
	{
		index = 0;
	}
	rc_sen->info->mouse_x -= (float)mouse_x[index] / (float)REMOTE_SMOOTH_TIMES; /* 减去旧值 */
	rc_sen->info->mouse_y -= (float)mouse_y[index] / (float)REMOTE_SMOOTH_TIMES; /* 减去旧值 */
	mouse_x[index] = rc_sen->info->mouse_vx;
	mouse_y[index] = rc_sen->info->mouse_vy;
	rc_sen->info->mouse_x += (float)mouse_x[index] / (float)REMOTE_SMOOTH_TIMES; /* 加入新值 */
	rc_sen->info->mouse_y += (float)mouse_y[index] / (float)REMOTE_SMOOTH_TIMES; /* 加入新值 */

	index++;

}
/* 解析遥控器帧 */
/* 解析下板遥控器原始帧 */
void rc_update(rc_dev_t *rc_sen, uint8_t *rxBuf)
{

	rc_data_t *rc_info = rc_sen->info;
	rc_info->offline_cnt=0; /* 收到帧则在线 */
	/* 遥控器 */
	rc_info->ch0 = (rxBuf[0] | rxBuf[1] << 8) & 0x07FF; /* 右横 */
	rc_info->ch0 -= 1024;
	rc_info->ch1 = (rxBuf[1] >> 3 | rxBuf[2] << 5) & 0x07FF; /* 右纵 */
	rc_info->ch1 -= 1024;
	rc_info->ch2 = (rxBuf[2] >> 6 | rxBuf[3] << 2 | rxBuf[4] << 10) & 0x07FF; /* 左横 */
	rc_info->ch2 -= 1024;
	rc_info->ch3 = (rxBuf[4] >> 1 | rxBuf[5] << 7) & 0x07FF; /* 左纵 */
	rc_info->ch3 -= 1024;

	rc_info->thumbwheel.value = ((int16_t)rxBuf[16] | ((int16_t)rxBuf[17] << 8)) & 0x07ff; /* 波轮 */
	rc_info->thumbwheel.value -= 1024;

	if(abs(rc_info->thumbwheel.value)>660)
	{
		rc_info->thumbwheel.value=0;
	}

	rc_info->s1.value = ((rxBuf[5] >> 4) & 0x000C) >> 2; /* S1 档位 */
	rc_info->s2.value = (rxBuf[5] >> 4) & 0x0003; /* S2 档位 */
	/*遥控器限位置零*/
	if(rc_dev.info->ch3== -660)
	{
		rc_dev.info->ch3=0;
	}

	/* 键鼠 */
	rc_info->mouse_vx = rxBuf[6]  | (rxBuf[7 ] << 8); /* 鼠标 X */
	rc_info->mouse_vy = rxBuf[8]  | (rxBuf[9 ] << 8); /* 鼠标 Y */
	rc_info->mouse_vz = rxBuf[10] | (rxBuf[11] << 8); /* 鼠标滚轮 */
  rc_info->mouse_btn_l.value = rxBuf[12] & 0x01; /* 左键 */
  rc_info->mouse_btn_r.value = rxBuf[13] & 0x01; /* 右键 */
  rc_info->key_v   =  rxBuf[14] | (rxBuf[15] << 8); /* 键盘位图 */
  rc_info->update_seq++;

  rc_info->W.value = 	KEY_PRESSED_W;
  rc_info->S.value =    KEY_PRESSED_S;
  rc_info->A.value = 	KEY_PRESSED_A;
  rc_info->D.value = 	KEY_PRESSED_D;
  rc_info->Shift.value = 	KEY_PRESSED_SHIFT;
  rc_info->Ctrl.value  = 	KEY_PRESSED_CTRL;
  rc_info->Q.value = 	KEY_PRESSED_Q;
  rc_info->E.value = 	KEY_PRESSED_E;
  rc_info->R.value = 	KEY_PRESSED_R;
  rc_info->F.value = 	KEY_PRESSED_F;
  rc_info->G.value = 	KEY_PRESSED_G;
  rc_info->Z.value = 	KEY_PRESSED_Z;
  rc_info->X.value = 	KEY_PRESSED_X;
  rc_info->C.value = 	KEY_PRESSED_C;
  rc_info->V.value = 	KEY_PRESSED_V;
  rc_info->B.value = 	KEY_PRESSED_B;

	rc_info->offline_cnt = 0;
	tt1 = tt2;
	tt2 = micros();
	ttp1 = tt2 - tt1;

}

/* 更新键盘状态 */
/* 刷新所有键鼠按键的边沿和长按状态 */
void keyboard_update(rc_data_t	*info)
{
  keyboard_status_update(&info->mouse_btn_l);
  keyboard_status_update(&info->mouse_btn_r);
  keyboard_status_update(&info->Q);
  keyboard_status_update(&info->W);
  keyboard_status_update(&info->E);
  keyboard_status_update(&info->R);
  keyboard_status_update(&info->A);
  keyboard_status_update(&info->S);
  keyboard_status_update(&info->D);
  keyboard_status_update(&info->F);
  keyboard_status_update(&info->G);
  keyboard_status_update(&info->Z);
  keyboard_status_update(&info->X);
  keyboard_status_update(&info->C);
  keyboard_status_update(&info->V);
  keyboard_status_update(&info->B);
  keyboard_status_update(&info->Shift);
  keyboard_status_update(&info->Ctrl);
}

/* 更新单个按键状态 */
/* 单键状态机：释放/按下/短按/长按 */
void keyboard_status_update(kb_key_t *key)
{
	key->last_status = key->status; /* 保存上拍状态 */

    switch(key->value)
    {
        case 0:
        {
            if(key->cnt != 0)
            {
				key->status = press_to_release; /* 释放升沿 */
				key->cnt = 0;
            }
            else
            {
				key->status = release; /* 持续释放 */
                key->cnt = 0;
            }
            break;
        }
        case 1:
        {
			key->cnt++; /* 按下计数 */
            if(key->cnt == 1)
            {
				key->status = release_to_press; /* 按下升沿 */
            }
            else if(key->cnt >= key->cnt_max)
            {
				key->status = long_press; /* 长按 */
				key->cnt = key->cnt_max;
            }
            else
            {
				key->status = short_press; /* 短按保持 */
            }
        }
    }
}


static uint8_t init_cnt = 0; /* 首帧同步标志 */

/* USART5 收帧入口，刷新遥控状态并检测离线 */
/* USART5 数据解析(遥控器) */
void USART5_rxDataHandler(uint8_t *rxBuf)
{
	// 更新遥控数据
	if(init_cnt != 0) /* 跳过首帧 */
	rc_dev.info->offline_cnt = 0;
	else
	init_cnt ++;
	rc_dev.update(&rc_dev, rxBuf);
	rc_dev.check(&rc_dev);


}



