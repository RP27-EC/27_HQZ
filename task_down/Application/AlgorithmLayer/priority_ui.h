/* priority_ui.h - UI 浼樺厛绾ц皟搴� */

#ifndef _PRIORITY_UI_H
#define _PRIORITY_UI_H
#include "main.h"
#include "stdbool.h"
#include "string.h"
#include "stdlib.h"
#include "ui_protocol.h"
/* enum */
typedef enum {
  LOW_PRIORITY = 0,  // 浣庝紭鍏堢骇
  MID_PRIORITY,  // 涓紭鍏堢骇
  HIGH_PRIORITY,  // 楂樹紭鍏堢骇
}ui_priority_e;

/* enum */
typedef enum
{
  SEND_CHAR_MODE = 0,
  SEND_GRAPHIC_MODE,
} ui_send_mode_e;

/* enum */
typedef enum {
    MESSAGE_SENT = 0,
    MESSAGE_NOT_SENT,
} ui_sent_state_e;

typedef enum {
  LINE = 0,  // 鐩寸嚎
  RECTANGEL,
  CIRCLE,  // 鍦�
  ELLIPSE,  // 妞渾
  ARC,  // 寮�
  FLOAT,
  INT,
  CHAR,  // 瀛楃
} ui_type_e;

/* enum */
typedef enum 
{
  UI_ERROR    ,
  UI_OK       ,
  UI_BUSY     ,
} ui_status_e;
/* __packed */
typedef __packed struct  {

  ui_priority_e priority;  // 浼樺厛绾�
  ui_type_e ui_type;  // ui绫诲瀷
  char name[3];  // 鍚嶇О




  operate_tpye_e operate_type;  // operate绫诲瀷
  uint8_t layer;  // 灞�
  graphic_color_e color;  // 棰滆壊
  uint16_t width;  // 瀹藉害
  uint16_t start_x;  // 璧峰x
  uint16_t start_y;  // 璧峰y
  uint16_t end_x;  // 缁撴潫x
  uint16_t end_y;  // 缁撴潫y

  uint16_t radius;  // 鍗婂緞
  uint16_t start_angel;  // 璧峰angel
  uint16_t end_angel;  // 缁撴潫angel
  uint16_t size;  // 瀛楀彿
  float float_num;  // float鏁伴噺
  uint16_t decimal;
  int32_t int_num;  // int鏁伴噺
  char text[30];  // 鏂囨湰
} ui_config_t;

/* __packed */
typedef __packed struct  {
  ui_sent_state_e sent_state;
  uint32_t updateTick;  // update鑺傛媿
  uint32_t  priority_value;  // 浼樺厛绾у€�
  ui_config_t ui_config;
} ui_info_t;

/* Node_u */
typedef struct Node_u 
{
  ui_info_t *ui;
  struct Node_u *next;
} Node_u;

/*test*/
ui_status_e Init_Ui_List(ui_info_t *dynamic_ui_info, uint8_t dynamic_ui_num, ui_info_t *const_ui_info, uint8_t const_ui_num);
void Ui_Send(void);
ui_status_e Enqueue_Ui_For_Sending(ui_info_t *ui_info);
#endif



