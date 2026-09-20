/**
 * @file ui_protocol.c
 * @brief ui协议层
 * @version 0.1
 */

#include "ui_protocol.h"
#include "crc.h"
#include "string.h"
#include "stdbool.h"
#include "stdio.h"
#include "judge_protocol.h"
#include "judge.h"

/* 配置区 begin */
#define UI_huart huart1 // 串口接口
extern UART_HandleTypeDef UI_huart;

client_info_t client_info =
{
    .robot_id = 1,
    .client_id = 0x0101,
};

__attribute__((section(".AXI_SRAM"))) uint8_t client_tx_buf[128];

/**
 * @brief 更新红蓝方机器人信息，在裁判系统接受中断中调用
 */
void client_info_update(void)
{
    switch (judge.info->robot_status.robot_id)
    {
    case 1: // 红方英雄
        client_info.robot_id = 1;
        client_info.client_id = 0x0101;
        break;
    case 2: // 红方工程
        client_info.robot_id = 2;
        client_info.client_id = 0x0102;
        break;
    case 3: // 红方步兵
        client_info.robot_id = 3;
        client_info.client_id = 0x0103;
        break;
    case 4: // 红方步兵
        client_info.robot_id = 4;
        client_info.client_id = 0x0104;
        break;
    case 5: // 红方步兵
        client_info.robot_id = 5;
        client_info.client_id = 0x0105;
        break;
    case 101: // 蓝方英雄
        client_info.robot_id = 101;
        client_info.client_id = 0x0165;
        break;
    case 102: // 蓝方工程
        client_info.robot_id = 102;
        client_info.client_id = 0x0166;
        break;
    case 103: // 蓝方步兵
        client_info.robot_id = 103;
        client_info.client_id = 0x0167;
        break;
    case 104: // 蓝方步兵
        client_info.robot_id = 104;
        client_info.client_id = 0x0168;
        break;
    case 105: // 蓝方步兵
        client_info.robot_id = 105;
        client_info.client_id = 0x0169;
        break;
    default: // 收不到裁判系统数据
        client_info.robot_id = 1;
        client_info.client_id = 0x0101;
        break;
    }
}
/* 配置区 end */

/******************************获取图像数据帧begin******************************/
/**
  * @brief  获取直线数据帧
  * @param
  * @retval 图形数据结构体
  */
graphic_data_struct_t draw_line(char *name, uint8_t operate_tpye, uint8_t layer, uint8_t color, uint16_t width, uint16_t start_x, uint16_t start_y, uint16_t end_x, uint16_t end_y)
{
    graphic_data_struct_t data;
    memcpy(data.graphic_name, name, 3);
    data.operate_tpye = operate_tpye;
    data.graphic_tpye = 0;
    data.layer = layer;
    data.color = color;
    data.start_angle = 0;
    data.end_angle = 0;
    data.width = width;
    data.start_x = start_x;
    data.start_y = start_y;
    data.radius = 0;
    data.end_x = end_x;
    data.end_y = end_y;
    return data;
}

/**
  * @brief  获取矩形数据帧
  * @param
  * @retval 图形数据结构体
  */
graphic_data_struct_t draw_rectangle(char *name, uint8_t operate_tpye, uint8_t layer, uint8_t color, uint16_t width, uint16_t start_x, uint16_t start_y, uint16_t end_x, uint16_t end_y)
{
    graphic_data_struct_t data;
    memcpy(data.graphic_name, name, 3);
    data.operate_tpye = operate_tpye;
    data.graphic_tpye = 1;
    data.layer = layer;
    data.color = color;
    data.start_angle = 0;
    data.end_angle = 0;
    data.width = width;
    data.start_x = start_x;
    data.start_y = start_y;
    data.radius = 0;
    data.end_x = end_x;
    data.end_y = end_y;
    return data;
}

/**
  * @brief  获取整圆数据帧
  * @param
  * @retval 图形数据结构体
  */
graphic_data_struct_t draw_circle(char *name, uint8_t operate_tpye, uint8_t layer, uint8_t color, uint16_t width, uint16_t ciclemid_x, uint16_t ciclemid_y, uint16_t radius)
{
    graphic_data_struct_t data;
    memcpy(data.graphic_name, name, 3);
    data.operate_tpye = operate_tpye;
    data.graphic_tpye = 2;
    data.layer = layer;
    data.color = color;
    data.start_angle = 0;
    data.end_angle = 0;
    data.width = width;
    data.start_x = ciclemid_x;
    data.start_y = ciclemid_y;
    data.radius = radius;
    data.end_x = 0;
    data.end_y = 0;
    return data;
}

/**
  * @brief  获取椭圆数据帧
  * @param
  * @retval 图形数据结构体
  */
graphic_data_struct_t draw_ellipse(char *name, uint8_t operate_tpye, uint8_t layer, uint8_t color, uint16_t width, uint16_t start_x, uint16_t start_y, uint16_t end_x, uint16_t end_y)
{
    graphic_data_struct_t data;
    memcpy(data.graphic_name, name, 3);
    data.operate_tpye = operate_tpye;
    data.graphic_tpye = 3;
    data.layer = layer;
    data.color = color;
    data.start_angle = 0;
    data.end_angle = 0;
    data.width = width;
    data.start_x = start_x;
    data.start_y = start_y;
    data.radius = 0;
    data.end_x = end_x;
    data.end_y = end_y;
    return data;
}

/**
  * @brief  获取圆弧数据帧
  * @param
  * @retval 图形数据结构体
  */
graphic_data_struct_t draw_arc(char *name, uint8_t operate_tpye, uint8_t layer, uint8_t color, uint16_t start_angle, uint16_t end_angle, uint16_t width, uint16_t circlemin_x, uint16_t circlemin_y, uint16_t end_x, uint16_t end_y)
{
    graphic_data_struct_t data;
    memcpy(data.graphic_name, name, 3);
    data.operate_tpye = operate_tpye;
    data.graphic_tpye = 4;
    data.layer = layer;
    data.color = color;
    data.start_angle = start_angle;
    data.end_angle = end_angle;
    data.width = width;
    data.start_x = circlemin_x;
    data.start_y = circlemin_y;
    data.radius = 0;
    data.end_x = end_x;
    data.end_y = end_y;
    return data;
}

/**
  * @brief  获取浮点数数据帧
  * @param
  * @retval 图形数据结构体
  */
graphic_data_struct_t draw_float(char *name, uint8_t operate_tpye, uint8_t layer, uint8_t color, uint16_t size, uint16_t decimal, uint16_t width, uint16_t start_x, uint16_t start_y, int32_t num)
{
    graphic_data_struct_t data;
    memcpy(data.graphic_name, name, 3);
    data.operate_tpye = operate_tpye;
    data.graphic_tpye = 5;
    data.layer = layer;
    data.color = color;
    data.start_angle = size;
    data.end_angle = decimal;
    data.width = width;
    data.start_x = start_x;
    data.start_y = start_y;
    data.radius = num;
    data.end_x = num >> 10;
    data.end_y = num >> 21;
    return data;
}

/**
  * @brief  获取整型数数据帧
  * @param
  * @retval 图形数据结构体
  */
graphic_data_struct_t draw_int(char *name, uint8_t operate_tpye, uint8_t layer, uint8_t color, uint16_t size, uint16_t width, uint16_t start_x, uint16_t start_y, int32_t num)
{
    graphic_data_struct_t data;
    memcpy(data.graphic_name, name, 3);
    data.operate_tpye = operate_tpye;
    data.graphic_tpye = 6;
    data.layer = layer;
    data.color = color;
    data.start_angle = size;
    data.end_angle = 0;
    data.width = width;
    data.start_x = start_x;
    data.start_y = start_y;
    data.radius = num;
    data.end_x = num >> 10;
    data.end_y = num >> 21;
    return data;
}

/**
  * @brief  获取字符数据帧
  * @param  operate_tpye 图形操作
    * @param  layer 图层数，0~9
    * @param  color 颜色
    * @param  size 字体大小
    * @param  length 字符长度
    * @param  width 线条宽度
    * @param  start_x 起点 x 坐标
    * @param  start_y 起点 y 坐标
  * @retval 图形数据结构体
  */
graphic_data_struct_t draw_char(char *name, uint8_t operate_tpye, uint8_t layer, uint8_t color, uint16_t size, uint16_t length, uint16_t width, uint16_t start_x, uint16_t start_y)
{
    graphic_data_struct_t data;
    memcpy(data.graphic_name, name, 3);
    data.operate_tpye = operate_tpye;
    data.graphic_tpye = 7;
    data.layer = layer;
    data.color = color;
    data.start_angle = size;
    data.end_angle = length;
    data.width = width;
    data.start_x = start_x;
    data.start_y = start_y;
    data.radius = 0;
    data.end_x = 0;
    data.end_y = 0;
    return data;
}
/******************************获取图像数据帧end******************************/

/******************************发送帧数据begin******************************/
/**
    * @brief  发送绘制一个图形帧数据
  * @param
  * @retval
  */
uint8_t client_send_single_graphic(ext_client_custom_graphic_single_t data)
{
    frame_t frame;
    ext_student_interactive_header_data_t data_header;
    frame.frame_header.SOF = 0xA5;
    frame.frame_header.data_length = LEN_ID_draw_one_graphic;
    frame.frame_header.seq = 0;
    memcpy(client_tx_buf, &frame.frame_header, 4);
    Append_CRC8_Check_Num(client_tx_buf, 5);
    frame.cmd_id = 0x301;
    memcpy(&client_tx_buf[5], (void *)&frame.cmd_id, 2);
    data_header.data_cmd_id = ID_draw_one_graphic;
    data_header.sender_ID = client_info.robot_id;
    data_header.receiver_ID = client_info.client_id;
    memcpy(&client_tx_buf[7], &data_header, 6);
    memcpy(&client_tx_buf[13], &data.grapic_data_struct, 15);
    Append_CRC16_Check_Sum(client_tx_buf, 5 + 2 + LEN_ID_draw_one_graphic + 2);
    return uart_send_data(client_tx_buf, 5 + 2 + LEN_ID_draw_one_graphic + 2);
}

/**
    * @brief  发送绘制二个图形帧数据
  * @param
  * @retval
  */
uint8_t client_send_double_graphic(ext_client_custom_graphic_double_t data)
{
    frame_t frame;
    ext_student_interactive_header_data_t data_header;
    frame.frame_header.SOF = 0xA5;
    frame.frame_header.data_length = LEN_ID_draw_two_graphic;
    frame.frame_header.seq = 0;
    memcpy(client_tx_buf, &frame.frame_header, 4);
    Append_CRC8_Check_Num(client_tx_buf, 5);
    frame.cmd_id = 0x301;
    memcpy(&client_tx_buf[5], (void *)&frame.cmd_id, 2);
    data_header.data_cmd_id = ID_draw_two_graphic;
    data_header.sender_ID = client_info.robot_id;
    data_header.receiver_ID = client_info.client_id;
    memcpy(&client_tx_buf[7], &data_header, 6);
    memcpy(&client_tx_buf[13], data.grapic_data_struct, 15 * 2);
    Append_CRC16_Check_Sum(client_tx_buf, 5 + 2 + LEN_ID_draw_two_graphic + 2);
    return uart_send_data(client_tx_buf, 5 + 2 + LEN_ID_draw_two_graphic + 2);
}

/**
    * @brief  发送绘制五个图形帧数据
  * @param
  * @retval
  */
uint8_t client_send_five_graphic(ext_client_custom_graphic_five_t data)
{
    frame_t frame;
    ext_student_interactive_header_data_t data_header;
    frame.frame_header.SOF = 0xA5;
    frame.frame_header.data_length = LEN_ID_draw_five_graphic;
    frame.frame_header.seq = 0;
    memcpy(client_tx_buf, &frame.frame_header, 4);
    Append_CRC8_Check_Num(client_tx_buf, 5);
    frame.cmd_id = 0x301;
    memcpy(&client_tx_buf[5], (void *)&frame.cmd_id, 2);
    data_header.data_cmd_id = ID_draw_five_graphic;
    data_header.sender_ID = client_info.robot_id;
    data_header.receiver_ID = client_info.client_id;
    memcpy(&client_tx_buf[7], &data_header, 6);
    memcpy(&client_tx_buf[13], data.grapic_data_struct, 15 * 5);
    Append_CRC16_Check_Sum(client_tx_buf, 5 + 2 + LEN_ID_draw_five_graphic + 2);
    return uart_send_data(client_tx_buf, 5 + 2 + LEN_ID_draw_five_graphic + 2);
}

/**
    * @brief  发送绘制七个图形帧数据
  * @param
  * @retval
  */
uint8_t client_send_seven_graphic(ext_client_custom_graphic_seven_t data)
{
    frame_t frame;
    ext_student_interactive_header_data_t data_header;
    frame.frame_header.SOF = 0xA5;
    frame.frame_header.data_length = LEN_ID_draw_seven_graphic;
    frame.frame_header.seq = 0;
    memcpy(client_tx_buf, &frame.frame_header, 4);
    Append_CRC8_Check_Num(client_tx_buf, 5);
    frame.cmd_id = 0x301;
    memcpy(&client_tx_buf[5], (void *)&frame.cmd_id, 2);
    data_header.data_cmd_id = ID_draw_seven_graphic;
    data_header.sender_ID = client_info.robot_id;
    data_header.receiver_ID = client_info.client_id;
    memcpy(&client_tx_buf[7], &data_header, 6);
    memcpy(&client_tx_buf[13], data.grapic_data_struct, 15 * 7);
    Append_CRC16_Check_Sum(client_tx_buf, 5 + 2 + LEN_ID_draw_seven_graphic + 2);
    return uart_send_data(client_tx_buf, 5 + 2 + LEN_ID_draw_seven_graphic + 2);
}

/**
    * @brief  发送绘制字符帧数据
  * @param
  * @retval
  */
uint8_t client_send_char(ext_client_custom_character_t data)
{
    frame_t frame;
    ext_student_interactive_header_data_t data_header;
    frame.frame_header.SOF = 0xA5;
    frame.frame_header.data_length = LEN_ID_draw_char_graphic;
    frame.frame_header.seq = 0;
    memcpy(client_tx_buf, &frame.frame_header, 4);
    Append_CRC8_Check_Num(client_tx_buf, 5);
    frame.cmd_id = 0x301;
    memcpy(&client_tx_buf[5], (void *)&frame.cmd_id, 2);
    data_header.data_cmd_id = ID_draw_char_graphic;
    data_header.sender_ID = client_info.robot_id;
    data_header.receiver_ID = client_info.client_id;
    memcpy(&client_tx_buf[7], &data_header, 6);
    memcpy(&client_tx_buf[13], &data.grapic_data_struct, 15);
    memcpy(&client_tx_buf[28], data.data, 30);
    Append_CRC16_Check_Sum(client_tx_buf, 5 + 2 + LEN_ID_draw_char_graphic + 2);
    return uart_send_data(client_tx_buf, 5 + 2 + LEN_ID_draw_char_graphic + 2);
}

/**
    * @brief  删除一个图层
  * @param  uint8_t delete_layer 删除的图层数
  * @retval
  */
uint8_t client_graphic_delete_update(uint8_t delete_layer)
{
    frame_t frame;
    ext_student_interactive_header_data_t data_header;
    frame.frame_header.SOF = 0xA5;
    frame.frame_header.data_length = LEN_ID_draw_char_graphic;
    frame.frame_header.seq = 0;
    memcpy(client_tx_buf, &frame.frame_header, 4);
    Append_CRC8_Check_Num(client_tx_buf, 5);
    frame.cmd_id = 0x301;
    memcpy(&client_tx_buf[5], (void *)&frame.cmd_id, 2);
    data_header.data_cmd_id = ID_delete_graphic;
    data_header.sender_ID = client_info.robot_id;
    data_header.receiver_ID = client_info.client_id;
    memcpy(&client_tx_buf[7], &data_header, 6);
    client_tx_buf[13] = 1;
    client_tx_buf[14] = delete_layer;
    Append_CRC16_Check_Sum(client_tx_buf, 5 + 2 + LEN_ID_delete_graphic + 2);
    return uart_send_data(client_tx_buf, 5 + 2 + LEN_ID_delete_graphic + 2);
}
/******************************发送帧数据end****************************************/

/******************************串口发送数据begin************************************/
/**
    * @brief  串口发送数据
  * @param
  * @retval
  */
uint8_t uart_send_data(uint8_t *txbuf, uint16_t length)
{
    return HAL_UART_Transmit_DMA(&UI_huart, txbuf, length);
}
/******************************串口发送数据end**************************************/