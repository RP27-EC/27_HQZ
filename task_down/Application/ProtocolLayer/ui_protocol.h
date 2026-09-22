/* ui_protocol.h - UI 显示协议 */



#ifndef __UI_PROTOCOL_H

#define __UI_PROTOCOL_H

#include "main.h"

#include "stdbool.h"

// 1920*1080

#define Client_mid_position_x 960

#define Client_mid_position_y 540



typedef struct __attribute__((packed))

{

    uint8_t SOF;

    uint16_t data_length;

    uint8_t seq;

    uint8_t CRC8;

} frame_header_t;


typedef struct __attribute__((packed))

{

    uint16_t data_cmd_id;

    uint16_t sender_ID;

    uint16_t receiver_ID;

} ext_student_interactive_header_data_t;




typedef enum

{



    ID_delete_graphic = 0x0100,

    ID_draw_one_graphic = 0x0101,

    ID_draw_two_graphic = 0x0102,

    ID_draw_five_graphic = 0x0103,

    ID_draw_seven_graphic = 0x0104,

    ID_draw_char_graphic = 0x0110,

} data_cmd_id_e;


enum

{

    LEN_ID_delete_graphic = 8,// 6+2

    LEN_ID_draw_one_graphic = 21,// 6+15

    LEN_ID_draw_two_graphic = 36,// 6+15*2

    LEN_ID_draw_five_graphic = 81,// 6+15*5

    LEN_ID_draw_seven_graphic = 111,// 6+15*7

    LEN_ID_draw_char_graphic = 51,
};


typedef enum

{

    NONE = 0,  

    ADD = 1,  

    MODIFY = 2,  

    DELETE = 3,  

} operate_tpye_e;


typedef enum

{

    RED_BLUE = 0,

    YELLOW = 1,

    GREEN = 2,

    ORANGE = 3,

    FUCHSIA = 4,

    PINK = 5,

    CYAN_BLUE = 6,

    BLACK = 7,

    WHITE = 8

} graphic_color_e;




typedef struct __attribute__((packed))

{

    uint8_t graphic_name[3];

    uint32_t operate_tpye : 3;

    uint32_t graphic_tpye : 3;

    uint32_t layer : 4;

    uint32_t color : 4;

    uint32_t start_angle : 9;

    uint32_t end_angle : 9;

    uint32_t width : 10;

    uint32_t start_x : 11;

    uint32_t start_y : 11;

    uint32_t radius : 10;

    uint32_t end_x : 11;

    uint32_t end_y : 11;

} graphic_data_struct_t;


typedef struct __attribute__((packed))

{

    graphic_data_struct_t grapic_data_struct;

} ext_client_custom_graphic_single_t;


typedef struct __attribute__((packed))

{

    graphic_data_struct_t grapic_data_struct[2];

} ext_client_custom_graphic_double_t;


typedef struct __attribute__((packed))

{

    graphic_data_struct_t grapic_data_struct[5];

} ext_client_custom_graphic_five_t;


typedef struct __attribute__((packed))

{

    graphic_data_struct_t grapic_data_struct[7];

} ext_client_custom_graphic_seven_t;


typedef struct __attribute__((packed))

{

    graphic_data_struct_t grapic_data_struct;

    char data[30];

} ext_client_custom_character_t;

typedef struct __attribute__((packed))

{

    uint8_t operate_tpye;

    uint8_t layer;

} ext_client_custom_graphic_delete_t;




typedef struct __attribute__((packed))

{

    frame_header_t frame_header;

    uint16_t cmd_id;

    ext_student_interactive_header_data_t data_header;

    uint16_t frame_tail;

} frame_t;


typedef struct

{

    uint8_t robot_id;

    uint16_t client_id;

} client_info_t;


void client_info_update(void);


graphic_data_struct_t draw_line(char *name,

                                uint8_t operate_tpye,

                                uint8_t layer,

                                uint8_t color,

                                uint16_t width,

                                uint16_t start_x,

                                uint16_t start_y,

                                uint16_t end_x,

                                uint16_t end_y);


graphic_data_struct_t draw_rectangle(char *name,

                                     uint8_t operate_tpye,

                                     uint8_t layer,

                                     uint8_t color,

                                     uint16_t width,

                                     uint16_t start_x,

                                     uint16_t start_y,

                                     uint16_t end_x,

                                     uint16_t end_y);


graphic_data_struct_t draw_circle(char *name,

                                  uint8_t operate_tpye,

                                  uint8_t layer,

                                  uint8_t color,

                                  uint16_t width,

                                  uint16_t start_x,

                                  uint16_t start_y,

                                  uint16_t radius);


graphic_data_struct_t draw_ellipse(char *name,

                                   uint8_t operate_tpye,

                                   uint8_t layer,

                                   uint8_t color,

                                   uint16_t width,

                                   uint16_t start_x,

                                   uint16_t start_y,

                                   uint16_t end_x,

                                   uint16_t end_y);


graphic_data_struct_t draw_arc(char *name,

                               uint8_t operate_tpye,

                               uint8_t layer,

                               uint8_t color,

                               uint16_t start_angle,

                               uint16_t end_angle,

                               uint16_t width,

                               uint16_t start_x,

                               uint16_t start_y,

                               uint16_t end_x,

                               uint16_t end_y);


graphic_data_struct_t draw_float(char *name,

                                 uint8_t operate_tpye,

                                 uint8_t layer,

                                 uint8_t color,

                                 uint16_t size,

                                 uint16_t decimal,

                                 uint16_t width,

                                 uint16_t start_x,

                                 uint16_t start_y,

                                 int32_t num);


graphic_data_struct_t draw_int(char *name,

                               uint8_t operate_tpye,

                               uint8_t layer,

                               uint8_t color,

                               uint16_t size,

                               uint16_t width,

                               uint16_t start_x,

                               uint16_t start_y,

                               int32_t num);


graphic_data_struct_t draw_char(char *name,

                                uint8_t operate_tpye,

                                uint8_t layer,

                                uint8_t color,

                                uint16_t size,

                                uint16_t length,

                                uint16_t width,

                                uint16_t start_x,

                                uint16_t start_y);

uint8_t client_send_single_graphic(ext_client_custom_graphic_single_t data);

uint8_t client_send_double_graphic(ext_client_custom_graphic_double_t data);

uint8_t client_send_five_graphic(ext_client_custom_graphic_five_t data);

uint8_t client_send_seven_graphic(ext_client_custom_graphic_seven_t data);

uint8_t client_send_char(ext_client_custom_character_t data);

uint8_t client_graphic_delete_update(uint8_t delete_layer);

uint8_t uart_send_data(uint8_t *txbuf, uint16_t length);

//****************************************************************************************************end

#endif

