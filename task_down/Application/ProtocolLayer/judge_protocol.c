/* judge_protocol.c - 裁判系统协议 */

#include "judge_protocol.h"
#include "string.h"
#include "crc.h"
#include "judge.h"
#include "drv_uart.h"
judge_frame_header_t judge_frame_header;
drv_judge_info_t drv_judge_info = {
	.frame_header = &judge_frame_header,
};

/* 长度匹配后才允许固定长复制 */
static uint8_t Judge_PayloadMatches(uint16_t command, uint16_t payload)
{
    switch (command)
    {
    case ID_game_status: return (payload == LEN_game_status) ? 1u : 0u;
    case ID_game_result: return (payload == LEN_game_result) ? 1u : 0u;
    case ID_game_robot_HP: return (payload == LEN_game_robot_HP) ? 1u : 0u;
    case ID_event_data: return (payload == LEN_event_data) ? 1u : 0u;
    case ID_referee_warning: return (payload == LEN_referee_warning) ? 1u : 0u;
    case ID_dart_info: return (payload == LEN_dart_info) ? 1u : 0u;
    case ID_robot_status: return (payload == LEN_robot_status) ? 1u : 0u;
    case ID_power_heat_data: return (payload == LEN_power_heat_data) ? 1u : 0u;
    case ID_robot_pos: return (payload == LEN_robot_pos) ? 1u : 0u;
    case ID_buff: return (payload == LEN_buff) ? 1u : 0u;
    case ID_hurt_data: return (payload == LEN_hurt_data) ? 1u : 0u;
    case ID_shoot_data: return (payload == LEN_shoot_data) ? 1u : 0u;
    case ID_projectile_allowance: return (payload == LEN_projectile_allowance) ? 1u : 0u;
    case ID_rfid_status: return (payload == LEN_rfid_status) ? 1u : 0u;
    case ID_robot_interaction_data: return (payload == LEN_robot_interaction_data) ? 1u : 0u;
    case ID_map_command: return (payload == LEN_map_command) ? 1u : 0u;
    case ID_map_robot_data: return (payload == LEN_map_robot_data) ? 1u : 0u;
    case ID_map_data: return (payload == LEN_map_data) ? 1u : 0u;
    case ID_custom_info: return (payload == LEN_custom_info) ? 1u : 0u;
    case ID_set_video_channel: return (payload == LEN_set_video_channel) ? 1u : 0u;
    case ID_query_video_channel: return (payload == LEN_query_video_channel) ? 1u : 0u;
    default: return 0u;
    }
}

/* 保留半帧，完整CRC帧才发布 */
void judge_receive(uint8_t *rxBuf, uint16_t size)
{
    static uint8_t stream[USART1_RX_BUF_LEN];
    static uint16_t used;
    uint16_t length;
    uint16_t payload;
    uint16_t command;
    uint16_t consumed;
    if ((rxBuf == NULL) || (size > USART1_RX_BUF_LEN))
    {
        return;
    }
    for (uint16_t i = 0u; i < size; i++)
    {
        stream[used++] = rxBuf[i];
        while (used >= 5u)
        {
            consumed = 1u;
            if ((stream[0] == 0xA5u) &&
                (Verify_CRC8_Check_Sum(stream, 5u) != 0u))
            {
                payload = (uint16_t)stream[1] | ((uint16_t)stream[2] << 8);
                if (payload <= (USART1_RX_BUF_LEN - 9u))
                {
                    length = payload + 9u;
                    if (used < length)
                    {
                        break;
                    }
                    if (Verify_CRC16_Check_Sum(stream, length) != 0u)
                    {
                        command = (uint16_t)stream[5] | ((uint16_t)stream[6] << 8);
                        drv_judge_info.frame_header->SOF = stream[0];
                        drv_judge_info.frame_header->data_length = payload;
                        drv_judge_info.frame_header->seq = stream[3];
                        drv_judge_info.frame_header->CRC8 = stream[4];
                        drv_judge_info.cmd_id = command;
                        memcpy(&drv_judge_info.frame_tail, stream + length - 2u, 2u);
                        if (Judge_PayloadMatches(command, payload) != 0u)
                        {
                            Judge_Data_Update(command, stream + 7u);
                        }
                        consumed = length;
                    }
                }
            }
            used -= consumed;
            memmove(stream, stream + consumed, used);
        }
    }
}

void USART1_rxDataHandler(uint8_t *rxBuf, uint16_t size)
{
    judge_receive(rxBuf, size);
}

