/* supercap.c - 超电通信设备 */

#include "supercap.h"
#include "drv_can.h"

SuperCap_t supercap;

/* 原始值线性映射到物理量 */
static float SuperCap_Scale(int16_t value, float min_value, float max_value)
{
    // 超电原始值域 -32000~32000
    float ratio = ((float)value + 32000.0f) / 64000.0f;

    return ratio * (max_value - min_value) + min_value;
}

/* 初始化 */
void SuperCap_Init(void)
{
    supercap.state = SUPERCAP_STATE_OFFLINE;
    supercap.feedback.chassis_power = 0;
    supercap.feedback.voltage_raw = 0;
    supercap.feedback.current_raw = 0;
    supercap.feedback.ability = 0u;
    supercap.feedback.pre_charge_mode = 0u;
    supercap.chassis_power = 0;
    supercap.cap_voltage = 0.0f;
    supercap.cap_current = 0.0f;
    supercap.tx_count = 0u;
    supercap.rx_count = 0u;
    supercap.last_rx_ms = 0u;
    // 上电即视为离线
    supercap.offline_count = SUPERCAP_OFFLINE_TIMEOUT_MS;
}

/* 下发控制帧 */
void SuperCap_Tx(void)
{
    SuperCap_Control_t control = {0};
    uint8_t tx_buf[8];

    control.power_buffer = SUPERCAP_POWER_BUFFER;
    control.power_limit = SUPERCAP_POWER_LIMIT;
    control.power_out_limit = SUPERCAP_POWER_OUT_LIMIT;
    control.power_in_limit = SUPERCAP_POWER_IN_LIMIT;
    control.cap_switch = SUPERCAP_CAP_SWITCH;
    control.turbo_mode = SUPERCAP_TURBO_MODE;
    control.pre_charge_enable = SUPERCAP_PRE_CHARGE_ENABLE;

    SuperCap_Protocol_Encode(&control, tx_buf);
    CAN_SendData(&hfdcan1, SUPERCAP_CAN_TX_ID, tx_buf);
    supercap.tx_count++;
}

/* 解析反馈并刷新状态 */
void SuperCap_Rx(const uint8_t *rx_buf)
{
    SuperCap_Protocol_Decode(rx_buf, &supercap.feedback);

    supercap.chassis_power = supercap.feedback.chassis_power;
    supercap.cap_voltage = SuperCap_Scale(supercap.feedback.voltage_raw, 0.0f, 25.0f);
    supercap.cap_current = SuperCap_Scale(supercap.feedback.current_raw, -16.0f, 16.0f);
    supercap.rx_count++;
    supercap.last_rx_ms = HAL_GetTick();
    supercap.offline_count = 0u;
    // 收到任意帧即置在线
    supercap.state = SUPERCAP_STATE_ONLINE;
}

/* 离线检测 */
void SuperCap_Heartbeat(void)
{
    if (supercap.state == SUPERCAP_STATE_OFFLINE)
    {
        return;
    }

    if ((HAL_GetTick() - supercap.last_rx_ms) >= SUPERCAP_OFFLINE_TIMEOUT_MS)
    {
    // 上电即视为离线
        supercap.offline_count = SUPERCAP_OFFLINE_TIMEOUT_MS;
        supercap.state = SUPERCAP_STATE_OFFLINE;
    }
    else if (supercap.offline_count < SUPERCAP_OFFLINE_TIMEOUT_MS)
    {
        supercap.offline_count++;
    }
}
