/* power_limit.c - 底盘功率限制 */

#include <math.h>
#include <stddef.h>
#include <float.h>

#include "power_limit.h"
#include "rp_math.h"

power_limit_state_t power_limit_state;

/* 沿用模板四轮系数 */
static const power_coeff_t power_coeff[WHEEL_CNT] = {
    {{1.4268163740611692f,  0.0004488821106870444f,  8.492604098272384e-05f,  1.7818822187359053e-06f, 1.3769792187274362e-07f, 3.5482352783775733e-07f}},
    {{1.316759451137222f,  -0.0003859926122313794f, -0.0001505750547614686f, 1.561834267598007e-06f,  1.6344718643622346e-07f, 4.450747190731389e-07f}},
    {{1.3732605217163856f, -0.0005512718723215252f,  0.00016888118401289773f, 1.6267287913987835e-06f, 1.535613047156326e-07f,  3.9997112666134343e-07f}},
    {{1.3732605217163856f, -0.0005512718723215252f,  0.00016888118401289773f, 1.6267287913987835e-06f, 1.535613047156326e-07f,  3.9997112666134343e-07f}},
};

void Power_Limit_Init(void)
{
    power_limit_state_t initial = {0};
    power_limit_state = initial;
    power_limit_state.target_power = CHASSIS_POWER_FALLBACK_W;
    power_limit_state.scale = 1.0f;
    power_limit_state.fallback_used = 1u;
}

static uint8_t Power_Limit_Finite(float value)
{
    return ((value >= -FLT_MAX) && (value <= FLT_MAX)) ? 1u : 0u;
}

/* I为原始计数，w为转子rpm */
static float Power_Predict_One(const power_coeff_t *coeff, float i, float w)
{
    return coeff->k[0]
         + coeff->k[1] * i
         + coeff->k[2] * w
         + coeff->k[3] * i * w
         + coeff->k[4] * i * i
         + coeff->k[5] * w * w;
}

/* 与电机驱动换算一致 */
static int16_t Power_Limit_Torque_To_Raw(float torque)
{
    float current_a = torque / _3508_TORQUE_CONSTANT;
    current_a = constrain(current_a, -_3508_MAX_CURRENT * 0.9f, _3508_MAX_CURRENT * 0.9f);
    return (int16_t)((current_a / _3508_MAX_CURRENT) * 16384.0f);
}

/* 不用回馈抵消其他轮耗电 */
static float Power_Limit_Predict(const float torque[WHEEL_CNT],
                                 const float rpm[WHEEL_CNT], float scale,
                                 uint8_t record)
{
    float sum = 0.0f;
    uint8_t i;

    for (i = 0u; i < WHEEL_CNT; i++)
    {
        float output = torque[i] * scale;
        int16_t raw = Power_Limit_Torque_To_Raw(output);
        float predicted = Power_Predict_One(&power_coeff[i], (float)raw, rpm[i]);

        if (Power_Limit_Finite(predicted) == 0u)
        {
            return NAN;
        }
        if (record != 0u)
        {
            power_limit_state.wheel_raw[i] = (float)raw;
            power_limit_state.wheel_predict[i] = predicted;
            power_limit_state.wheel_torque_limited[i] = output;
        }
        if (predicted > 0.0f)
        {
            sum += predicted;
        }
    }
    return sum;
}

float Power_Limit_GetTarget(dev_work_state_t judge_status, uint8_t power_data_valid,
                            uint16_t limit_w, uint16_t buffer_j)
{
    float buffer;
    float factor;

    power_limit_state.judge_online = (judge_status == DEV_ONLINE) ? 1u : 0u;
    power_limit_state.fallback_used = 1u;
    power_limit_state.buffer_energy = 0.0f;
    power_limit_state.target_power = CHASSIS_POWER_FALLBACK_W;

    if ((judge_status != DEV_ONLINE) || (power_data_valid == 0u) ||
        ((float)limit_w < CHASSIS_POWER_MIN_W) ||
        ((float)limit_w > CHASSIS_POWER_MAX_W))
    {
        return power_limit_state.target_power;
    }

    buffer = constrain((float)buffer_j, 0.0f, CHASSIS_POWER_BUFFER_FULL_J);
    factor = CHASSIS_POWER_BUFFER_FLOOR +
             (1.0f - CHASSIS_POWER_BUFFER_FLOOR) * buffer / CHASSIS_POWER_BUFFER_FULL_J;
    power_limit_state.buffer_energy = buffer;
    power_limit_state.fallback_used = 0u;
    power_limit_state.target_power = (float)limit_w * CHASSIS_POWER_SAFETY_K * factor;
    return power_limit_state.target_power;
}

void Power_Limit_SetCapFeedback(int16_t raw, uint8_t online)
{
    power_limit_state.cap_online = (online != 0u) ? 1u : 0u;
    if (online != 0u)
    {
        power_limit_state.cap_power_raw = raw;
    }
}

static void Power_Limit_InvalidOutput(float torque_out[WHEEL_CNT])
{
    uint8_t i;

    power_limit_state.requested_power = NAN;
    power_limit_state.estimate_power = NAN;
    power_limit_state.scale = 0.0f;
    power_limit_state.limited = 1u;
    power_limit_state.target_unreachable = 1u;
    for (i = 0u; i < WHEEL_CNT; i++)
    {
        if (torque_out != NULL)
        {
            torque_out[i] = 0.0f;
        }
        power_limit_state.wheel_raw[i] = 0.0f;
        power_limit_state.wheel_predict[i] = NAN;
        power_limit_state.wheel_torque_limited[i] = 0.0f;
    }
}

void Power_Limit_Apply(float torque_out[WHEEL_CNT], rm_motor_t *const motor[WHEEL_CNT])
{
    float candidate[WHEEL_CNT];
    float rpm[WHEEL_CNT];
    float requested;
    float zero_power;
    float final_power;
    float scale = 1.0f;
    float target = power_limit_state.target_power;
    uint8_t i;

    if ((torque_out == NULL) || (motor == NULL))
    {
        Power_Limit_InvalidOutput(torque_out);
        return;
    }
    for (i = 0u; i < WHEEL_CNT; i++)
    {
        if ((motor[i] == NULL) || (motor[i]->rx_info == NULL) ||
            (Power_Limit_Finite(torque_out[i]) == 0u))
        {
            Power_Limit_InvalidOutput(torque_out);
            return;
        }
        candidate[i] = torque_out[i];
        rpm[i] = (float)motor[i]->rx_info->encoder_speed;
    }

    requested = Power_Limit_Predict(candidate, rpm, 1.0f, 0u);
    zero_power = Power_Limit_Predict(candidate, rpm, 0.0f, 0u);
    if ((Power_Limit_Finite(requested) == 0u) ||
        (Power_Limit_Finite(zero_power) == 0u))
    {
        Power_Limit_InvalidOutput(torque_out);
        return;
    }
    power_limit_state.requested_power = requested;
    power_limit_state.target_unreachable = 0u;

#if (CHASSIS_POWER_LIMIT_ENABLE != 0u)
    if ((Power_Limit_Finite(target) == 0u) || (target < 0.0f))
    {
        scale = 0.0f;
        power_limit_state.target_unreachable = 1u;
    }
    else if (zero_power > target)
    {
        /* 不制造额外制动力矩 */
        scale = 0.0f;
        power_limit_state.target_unreachable = 1u;
    }
    else if (requested > target)
    {
        float lower = 0.0f;
        float upper = 1.0f;
        uint8_t step;

        /* 公共比例保留方向与配比 */
        for (step = 0u; step < CHASSIS_POWER_SEARCH_STEPS; step++)
        {
            float middle = (lower + upper) * 0.5f;
            float predicted = Power_Limit_Predict(candidate, rpm, middle, 0u);
            if ((Power_Limit_Finite(predicted) != 0u) && (predicted <= target))
            {
                lower = middle;
            }
            else
            {
                upper = middle;
            }
        }
        scale = lower;
    }
#else
    (void)target;
#endif

    for (i = 0u; i < WHEEL_CNT; i++)
    {
        torque_out[i] = candidate[i] * scale;
    }
    final_power = Power_Limit_Predict(torque_out, rpm, 1.0f, 1u);
#if (CHASSIS_POWER_LIMIT_ENABLE != 0u)
    if ((Power_Limit_Finite(final_power) == 0u) ||
        ((power_limit_state.target_unreachable == 0u) && (final_power > target)))
    {
        /* 复算失败则回到零输出 */
        scale = 0.0f;
        for (i = 0u; i < WHEEL_CNT; i++)
        {
            torque_out[i] = 0.0f;
        }
        final_power = Power_Limit_Predict(torque_out, rpm, 1.0f, 1u);
        power_limit_state.target_unreachable =
            ((Power_Limit_Finite(final_power) == 0u) || (final_power > target)) ? 1u : 0u;
    }
#endif
    power_limit_state.estimate_power = final_power;
    power_limit_state.scale = scale;
    power_limit_state.limited = (scale < 1.0f) ? 1u : 0u;
}
