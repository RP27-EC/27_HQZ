/* power_limit.c - 底盘功率限制 */

#include <math.h>
#include <stddef.h>
#include <float.h>

#include "power_limit.h"
#include "rp_math.h"

power_limit_state_t power_limit_state;

typedef struct
{
    uint32_t buffer_seq; // 已处理帧序号，循环计数
    uint32_t buffer_tick; // 已处理接收时刻，ms
    uint8_t online; // 在线预算已初始化，0/1
} power_buffer_runtime_t;

static power_buffer_runtime_t buffer_runtime;

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
    power_buffer_runtime_t runtime_initial = {0};
    power_limit_state = initial;
    buffer_runtime = runtime_initial;
    power_limit_state.target_power = CHASSIS_POWER_FALLBACK_W;
    power_limit_state.base_budget_w = CHASSIS_POWER_FALLBACK_W;
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

float Power_Limit_GetTarget(const judge_power_snapshot_t *snapshot, uint8_t active)
{
    float buffer;
    float limit;
    float integral_max;
    float guard_limit;
    float desired;
    float dt = 0.0f;
    uint8_t new_frame = 0u;
    uint8_t first_frame = 0u;

    power_limit_state.judge_online =
        ((snapshot != NULL) && (snapshot->judge_online != 0u)) ? 1u : 0u;
    if ((snapshot == NULL) || (snapshot->judge_online == 0u) ||
        (snapshot->valid == 0u) ||
        ((float)snapshot->limit_w < CHASSIS_POWER_MIN_W) ||
        ((float)snapshot->limit_w > CHASSIS_POWER_MAX_W))
    {
        power_buffer_runtime_t initial = {0};
        buffer_runtime = initial;
        power_limit_state.fallback_used = 1u;
        power_limit_state.buffer_energy = 0.0f;
        power_limit_state.buffer_error = 0.0f;
        power_limit_state.buffer_integral_w = 0.0f;
        power_limit_state.buffer_budget_w = CHASSIS_POWER_FALLBACK_W;
        power_limit_state.base_budget_w = CHASSIS_POWER_FALLBACK_W;
        power_limit_state.buffer_guard_active = 0u;
        power_limit_state.target_power = CHASSIS_POWER_FALLBACK_W;
        return power_limit_state.target_power;
    }

    limit = fmaxf(CHASSIS_POWER_MIN_W,
                  (float)snapshot->limit_w - CHASSIS_POWER_MARGIN_W);
    power_limit_state.base_budget_w = limit;
    buffer = constrain((float)snapshot->buffer_j, 0.0f, CHASSIS_POWER_BUFFER_FULL_J);
    integral_max = limit - CHASSIS_POWER_MIN_W;
    power_limit_state.buffer_energy = buffer;
    power_limit_state.fallback_used = 0u;

    if (buffer_runtime.online == 0u)
    {
        power_limit_state.target_power = fminf(CHASSIS_POWER_FALLBACK_W, limit);
        power_limit_state.buffer_integral_w = limit - power_limit_state.target_power;
        power_limit_state.buffer_error = 0.0f;
        buffer_runtime.online = 1u;
        first_frame = 1u;
        new_frame = 1u;
    }
    else if (snapshot->buffer_seq != buffer_runtime.buffer_seq)
    {
        uint32_t elapsed = snapshot->buffer_tick - buffer_runtime.buffer_tick;
        if (elapsed > CHASSIS_POWER_BUFFER_DT_MAX_MS)
        {
            elapsed = CHASSIS_POWER_BUFFER_DT_MAX_MS;
        }
        dt = (float)elapsed * 0.001f;
        new_frame = 1u;
    }
    power_limit_state.buffer_integral_w =
        constrain(power_limit_state.buffer_integral_w, 0.0f, integral_max);

    guard_limit = limit;
    power_limit_state.buffer_guard_active =
        (buffer < CHASSIS_POWER_BUFFER_GUARD_J) ? 1u : 0u;
    if (power_limit_state.buffer_guard_active != 0u)
    {
        guard_limit = CHASSIS_POWER_MIN_W + integral_max *
                      buffer / CHASSIS_POWER_BUFFER_GUARD_J;
    }

    if (new_frame != 0u)
    {
        float error = 0.0f;
        float low = CHASSIS_POWER_BUFFER_TARGET_J - CHASSIS_POWER_BUFFER_BAND_J;

        /* 低缓冲帧不因松杆丢弃 */
        buffer_runtime.buffer_seq = snapshot->buffer_seq;
        buffer_runtime.buffer_tick = snapshot->buffer_tick;
        if (buffer < low)
        {
            error = low - buffer;
        }
        power_limit_state.buffer_error = error;
        if (first_frame == 0u)
        {
            float delta = CHASSIS_POWER_BUFFER_KI * error * dt;
            /* 停车满缓冲不释放扣减 */
            if ((buffer >= CHASSIS_POWER_BUFFER_FULL_J) && (active != 0u))
            {
                delta = -CHASSIS_POWER_BUFFER_RELEASE_W_S * dt;
            }
            float proposed = constrain(power_limit_state.buffer_integral_w +
                delta, 0.0f, integral_max);
            float proposed_budget = limit - CHASSIS_POWER_BUFFER_KP * error - proposed;
            /* 禁止积分加深预算饱和 */
            if (!((delta > 0.0f && proposed_budget < CHASSIS_POWER_MIN_W) ||
                  (delta < 0.0f && proposed_budget > guard_limit)))
            {
                power_limit_state.buffer_integral_w = proposed;
            }
        }
    }

    power_limit_state.buffer_budget_w = limit -
        CHASSIS_POWER_BUFFER_KP * power_limit_state.buffer_error -
        power_limit_state.buffer_integral_w;
    desired = constrain(power_limit_state.buffer_budget_w,
                        CHASSIS_POWER_MIN_W, guard_limit);
    /* 降额立即，恢复仅随新帧 */
    if (desired < power_limit_state.target_power)
    {
        power_limit_state.target_power = desired;
    }
    else if ((active != 0u) && (new_frame != 0u))
    {
        power_limit_state.target_power = fminf(desired,
            power_limit_state.target_power + CHASSIS_POWER_RECOVER_W_S * dt);
    }
    power_limit_state.target_power = constrain(power_limit_state.target_power,
                                              CHASSIS_POWER_MIN_W, guard_limit);
    return power_limit_state.target_power;
}

void Power_Limit_SetCapFeedback(int16_t power_raw, float voltage_v, float current_a,
                                uint8_t ability, uint8_t online)
{
    power_limit_state.cap_online = (online != 0u) ? 1u : 0u;
    if (online == 0u)
    {
        /* 掉线保留最后值，仅置在线标志，避免曲线出现归零假台阶 */
        return;
    }

    power_limit_state.cap_power_raw = power_raw;
    power_limit_state.cap_voltage = voltage_v;
    power_limit_state.cap_current = current_a;
    power_limit_state.cap_ability = (ability != 0u) ? 1u : 0u;
    /* 电容净功率，正=放电，用于判定 0x211[0:1] 字节序与量纲 */
    power_limit_state.cap_net_power = voltage_v * current_a;
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
