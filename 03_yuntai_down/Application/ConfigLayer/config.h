///* ------------------------------------------------------------------------------------------------------------------------------------------------------------------- 
// * @Files: config.h
// * 
// * @Author: Ye Jinyi
// * 
// * @First  Edit Date: 2024.12.4
// * @Latest Edit Date: 
// * 
// * @illustrate: 用来全局配置
// * 
// * @attention: 
// * 
// *-------------------------------------------------------------------------------------------------------------------------------------------------------------------*/

//#ifndef _CONFIG_H
//#define _CONFIG_H

///* Macro ------------------------------------------------------------------------------------------------------------------------------------------------------------ */
///* 使用第几批板 */
//#define _USE_BOARD_1_x


///* 超电使用模式，需要且仅有一个能置1 */
//#define _ON_CAR_USING_              0U      // 上车使用
//#define _BEFORE_CAR_TEST_           1U      // 上车前的最终测试
//#define _CALIBRATE_CAP_I_           0U      // 校准电容组电流
//#define _CALIBRATE_EXCEPT_CAP_I_    0U      // 校准电容组电流之外的信息
//#define _PWM_TEST_                  0U      // 测试 PWM 波及 CAP 端输出

//// 多定义或少定义编译会报错
//#if ((_ON_CAR_USING_ + _BEFORE_CAR_TEST_ + _CALIBRATE_CAP_I_ + _CALIBRATE_EXCEPT_CAP_I_ + _PWM_TEST_) < 1U)
//    #error "The definition of supercap usage mode is missing in config.h"
//#endif
//#if ((_ON_CAR_USING_ + _BEFORE_CAR_TEST_ + _CALIBRATE_CAP_I_ + _CALIBRATE_EXCEPT_CAP_I_ + _PWM_TEST_) > 1U)
//    #error "Redundant definition of supercap usage mode in config.h"
//#endif


///* 串口打印信息开关 */
//#define _I_NEED_TO_PRINT_INFO_


///* 外设有关参数 */
///* hrtim */
//#define HRTIM_PERIOD     (27200.f)      // hrtim周期
//#define HRTIM_PERIOD_0_5 (13600.f)      // hrtim半周期
//#define MIN_CMP_VALUE    (100.f)        // hrtim CCR最小比较值
//#define MAX_CMP_VALUE    (27100.f)      // hrtim CCR最大比较值
///* adc */
//#define INA_V_REF        (3.3f)          // 参考电压
//#define CHIP_V_REF       (3.3f)
//#define ADC_SCALE        (4095.f)       // ADC数据范围


///* 数据处理相关部分 */
//typedef enum {
//    BAT_V  = 0,      // 电源电压
//    CAP_I  = 1,      // 电容电流
//    CAP_V  = 2,      // 电容电压
//    BAT_I  = 3,      // 电源电流
//    MEASURE_NUM,     // 测量参数总数
//} argument_idx_e;    // 测量参数在数组中的索引


//#endif
