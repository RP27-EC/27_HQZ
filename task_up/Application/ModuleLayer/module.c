/* module.c - 模块统一入口 */

#include "module.h"
#include "gimbal.h"
#include "lift.h"

/* 模块初始化 */
void Module_Init(void)
{
    // 先绑函数指针再调用
    Gimbal_Init(&Gimbal);
    Lift_Init();
}

/* 模块周期调度, 1ms */
void Module_Work(void)
{
    Gimbal.work(&Gimbal);
    Lift_Work();
}


