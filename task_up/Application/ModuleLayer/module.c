/* module.c - 模块统一入口 */

#include "module.h"
#include "gimbal.h"
#include "lift.h"

void Module_Init(void)
{
    /* 直接初始化后才可使用绑定接口 */
    Gimbal_Init(&Gimbal);
    Lift_Init();
}

void Module_Work(void)
{
    Gimbal.work(&Gimbal);
    Lift_Work();
}

