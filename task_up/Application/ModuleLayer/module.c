/* module.c - 模块统一入口 */

#include "module.h"
#include "gimbal.h"
#include "lift.h"

void Module_Init(void)
{
    /* Gimbal.init is NULL until Gimbal_Init() sets the function pointers. */
    Gimbal_Init(&Gimbal);
    Lift_Init();
}

void Module_Work(void)
{
    Gimbal.work(&Gimbal);
    Lift_Work();
}

