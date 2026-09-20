#include "module.h"
#include "gimbal.h"

void Module_Init(void)
{
    /* Gimbal.init is NULL until Gimbal_Init() sets the function pointers. */
    Gimbal_Init(&Gimbal);
}

void Module_Work(void)
{
    Gimbal.work(&Gimbal);
}
