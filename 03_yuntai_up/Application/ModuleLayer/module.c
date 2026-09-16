#include "module.h"
#include "gimbal.h"

void Module_Init(void)
{
    Gimbal.init(&Gimbal);
}

void Module_Work(void)
{
    Gimbal.work(&Gimbal);
}
