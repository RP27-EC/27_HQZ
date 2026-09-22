/* Command.c - 指令服务 */
#include "command.h"
void User_Status_Update(command_t *command);
void Cmd_Switch_Run(command_t *command);
void Cmd_Switch_Finish(command_t *command);
void Cmd_Value_Update(command_t *command);
void Cmd_HeartBeat(command_t *command);
void Cmd_Class_Update(command_t *command, bool condition);
void Cmd_Clean(command_t *command);
/* 指令分类初始化 */
void Cmd_Class_Init(command_t *command)
{
  command->user_value = false;
  command->user_value_last = false;
  command->user_status = KEEP_U;

  command->cmd_value = false;
  command->cmd_status = FINISH_C;

  command->run_time = 0;

  command->update = Cmd_Class_Update;
  command->s_run = Cmd_Switch_Run;
  command->s_finish = Cmd_Switch_Finish;
  command->clean = Cmd_Clean;
  command->heartbeat = Cmd_HeartBeat;

  command->init_flag = INIT_C;  // 初始化标志
}
/* User_Status_Update */
void User_Status_Update(command_t *command)
{

  static uint32_t last_lock_tick;

  if (command->user_value == true && command->user_value_last == false)
  {

    if (command->Trigger_lock.Trigger_lock_on == 1)
    {
      if (HAL_GetTick() - last_lock_tick > command->Trigger_lock.lock_time)
      {

        command->user_status = SWITCH_HIGHT_U;

      }
    }

    else
    {
      command->user_status = SWITCH_HIGHT_U;
    }
    last_lock_tick = HAL_GetTick();  // lastlock节拍
  }
  else if (command->user_value == false && command->user_value_last == true)
  {
    command->user_status = SWITCH_LOW_U;
  }
  else
  {
    command->user_status = KEEP_U;
  }

  command->user_value_last = command->user_value;
}

/* 指令服务心跳 */
void Cmd_HeartBeat(command_t *command)
{

  if (command->cmd_status == RUNING_C)  // 命令状态
  {
    if (command->run_time_max != OUT_TIME_OFF)  // 输出时间OFF
    {
      command->run_time++;
    }
  }
  else
  {
    command->run_time = 0;
  }


  if (command->run_time > command->run_time_max)
  {
    Cmd_Clean(command);
  }

   Cmd_Value_Update(command);
}

/* 刷新指令数值 */
void Cmd_Value_Update(command_t *command)
{

  switch (command->cmd_type)
  {
  case RISE_TRIGER_C:
    if (command->user_status == SWITCH_HIGHT_U)
    {
      command->cmd_value = true;
    }
    else
    {
      command->cmd_value = false;
    }
    break;

  case FALL_TRIGER_C:
    if (command->user_status == SWITCH_LOW_U)
    {
      command->cmd_value = true;
    }
    else
    {
      command->cmd_value = false;
    }
    break;

  case HIGH_TRIGER_C:
    if (command->user_value == true)
    {
      command->cmd_value = true;
    }
    else
    {
      command->cmd_value = false;
    }
    break;

  case LOW_TRIGER_C:
    if (command->user_value == false)
    {
      command->cmd_value = true;
    }
    else
    {
      command->cmd_value = false;
    }
    break;

  case NO_CMD:
    command->cmd_value = false;
    break;

  default:
    break;
  }
}

/* 指令分类刷新 */
void Cmd_Class_Update(command_t *command, bool condition)
{
  if (command->init_flag == INIT_C)
  {
    if (condition == true)
    {
      command->user_value = true;
    }
    else
    {
      command->user_value = false;
    }
  }


  User_Status_Update(command);

 // Cmd_Value_Update(command);
}

/* 清空指令队列 */
void Cmd_Clean(command_t *command)
{
  command->user_value = false;
  command->user_value_last = false;
  command->user_status = KEEP_U;

  command->cmd_value = false;
  command->cmd_status = FINISH_C;

  command->run_time = 0;
}

/* 指令切换执行 */
void Cmd_Switch_Run(command_t *command)
{
  command->cmd_status = RUNING_C;
}

/* 指令切换完成 */
void Cmd_Switch_Finish(command_t *command)
{
  command->cmd_status = FINISH_C;
}


