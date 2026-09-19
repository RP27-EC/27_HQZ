/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Minimal FreeRTOS tasks for the gimbal down board.
  ******************************************************************************
  */
/* USER CODE END Header */

#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Definitions for MonitorTask */
osThreadId_t MonitorTaskHandle;
const osThreadAttr_t MonitorTask_attributes = {
  .name = "MonitorTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal,
};

/* Definitions for CtrlTask */
osThreadId_t CtrlTaskHandle;
const osThreadAttr_t CtrlTask_attributes = {
  .name = "CtrlTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal,
};

/* Definitions for CommandTask */
osThreadId_t CommandTaskHandle;
const osThreadAttr_t CommandTask_attributes = {
  .name = "CommandTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};

/* Definitions for ConnectTask */
osThreadId_t ConnectTaskHandle;
const osThreadAttr_t ConnectTask_attributes = {
  .name = "ConnectTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};

void StartMonitorTask(void *argument);
void StartCtrlTask(void *argument);
void StartCommandTask(void *argument);
void StartConnectTask(void *argument);

void MX_FREERTOS_Init(void);

void MX_FREERTOS_Init(void)
{
  MonitorTaskHandle = osThreadNew(StartMonitorTask, NULL, &MonitorTask_attributes);
  CtrlTaskHandle = osThreadNew(StartCtrlTask, NULL, &CtrlTask_attributes);
  CommandTaskHandle = osThreadNew(StartCommandTask, NULL, &CommandTask_attributes);
  ConnectTaskHandle = osThreadNew(StartConnectTask, NULL, &ConnectTask_attributes);
}
