/**
 * @file  device.c
 */

/* Includes ------------------------------------------------------------------*/
#include "device.h"
#include "judge.h"
#include "cap.h"
#include "board_protocol.h"
#include "rc_sensor.h"
#include "infantry.h"
#include "chassis.h"
#include "gimbal.h"
#include "launch.h"
#include "vision.h"
#include "ui.h"
#include "board_comm_config.h"
#include "chassis_config.h"
#include "chassis_control.h"
#include "chassis_input.h"

/* Private macro -------------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Exported variables --------------------------------------------------------*/


/* Private functions ---------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/
void DEVICE_Init(void)
{
#if BOARD_COMM_DEBUG
    rc_sensor.init(&rc_sensor);
    board.init(&board);
#if CHASSIS_BRINGUP_ENABLE
    rm_motor_list_init();
    Chassis_Input_Init();
    Chassis_Control_Init();
#endif
#else
    imu_sensor.init(&imu_sensor);
    rc_sensor.init(&rc_sensor);
    rm_motor_list_init();
    kt_motor_list_init();
    ht_motor_list_init();
    dm_motor_list_init();

    judge.init(&judge);
    cap.init(&cap);
    board.init(&board);

    chassis.init(&chassis);
    gimbal.init(&gimbal);
    launch.init(&launch);
    vision.init(&vision);
    My_Ui_Init();
    infantry.init(&infantry);
#endif
}
