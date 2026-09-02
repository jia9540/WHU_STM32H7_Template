#ifndef SEIZE_SKY_H
#define SEIZE_SKY_H


#include <stdbool.h>
#include <stdint.h>
#include "main.h"
#include "UnitreeMotor.h"
#include "DJmotor.h"
#include "cmsis_os.h"
#include "gpio.h"
#include "kinematics.h"
#include "fdcan.h"



//  初始状态
#define ARM_U1_START_POS    0.0f
#define ARM_U2_START_POS    0.0f
#define ARM_DJ_START_POS    0.0f


//  准备状态
#define ARM_U1_READY_POS    0.64f
#define ARM_U2_READY_POS    0.52f
#define ARM_DJ_READY_POS    -240.0f
// //  准备状态
// #define ARM_U1_READY_POS    500.f
// #define ARM_U2_READY_POS    400.f
// #define ARM_DJ_READY_POS    0.0f


//  持块状态
#define ARM_U1_KEEP_POS    0.64f
#define ARM_U2_KEEP_POS    0.52f
#define ARM_DJ_KEEP_POS    -240.0f


//  存贮状态
#define ARM_U1_STORE_POS    0.64f
#define ARM_U2_STORE_POS    0.52f
#define ARM_DJ_STORE_POS    -240.0f

//取天空块
#define ARM_U1_SKY_POS    0.672f
#define ARM_U2_SKY_POS    -0.31f
#define ARM_DJ_SKY_POS    -210.0f

//  底层取块
#define ARM_U1_LOW_POS      0.64f
#define ARM_U2_LOW_POS      0.38f
#define ARM_DJ_LOW_POS      -230.0f


//底层放块
#define ARM_U1_LOW1_POS      0.64f
#define ARM_U2_LOW1_POS      0.52f
#define ARM_DJ_LOW1_POS      -240.0f


//  中层取块
#define ARM_U1_MID_POS      0.64f
#define ARM_U2_MID_POS      0.45f
#define ARM_DJ_MID_POS      -140.0f


//中层放块
#define ARM_U1_MID1_POS      0.64f
#define ARM_U2_MID1_POS      0.52f
#define ARM_DJ_MID1_POS      -140.0f



//  高层放块
#define ARM_U1_HIGH_POS     0.90f
#define ARM_U2_HIGH_POS     1.50f
#define ARM_DJ_HIGH_POS     -185.0f



extern volatile uint8_t level_flag;
extern volatile uint8_t Is_pick;
extern volatile uint8_t Is_place;
extern volatile uint8_t Is_store;
extern volatile uint8_t Is_ready;
extern volatile uint8_t Is_reset;
extern volatile uint8_t Is_on;
extern volatile uint8_t Is_open;
extern volatile uint8_t Is_ok;



typedef struct
{
    float ARM_U1_POS;
    float ARM_U2_POS;
    float ARM_DJ_ROS;
}ARM_POS_t;



typedef enum
{
    ARM_STATE_NONE = 0,
    ARM_STATE_READY,       // 准备状态 
    ARM_STATE_LOW,         // 底层取块 
    ARM_STATE_MID,         // 中层取块状态 
    ARM_STATE_SKY,         // 取天空块
    ARM_STATE_KEEP,        //持块状态
    ARM_STATE_STORT,       // 储存状态 
    ARM_STATE_LOW1,        //放一层
    ARM_STATE_MID1,        //放二层
    ARM_STATE_HIGH         //放三层
}ArmState_t;


typedef struct
{
    float start_angle;
    float target;
} ArmInterpolationPoint_t;


typedef struct
{
    volatile ArmState_t state;

    volatile ArmState_t last_state;

    float time;

    float total_time;

    ArmInterpolationPoint_t u1;

    ArmInterpolationPoint_t u2;

    ArmInterpolationPoint_t dj;

    volatile bool running;

    volatile bool finish;

} ArmControl_t;

extern ArmControl_t ArmControl;

void Relay_ON(void);
void Relay_OFF(void);
void Arm_Control_Init(void);
void Arm_State_Update(void);
void Arm_Control_Task(void *argument);
void Arm_Motor_Enable(void);
void Arm_Motor_Disable(void);
void Arm_Receive(FDCAN_RxHeaderTypeDef Rxheader, uint8_t *Rx_data);

void Arm_Transmit(void);





#endif
