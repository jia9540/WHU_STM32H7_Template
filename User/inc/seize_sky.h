#ifndef SEIZE_SKY_H
#define SEIZE_SKY_H


#include <stdbool.h>
#include <stdint.h>
#include "main.h"
#include "UnitreeMotor.h"
#include "DJmotor.h"
#include "cmsis_os.h"
#include "gpio.h"


//  初始状态
#define ARM_U1_START_POS    0.088f
#define ARM_U2_START_POS    0.863f
#define ARM_DJ_START_POS    0.0f


//  准备状态
#define ARM_U1_READY_POS    0.832f
#define ARM_U2_READY_POS    1.3f
#define ARM_DJ_READY_POS    0.0f


//  持块状态
#define ARM_U1_KEEP_POS    0.832f
#define ARM_U2_KEEP_POS    1.003f
#define ARM_DJ_KEEP_POS    0.0f


//  存贮状态
#define ARM_U1_STORE_POS    0.832f
#define ARM_U2_STORE_POS    1.003f
#define ARM_DJ_STORE_POS    0.0f

//取天空块
#define ARM_U1_SKY_POS    0.82f
#define ARM_U2_SKY_POS    0.645f
#define ARM_DJ_SKY_POS    0.0f

//  底层取块
#define ARM_U1_LOW_POS      0.832f
#define ARM_U2_LOW_POS      1.003f
#define ARM_DJ_LOW_POS      0.0f


//底层放块
#define ARM_U1_LOW1_POS      0.872f
#define ARM_U2_LOW1_POS      1.114f
#define ARM_DJ_LOW1_POS      0.0f


//  中层取块
#define ARM_U1_MID_POS      0.2675f
#define ARM_U2_MID_POS      1.224f
#define ARM_DJ_MID_POS      0.0f


//中层放块
#define ARM_U1_MID1_POS      0.27f
#define ARM_U2_MID1_POS      1.6f
#define ARM_DJ_MID1_POS      0.0f



//  高层放块
#define ARM_U1_HIGH_POS     0.878f
#define ARM_U2_HIGH_POS     1.89f
#define ARM_DJ_HIGH_POS     0.0f



extern volatile uint8_t level_flag;
extern volatile uint8_t Is_pick;
extern volatile uint8_t Is_place;
extern volatile uint8_t Is_store;
extern volatile uint8_t Is_ready;
extern volatile uint8_t Is_reset;
extern volatile uint8_t Is_on;
extern volatile uint8_t Is_open;



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






#endif
