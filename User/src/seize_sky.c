#include "seize_sky.h"

#include "DJmotor.h"

#define ARM_INTERPOLATION_DT    0.002f     
#define ARM_EPSILON             0.0001f
#define ARM_MOVE_TIME           3.0f


volatile uint8_t level_flag=0;
volatile uint8_t Is_pick=0;
volatile uint8_t Is_place=0;
volatile uint8_t Is_store=0;
volatile uint8_t Is_ready=0;
volatile uint8_t Is_reset=0;
ArmControl_t ArmControl;




void Relay_ON(void)
{
    HAL_GPIO_WritePin(GPIOA,GPIO_PIN_9,GPIO_PIN_SET);
}

void Relay_OFF(void)
{
    HAL_GPIO_WritePin(GPIOA,GPIO_PIN_9,GPIO_PIN_RESET);
}



//时间归一化,计算轨迹设定角度
static float Target_Quintic_Interpolation(float start_angle,float target,float time,float total_time)
{
    float t;
    float t2;
    float t3;
    float t4;
    float t5;
    float s;
    if (total_time<=0)
    {
        return target;
    }

    t=time/total_time;

    if(t<=0.0f)
    {
        t=0.0f;
    }
    else if(t>=1.0f)
    {
        t=1.0;
    }


    t2=t*t;
    t3=t2*t;
    t4=t3*t;
    t5=t4*t;

    s=10.0f*t5-15.0f*t4+6.0f*t3;

    return start_angle+(target-start_angle)*s;

}

//开始轨迹计算
static void Arm_Interpolation_Start(float u1_target,float u2_target, float dj_target, float move_time)
{
    ArmControl.u1.start_angle = Unitree_motors[0].data.position;
    ArmControl.u2.start_angle = Unitree_motors[1].data.position;
    ArmControl.dj.start_angle = DJmotor[0].valNow.angle_deg;

    ArmControl.u1.target = u1_target;
    ArmControl.u2.target = u2_target;
    ArmControl.dj.target = dj_target;

    ArmControl.time = 0.0f;
    ArmControl.total_time = move_time;

    ArmControl.running = true;
    ArmControl.finish = false;

}


//轨迹点更新
static void Arm_Interpolation_Update(void)
{
    if (ArmControl.running == false)
    {
        return ;
    }

   

    Unitree_motors[0].cmd.position =Target_Quintic_Interpolation(ArmControl.u1.start_angle,ArmControl.u1.target,ArmControl.time,ARM_MOVE_TIME);
    Unitree_motors[1].cmd.position = Target_Quintic_Interpolation(ArmControl.u2.start_angle,ArmControl.u2.target,ArmControl.time,ARM_MOVE_TIME);
    DJmotor[0].valSet.angle_deg =  Target_Quintic_Interpolation(ArmControl.dj.start_angle,ArmControl.dj.target,ArmControl.time,ARM_MOVE_TIME);

    ArmControl.time += ARM_INTERPOLATION_DT;

    if (ArmControl.time >= ARM_MOVE_TIME)
    {
        ArmControl.time = ARM_MOVE_TIME;
        Unitree_motors[0].cmd.position = ArmControl.u1.target;
        Unitree_motors[1].cmd.position = ArmControl.u2.target;
        DJmotor[0].valSet.angle_deg = ArmControl.dj.target;
        ArmControl.running = false;
        ArmControl.finish = true;
    }

}



//机械臂位置、模式初始化
void Arm_Control_Init(void)
{
    ArmControl.state=ARM_STATE_NONE;
    ArmControl.last_state=ARM_STATE_NONE;

    ArmControl.time = 0.0f;
    ArmControl.total_time = ARM_MOVE_TIME;

    ArmControl.running = false;
    ArmControl.finish = true;

    ArmControl.u1.start_angle = 0.0f;
    ArmControl.u1.target = ARM_U1_START_POS;

    ArmControl.u2.start_angle = 0.0f;
    ArmControl.u2.target = ARM_U2_START_POS;

    ArmControl.dj.start_angle = 0.0f;
    ArmControl.dj.target = ARM_DJ_START_POS;

}



// void Arm_Control_SetState(ArmState_t state)
// {
//     if (state < ARM_STATE_NONE || state > ARM_STATE_HIGH)
//     {
//         return;
//     }
//      ArmControl.state = state;
// }


// ArmState_t Arm_Control_GetState(void)
// {
//     return ArmControl.state;
// }





//其中一种状态，其他状态最后根据具体情况添加
//准备状态
static void Arm_Ready_Process(void)
{
    if (ArmControl.running == false &&
        ArmControl.finish == false)
    {
        Arm_Interpolation_Start(ARM_U1_READY_POS,ARM_U2_READY_POS,ARM_DJ_READY_POS,ARM_MOVE_TIME);
    }
}

//起始状态
static void Arm_NONE_Process(void)
{
    if (ArmControl.running == false &&
        ArmControl.finish == false)
    {
        Arm_Interpolation_Start(ARM_U1_START_POS,ARM_U2_START_POS,ARM_DJ_START_POS,ARM_MOVE_TIME);
    }
}

//底层取块
static void Arm_LOW_Process(void)
{
    if (ArmControl.running == false &&
        ArmControl.finish == false)
    {
        Arm_Interpolation_Start(ARM_U1_LOW_POS,ARM_U2_LOW_POS,ARM_DJ_LOW_POS,ARM_MOVE_TIME);
    }
}

//二层取块
static void Arm_MID_Process(void)
{
    if (ArmControl.running == false &&
        ArmControl.finish == false)
    {
        Arm_Interpolation_Start(ARM_U1_MID_POS,ARM_U2_MID_POS,ARM_DJ_MID_POS,ARM_MOVE_TIME);
    }
}

//取天空块
static void Arm_SKY_Process(void)
{
    if (ArmControl.running == false &&
        ArmControl.finish == false)
    {
        Arm_Interpolation_Start(ARM_U1_SKY_POS,ARM_U2_SKY_POS,ARM_DJ_SKY_POS,ARM_MOVE_TIME);
    }
}

//储存状态
static void Arm_STORT_Process(void)
{
    if (ArmControl.running == false &&
        ArmControl.finish == false)
    {
        Arm_Interpolation_Start(ARM_U1_STORE_POS,ARM_U2_STORE_POS,ARM_DJ_STORE_POS,ARM_MOVE_TIME);
    }
}

//底层放块
static void Arm_LOW1_Process(void)
{
    if (ArmControl.running == false &&
        ArmControl.finish == false)
    {
        Arm_Interpolation_Start(ARM_U1_LOW1_POS,ARM_U2_LOW1_POS,ARM_DJ_LOW1_POS,ARM_MOVE_TIME);
    }
}

//二层放块
static void Arm_MID1_Process(void)
{
    if (ArmControl.running == false &&
        ArmControl.finish == false)
    {
        Arm_Interpolation_Start(ARM_U1_MID1_POS,ARM_U2_MID1_POS,ARM_DJ_MID1_POS,ARM_MOVE_TIME);
    }
}


//三层放块
static void Arm_HIGH_Process(void)
{
    if (ArmControl.running == false &&
        ArmControl.finish == false)
    {
        Arm_Interpolation_Start(ARM_U1_HIGH_POS,ARM_U2_HIGH_POS,ARM_DJ_HIGH_POS,ARM_MOVE_TIME);
    }
}


//持块状态
static void Arm_KEEP_Process(void)
{
    if (ArmControl.running == false &&
        ArmControl.finish == false)
    {
        Arm_Interpolation_Start(ARM_U1_KEEP_POS,ARM_U2_KEEP_POS,ARM_DJ_KEEP_POS,ARM_MOVE_TIME);
    }
}





void Arm_State_Update(void)
{

    if (Is_reset)
    {
        if (ArmControl.state!= ARM_STATE_NONE)
        {
            ArmControl.state = ARM_STATE_NONE;
        }
            Is_pick=0;
            Is_place=0;
            Is_store=0;
            Is_reset=0;
            Is_ready=0;
        return;
    }



    if (Is_ready)
    {
        if (ArmControl.state != ARM_STATE_READY)
        {
            ArmControl.state = ARM_STATE_READY;
 
        }
            Is_pick=0;
            Is_place=0;
            Is_store=0;
            Is_reset=0;
            Is_ready=0;
        return;
    }



    if (Is_store)
    {
        if (ArmControl.state != ARM_STATE_STORT)
        {
            ArmControl.state = ARM_STATE_STORT;
        }
        Is_pick=0;
        Is_place=0;
        Is_store=0;
        Is_ready=0;
        Is_reset=0;
        return;
    }


    if (Is_pick==1&&Is_place==0)
    {
        ArmState_t new_state;

        switch (level_flag)
        {
            case 0:
                new_state = ARM_STATE_SKY;
                break;

            case 1:
                new_state = ARM_STATE_LOW;
                break;

            case 2:
                new_state = ARM_STATE_MID;
                break;

            default:
                new_state = ARM_STATE_MID;
                break;
        }


        if (ArmControl.state != new_state)
        {
            ArmControl.state = new_state;

        }
        Is_pick=0;
        Is_place=0;
        Is_store=0;
        Is_ready=0;
        Is_reset=0;

        return;
    }


   
    if (Is_place==1&&Is_pick==0)
    {
        ArmState_t new_state;

        switch (level_flag)
        {
            case 1:
                new_state = ARM_STATE_LOW1;
                break;

            case 2:
                new_state = ARM_STATE_MID1;
                break;

            case 3:
                new_state = ARM_STATE_HIGH;
                break;

            default:
            new_state = ARM_STATE_LOW1;
                break;
        }


        if (ArmControl.state != new_state)
        {
            ArmControl.state = new_state;
           
        }

        Is_pick=0;
        Is_place=0;
        Is_store=0;
        Is_ready=0;
        Is_reset=0;

        return;
    }


    Is_pick=0;
    Is_place=0;
    Is_store=0;
    Is_ready=0;
    Is_reset=0;
}





//状态更新
void Arm_Control_Task(void *argument)
{
    (void)argument;
    for (;;)
    {
        osDelay(2);
         if (ArmControl.state != ArmControl.last_state)
        {
             ArmControl.running = false;
            ArmControl.finish = false;
             ArmControl.last_state = ArmControl.state;
        }
    //     switch (ArmControl.state)
    //     {
    //         case ARM_STATE_NONE:
    //             Arm_NONE_Process();
    //             break;


    //         case ARM_STATE_READY:

    //             Arm_Ready_Process();

    //             break;


    //         case ARM_STATE_STORT:

    //             Arm_STORT_Process();
    //             break;


    //         case ARM_STATE_KEEP:
    //             Arm_KEEP_Process();
    //             break;

    //         case ARM_STATE_LOW:
    //             Arm_LOW_Process();
    //             break;
            

    //         case ARM_STATE_LOW1:
    //             Arm_LOW1_Process();
    //             break;


    //         case ARM_STATE_MID:
    //             Arm_MID_Process();
    //             break;



    //         case ARM_STATE_MID1:
    //             Arm_MID1_Process();
    //             break;


    //         case ARM_STATE_HIGH:
    //             Arm_HIGH_Process();
    //             break;

    //         case ARM_STATE_SKY:
    //             Arm_SKY_Process();
    //             break;


    //         default:
    //         ArmControl.running = false;
    //             ArmControl.finish = true;

    //             break;
    //     }
    //     if (ArmControl.running == true)
    //     {
    //         Arm_Interpolation_Update();
    //     }
    }
}





void Arm_Motor_Enable(void)
{
    Unitree_motors[0].enable = true;
    Unitree_motors[1].enable = true;

    DJmotor[0].Begin = true;

    DJmotor[0].MODE_Set = DJ_Position;
}


void Arm_Motor_Disable(void)
{
    Unitree_motors[0].enable = false;
    Unitree_motors[1].enable = false;

    DJmotor[0].Begin = false;

    DJmotor[0].MODE_Set = DJ_Disable;
}



void Arm_Receive(FDCAN_RxHeaderTypeDef Rxheader, uint8_t *Rx_data)
{
    if (Rxheader.IdType == FDCAN_EXTENDED_ID)
           {
               switch (Rxheader.Identifier)
               {

           

                   case 0x01020211U:

                       if (Rx_data[0] == 'E')
                       {
                           Arm_Motor_Enable();
                       }
                       else if (Rx_data[0] == 'D')
                       {
                           Arm_Motor_Disable();
                       }

                       break;


                   //取块准备
                   case 0x01020301U:

                       if (Rx_data[0] == 'P')
                       {
                           Is_pick  = 0U;
                           Is_place = 0U;
                           Is_store = 0U;
                           Is_reset = 0U;

                           Is_ready = 1U;
                       }

                       break;

                   //位置调节
                   case 0x01020302U:

                       if (Rx_data[0] <= 3U)
                       {
                           level_flag = Rx_data[0];
                       }

                       break;


                   //存块
                   case 0x01020303U:

                       if (Rx_data[0] == 'S')
                       {
                           Is_pick  = 0U;
                           Is_place = 0U;
                           Is_ready = 0U;
                           Is_reset = 0U;

                           Is_store = 1U;
                       }

                       break;


                   //取存块
                   case 0x01020304U:

                       if (Rx_data[0] == 'G')
                       {
                           Is_place = 0U;
                           Is_store = 1U;
                           Is_ready = 0U;
                           Is_reset = 0U;

                           Is_pick = 0U;
                       }

                       break;


                   //放块
                   case 0x01020305U:

                       if (Rx_data[0] == 'R')
                       {
                           Is_pick  = 0U;
                           Is_store = 0U;
                           Is_ready = 0U;
                           Is_reset = 0U;

                           Is_place = 1U;
                       }

                       break;


                   //返回零位
                   case 0x01020306U:

                       if (Rx_data[0] == 'Z')
                       {
                           Is_pick  = 0U;
                           Is_place = 0U;
                           Is_store = 0U;
                           Is_ready = 0U;

                           Is_reset = 1U;
                       }

                       break;

                   //取块开始
                   case 0x01020307U:

                       if (Rx_data[0] == 'T')
                       {
                           Is_place = 0U;
                           Is_store = 0U;
                           Is_ready = 0U;
                           Is_reset = 0U;

                           Is_pick = 1U;
                       }

                       break;


                   //放块准备
                   case 0x01020308U:

                       if (Rx_data[0] == 'F')
                       {
                           Is_pick  = 0U;
                           Is_place = 0U;
                           Is_store = 0U;
                           Is_reset = 0U;

                           Is_ready = 1U;
                       }

                       break;


                   default:

                       break;
               }
           }

}
