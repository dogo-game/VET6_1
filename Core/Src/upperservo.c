#include "upperservo.h"
#include "Caculate.h"
#include "param.h"
#include "DJI.h"
#include "decode.h"
#include "cmsis_os.h"
#include "wtr_can.h"
#include "math.h"
#include <stdbool.h>
#define maxspeed        600
#define maxAcceleration 10
gantrystate mygantry;


void upperservotask(void const *argument)
{
    /* USER CODE BEGIN upperservotask */
    mygantry.gantrypos.x = 0;
    mygantry.gantrypos.y = 0;
    osDelay(50);
    TickType_t WheelCorrect_StartTick = xTaskGetTickCount();
    /* Infinite loop */
    for (;;) {
        // TickType_t WheelCorrect_NowTick     = xTaskGetTickCount();
        // TickType_t WheelCorrect_ElapsedTick = WheelCorrect_NowTick - WheelCorrect_StartTick;
        // float timeSec                       = (WheelCorrect_ElapsedTick / (1000.0)); // 获取当前时间/s
        //  VelocityPlanning(0,
        //                      maxspeed ,
        //                     maxAcceleration,
        //                      mygantry.gantrypos.y, timeSec,
        //                      &hDJI[1].posPID.ref);
        // STP_23L_Decode(Rxbuffer_1, &Lidar1);//激光是长轴的
        // STP_23L_Decode(Rxbuffer_2, &Lidar2);//激光是短轴的
        positionServo(mygantry.gantrypos.x, mygantry.Motor_X);
        positionServo(mygantry.gantrypos.y, mygantry.Motor_Y);//
        positionServo(-mygantry.gantrypos.y, mygantry.Motor_Y2);
        // positionServo_lidar(mygantry.gantrypos.x, mygantry.Motor_X, Lidar1); // x轴长
        // positionServo_lidar(mygantry.gantrypos.y, mygantry.Motor_Y, Lidar2); // y轴宽

        positionServo(mygantry.gantrypos.z, mygantry.Motor_Z);
        positionServo(mygantry.gantrypos.yaw, mygantry.Motor_yaw);
 

        CanTransmit_DJI_1234(&hcan1, mygantry.Motor_yaw->speedPID.output,
                             mygantry.Motor_Y->speedPID.output,
                              mygantry.Motor_Y2->speedPID.output,
                             mygantry.Motor_X->speedPID.output);

        CanTransmit_DJI_5678(&hcan1,
                             mygantry.Motor_Z->speedPID.output,
                             0,
                             0,
                             0);
        osDelay(1);
    }
    /* USER CODE END upperservotask */
}

void gantry_Motor_init() // 电机初始化
{
    hDJI[0].motorType = M2006; // yaw
    hDJI[1].motorType = M2006; // y
    hDJI[2].motorType = M2006; // y
    hDJI[3].motorType = M2006; // x
    hDJI[4].motorType = M3508; // z

    mygantry.Motor_X   = &hDJI[3];
    mygantry.Motor_Y   = &hDJI[1];
    mygantry.Motor_Y2   = &hDJI[2];
    mygantry.Motor_Z   = &hDJI[4];
    mygantry.Motor_yaw = &hDJI[0];

    DJI_Init();
    osDelay(1000);
    mygantry.Motor_X->speedPID.outputMax   = hDJI[3].speedPID.outputMax;
    mygantry.Motor_Y->speedPID.outputMax   = hDJI[1].speedPID.outputMax;
     mygantry.Motor_Y2->speedPID.outputMax   = hDJI[1].speedPID.outputMax;
    mygantry.Motor_Z->speedPID.outputMax   = hDJI[4].speedPID.outputMax;
    mygantry.Motor_yaw->speedPID.outputMax = hDJI[0].speedPID.outputMax;

    mygantry.Motor_X->posPID.outputMax   = hDJI[3].posPID.outputMax;
    mygantry.Motor_Y->posPID.outputMax   = hDJI[1].posPID.outputMax;
     mygantry.Motor_Y2->posPID.outputMax   = hDJI[1].posPID.outputMax;
    mygantry.Motor_Z->posPID.outputMax   = hDJI[4].posPID.outputMax;
    mygantry.Motor_yaw->posPID.outputMax = hDJI[0].posPID.outputMax;

    CANFilterInit(&hcan1);
    CANFilterInit(&hcan2);
}

// Function to reset PID parameters
void pid_reset(PID_t *pid, float kp, float ki, float kd)
{
    pid->KP = kp;
    pid->KI = ki;
    pid->KD = kd;
}
