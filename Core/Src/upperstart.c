#ifndef __UPEERSTART_H__
#define __UPEERSTART_H__
#include "upperservo.h"
#include "retarget.h"
#include "param.h"
#include "cmsis_os.h"
#include "tim.h"
#include "mi_motor.h"
void StartDefaultTask(void const * argument)
{
  /* USER CODE BEGIN StartDefaultTask */

  /* initialize motors incluing motortypes、xyz of motors、PID of motors*/
  
  gantry_Motor_init(); 
  osDelay(30);
  init_cybergear(&mi_motor[0], 0x7F, Motion_mode);
  
  set_zeropos_cybergear(&mi_motor[0]);
  osDelay(30);
  HAL_UART_Receive_IT(&huart3, usart3_rx, 1);
  HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_3);
osDelay(30);
  RetargetInit(&huart2);
  osDelay(30);
  /* Infinite loop */
  for(;;)
  {
  //  for(int i = 0; i < 5; i++)
  //   {
  //     float err = hDJI[i].posPID.cur_error; // Get the current angle of each motor
  //     float out = hDJI[i].speedPID.output; // Get the output of the position PID
  //     float ref = hDJI[i].posPID.ref;
  //     printf("%f,%f,%f\n",err,out,ref); // Print the current angle of motor 4 to USART2
  //   }
  //   //  for(uint16_t i=0;i<4;i++)
    //  {
    //   if(hDJI[i].posPID.cur_error<2000&&hDJI[i].posPID.cur_error>-2000)
    //   {
    //     hDJI[i].speedPID.outputMax=2000;

    //   }


    //  }
    // float err = hDJI[1].posPID.cur_error; // Get the current angle of each motor
    //   float out = hDJI[1].speedPID.output; // Get the output of the position PID
    //   float ref = hDJI[1].posPID.output;
    //   printf("%f,%f,%f\n",err,out,ref); // Print the current angle of motor 4 to USART2
    // printf("\n");
   // printf("%f\n",mi_motor[0].Angle); 
    osDelay(20);
  }
  /* USER CODE END StartDefaultTask */
}



#endif // !1
