#ifndef _COMMON
#define _COMMON

#include "main.h"

typedef enum
{
	Power_On=0,
	Power_Normal=50,
	Power_Off=100
}Power_State_t;

void SCAP_Init(void);
void SCAP_Main(void);
void ADC_DMA_Callback(void);
void ADC_Inject_Callback(uint8_t ADC_Num);
void TIM17_Callback(void);
void Limit_Min_Max(float *Data,float Min,float Max);
void Para_Init(void);
void Get_Value(void);
void PID_Calc(void);
void Update_PWM(void);
void Set_Compare(void);
void Set_Mosfet_Driver(void);
void State_Machine(void);
void Vofa_Send(void);
void CAN_Filter_Init(void);
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan);
void CAN_Send(void);

#endif
