#include "common.h"
#include "pid.h"

#include "adc.h"
#include "can.h"
#include "dma.h"
#include "hrtim.h"
#include "string.h"
#include "tim.h"
#include "usart.h"

#define Max_VScap 21.4f
#define Max_VBus 27.0f
#define Max_IScap 20.0f
#define VMonitorIn_ThDown 18.0f
#define VMonitorIn_ThOn 20.0f

#define PWM_PERIOD 46080

// #define HARDWARE_VERDION1

#ifdef HARDWARE_VERDION1
#define K_ILoad 186.18f
#define K_IBatt 372.36f
#else
#define K_ILoad 186.18f * 2.0f
#define K_IBatt 372.36f * 2.0f
#endif

#define K_IScap 93.09f
#define K_VMonitorIn 110.000f
#define K_VBuckOut 110.000f
#define K_VBus 110.000f
#define K_VScap 110.000f
#define b_ILoad 0.0f
#define b_IBatt 0.0f
#define b_IScap 2048.0f
#define b_VMonitorIn 0.0f
#define b_VBuckOut 0.0f
#define b_VBus 0.0f
#define b_VScap 0.0f

#define ADC1_SIZE 3
#define ADC2_SIZE 2

#define Filter_Factor 0.33f

uint8_t VScap_Low = 0;

uint8_t can_rxdata[8];
uint8_t can_txdata[8];
CAN_TxHeaderTypeDef hcan_txheader;

float VMonitorIn = 0.0f; // 母线电管侧电压
float VBuckOut = 0.0f;   // 电容继电器前电压
float VBus = 0.0f;       // 母线电压
float VScap = 0.0f;      // 电容端电压

float ILoad = 0.0f;   // 底盘电流
float IScap = 0.0f;   // 电容充放电流
float IBatt = 0.0f;   // 电管侧输入电流
float IBuckIn = 0.0f; // 半桥高压侧电流

float VBus_REF = 0.0f;  // 半桥高压侧恒压环参考值
float VScap_REF = 0.0f; // BUCK输出恒压环参考值
float VBuckOut_REF = 0.0f;
float IBuckIn_REF = 0.0f; // BUCK输入恒流环参考值
float IScap_REF = 0.0f;   // BUCK输出恒流环参考值

float POWER_BATT = 0.0f;            // 电池输出功率
float POWER_LOAD = 0.0f;            // 底盘功率本地测量值
float POWER_SCAP = 0.0f;            // 电容功率
float POWER_EFFECIENCY = 0.0f;      // 控制器总效率
float POWER_BUCK_EFFECIENCY = 0.0f; // BUCK环节效率
float CAP_SOC = 0.0f;

Power_State_t Power_State = Power_On;
float POWER_LIMIT = 0, POWER_SENS = 0, POWER_BUFFER = 0,
      POWER_MEASUREMENT_ERRO = 0;

float PWM_DUTY = 0.0f;
float PWM = PWM_PERIOD * 0.98f;

uint16_t ADC1_Buf[ADC1_SIZE], ADC2_Buf[ADC2_SIZE];
uint16_t ADC1_Inject[1], ADC2_Inject[1];
float ADC1_Data[ADC1_SIZE + 1], ADC2_Data[ADC2_SIZE + 1];

pid_typedef PID_IScap, PID_IBuckIn, PID_VBuckOut, PID_VBus; //,PID_VScap;

void SCAP_Init(void) {
  Para_Init();

  // ADC Cali
  HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED);
  HAL_Delay(100);
  HAL_ADCEx_Calibration_Start(&hadc2, ADC_SINGLE_ENDED);
  HAL_Delay(100);

  // ADC Start
  HAL_ADC_Start_DMA(&hadc1, (uint32_t *)ADC1_Buf, ADC1_SIZE);
  HAL_ADC_Start_DMA(&hadc2, (uint32_t *)ADC2_Buf, ADC2_SIZE);
  hdma_adc1.Instance->CCR &= ~DMA_IT_HT;
  __HAL_ADC_ENABLE_IT(&hadc1, ADC_IT_JEOC);
  __HAL_ADC_ENABLE_IT(&hadc2, ADC_IT_JEOC);
  HAL_ADCEx_InjectedStart(&hadc1);
  HAL_ADCEx_InjectedStart(&hadc2);

  // GPIO Init
  HAL_GPIO_WritePin(DRVOFF_GPIO_Port, DRVOFF_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(RELAY_GPIO_Port, RELAY_Pin, GPIO_PIN_RESET);

  // Set Hrtim CMP
  HRTIM1->sTimerxRegs[HRTIM_TIMERINDEX_TIMER_B].CMP1xR = PWM_PERIOD * 0.98f;
  HRTIM1->sTimerxRegs[HRTIM_TIMERINDEX_TIMER_B].CMP2xR = PWM_PERIOD * 0.98f / 2;
  HRTIM1->sTimerxRegs[HRTIM_TIMERINDEX_TIMER_B].CMP3xR = PWM_PERIOD * 0.1f;

  // Start Hrtim
  HAL_HRTIM_WaveformCounterStart(&hhrtim1, HRTIM_TIMERID_MASTER);
  HAL_HRTIM_WaveformCounterStart(&hhrtim1, HRTIM_TIMERID_TIMER_B);
  HAL_HRTIM_WaveformOutputStart(&hhrtim1, HRTIM_OUTPUT_TB1 | HRTIM_OUTPUT_TB2);

  HAL_Delay(500);
  // Start TIM17
  HAL_TIM_Base_Start_IT(&htim17);
}

uint32_t Main_Cnt = 0;
void SCAP_Main(void) {
  State_Machine();

  if (Main_Cnt % 10 == 0)
    CAN_Send();
  if (Main_Cnt % 20 == 0)
    Vofa_Send();
  if (Main_Cnt % 200 == 0)
    HAL_GPIO_TogglePin(MOD_GPIO_Port, MOD_Pin);

  Main_Cnt++;
  HAL_Delay(0);
}

void State_Machine(void) {
  switch (Power_State) {
  case Power_On: {
    VBuckOut_REF = VScap;
    if (VBuckOut - VScap < 2.0f && VBuckOut - VScap > -2.0f) {
      Power_State = Power_Normal;
      VBuckOut_REF = Max_VScap;
      HAL_GPIO_WritePin(RELAY_GPIO_Port, RELAY_Pin, GPIO_PIN_SET);
    }
    break;
  }
  case Power_Normal: {
    if (VMonitorIn < VMonitorIn_ThDown) {
      Power_State = Power_Off;
      HAL_GPIO_WritePin(RELAY_GPIO_Port, RELAY_Pin, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(DRVOFF_GPIO_Port, DRVOFF_Pin, GPIO_PIN_SET);
    }
    break;
  }
  case Power_Off: {
    if (VMonitorIn > VMonitorIn_ThOn) {
      Power_State = Power_On;
    }
    break;
  }
  }
}

uint32_t TIM17_Cnt = 0;
void TIM17_Callback(void) {
  Get_Value();
  PID_Calc();
  Update_PWM();
  Limit_Min_Max(&PWM, PWM_PERIOD * 0.05f, PWM_PERIOD * 0.98f);
  Set_Compare();
  Set_Mosfet_Driver();

  TIM17_Cnt++;
}

void Para_Init(void) {
  pid_init(&PID_IScap, 90.0f, 20, 0);
  pid_init(&PID_IBuckIn, 100.0f, 50.f, 0);
  pid_init(&PID_VBus, 100.0f, 20.0f, 0);
  //	pid_init(&PID_VScap,100.0f,20.0f,0);
  pid_init(&PID_VBuckOut, 100.0f, 20.0f, 0);

  PWM = 44000;
  POWER_LIMIT = 40; // Default 40W
  IScap_REF = Max_IScap;
  IBuckIn_REF = POWER_LIMIT / VMonitorIn - ILoad;
  //	VScap_REF=Max_VScap;
  VBus_REF = Max_VBus;
}

void Get_Value(void) {
  IBatt = (ADC2_Data[ADC2_SIZE] - b_IBatt) / K_IBatt;
  ILoad = (ADC1_Data[ADC1_SIZE] - b_ILoad) / K_ILoad;
  IScap = (ADC1_Data[0] - b_IScap) / K_IScap;
  IBuckIn = IBatt - ILoad - 0.05f;
  VMonitorIn = (ADC1_Data[2] - b_VMonitorIn) / K_VMonitorIn;
  VBuckOut = (ADC2_Data[0] - b_VScap) / K_VScap;
  VScap = (ADC1_Data[1] - b_VBuckOut) / K_VBuckOut;
  VBus = (ADC2_Data[1] - b_VBus) / K_VBus;

  POWER_LOAD = VBus * ILoad;
  POWER_SCAP = VScap * IScap;
  POWER_BATT = VMonitorIn * IBatt;
  CAP_SOC = VScap * VScap / Max_VScap / Max_VScap;
  //	POWER_EFFECIENCY=100*(POWER_LOAD+POWER_SCAP)/POWER_BATT;
  //	POWER_BUCK_EFFECIENCY=100*POWER_SCAP/VMonitorIn/IBuckIn;
}

void PID_Calc(void) {
  IBuckIn_REF = POWER_LIMIT / VMonitorIn - ILoad;

  if (VScap < 6.0f)
    VScap_Low = 1;
  else if (VScap > 10.0f)
    VScap_Low = 0;
  if (VScap_Low == 1 && IBuckIn_REF < 0.0f)
    IBuckIn_REF = 0.0f;

  if (IBuckIn_REF > 10.0f)
    IBuckIn_REF = 10.0f;
  else if (IBuckIn_REF < -30.0f)
    IBuckIn_REF = -30.0f;

  pid_increment_type(&PID_IBuckIn, IBuckIn_REF, IBuckIn);
  pid_increment_type(&PID_IScap, IScap_REF, IScap);
  //	pid_increment_type(&PID_VScap,VScap_REF,VScap);
  pid_increment_type(&PID_VBus, VBus_REF, VBus);
  pid_increment_type(&PID_VBuckOut, VBuckOut_REF, VBuckOut);
}

void Update_PWM(void) {
  if (Power_State == Power_On) {
    PWM += PID_VBuckOut.output;
  } else {
    if (IBuckIn_REF > 0) {
      PWM += PID_IBuckIn.output < PID_VBuckOut.output
                 ? (PID_IBuckIn.output < PID_IScap.output ? PID_IBuckIn.output
                                                          : PID_IScap.output)
                 : (PID_VBuckOut.output < PID_IScap.output ? PID_VBuckOut.output
                                                           : PID_IScap.output);
    } else {
      PWM += PID_IBuckIn.output;
    }
  }
}

void Set_Compare(void) {
  HRTIM1->sTimerxRegs[HRTIM_TIMERINDEX_TIMER_B].CMP1xR = PWM;
  if (PWM > PWM_PERIOD * 0.5f)
    HRTIM1->sTimerxRegs[HRTIM_TIMERINDEX_TIMER_B].CMP2xR = PWM_PERIOD * 0.1f;
  else
    HRTIM1->sTimerxRegs[HRTIM_TIMERINDEX_TIMER_B].CMP2xR = PWM_PERIOD * 0.6f;
  HRTIM1->sTimerxRegs[HRTIM_TIMERINDEX_TIMER_B].CMP3xR = PWM / 2;
}

void Set_Mosfet_Driver(void) {
  switch (Power_State) {
  case (Power_On):
  case (Power_Normal): {
    HAL_GPIO_WritePin(DRVOFF_GPIO_Port, DRVOFF_Pin, GPIO_PIN_RESET);
    break;
  }
  case (Power_Off): {
    HAL_GPIO_WritePin(DRVOFF_GPIO_Port, DRVOFF_Pin, GPIO_PIN_SET);
    break;
  }
  }
}

#define VOFA_NUM 11
uint8_t Vofa_Buffer[VOFA_NUM * 4 + 4];
float Vofa_Temp[VOFA_NUM];
void Vofa_Send(void) {
  Vofa_Temp[0] = POWER_SENS;
  Vofa_Temp[1] = POWER_BATT;
  Vofa_Temp[2] = VMonitorIn;
  Vofa_Temp[3] = IBatt;
  Vofa_Temp[4] = POWER_LOAD;
  Vofa_Temp[5] = VScap;
  PWM_DUTY = PWM / 46080.0f;
  Vofa_Temp[6] = PWM_DUTY * 100;
  Vofa_Temp[7] = ILoad;
  Vofa_Temp[8] = VBus;
  Vofa_Temp[9] = Power_State;
  Vofa_Temp[10] = POWER_SCAP;
  memcpy(Vofa_Buffer, (uint8_t *)&Vofa_Temp, sizeof(Vofa_Temp));
  Vofa_Buffer[VOFA_NUM * 4] = 0x00;
  Vofa_Buffer[VOFA_NUM * 4 + 1] = 0x00;
  Vofa_Buffer[VOFA_NUM * 4 + 2] = 0x80;
  Vofa_Buffer[VOFA_NUM * 4 + 3] = 0x7f;
  HAL_USART_Transmit(&husart1, (uint8_t *)Vofa_Buffer, VOFA_NUM * 4 + 4, 0xFF);
}

CAN_FilterTypeDef hcan_FilterConfig;
void CAN_Filter_Init(void) {
  hcan_FilterConfig.FilterBank = 0;
  hcan_FilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
  hcan_FilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
  hcan_FilterConfig.FilterIdHigh = (((uint32_t)0x140 << 21) & 0xffff0000) >> 16;
  hcan_FilterConfig.FilterIdLow =
      (((uint32_t)0x140 << 21) | CAN_ID_STD) & 0xffff;
  hcan_FilterConfig.FilterMaskIdHigh =
      (((uint32_t)0x140 << 21) & 0xffff0000) >> 16;
  hcan_FilterConfig.FilterMaskIdLow =
      (((uint32_t)0x140 << 21) | CAN_ID_STD) & 0xffff;
  hcan_FilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;
  hcan_FilterConfig.FilterActivation = ENABLE;
  HAL_CAN_ConfigFilter(&hcan, &hcan_FilterConfig);
  HAL_CAN_Start(&hcan);
  HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING);
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) {
  CAN_RxHeaderTypeDef hcan_rxheader;
  __HAL_CAN_CLEAR_FLAG(hcan, CAN_IT_RX_FIFO0_MSG_PENDING);
  HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &hcan_rxheader, can_rxdata);
  if (hcan_rxheader.StdId == 0x140) {
    POWER_LIMIT =
        (((uint16_t)can_rxdata[0] | ((uint16_t)can_rxdata[1] << 8)) / 100.f);
    POWER_SENS =
        ((uint16_t)can_rxdata[2] | ((uint16_t)can_rxdata[3] << 8)) / 100.f;
    POWER_BUFFER =
        ((uint16_t)can_rxdata[4] | ((uint16_t)can_rxdata[5] << 8)) / 100.f;

    if (POWER_LIMIT < 30.0f)
      POWER_LIMIT = 30.0f;
    else if (POWER_LIMIT > 140.0f)
      POWER_LIMIT = 140.0f;

    // 	if(POWER_SENS>POWER_LIMIT)POWER_MEASUREMENT_ERRO+=0.1f;
    // 	if(POWER_SENS<POWER_LIMIT-5.0f&&VScap<Max_VScap-2.0f)POWER_MEASUREMENT_ERRO-=0.02f;
    // 	if(POWER_MEASUREMENT_ERRO>10.0f)POWER_MEASUREMENT_ERRO=10.0f;
    // 	if(POWER_MEASUREMENT_ERRO<-10.0f)POWER_MEASUREMENT_ERRO=-10.0f;
    // 	POWER_LIMIT-=POWER_MEASUREMENT_ERRO;
  }
}

void CAN_Send(void) {
  uint32_t mailBoxId = 0;

  if (CAP_SOC < 0.05f)
    CAP_SOC = 0.05f;
  hcan_txheader.StdId = 0x130;
  hcan_txheader.IDE = CAN_ID_STD;
  hcan_txheader.RTR = CAN_RTR_DATA;
  hcan_txheader.DLC = 8;
  can_txdata[0] = (uint8_t)((uint16_t)(CAP_SOC * 32768));
  can_txdata[1] = (uint8_t)((uint16_t)(CAP_SOC * 32768) >> 8);
  can_txdata[2] = (uint8_t)((uint16_t)(POWER_LOAD * 100));
  can_txdata[3] = (uint8_t)((uint16_t)(POWER_LOAD * 100) >> 8);
  can_txdata[4] = 0;
  can_txdata[5] = 0;
  can_txdata[6] = 0;
  can_txdata[7] = 0;

  HAL_CAN_AddTxMessage(&hcan, &hcan_txheader, can_txdata, &mailBoxId);
}

uint32_t ADC_DMA_Cnt = 0;
void ADC_DMA_Callback(void) {
  uint8_t i;

  for (i = 0; i < ADC1_SIZE; i++)
    ADC1_Data[i] =
        ADC1_Data[i] * (1 - Filter_Factor) + ADC1_Buf[i] * Filter_Factor;
  for (i = 0; i < ADC2_SIZE; i++)
    ADC2_Data[i] =
        ADC2_Data[i] * (1 - Filter_Factor) + ADC2_Buf[i] * Filter_Factor;

  ADC_DMA_Cnt++;
}

uint32_t ADC1_Inject_Cnt = 0, ADC2_Inject_Cnt = 0;
void ADC_Inject_Callback(uint8_t ADC_Num) {
  if (ADC_Num == 1) {
    ADC1_Inject[0] = ADC1->JDR1;
    ADC1_Data[ADC1_SIZE] = ADC1_Data[ADC1_SIZE] * (1 - Filter_Factor) +
                           ADC1_Inject[0] * Filter_Factor;
    ADC1_Inject_Cnt++;
  } else if (ADC_Num == 2) {
    ADC2_Inject[0] = ADC2->JDR1;
    ADC2_Data[ADC2_SIZE] = ADC2_Data[ADC2_SIZE] * (1 - Filter_Factor) +
                           ADC2_Inject[0] * Filter_Factor;
    ADC2_Inject_Cnt++;
  }
}

void Limit_Min_Max(float *Data, float Min, float Max) {
  if (*Data < Min)
    *Data = Min;
  else if (*Data > Max)
    *Data = Max;
}
