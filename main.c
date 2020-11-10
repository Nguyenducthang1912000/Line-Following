/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "MotorControl.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define ADC1_DR_Address    ((uint32_t)0x4001244C)
#define ADC_Sample_Times	50000// So Lan doc ADC Lay nguong
#define NumberOfSensor 8

#define DiThang 0
#define LechPhai 1
#define LechTrai -1
#define MatLine 2
#define ChuyenLaneTrai -1
#define ChuyenLanePhai 1
#define Coduong 1
#define Khongduong -1

#define ServoStep 0.3;
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define sbi(Reg, Bit) (Reg |= (1<<Bit))
#define cbi(Reg, Bit) (Reg &= ~(1<<Bit))
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

// Sensor Related Variable
volatile uint16_t Sensor_ADC_Value[8];
uint16_t Sensor_Threshold[] = {3800, 3000,3915, 3000, 3000 ,3900 , 3800, 3500};
uint8_t GetThreshold_Flag = 0;
int8_t MaxAngle = 70;
// Motor Related Variable

uint8_t PrevLine = 0;
uint8_t LineDetect = 0;
int8_t CarState = 0;
int8_t ChuyenLaneFlag = 0 ;
int8_t SpecialCaseFlag = 0;
int8_t StraightLineFlag = 0;
float ServoAngle = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM1_Init(void);
static void MX_ADC1_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
/* USER CODE BEGIN PFP */

void GetThreshold();
void Sensor_Convert_A2D();
void Sensor_Print_Thres();
void Sensor_PrintValue();
void Sensor_Print_LineDetect();

void Cuavuongtrai();
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
PUTCHAR_PROTOTYPE
{
	while (LL_USART_IsActiveFlag_TC(USART1)==0)
	{}
	LL_USART_TransmitData8(USART1,(uint8_t)ch);

  	return ch;
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

	uint16_t MaxSpeed = 7200;

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */

  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_AFIO);
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_PWR);

  NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);

  /* System interrupt init*/
  /* MemoryManagement_IRQn interrupt configuration */
  NVIC_SetPriority(MemoryManagement_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),1, 0));
  /* BusFault_IRQn interrupt configuration */
  NVIC_SetPriority(BusFault_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),1, 0));
  /* UsageFault_IRQn interrupt configuration */
  NVIC_SetPriority(UsageFault_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),1, 0));
  /* SVCall_IRQn interrupt configuration */
  NVIC_SetPriority(SVCall_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),1, 0));
  /* DebugMonitor_IRQn interrupt configuration */
  NVIC_SetPriority(DebugMonitor_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),1, 0));
  /* PendSV_IRQn interrupt configuration */
  NVIC_SetPriority(PendSV_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),1, 0));
  /* SysTick_IRQn interrupt configuration */
  NVIC_SetPriority(SysTick_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),1, 0));

  /** NOJTAG: JTAG-DP Disabled and SW-DP Enabled
  */
  LL_GPIO_AF_Remap_SWJ_NOJTAG();

  /* USER CODE BEGIN Init */
  setvbuf(stdin, NULL, _IONBF, 0);
  setvbuf(stdout, NULL, _IONBF, 0);
  setvbuf(stderr, NULL, _IONBF, 0);
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_TIM1_Init();
  MX_ADC1_Init();
  MX_I2C1_Init();
  MX_USART1_UART_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */
  MotorL_EnablePWM();
  MotorR_EnablePWM();
  MotorL_SetPWM(0);
  MotorR_SetPWM(0);
  Servo_SetAngle(2);
  OC2_IT_Setmillis(2.5);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

//  uint32_t Count = LL_TIM_GetCounter(TIM2);
  while (1)
  {

	  Sensor_Convert_A2D();
//	  Sensor_Print_Thres();
//    Sensor_PrintValue();
//	  Sensor_Print_LineDetect();
	  if(Btn1_Flag == 1)
	  {
		  Btn1_Flag = 0;
		  Sensor_Print_LineDetect();
	  }
	  if(Btn2_Flag == 1)
	  	  {
	  		  Btn2_Flag = 0;
	  		  ServoAngle += ServoStep;
	  		  Servo_SetAngle(ServoAngle);
	  		  printf("Servo Angle %g \n", ServoAngle);
	  	  }
	  if(Btn3_Flag == 1)
	  	  {
	  		  Btn3_Flag = 0;
	  		  ServoAngle -= ServoStep;
	  		  Servo_SetAngle(ServoAngle);
	  		printf("Servo Angle %g \n", ServoAngle);
	  	  }

	  if(LineDetect == 0b00011000 ||
		 LineDetect == 0b00011100 ||
		 LineDetect == 0b00111000 ||
	  	 LineDetect == 0b00001000 ||
	  	 LineDetect == 0b00010000)
	  {
		  CarState = DiThang;
		  MotorL_SetPWM(MaxSpeed);
		  MotorR_SetPWM(MaxSpeed);
		  Servo_SetAngle(2);
		  StraightLineFlag = 1;
	  }
	  else if(LineDetect == 0xFF || LineDetect == 0b11011111 || LineDetect == 0b10111111 || LineDetect == 0b10011111)
	  {
		  SpecialCaseFlag = 1;
		  MaxSpeed = 4000;
		  MotorR_SetPWM(1700);
		  MotorL_SetPWM(1700);
		  Servo_SetAngle(4);
	  }
	  else if(LineDetect == 0b10000000 ||
			  LineDetect == 0b11000000 ||
			  LineDetect == 0b11100000 ||
			  LineDetect == 0b01100000 ||
			  LineDetect == 0b01110000 ||
			  LineDetect == 0b00110000)
	  {
		  CarState = LechPhai;
	  }
	  else if (LineDetect == 0b00000001 ||
			   LineDetect == 0b00000011 ||
			   LineDetect == 0b00000111 ||
			   LineDetect == 0b00001110 ||
			   LineDetect == 0b00001100 ||
			   LineDetect == 0b00000110 )
	  {
		  CarState = LechTrai;
	  }
	  else if( LineDetect == 0b00111111 ||
			   LineDetect == 0b00011111 ||
			   LineDetect == 0b00001111 )
	  {
		  ChuyenLaneFlag = ChuyenLanePhai;
	  }
	  else if(LineDetect == 0b11110000 ||
			  LineDetect == 0b11111000 ||
			  LineDetect == 0b11111100 )
	  {
		  ChuyenLaneFlag = ChuyenLaneTrai;
	  }
	  else if(LineDetect == 0)
	  {
		  	  	if (SpecialCaseFlag == 0)
		  	  {
			  if(ChuyenLaneFlag == ChuyenLaneTrai)
			  {
				  printf("chUYEN");
				  ChuyenLaneFlag = 0;
				  MotorR_SetPWM(MaxSpeed);
				  MotorL_SetPWM(MaxSpeed * 0.7);
				  Servo_SetAngle(-31);
				  CarState = LechPhai;
				  printf("HE");
			  }
			  else if (ChuyenLaneFlag == ChuyenLanePhai)
			  {
				  printf("Chuyen lan");
				  ChuyenLaneFlag = 0;
				  MotorL_SetPWM(MaxSpeed);
				  MotorR_SetPWM(MaxSpeed * 0.7);
				  Servo_SetAngle(38);
				  CarState = LechTrai;
				  printf("het chuyen");
			  }
		 	}
		  //Truong hop cua vuong va mat line

		  // Truong hop chuyen lan
		  	  	else if(SpecialCaseFlag == 1 && ChuyenLaneFlag != ChuyenLanePhai &&
				  ChuyenLaneFlag != ChuyenLaneTrai && StraightLineFlag == 1)
		  	  	{
		  	  		SpecialCaseFlag = 0;
		  	  		StraightLineFlag = 0;
				  while (LineDetect == 0)
				  {
					  MotorL_SetPWM(7200);
					  MotorR_SetPWM(7200);
					  Servo_SetAngle(5);
					  Sensor_Convert_A2D();
					  CarState = DiThang;
				  }
			  MaxSpeed = 7200;
		  	  	}
		  else if (SpecialCaseFlag == 1 && ChuyenLaneFlag == ChuyenLaneTrai)
				  {
			  	  	  	  ChuyenLaneFlag = 0;
			  	  	  	  SpecialCaseFlag = 0;
						  while (LineDetect != 0b00011000 &&
								 LineDetect != 0b00001000 &&
								 LineDetect != 0b00010000 &&
								 LineDetect != 0b00111000 &&
								 LineDetect != 0b00011100 )
						  {

							  MotorL_SetPWM(-7200);
							  MotorR_SetPWM(7200);
							  Servo_SetAngle(-70);
							  Sensor_Convert_A2D();
						  }
							  MaxSpeed = 7200;
				  }
	  	  else if (SpecialCaseFlag == 1 && ChuyenLaneFlag == ChuyenLanePhai)
		  	  {
	  		ChuyenLaneFlag = 0;
	  		SpecialCaseFlag = 0;
			  while (LineDetect != 0b00011000 &&
					 LineDetect != 0b00001000 &&
					 LineDetect != 0b00010000 &&
					 LineDetect != 0b00011100)
			  {
				  ChuyenLaneFlag = 0;
				  SpecialCaseFlag = 0;
				  MotorL_SetPWM(7200);
				  MotorR_SetPWM(-7200);
				  Servo_SetAngle(70);
				  Sensor_Convert_A2D();
			  }
				  MaxSpeed = 7200;
		  }

	  }
	  if (CarState == LechTrai)
	  {
		  switch (LineDetect)
		  {
			  case	0b00000000:
				  while(LineDetect != 0b00000001)
				  {
				  MotorR_SetPWM(MaxSpeed*0.52);
				  MotorL_SetPWM(MaxSpeed);
				  Servo_SetAngle(48);
				  Sensor_Convert_A2D();
				  }
				  break;
			  case	0b00000001:
				  MotorR_SetPWM(MaxSpeed*0.63);
				  MotorL_SetPWM(MaxSpeed);
				  Servo_SetAngle(33.3);
				  break;
			  case 0b00000011:
				  MotorR_SetPWM(MaxSpeed*0.72);
				  MotorL_SetPWM(MaxSpeed);
				  Servo_SetAngle(30.3);
				  break;
			  case 0b00000111:
				  MotorR_SetPWM(MaxSpeed*0.80);
				  MotorL_SetPWM(MaxSpeed);
				  Servo_SetAngle(24.9);
				  break;
			  case 0b00000110:
				  MotorR_SetPWM(MaxSpeed);
				  MotorL_SetPWM(MaxSpeed);
				  Servo_SetAngle(24.9);
				  break;
			  case 0b00001100:
				  MotorR_SetPWM(MaxSpeed);
				  MotorL_SetPWM(MaxSpeed);
				  Servo_SetAngle(12.6);
				  break;
			  case 0b00001110:
				  MotorR_SetPWM(MaxSpeed);
				  MotorL_SetPWM(MaxSpeed);
				  Servo_SetAngle(18.9);
				  break;
			  case 0b00001000:
			  	  MotorL_SetPWM(MaxSpeed);
			  	  MotorR_SetPWM(MaxSpeed);
			  	  Servo_SetAngle(2);
			  	  break;

		  }
		  continue;
	  }
	  if (CarState == LechPhai)
	  {
		  switch (LineDetect)
		  {
	  	  	  case	0b00000000:
	  	  		  while(LineDetect != 0b10000000){
	  	  	  	  MotorR_SetPWM(MaxSpeed*0.52);
	  	  	  	  MotorL_SetPWM(MaxSpeed);
	  	  	  	  Servo_SetAngle(-41.7);
	  	  	  	  Sensor_Convert_A2D();
	  	  		  }
	  	  	  	  break;
			  case	0b10000000:
				  MotorL_SetPWM(MaxSpeed*0.63);
				  MotorR_SetPWM(MaxSpeed*0.97);
				  Servo_SetAngle(-35.1);
				  break;
			  case 0b11000000:
				  MotorL_SetPWM(MaxSpeed*0.83);
				  MotorR_SetPWM(MaxSpeed*0.98);
				  Servo_SetAngle(-27.3);
				  break;
			  case 0b11100000:
				  MotorL_SetPWM(MaxSpeed*0.88);
				  MotorR_SetPWM(MaxSpeed);
				  Servo_SetAngle(-25.2);
				  break;
			  case 0b01100000:
				  MotorL_SetPWM(MaxSpeed);
				  MotorR_SetPWM(MaxSpeed);
				  Servo_SetAngle(-20.1);
				  break;
			  case 0b01110000:
				  MotorL_SetPWM(MaxSpeed);
				  MotorR_SetPWM(MaxSpeed);
				  Servo_SetAngle(-13.8);
				  break;
			  case 0b00110000:
				  MotorL_SetPWM(MaxSpeed);
				  MotorR_SetPWM(MaxSpeed);
				  Servo_SetAngle(-9.3);
				  break;
			 case 0b00010000:
				  MotorL_SetPWM(MaxSpeed);
				  MotorR_SetPWM(MaxSpeed);
				  Servo_SetAngle(2);
				  break;
		  }
		  continue;
	  }
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}


/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  LL_FLASH_SetLatency(LL_FLASH_LATENCY_2);
  while(LL_FLASH_GetLatency()!= LL_FLASH_LATENCY_2)
  {
  }
  LL_RCC_HSE_Enable();

   /* Wait till HSE is ready */
  while(LL_RCC_HSE_IsReady() != 1)
  {

  }
  LL_RCC_PLL_ConfigDomain_SYS(LL_RCC_PLLSOURCE_HSE_DIV_1, LL_RCC_PLL_MUL_9);
  LL_RCC_PLL_Enable();

   /* Wait till PLL is ready */
  while(LL_RCC_PLL_IsReady() != 1)
  {

  }
  LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);
  LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_2);
  LL_RCC_SetAPB2Prescaler(LL_RCC_APB2_DIV_1);
  LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_PLL);

   /* Wait till System clock is ready */
  while(LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_PLL)
  {

  }
  LL_Init1msTick(72000000);
  LL_SetSystemCoreClock(72000000);
  LL_RCC_SetADCClockSource(LL_RCC_ADC_CLKSRC_PCLK2_DIV_6);
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  LL_ADC_InitTypeDef ADC_InitStruct = {0};
  LL_ADC_CommonInitTypeDef ADC_CommonInitStruct = {0};
  LL_ADC_REG_InitTypeDef ADC_REG_InitStruct = {0};

  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* Peripheral clock enable */
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_ADC1);

  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_GPIOA);
  /**ADC1 GPIO Configuration
  PA0-WKUP   ------> ADC1_IN0
  PA1   ------> ADC1_IN1
  PA2   ------> ADC1_IN2
  PA3   ------> ADC1_IN3
  PA4   ------> ADC1_IN4
  PA5   ------> ADC1_IN5
  PA6   ------> ADC1_IN6
  PA7   ------> ADC1_IN7
  */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_0|LL_GPIO_PIN_1|LL_GPIO_PIN_2|LL_GPIO_PIN_3
                          |LL_GPIO_PIN_4|LL_GPIO_PIN_5|LL_GPIO_PIN_6|LL_GPIO_PIN_7;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ANALOG;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* ADC1 DMA Init */

  /* ADC1 Init */
  LL_DMA_SetDataTransferDirection(DMA1, LL_DMA_CHANNEL_1, LL_DMA_DIRECTION_PERIPH_TO_MEMORY);

  LL_DMA_SetChannelPriorityLevel(DMA1, LL_DMA_CHANNEL_1, LL_DMA_PRIORITY_HIGH);

  LL_DMA_SetMode(DMA1, LL_DMA_CHANNEL_1, LL_DMA_MODE_CIRCULAR);

  LL_DMA_SetPeriphIncMode(DMA1, LL_DMA_CHANNEL_1, LL_DMA_PERIPH_NOINCREMENT);

  LL_DMA_SetMemoryIncMode(DMA1, LL_DMA_CHANNEL_1, LL_DMA_MEMORY_INCREMENT);

  LL_DMA_SetPeriphSize(DMA1, LL_DMA_CHANNEL_1, LL_DMA_PDATAALIGN_HALFWORD);

  LL_DMA_SetMemorySize(DMA1, LL_DMA_CHANNEL_1, LL_DMA_MDATAALIGN_HALFWORD);

  /* USER CODE BEGIN ADC1_Init 1 */
  LL_DMA_SetDataLength(DMA1,LL_DMA_CHANNEL_1,8);
  LL_DMA_SetMemoryAddress(DMA1, LL_DMA_CHANNEL_1, (uint32_t) &Sensor_ADC_Value);
  LL_DMA_SetPeriphAddress(DMA1,LL_DMA_CHANNEL_1,ADC1_DR_Address);
  LL_DMA_EnableChannel(DMA1,LL_DMA_CHANNEL_1);
  /* USER CODE END ADC1_Init 1 */
  /** Common config
  */
  ADC_InitStruct.DataAlignment = LL_ADC_DATA_ALIGN_RIGHT;
  ADC_InitStruct.SequencersScanMode = LL_ADC_SEQ_SCAN_ENABLE;
  LL_ADC_Init(ADC1, &ADC_InitStruct);
  ADC_CommonInitStruct.Multimode = LL_ADC_MULTI_INDEPENDENT;
  LL_ADC_CommonInit(__LL_ADC_COMMON_INSTANCE(ADC1), &ADC_CommonInitStruct);
  ADC_REG_InitStruct.TriggerSource = LL_ADC_REG_TRIG_SOFTWARE;
  ADC_REG_InitStruct.SequencerLength = LL_ADC_REG_SEQ_SCAN_ENABLE_8RANKS;
  ADC_REG_InitStruct.SequencerDiscont = LL_ADC_REG_SEQ_DISCONT_DISABLE;
  ADC_REG_InitStruct.ContinuousMode = LL_ADC_REG_CONV_CONTINUOUS;
  ADC_REG_InitStruct.DMATransfer = LL_ADC_REG_DMA_TRANSFER_UNLIMITED;
  LL_ADC_REG_Init(ADC1, &ADC_REG_InitStruct);
  /** Configure Regular Channel
  */
  LL_ADC_REG_SetSequencerRanks(ADC1, LL_ADC_REG_RANK_1, LL_ADC_CHANNEL_0);
  LL_ADC_SetChannelSamplingTime(ADC1, LL_ADC_CHANNEL_0, LL_ADC_SAMPLINGTIME_28CYCLES_5);
  /** Configure Regular Channel
  */
  LL_ADC_REG_SetSequencerRanks(ADC1, LL_ADC_REG_RANK_2, LL_ADC_CHANNEL_1);
  LL_ADC_SetChannelSamplingTime(ADC1, LL_ADC_CHANNEL_1, LL_ADC_SAMPLINGTIME_28CYCLES_5);
  /** Configure Regular Channel
  */
  LL_ADC_REG_SetSequencerRanks(ADC1, LL_ADC_REG_RANK_3, LL_ADC_CHANNEL_2);
  LL_ADC_SetChannelSamplingTime(ADC1, LL_ADC_CHANNEL_2, LL_ADC_SAMPLINGTIME_28CYCLES_5);
  /** Configure Regular Channel
  */
  LL_ADC_REG_SetSequencerRanks(ADC1, LL_ADC_REG_RANK_4, LL_ADC_CHANNEL_3);
  LL_ADC_SetChannelSamplingTime(ADC1, LL_ADC_CHANNEL_3, LL_ADC_SAMPLINGTIME_28CYCLES_5);
  /** Configure Regular Channel
  */
  LL_ADC_REG_SetSequencerRanks(ADC1, LL_ADC_REG_RANK_5, LL_ADC_CHANNEL_4);
  LL_ADC_SetChannelSamplingTime(ADC1, LL_ADC_CHANNEL_4, LL_ADC_SAMPLINGTIME_28CYCLES_5);
  /** Configure Regular Channel
  */
  LL_ADC_REG_SetSequencerRanks(ADC1, LL_ADC_REG_RANK_6, LL_ADC_CHANNEL_5);
  LL_ADC_SetChannelSamplingTime(ADC1, LL_ADC_CHANNEL_5, LL_ADC_SAMPLINGTIME_28CYCLES_5);
  /** Configure Regular Channel
  */
  LL_ADC_REG_SetSequencerRanks(ADC1, LL_ADC_REG_RANK_7, LL_ADC_CHANNEL_6);
  LL_ADC_SetChannelSamplingTime(ADC1, LL_ADC_CHANNEL_6, LL_ADC_SAMPLINGTIME_28CYCLES_5);
  /** Configure Regular Channel
  */
  LL_ADC_REG_SetSequencerRanks(ADC1, LL_ADC_REG_RANK_8, LL_ADC_CHANNEL_7);
  LL_ADC_SetChannelSamplingTime(ADC1, LL_ADC_CHANNEL_7, LL_ADC_SAMPLINGTIME_28CYCLES_5);
  /* USER CODE BEGIN ADC1_Init 2 */

  LL_ADC_REG_SetDMATransfer(ADC1,LL_ADC_REG_DMA_TRANSFER_UNLIMITED);
  /* Khoi dong bo ADC */
  LL_ADC_Enable(ADC1);
  LL_ADC_StartCalibration(ADC1);

  	/* Cho trang thai cablib duoc bat *
  	 *
  	 */
  while(LL_ADC_IsCalibrationOnGoing(ADC1));

  	/* Bat dau chuyen doi ADC */
  LL_ADC_REG_StartConversionSWStart (ADC1);
  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  LL_I2C_InitTypeDef I2C_InitStruct = {0};

  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_GPIOB);
  /**I2C1 GPIO Configuration
  PB8   ------> I2C1_SCL
  PB9   ------> I2C1_SDA
  */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_8|LL_GPIO_PIN_9;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_OPENDRAIN;
  LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  LL_GPIO_AF_EnableRemap_I2C1();

  /* Peripheral clock enable */
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_I2C1);

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  /** I2C Initialization
  */
  LL_I2C_DisableOwnAddress2(I2C1);
  LL_I2C_DisableGeneralCall(I2C1);
  LL_I2C_EnableClockStretching(I2C1);
  I2C_InitStruct.PeripheralMode = LL_I2C_MODE_I2C;
  I2C_InitStruct.ClockSpeed = 100000;
  I2C_InitStruct.DutyCycle = LL_I2C_DUTYCYCLE_2;
  I2C_InitStruct.OwnAddress1 = 0;
  I2C_InitStruct.TypeAcknowledge = LL_I2C_ACK;
  I2C_InitStruct.OwnAddrSize = LL_I2C_OWNADDRESS1_7BIT;
  LL_I2C_Init(I2C1, &I2C_InitStruct);
  LL_I2C_SetOwnAddress2(I2C1, 0);
  /* USER CODE BEGIN I2C1_Init 2 */
  LL_I2C_Enable(I2C1);

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  LL_TIM_InitTypeDef TIM_InitStruct = {0};
  LL_TIM_OC_InitTypeDef TIM_OC_InitStruct = {0};
  LL_TIM_BDTR_InitTypeDef TIM_BDTRInitStruct = {0};

  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* Peripheral clock enable */
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_TIM1);

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  TIM_InitStruct.Prescaler = 0;
  TIM_InitStruct.CounterMode = LL_TIM_COUNTERMODE_UP;
  TIM_InitStruct.Autoreload = 7199;
  TIM_InitStruct.ClockDivision = LL_TIM_CLOCKDIVISION_DIV1;
  TIM_InitStruct.RepetitionCounter = 0;
  LL_TIM_Init(TIM1, &TIM_InitStruct);
  LL_TIM_DisableARRPreload(TIM1);
  LL_TIM_OC_EnablePreload(TIM1, LL_TIM_CHANNEL_CH1);
  TIM_OC_InitStruct.OCMode = LL_TIM_OCMODE_PWM1;
  TIM_OC_InitStruct.OCState = LL_TIM_OCSTATE_DISABLE;
  TIM_OC_InitStruct.OCNState = LL_TIM_OCSTATE_DISABLE;
  TIM_OC_InitStruct.CompareValue = 0;
  TIM_OC_InitStruct.OCPolarity = LL_TIM_OCPOLARITY_HIGH;
  TIM_OC_InitStruct.OCNPolarity = LL_TIM_OCPOLARITY_HIGH;
  TIM_OC_InitStruct.OCIdleState = LL_TIM_OCIDLESTATE_LOW;
  TIM_OC_InitStruct.OCNIdleState = LL_TIM_OCIDLESTATE_LOW;
  LL_TIM_OC_Init(TIM1, LL_TIM_CHANNEL_CH1, &TIM_OC_InitStruct);
  LL_TIM_OC_DisableFast(TIM1, LL_TIM_CHANNEL_CH1);
  LL_TIM_OC_EnablePreload(TIM1, LL_TIM_CHANNEL_CH3);
  TIM_OC_InitStruct.OCState = LL_TIM_OCSTATE_DISABLE;
  TIM_OC_InitStruct.OCNState = LL_TIM_OCSTATE_DISABLE;
  LL_TIM_OC_Init(TIM1, LL_TIM_CHANNEL_CH3, &TIM_OC_InitStruct);
  LL_TIM_OC_DisableFast(TIM1, LL_TIM_CHANNEL_CH3);
  LL_TIM_SetTriggerOutput(TIM1, LL_TIM_TRGO_RESET);
  LL_TIM_DisableMasterSlaveMode(TIM1);
  TIM_BDTRInitStruct.OSSRState = LL_TIM_OSSR_DISABLE;
  TIM_BDTRInitStruct.OSSIState = LL_TIM_OSSI_DISABLE;
  TIM_BDTRInitStruct.LockLevel = LL_TIM_LOCKLEVEL_OFF;
  TIM_BDTRInitStruct.DeadTime = 0;
  TIM_BDTRInitStruct.BreakState = LL_TIM_BREAK_DISABLE;
  TIM_BDTRInitStruct.BreakPolarity = LL_TIM_BREAK_POLARITY_HIGH;
  TIM_BDTRInitStruct.AutomaticOutput = LL_TIM_AUTOMATICOUTPUT_DISABLE;
  LL_TIM_BDTR_Init(TIM1, &TIM_BDTRInitStruct);
  /* USER CODE BEGIN TIM1_Init 2 */
  LL_TIM_EnableIT_UPDATE(TIM1);
  LL_TIM_SetCounter(TIM1,0);
  LL_TIM_EnableAllOutputs(TIM1);
  LL_TIM_EnableCounter(TIM1);

  /* USER CODE END TIM1_Init 2 */
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_GPIOA);
  /**TIM1 GPIO Configuration
  PA8   ------> TIM1_CH1
  PA10   ------> TIM1_CH3
  */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_8|LL_GPIO_PIN_10;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  LL_TIM_InitTypeDef TIM_InitStruct = {0};

  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* Peripheral clock enable */
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM2);

  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_GPIOA);
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_GPIOB);
  /**TIM2 GPIO Configuration
  PA15   ------> TIM2_CH1
  PB3   ------> TIM2_CH2
  */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_15;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_FLOATING;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = LL_GPIO_PIN_3;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_FLOATING;
  LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* TIM2 interrupt Init */
  NVIC_SetPriority(TIM2_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),2, 0));
  NVIC_EnableIRQ(TIM2_IRQn);

  /* USER CODE BEGIN TIM2_Init 1 */
  LL_GPIO_AF_EnableRemap_TIM2();
  /* USER CODE END TIM2_Init 1 */
  LL_TIM_SetEncoderMode(TIM2, LL_TIM_ENCODERMODE_X4_TI12);
  LL_TIM_IC_SetActiveInput(TIM2, LL_TIM_CHANNEL_CH1, LL_TIM_ACTIVEINPUT_DIRECTTI);
  LL_TIM_IC_SetPrescaler(TIM2, LL_TIM_CHANNEL_CH1, LL_TIM_ICPSC_DIV1);
  LL_TIM_IC_SetFilter(TIM2, LL_TIM_CHANNEL_CH1, LL_TIM_IC_FILTER_FDIV1);
  LL_TIM_IC_SetPolarity(TIM2, LL_TIM_CHANNEL_CH1, LL_TIM_IC_POLARITY_RISING);
  LL_TIM_IC_SetActiveInput(TIM2, LL_TIM_CHANNEL_CH2, LL_TIM_ACTIVEINPUT_DIRECTTI);
  LL_TIM_IC_SetPrescaler(TIM2, LL_TIM_CHANNEL_CH2, LL_TIM_ICPSC_DIV1);
  LL_TIM_IC_SetFilter(TIM2, LL_TIM_CHANNEL_CH2, LL_TIM_IC_FILTER_FDIV1);
  LL_TIM_IC_SetPolarity(TIM2, LL_TIM_CHANNEL_CH2, LL_TIM_IC_POLARITY_RISING);
  TIM_InitStruct.Prescaler = 0;
  TIM_InitStruct.CounterMode = LL_TIM_COUNTERMODE_UP;
  TIM_InitStruct.Autoreload = 65535;
  TIM_InitStruct.ClockDivision = LL_TIM_CLOCKDIVISION_DIV1;
  LL_TIM_Init(TIM2, &TIM_InitStruct);
  LL_TIM_DisableARRPreload(TIM2);
  LL_TIM_SetTriggerOutput(TIM2, LL_TIM_TRGO_RESET);
  LL_TIM_DisableMasterSlaveMode(TIM2);
  /* USER CODE BEGIN TIM2_Init 2 */
//  LL_TIM_CC_EnableChannel(TIM2,LL_TIM_CHANNEL_CH1);
//  LL_TIM_CC_EnableChannel(TIM2,LL_TIM_CHANNEL_CH2);
  LL_TIM_SetCounter(TIM2,0);
  LL_TIM_ClearFlag_UPDATE(TIM2);
  LL_TIM_EnableIT_UPDATE(TIM2);
  LL_TIM_EnableCounter(TIM2);
  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  LL_TIM_InitTypeDef TIM_InitStruct = {0};

  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* Peripheral clock enable */
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM3);

  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_GPIOB);
  /**TIM3 GPIO Configuration
  PB4   ------> TIM3_CH1
  PB5   ------> TIM3_CH2
  */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_4|LL_GPIO_PIN_5;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_FLOATING;
  LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* TIM3 interrupt Init */
  NVIC_SetPriority(TIM3_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),2, 0));
  NVIC_EnableIRQ(TIM3_IRQn);

  /* USER CODE BEGIN TIM3_Init 1 */
  LL_GPIO_AF_RemapPartial_TIM3();
  /* USER CODE END TIM3_Init 1 */
  LL_TIM_SetEncoderMode(TIM3, LL_TIM_ENCODERMODE_X4_TI12);
  LL_TIM_IC_SetActiveInput(TIM3, LL_TIM_CHANNEL_CH1, LL_TIM_ACTIVEINPUT_DIRECTTI);
  LL_TIM_IC_SetPrescaler(TIM3, LL_TIM_CHANNEL_CH1, LL_TIM_ICPSC_DIV1);
  LL_TIM_IC_SetFilter(TIM3, LL_TIM_CHANNEL_CH1, LL_TIM_IC_FILTER_FDIV1);
  LL_TIM_IC_SetPolarity(TIM3, LL_TIM_CHANNEL_CH1, LL_TIM_IC_POLARITY_RISING);
  LL_TIM_IC_SetActiveInput(TIM3, LL_TIM_CHANNEL_CH2, LL_TIM_ACTIVEINPUT_DIRECTTI);
  LL_TIM_IC_SetPrescaler(TIM3, LL_TIM_CHANNEL_CH2, LL_TIM_ICPSC_DIV1);
  LL_TIM_IC_SetFilter(TIM3, LL_TIM_CHANNEL_CH2, LL_TIM_IC_FILTER_FDIV1);
  LL_TIM_IC_SetPolarity(TIM3, LL_TIM_CHANNEL_CH2, LL_TIM_IC_POLARITY_RISING);
  TIM_InitStruct.Prescaler = 0;
  TIM_InitStruct.CounterMode = LL_TIM_COUNTERMODE_UP;
  TIM_InitStruct.Autoreload = 65535;
  TIM_InitStruct.ClockDivision = LL_TIM_CLOCKDIVISION_DIV1;
  LL_TIM_Init(TIM3, &TIM_InitStruct);
  LL_TIM_DisableARRPreload(TIM3);
  LL_TIM_SetTriggerOutput(TIM3, LL_TIM_TRGO_RESET);
  LL_TIM_DisableMasterSlaveMode(TIM3);
  /* USER CODE BEGIN TIM3_Init 2 */
  LL_TIM_SetCounter(TIM3,0);
   LL_TIM_EnableCounter(TIM3);
   LL_TIM_ClearFlag_UPDATE(TIM3);
   LL_TIM_EnableIT_UPDATE(TIM3);
  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  LL_TIM_InitTypeDef TIM_InitStruct = {0};
  LL_TIM_OC_InitTypeDef TIM_OC_InitStruct = {0};

  /* Peripheral clock enable */
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM4);

  /* TIM4 interrupt Init */
  NVIC_SetPriority(TIM4_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),0, 0));
  NVIC_EnableIRQ(TIM4_IRQn);

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  TIM_InitStruct.Prescaler = 24;
  TIM_InitStruct.CounterMode = LL_TIM_COUNTERMODE_UP;
  TIM_InitStruct.Autoreload = 59999;
  TIM_InitStruct.ClockDivision = LL_TIM_CLOCKDIVISION_DIV1;
  LL_TIM_Init(TIM4, &TIM_InitStruct);
  LL_TIM_DisableARRPreload(TIM4);
  LL_TIM_SetClockSource(TIM4, LL_TIM_CLOCKSOURCE_INTERNAL);
  LL_TIM_OC_EnablePreload(TIM4, LL_TIM_CHANNEL_CH1);
  TIM_OC_InitStruct.OCMode = LL_TIM_OCMODE_PWM1;
  TIM_OC_InitStruct.OCState = LL_TIM_OCSTATE_DISABLE;
  TIM_OC_InitStruct.OCNState = LL_TIM_OCSTATE_DISABLE;
  TIM_OC_InitStruct.CompareValue = 4500;
  TIM_OC_InitStruct.OCPolarity = LL_TIM_OCPOLARITY_HIGH;
  LL_TIM_OC_Init(TIM4, LL_TIM_CHANNEL_CH1, &TIM_OC_InitStruct);
  LL_TIM_OC_DisableFast(TIM4, LL_TIM_CHANNEL_CH1);
  TIM_OC_InitStruct.OCMode = LL_TIM_OCMODE_FROZEN;
  TIM_OC_InitStruct.OCState = LL_TIM_OCSTATE_DISABLE;
  TIM_OC_InitStruct.OCNState = LL_TIM_OCSTATE_DISABLE;
  LL_TIM_OC_Init(TIM4, LL_TIM_CHANNEL_CH2, &TIM_OC_InitStruct);
  LL_TIM_OC_DisableFast(TIM4, LL_TIM_CHANNEL_CH2);
  LL_TIM_SetTriggerOutput(TIM4, LL_TIM_TRGO_RESET);
  LL_TIM_DisableMasterSlaveMode(TIM4);
  LL_TIM_OC_DisablePreload(TIM4, LL_TIM_CHANNEL_CH1);
  /* USER CODE BEGIN TIM4_Init 2 */

  LL_TIM_EnableIT_UPDATE(TIM4);
  LL_TIM_ClearFlag_UPDATE(TIM4);

  LL_TIM_EnableIT_CC1(TIM4);
  LL_TIM_ClearFlag_CC1(TIM4);
//
  LL_TIM_EnableIT_CC2(TIM4);
  LL_TIM_ClearFlag_CC2(TIM4);


  LL_TIM_SetCounter(TIM4,0);
  LL_TIM_EnableCounter(TIM4);


  /* USER CODE END TIM4_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  LL_USART_InitTypeDef USART_InitStruct = {0};

  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* Peripheral clock enable */
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_USART1);

  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_GPIOB);
  /**USART1 GPIO Configuration
  PB6   ------> USART1_TX
  PB7   ------> USART1_RX
  */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_6;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = LL_GPIO_PIN_7;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_FLOATING;
  LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  LL_GPIO_AF_EnableRemap_USART1();

  /* USART1 interrupt Init */
  NVIC_SetPriority(USART1_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),2, 0));
  NVIC_EnableIRQ(USART1_IRQn);

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  USART_InitStruct.BaudRate = 115200;
  USART_InitStruct.DataWidth = LL_USART_DATAWIDTH_9B;
  USART_InitStruct.StopBits = LL_USART_STOPBITS_1;
  USART_InitStruct.Parity = LL_USART_PARITY_NONE;
  USART_InitStruct.TransferDirection = LL_USART_DIRECTION_TX_RX;
  USART_InitStruct.HardwareFlowControl = LL_USART_HWCONTROL_NONE;
  USART_InitStruct.OverSampling = LL_USART_OVERSAMPLING_16;
  LL_USART_Init(USART1, &USART_InitStruct);
  LL_USART_ConfigAsyncMode(USART1);
  LL_USART_Enable(USART1);
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* Init with LL driver */
  /* DMA controller clock enable */
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_DMA1);

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  NVIC_SetPriority(DMA1_Channel1_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),1, 0));
  NVIC_EnableIRQ(DMA1_Channel1_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  LL_EXTI_InitTypeDef EXTI_InitStruct = {0};
  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_GPIOC);
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_GPIOD);
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_GPIOA);
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_GPIOB);

  /**/
  LL_GPIO_ResetOutputPin(GPIOC, LL_GPIO_PIN_13);

  /**/
  LL_GPIO_ResetOutputPin(GPIOB, LL_GPIO_PIN_15);

  /**/
  LL_GPIO_ResetOutputPin(GPIOA, LL_GPIO_PIN_9|LL_GPIO_PIN_11);

  /**/
  GPIO_InitStruct.Pin = LL_GPIO_PIN_13;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  LL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = LL_GPIO_PIN_15;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = LL_GPIO_PIN_9;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = LL_GPIO_PIN_11;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /**/
  LL_GPIO_AF_SetEXTISource(LL_GPIO_AF_EXTI_PORTB, LL_GPIO_AF_EXTI_LINE12);

  /**/
  LL_GPIO_AF_SetEXTISource(LL_GPIO_AF_EXTI_PORTB, LL_GPIO_AF_EXTI_LINE13);

  /**/
  LL_GPIO_AF_SetEXTISource(LL_GPIO_AF_EXTI_PORTB, LL_GPIO_AF_EXTI_LINE14);

  /**/
  EXTI_InitStruct.Line_0_31 = LL_EXTI_LINE_12;
  EXTI_InitStruct.LineCommand = ENABLE;
  EXTI_InitStruct.Mode = LL_EXTI_MODE_IT;
  EXTI_InitStruct.Trigger = LL_EXTI_TRIGGER_RISING;
  LL_EXTI_Init(&EXTI_InitStruct);

  /**/
  EXTI_InitStruct.Line_0_31 = LL_EXTI_LINE_13;
  EXTI_InitStruct.LineCommand = ENABLE;
  EXTI_InitStruct.Mode = LL_EXTI_MODE_IT;
  EXTI_InitStruct.Trigger = LL_EXTI_TRIGGER_RISING;
  LL_EXTI_Init(&EXTI_InitStruct);

  /**/
  EXTI_InitStruct.Line_0_31 = LL_EXTI_LINE_14;
  EXTI_InitStruct.LineCommand = ENABLE;
  EXTI_InitStruct.Mode = LL_EXTI_MODE_IT;
  EXTI_InitStruct.Trigger = LL_EXTI_TRIGGER_RISING;
  LL_EXTI_Init(&EXTI_InitStruct);

  /**/
  LL_GPIO_SetPinMode(GPIOB, LL_GPIO_PIN_12, LL_GPIO_MODE_FLOATING);

  /**/
  LL_GPIO_SetPinMode(GPIOB, LL_GPIO_PIN_13, LL_GPIO_MODE_FLOATING);

  /**/
  LL_GPIO_SetPinMode(GPIOB, LL_GPIO_PIN_14, LL_GPIO_MODE_FLOATING);

  /* EXTI interrupt init*/
  NVIC_SetPriority(EXTI15_10_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),2, 0));
  NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */

void GetThreshold()
{
//	  printf("Getting White Line");
	  uint16_t WhiteValue[] = {0, 0, 0, 0, 0, 0, 0, 0};
	 for(int i = 0; i < ADC_Sample_Times; ++i)
	 {
		 for(int i = 0; i < NumberOfSensor; ++i)
		 {
			 if(WhiteValue[i] > Sensor_ADC_Value[i])
			 {
				 WhiteValue[i] = Sensor_ADC_Value[i];
			 }
		 }
	 }
	 LL_mDelay(1000);
//	 printf("Getting Black Line");
	 LL_mDelay(1000);
	 uint16_t BlackValue[] = {4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095};
	 for(int i = 0; i < ADC_Sample_Times; ++i)
	 {
		 for(int i = 0; i < NumberOfSensor; ++i)
		 {
			 if(BlackValue[i] > Sensor_ADC_Value[i])
			 {
				BlackValue[i] = Sensor_ADC_Value[i];
			 }
		 }
	 }
	 printf("Done");
	 for(int i = 0; i < 8; ++i)
	 {
		 Sensor_Threshold[i] = (BlackValue[i] + WhiteValue[i])/2;
	 }
}
void Sensor_Convert_A2D()
{
	LineDetect = 0;
	for(int i = 0; i < 8; ++i)
	  {
		  if(Sensor_ADC_Value[i] < Sensor_Threshold[i])
		  {
			  sbi(LineDetect, (7-i));
//			  printf("1 ");
		  }else
		  {
//			  printf("0 ");
		  }
	  };
//	printf("\n");
//	LL_mDelay(120);
}


void Sensor_Print_Thres()
{
	printf("Threshold Val:  ");
	  for(int i = 0; i < 8; ++i)
	 {
		  printf("%u ", Sensor_Threshold[i]);
	 }
	  printf("\n");
}
void Sensor_PrintValue()
{
	printf("Sensor Val: ");
  for(int i = 0; i < 8; ++i)
  {
	  printf("%u ", Sensor_ADC_Value[i]);
  };
  printf("\n");

}

void Sensor_Print_LineDetect()
{

		char buffer[8];
		itoa (LineDetect,buffer,2);
		printf ("binary: %s\n",buffer);


}

void Sensor_Print_LineDetect();
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
