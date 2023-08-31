/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
//#include "stdio.h"
#include "sinTable_16KHz.h"
#include "math.h"
#include "inverter.h"


#ifdef LCD
#include "lcd.h"
#endif

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define X_UART							0
#define X_EEPROM						0

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

//================= EEPROM
#define EEPROM_ADDRESS              	((uint8_t)0xA0)
#define FAULT_BUFFER		         	((uint8_t)12)        	// 12 byte = 12 faults
#define EEPROM_DEFAULT_PARAMS      	((uint8_t)16)        	// 16 parameters to read/write

//================= UART
#define UART_TIMEOUT					((uint8_t)50)
//================= GPIO
//----------------- Output Active Low
//#define LD_FAULT_INACTIVE				HAL_GPIO_WritePin(LD_R_GPIO_Port, LD_R_Pin, GPIO_PIN_SET)
//#define LD_FAULT_ACTIVE				HAL_GPIO_WritePin(LD_R_GPIO_Port, LD_R_Pin, GPIO_PIN_RESET)
//#define LD_FAULT_TOGGLE				HAL_GPIO_TogglePin(LD_R_GPIO_Port, LD_R_Pin)
//
//#define LD_BYPSS_INACTIVE				HAL_GPIO_WritePin(LD_B_GPIO_Port, LD_B_Pin, GPIO_PIN_SET)
//#define LD_BYPSS_ACTIVE				HAL_GPIO_WritePin(LD_B_GPIO_Port, LD_B_Pin, GPIO_PIN_RESET)
//#define LD_BYPSS_TOGGLE				HAL_GPIO_TogglePin(LD_B_GPIO_Port, LD_B_Pin)
//
//#define LD_RUN_INACTIVE				HAL_GPIO_WritePin(LD_G_GPIO_Port, LD_G_Pin, GPIO_PIN_SET)
//#define LD_RUN_ACTIVE					HAL_GPIO_WritePin(LD_G_GPIO_Port, LD_G_Pin, GPIO_PIN_RESET)
//#define LD_RUN_TOGGLE					HAL_GPIO_TogglePin(LD_G_GPIO_Port, LD_G_Pin)



#define LD_FAULT_INACTIVE				GPIOC->BSRR = (uint32_t)GPIO_PIN_13;  			// SET
#define LD_FAULT_ACTIVE				GPIOC->BSRR = (uint32_t)GPIO_PIN_13 << 16u;  	// RESET
#define LD_FAULT_TOGGLE				HAL_GPIO_TogglePin(LD_R_GPIO_Port, LD_R_Pin)

#define LD_BYPSS_INACTIVE				GPIOC->BSRR = (uint32_t)GPIO_PIN_14;  			// SET
#define LD_BYPSS_ACTIVE				GPIOC->BSRR = (uint32_t)GPIO_PIN_14 << 16u;  	// RESET
#define LD_BYPSS_TOGGLE				HAL_GPIO_TogglePin(LD_B_GPIO_Port, LD_B_Pin)

#define LD_RUN_INACTIVE				GPIOC->BSRR = (uint32_t)GPIO_PIN_15;  			// SET
#define LD_RUN_ACTIVE					GPIOC->BSRR = (uint32_t)GPIO_PIN_15 << 16u;  	// RESET
#define LD_RUN_TOGGLE					HAL_GPIO_TogglePin(LD_G_GPIO_Port, LD_G_Pin)

//----------------- Output Active High
#define SLAVE_MOSFET_HIGH_INACTIVE	HAL_GPIO_WritePin(HOB_GPIO_Port, HOB_Pin, GPIO_PIN_RESET)
#define SLAVE_MOSFET_HIGH_ACTIVE		HAL_GPIO_WritePin(HOB_GPIO_Port, HOB_Pin, GPIO_PIN_SET)

#define SLAVE_MOSFET_LOW_INACTIVE		HAL_GPIO_WritePin(LOB_GPIO_Port, LOB_Pin, GPIO_PIN_RESET)
#define SLAVE_MOSFET_LOW_ACTIVE		HAL_GPIO_WritePin(LOB_GPIO_Port, LOB_Pin, GPIO_PIN_SET)

#define GATEDRIVE_INACTIVE			HAL_GPIO_WritePin(GATE_SD_GPIO_Port, GATE_SD_Pin, GPIO_PIN_RESET)
#define GATEDRIVE_ACTIVE				HAL_GPIO_WritePin(GATE_SD_GPIO_Port, GATE_SD_Pin, GPIO_PIN_SET)

#define FAN_INACTIVE					HAL_GPIO_WritePin(FAN_GPIO_Port, FAN_Pin, GPIO_PIN_RESET)
#define FAN_ACTIVE						HAL_GPIO_WritePin(FAN_GPIO_Port, FAN_Pin, GPIO_PIN_SET)

//#define RY_BYPASS_INACTIVE			HAL_GPIO_WritePin(BYPSS_GPIO_Port, BYPSS_Pin, GPIO_PIN_RESET)
//#define RY_BYPASS_ACTIVE				HAL_GPIO_WritePin(BYPSS_GPIO_Port, BYPSS_Pin, GPIO_PIN_SET)
//
//#define TESTPOINT_OFF					HAL_GPIO_WritePin(DBG_GPIO_Port, DBG_Pin, GPIO_PIN_RESET)
//#define TESTPOINT_ON					HAL_GPIO_WritePin(DBG_GPIO_Port, DBG_Pin, GPIO_PIN_SET)

//----------------- Digital Input
#define KEY_MODE						((uint8_t)0x01)		// 0000-0001
#define KEY_ENT						((uint8_t)0x02)   	// 0000-0010
#define KEY_UP	 						((uint8_t)0x04)		// 0000-0100
#define KEY_DWN						((uint8_t)0x08)		// 0000-1000


//=================
#define OFF								0
#define ON								1

#define RESET							0
#define SET								1

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;
DMA_HandleTypeDef hdma_adc1;

I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
//================= Structure
SPWM_HandleTypeDef spwm;	// inverter.h
PI_HandleTypeDef pi_control;
DISPLAY_HandleTypedef txData;

keyStatus_HandleTypedef key;
keyCounter_HandleTypedef count;
keyFlag_HandleTypedef flag;

protectionCounter_HandleTypedef cnt;
protectionFlag_HandleTypedef flg;
temperature_HandleTypedef ntc;

Fault_HandleTypedef fltCnt;		// fault Count
flagFault_HandleTypedef fltFlg;	// fault Flag

#ifdef LCD

#endif
//------------------------------------------------------------------------

enum enum_boardState {
	boardState_InverterOFF 	 = 0,
	boardState_CheckDCBus 	 = 1,
	boardState_RLyConnect 	 = 2,
	boardState_InverterON	 = 3,
	boardState_TripCondition = 4
};

enum enum_boardState boardState = boardState_InverterOFF;

//------------------------------------------------------------------------
typedef enum{
	powerState_Off,
	powerState_On
}enum_powerState;

typedef enum{
	machineState_Error,
	machineState_StartUp,
	machineState_Operate
}enum_machineState;

typedef enum{
	invState_Off,
	invState_On
}enum_invState;

typedef enum{
	phaseState_Positive,
	phaseState_Negative
}enum_phaseState;

typedef enum{
	modeState_Normal,
	modeState_Bypass
}enum_modeState;

typedef enum{
	faultState_Clear,

	faultState_InputOverCurrent,
	faultState_InputUnderVoltage,
	faultState_InputOverVoltage,

	faultState_OutputOverVoltage,
	faultState_OutputOverCurrent,
	faultState_OutputOverPower,

	faultState_MosfetOverTemp,
	faultState_TransOverTemp

}enum_faultState;

typedef enum{
	displayState_OFF,
	displayState_ON
}enum_displayState;

typedef enum{
	indicatorState_Home,
	indicatorState_Error
}enum_indicatorState;

typedef enum{
	keyState_Home,
	keyState_Selection,
	keyState_Setting
}enum_keyState;

volatile enum_powerState		powerState		= powerState_Off;
volatile enum_machineState		machineState	= machineState_Error;
volatile enum_invState			invState		= invState_Off;
volatile enum_phaseState		phaseState		= phaseState_Positive;
volatile enum_modeState		modeState		= modeState_Normal;
volatile enum_faultState		faultState		= faultState_Clear;
volatile enum_displayState		displatState	= displayState_ON;
volatile enum_indicatorState	indicatorState	= indicatorState_Home;
volatile enum_keyState			keyState		= keyState_Home;
//================= PID Control

int dynamicRef;
//================= PWM, INVERTER
uint16_t sampling =0; 					// Count for sampling PWM value 0 - 180 Degree

uint32_t dutyCycle_temp =0;					// temporary duty cycle before fine adjust by voltage loop
uint32_t dutyCycle =0;						// duty cycle before fine adjust by voltage loop
uint32_t dutyCyclePrv =0;					// old cycle before fine adjust by voltage loop

uint16_t ccr3 = CAPTURE_POINT;				// ADCS capture position every sampling point
uint16_t ccr1 = DUTY1, ccr2 = DUTY2; 		// use demo mode for check hardware

uint8_t	 flag_zeroCross = RESET;
//================= ADCS
volatile uint8_t 	convCompleted = 0;    	// Complete Conversion flag
volatile uint32_t 	adcPair[3];
volatile uint32_t 	adcVal[3];
volatile uint16_t	adc1[3];
volatile uint16_t 	adc2[3];

volatile uint16_t adcOffset;
volatile uint16_t adcVRef;
volatile uint16_t invVinInst;
volatile uint16_t invIinInst;
volatile uint16_t invVoutInst;
volatile uint16_t invIoutInst;
volatile uint16_t invMosfetTemp;
volatile uint16_t invTransfTemp;

//================= PI Regulator
//static int32_t Kp;
//static int32_t Ki;
static int32_t Int_term;
static int32_t VoutDelta;

volatile uint32_t VoutT;
volatile uint16_t VoutRange;
volatile uint16_t CTMax, CTMin;

int32_t  PI(uint32_t setpoint, uint32_t feedback);					// AN4449

//================= EEPROM
uint8_t write_buffer_init[4] = {1, 2, 4, 9};
uint8_t write_buffer[4] = {20, 30, 59, 87};
uint8_t read_buffer[4];

uint8_t flag_Read 	= RESET;
uint8_t flag_Write 	= RESET;

//================= UART
uint8_t flag_tx 	= RESET;
uint8_t flag_rx 	= RESET;

uint8_t tx_buffer[8] = {0x01,0x02,0x04,0x08,0x80,0x40,0x20,0x10};
uint8_t rx_buffer[8];
uint8_t tx_echo[8];

uint8_t tx_text_buffer[15] = "STM32F103C6T6\n\r";

//================= GPIO
uint16_t cntTick;
uint8_t  prv_keyState;
uint8_t  indexMenu =0, indexValue =0;

//================= etc
uint16_t *p, *q, *r;
uint16_t buff_p = 123;
uint32_t xsetpoint = 2000, xfeedback =0;
int data1, data2;

uint32_t delayTime =0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM1_Init(void);
static void MX_ADC2_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */
//================= systems

void TIM1_ISR(void);					// 16 KHz	// Highest  Priority
void TIM2_ISR(void);					//  8 KHz	// High 	Priority
void faultCheck(void);
int16_t convertTemp(int16_t adcNTC);

void sysTick(void);             		//  1 KHz   // Low  Priority
	void powerConvertion(void);    	//
	void readGPIO(void);

//=================
	void InitParameters(void);			// Parameters
	void InitPeripheral(void);			// Initial GPIO
	void ClearParameters(void);		// Clear Parameters

static void Vout_Check(void);
static void Reset_PI(void);

//==== Need Highest Clock
void EEPromWrite(void);
void EEPromRead(void);
void UartTransmit(void);
void execDISP(void);

void delay_ms(uint32_t del);
void mainProgram(void);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

#ifdef X_SWV

int _write(int file, char *ptr, int len)
{
	int i=0;
	for(i =0; i<len; i++)
		ITM_SendChar((*ptr++));
	return len;
}

#endif

void delay_ms(uint32_t del)
{
	delayTime = del;
	while(delayTime > 0);
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_TIM1_Init();
  MX_ADC2_Init();
  MX_I2C1_Init();
  MX_USART1_UART_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */

  HAL_UART_Receive_IT(&huart1, tx_echo, 8);

  spwm.maxPeriod = PWM_PERIOD_MAX;
  spwm.minPeriod = PWM_PERIOD_MIN;
  spwm.currentDuty = 20;
  spwm.percentMod = 0;
  spwm.pwm_deadTime = 0;

  InitParameters();
  InitPeripheral();

  //HAL_TIM_Base_Start_IT(&htim3);
  HAL_TIM_Base_Start_IT(&htim2);				 // Slave of TIM1 General Task
  HAL_TIM_Base_Start_IT(&htim1);				 // Inverter Task

  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);     // 32 KHz

  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);     // PWM AH
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);     // PWM BH
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);	 // OCREF for Trigger ADC Conversion
  //HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);

  HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);  // PWM AL
  HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);  // PWM BL
  //HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3);

  HAL_ADC_Start(&hadc2);						 // start ADC2 (slave) first!
  HAL_ADCEx_MultiModeStart_DMA(&hadc1, (uint32_t*) adcPair, 3);

#ifdef LCD

  //LCD_PWRON();
  //delay_ms(100);
  LCD_INIT();
  LCD_CLEAR_DISPLAY();
  LCD_LOCATE(1, 1);
  //LCD_printstring(" STM32F103C6T6 \n");
  LCD_printstring(" STM32F103C6T6  ");
  LCD_LOCATE(2, 1);
  //LCD_printstring("pureSineInverter\n");
  LCD_printstring("pureSineInverter");
#ifdef USE_HAL_DELAY
	HAL_Delay(500);
#else
	delay_ms(500);
#endif

  LCD_DISP_OFF();
#ifdef USE_HAL_DELAY
	HAL_Delay(200);
#else
	delay_ms(200);
#endif
  LCD_DISP_ON();
  LCD_CLEAR_DISPLAY();


#endif

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  	 mainProgram();
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL14;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV4;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
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

  ADC_MultiModeTypeDef multimode = {0};
  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T1_CC3;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 3;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the ADC multi-mode
  */
  multimode.Mode = ADC_DUALMODE_REGSIMULT;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_7CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = ADC_REGULAR_RANK_3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief ADC2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC2_Init(void)
{

  /* USER CODE BEGIN ADC2_Init 0 */

  /* USER CODE END ADC2_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC2_Init 1 */

  /* USER CODE END ADC2_Init 1 */

  /** Common config
  */
  hadc2.Instance = ADC2;
  hadc2.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc2.Init.ContinuousConvMode = DISABLE;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc2.Init.NbrOfConversion = 3;
  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_7CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_5;
  sConfig.Rank = ADC_REGULAR_RANK_3;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC2_Init 2 */

  /* USER CODE END ADC2_Init 2 */

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

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 400000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = ((TIM_CLOCK_DIVIDER) - 1);
  htim1.Init.CounterMode = TIM_COUNTERMODE_CENTERALIGNED1;
  htim1.Init.Period = ((PWM_PERIOD_CYCLES)/2);
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = (REP_COUNTER);
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_OC3REF;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.Pulse = 1;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = ((DEAD_TIME_COUNTS)/2);
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_ENABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_ENABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_SlaveConfigTypeDef sSlaveConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = ((TIM_CLOCK_DIVIDER) - 1);
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = ((PWM_PERIOD_CYCLES)*2);
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sSlaveConfig.SlaveMode = TIM_SLAVEMODE_DISABLE;
  sSlaveConfig.InputTrigger = TIM_TS_ITR0;
  if (HAL_TIM_SlaveConfigSynchro(&htim2, &sSlaveConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

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

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, LD_FLT_Pin|LD_BYPSS_Pin|LD_RUN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, RS_Pin|LOB_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, EN_Pin|D4_Pin|D5_Pin|D6_Pin
                          |D7_Pin|HOB_Pin|GATE_SD_Pin|FAN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : LD_FLT_Pin LD_BYPSS_Pin LD_RUN_Pin */
  GPIO_InitStruct.Pin = LD_FLT_Pin|LD_BYPSS_Pin|LD_RUN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : THMO_Pin */
  GPIO_InitStruct.Pin = THMO_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(THMO_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : RS_Pin LOB_Pin */
  GPIO_InitStruct.Pin = RS_Pin|LOB_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : EN_Pin D4_Pin D5_Pin D6_Pin
                           D7_Pin HOB_Pin GATE_SD_Pin FAN_Pin */
  GPIO_InitStruct.Pin = EN_Pin|D4_Pin|D5_Pin|D6_Pin
                          |D7_Pin|HOB_Pin|GATE_SD_Pin|FAN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : KEY_DWN_Pin KEY_UP_Pin KEY_ENT_Pin */
  GPIO_InitStruct.Pin = KEY_DWN_Pin|KEY_UP_Pin|KEY_ENT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : KEY_MODE_Pin */
  GPIO_InitStruct.Pin = KEY_MODE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(KEY_MODE_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

void TIM1_ISR(void) // 16 KHz
{
	GPIOC->BSRR = (uint32_t)GPIO_PIN_14;  // High
//-------------------------------------------------
//  readADC Values
	if(	convCompleted == 1)	 // End Of Conversion
	{
		convCompleted =0;
		readADCSignals(&adcOffset, &adcVRef, &invVinInst, &invIinInst, &invVoutInst, &invIoutInst);
	}
	GPIOC->BSRR = (uint32_t)GPIO_PIN_13 << 16u;  // RESET
//-------------------------------------------------
//  Generated samples
	sampling++;
	if(sampling > 160)
	{
		sampling =0;
		flag_zeroCross = SET;

		HAL_GPIO_TogglePin(LD_RUN_GPIO_Port, LD_RUN_Pin);		// 180 Degree

		//printf("ADC Value = %d\n", adc1[0]);
	}

//-------------------------------------------------
//  Feedback  PI Control

//  OPEN_LOOP
#if (CONTROLLEVEL == OPEN_LOOP)

	dutyCycle_temp = sinTable_160[sampling];
#endif

//  CLOSED_VOLTAGE
#if (CONTROLLEVEL == CLOSED_VOLTAGE)

#endif

//  CLOSED_CURRENT
#if (CONTROLLEVEL == CLOSED_CURRENT)

	dutyCycle_temp = sinTable_160[sampling];

	dynamicRef = PI(xsetpoint, xfeedback/*invVoutInst*/);   		// return new dutyCycle

#endif

//  CLOSED_VOLTAGE_CURRENT
#if (CONTROLLEVEL == CLOSED_VOLTAGE_CURRENT)

#endif

//-------------------------------------------------
//  PWM Driver for the inverter

//  dutyCyclePrv + ((int) plant)
    spwm.currentDuty = dutyCycle_temp;


//-------------------------------------------------
//	Data Stored
    dutyCyclePrv = dutyCycle;
//-------------------------------------------------
//	Update gui

		switch(phaseState)
		{
		case phaseState_Negative:

				TIM1->CCR1 = dutyCycle_temp;
				TIM1->CCR2 = 0;


				SLAVE_MOSFET_HIGH_INACTIVE;
				SLAVE_MOSFET_LOW_ACTIVE;

				if(flag_zeroCross == SET)
				{
					flag_zeroCross = RESET;
					phaseState = phaseState_Positive;
				}
			break;

		case phaseState_Positive:
				TIM1->CCR1 = 0;
				TIM1->CCR2 = dutyCycle_temp;

				SLAVE_MOSFET_HIGH_ACTIVE;
				SLAVE_MOSFET_LOW_INACTIVE;

				if(flag_zeroCross == SET)
				{
					flag_zeroCross = RESET;
					phaseState = phaseState_Negative;
				}
			break;
		}


		GPIOC->BSRR = (uint32_t)GPIO_PIN_14 << 16u;  // RESET

}

//==============================================================
/*
 *
 *
 *
 */
void TIM2_ISR(void)
{
	HAL_GPIO_TogglePin(FAN_GPIO_Port, FAN_Pin);

#ifdef DEVELOP_MODE
	faultState = faultState_Clear;
	powerState = powerState_On;
#else
	faultCheck();	// fault check

#endif


	if((faultState != faultState_Clear)||(powerState == powerState_Off)) 		// some fault is occur or power switch is off
	{
		machineState = machineState_Error;			// go to error state instantly
	}

	switch(machineState)
	{
		case machineState_Error :		// default
				invState = invState_Off;
				sampling = 0;

			break;

		case machineState_StartUp :		// soft-start
				invState = invState_On;

			break;

		case machineState_Operate :		// normal operation
				invState = invState_On;

			break;
	}

	//---- Gate Driver Control
	switch(invState)
	{
		case invState_Off :
				TIM1->CCR1	= 0;	// set dutyCycle = 0
				TIM1->CCR2  = 0;	// set dutyCycle = 0
				GATEDRIVE_INACTIVE;	// set High to GaveDrive Shutdown pin

			break;

		case invState_On :
				GATEDRIVE_ACTIVE;	// set Low to GaveDrive Shutdown pin
			break;
	}


}


//==============================================================
/*
 * input : analog input
 * return : fault status
 *
*/
void faultCheck(void)
{

	//----index 1 input OCP
	if((invIinInst > INPUT_OVER_CURRENT)&&(flg.InputOverCurrent == RESET))
	{
		cnt.InputOverCurrent++;
		if(cnt.InputOverCurrent > CNT_100MS)
		{
			if(faultState == faultState_Clear)
			{
				faultState = faultState_InputOverCurrent;
				flg.InputOverCurrent = SET;
			}
		}
	}
	else if((invIinInst < INPUT_STBY_CURRENT)&&(flg.InputOverCurrent == SET))
	{
		cnt.InputOverCurrent++;
		if(cnt.InputOverCurrent > CNT_1S)
		{
			flg.InputOverCurrent = RESET;
			if(faultState == faultState_InputOverCurrent)
			{
				faultState = faultState_Clear;
			}
		}
	}
	else
	{
		cnt.InputOverCurrent = 0;
	}

	//----index 2 index UVP
	if((invVinInst < INPUT_UNDER_VOLTAGE_CUTOUT)&&(flg.InputUnderVoltage == RESET))
	{
		cnt.InputUnderVoltage++;
		if(cnt.InputUnderVoltage > CNT_1S)
		{
			if(faultState == faultState_Clear)
			{
				faultState = faultState_InputUnderVoltage;
				flg.InputUnderVoltage = SET;
			}
		}
	}
	else if((invVinInst > INPUT_UNDER_VOLTAGE_CUTIN)&&(flg.InputUnderVoltage == SET))
	{
		cnt.InputUnderVoltage++;
		if(cnt.InputUnderVoltage > CNT_1S)
		{
			flg.InputUnderVoltage = RESET;
			if(faultState == faultState_InputUnderVoltage)
			{
				faultState = faultState_Clear;
			}
		}
	}
	else
	{
		cnt.InputUnderVoltage = 0;
	}

	//----index 3 input OVP
	if((invVinInst > INPUT_OVER_VOLTAGE_CUTOUT)&&(flg.InputOverVoltage == RESET))
	{
		cnt.InputOverVoltage++;
		if(cnt.InputOverVoltage > CNT_200MS)
		{
			if(faultState == faultState_Clear)
			{
				faultState = faultState_InputOverVoltage;
				flg.InputOverVoltage = SET;
			}

		}
	}
	else if((invVinInst < INPUT_OVER_VOLTAGE_CUTIN)&&(flg.InputOverVoltage == SET))
	{
		cnt.InputOverVoltage++;
		if(cnt.InputOverVoltage > CNT_500MS)
		{
			flg.InputOverVoltage = RESET;
			if(faultState == faultState_InputOverVoltage)
			{
				faultState = faultState_Clear;
			}
		}
	}
	else
	{
		cnt.InputOverVoltage = 0;
	}

	//----index 4 output OVP
	if((invVoutInst > OUTPUT_OVER_VOLTAGE)&&(flg.OutputOverVoltage == RESET))
	{
		cnt.OutputOverVoltage++;
		if(cnt.OutputOverVoltage > CNT_100MS)
		{
			if(faultState == faultState_Clear)
			{
				faultState = faultState_OutputOverVoltage;
				flg.OutputOverVoltage = SET;
			}
		}
	}
	else if((invVoutInst <= OUTPUT_VOLTAGE)&&(flg.OutputOverVoltage == SET))
	{
		cnt.OutputOverVoltage++;
		if(cnt.OutputOverVoltage > CNT_500MS)
		{
			flg.OutputOverVoltage = RESET;
			if(faultState == faultState_OutputOverVoltage)
			{
				faultState = faultState_Clear;
			}
		}
	}
	else
	{
		cnt.OutputOverVoltage = 0;
	}

	//----index 5 output OCP
	if((invIoutInst > OUTPUT_OVER_CURRENT)&&(flg.OutputOverCurrent == RESET))
	{
		cnt.OutputOverCurrent++;
		if(cnt.OutputOverCurrent > CNT_200MS)
		{
			if(faultState == faultState_Clear)
			{
				faultState = faultState_OutputOverCurrent;
				flg.OutputOverCurrent = SET;
			}
		}
	}
	else if((invIoutInst < OUTPUT_STBY_CURRENT)&&(flg.OutputOverCurrent == SET))
	{
		cnt.OutputOverCurrent++;
		if(cnt.OutputOverCurrent > CNT_1S)
		{
			flg.OutputOverCurrent = RESET;
			if(faultState == faultState_OutputOverCurrent)
			{
				faultState = faultState_Clear;
			}
		}
	}
	else
	{
		cnt.OutputOverCurrent = 0;
	}

	//----index 6 output OP

	//----index 7 mosfet OTP
	if((ntc.mosfet > TEMP_70C)&&(flg.MosfetOverTemp == RESET))
	{
		cnt.MosfetOverTemp++;
		if(cnt.MosfetOverTemp > CNT_3S)
		{
			if(faultState == faultState_Clear)
			{
				faultState = faultState_MosfetOverTemp;
				flg.MosfetOverTemp = SET;
			}
		}
	}
	else if((ntc.mosfet < TEMP_50C)&&(flg.MosfetOverTemp == SET))
	{
		cnt.MosfetOverTemp++;
		if(cnt.MosfetOverTemp > CNT_1S)
		{
			flg.MosfetOverTemp = RESET;
			if(faultState == faultState_MosfetOverTemp)
			{
				faultState = faultState_Clear;
			}
		}
	}
	else
	{
		cnt.MosfetOverTemp = 0;
	}

	//----index 8 trans  OTP
	if((ntc.transformer > TEMP_70C)&&(flg.TransOverTemp == RESET))
	{
		cnt.TransOverTemp++;
		if(cnt.TransOverTemp > CNT_2S)
		{
			if(faultState == faultState_Clear)
			{
				faultState = faultState_TransOverTemp;
				flg.TransOverTemp = SET;
			}
		}
	}
	else if((ntc.transformer < TEMP_50C)&&(flg.TransOverTemp == SET))
	{
		cnt.TransOverTemp++;
		if(cnt.TransOverTemp > CNT_500MS)
		{
			flg.TransOverTemp = RESET;
			if(faultState == faultState_TransOverTemp)
			{
				faultState = faultState_Clear;
			}
		}
	}
	else
	{
		cnt.TransOverTemp = 0;
	}
}

//==============================================================
/*
 * input : analog input
 * return : fault status
 *
*/
int16_t convertTemp(int16_t adcNTC)
{
	float fx, fy;
	float a2, a1, a0;
	int16_t shx;

	fx = adcNTC;
	fx = (fx/10);
	//------------------------
	//------------------------
	// :NTSA0154JZ007 (150K @ 25C)
	// R Pull-Up = 47K, Rs = 100
	// Min= (-20), Max= 120

	if(adcNTC >= 976)                    // (-20) to (-10)c
	{ a2= (-48.136); a1= (9005.8); a0= (-421443);
	}
	else if(adcNTC >= 943)               // (-10) to (0)c
	{ a2= (-16.833); a1= (2924.4); a0= (-126083);
	}
	//--------------------------------------------------------------
	else if(adcNTC >= 893)               // (0) to (10)c
	{ a2= (-6.9405); a1= (1076.5); a0= (-39788);
	}

	else if(adcNTC >= 822)               // (10) to (20)c
	{ a2= (-2.9832); a1= (370.43); a0= (-8293.3);
	}

	else if(adcNTC >= 731)               // (20) to (30)c
	{ a2= (-1.0587); a1= (54.887); a0= (4643.8);
	}

	else if(adcNTC >= 625)               // (30) to (40)c
	{ a2= (-0.2825); a1= (-55.922); a0= (8598.7);
	}

	else if(adcNTC >= 513)               // (40) to (50)c
	{ a2= (-0.0402); a1= (-85.068); a0= (9469.5);
	}

	else if(adcNTC >= 407)              // (50) to (60)c
	{ a2= (0.3495); a1= (-126.29); a0= (10559);
	}

	else if(adcNTC >= 314)             // (60) to (70)c
	{ a2= (1.0415); a1= (-182.59); a0= (11707);
	}

    else if(adcNTC >= 238)             // (70) to (80)c
	{ a2= (2.0916); a1= (-246.39); a0= (12676);
	}

    else if(adcNTC >= 178)             // (80) to (90)c
	{ a2= (3.8651); a1= (-327.64); a0= (13606);
	}

    else if(adcNTC >= 133)            // (90) to (100)c
	{ a2= (8.2796); a1= (-478.93); a0= (14905);
	}

	else if(adcNTC >= 99)            // (100) to (110)c
	{ a2= (13.425); a1= (-605.15); a0= (15674);
	}

	else //(adcNTC < 99)             // over (110)c
	{ a2= (36.593); a1= (-1047.3); a0= (17791);
	}

	//------------------------
	fy= (a2 * fx);
	fy= (fy + a1) * fx;
	fy= (fy + a0);
	//------------------------
	if(fy < (-12500)) fy= (-12500);
	if(fy > 12500) fy= 12500;
	//------------------------
	shx= fy;
	return(shx);
}

//==============================================================
/*
 * input : dutyCycle follow SineTable
 * return : dutyCycle for drive Full-Bridge Converter
 *
*/
int32_t PI(uint32_t setpoint, uint32_t feedback)  // setpoint = sinTable @ every time
{
	uint32_t buf_setpoint = setpoint;
	uint32_t buf_feedback = feedback;
	int32_t pi_out;
	//int32_t seterr,

	//int32_t error;

	//error = (int32_t)buf_feedback - (int32_t)buf_setpoint;
	//seterr = (-pi_control.kp * error) / 200;	// P term
	//Int_term = Int_term + ((-pi_control.ki * error) / 200); // I term

	pi_control.error = (int32_t)buf_feedback - (int32_t)buf_setpoint;
	pi_control.ProportionalTerm = (-pi_control.kp * pi_control.error) / 200;	// P term
	pi_control.xx = ((-pi_control.ki * pi_control.error) / 200);

	pi_control.IntegralTerm  = pi_control.IntegralTerm + pi_control.xx; //((-(pi_control.ki) * (pi_control.error)) / 200); // I term

	if(pi_control.IntegralTerm > SAT_LIMIT)
	{
		pi_control.IntegralTerm = SAT_LIMIT;
	}
	if(pi_control.IntegralTerm < -(SAT_LIMIT))
	{
		pi_control.IntegralTerm = -(SAT_LIMIT);
	}

	pi_out = pi_control.ProportionalTerm + pi_control.IntegralTerm;
	pi_out += setpoint;

	//----- error counter
	//***************************
	if(pi_out >= MAX_DUTY)		// out of control
	{
		pi_out = MAX_DUTY;
		CTMax++;
	}
	else
	{
		if(CTMax > 0) CTMax--;
	}
	//***************************
	if(pi_out <= MIN_DUTY)
	{
		pi_out = MIN_DUTY;
		CTMin++;
	}
	else
	{
		if(CTMin != 0) CTMin--;
	}


	return  pi_out;
}

//==============================================================
void sysTick(void)		// 1 mS
{
	//LD_BYPSS_TOGGLE;

	cntTick++;
	if(cntTick > 1000)
	{
		cntTick = 0;

		ccr1 += 50;
		if(ccr1 >= (PWM_PERIOD_CYCLES/2)) ccr1 = 0;

	}

	readGPIO();

	//---- FAN Control
	if((ntc.mosfet > TEMP_45C)&&(powerState = powerState_On))
	{
		ntc.cnt_mosfet++;
		if(ntc.cnt_mosfet > _1SEC)
		{	ntc.cnt_mosfet = 0;
			FAN_ACTIVE;
		}
	}
	else if((ntc.mosfet < TEMP_35C)&&(powerState = powerState_On))
	{
		ntc.cnt_mosfet++;
		if(ntc.cnt_mosfet > _1SEC)
		{	ntc.cnt_mosfet = 0;
			FAN_INACTIVE;
		}
	}
	else
	{
		ntc.cnt_mosfet = 0;
	}


	//---- FAN Control
	if((ntc.transformer > TEMP_60C)&&(powerState = powerState_On))
	{
		ntc.cnt_transformer++;
		if(ntc.cnt_transformer > _1SEC)
		{	ntc.cnt_transformer = 0;
			FAN_ACTIVE;
		}
	}
	else if((ntc.transformer < TEMP_50C)&&(powerState = powerState_On))
	{
		ntc.cnt_transformer++;
		if(ntc.cnt_transformer > _1SEC)
		{	ntc.cnt_transformer = 0;
			FAN_INACTIVE;
		}
	}
	else
	{
		ntc.cnt_transformer = 0;
	}

}

//==============================================================
void powerConvertion(void)  // Drive by TIM2 ISR
{

	p = &buff_p;
	q = &buff_p;


#if (CONV_MODE == HARDWARE_CHECK)


#endif

#if (CONV_MODE == PURESINE_INVTR)

#endif

}

//==============================================================
void readGPIO(void)	// read input switch, thermo_cutoff
{
	uint8_t	activeKey =0;

	key.Mode 	= HAL_GPIO_ReadPin(KEY_MODE_GPIO_Port, KEY_MODE_Pin);
	key.Enter 	= HAL_GPIO_ReadPin(KEY_ENT_GPIO_Port, KEY_ENT_Pin);
	key.Up		= HAL_GPIO_ReadPin(KEY_UP_GPIO_Port, KEY_UP_Pin);
	key.Down 	= HAL_GPIO_ReadPin(KEY_DWN_GPIO_Port, KEY_DWN_Pin);
	key.Power	= HAL_GPIO_ReadPin(THMO_GPIO_Port, THMO_Pin);

	//------------------- Check power Switch
	if(key.Power == LOW)
	{
		flag.key_Power = SET;
		powerState = powerState_On;
	}
	else
	{
		flag.key_Power = RESET;
		powerState = powerState_Off;
	}
	//----------------------------------------------------------------------

	if(key.Mode == LOW) 	activeKey |= KEY_MODE;		// 0x01
	if(key.Enter == LOW) 	activeKey |= KEY_ENT;		// 0x02
	if(key.Up == LOW)		activeKey |= KEY_UP;		// 0x04
	if(key.Down == LOW)		activeKey |= KEY_DWN;		// 0x08

	key.Value = activeKey;	// Max Value = 0x0F = 0b00001111

	if(key.Value == KEY_MODE) 	count.key_Mode++; else count.key_Mode =0;
	if(key.Value == KEY_ENT) 	count.key_Enter++; else count.key_Enter =0;
	if(key.Value == KEY_UP) 	count.key_Up++; else count.key_Up =0;
	if(key.Value == KEY_DWN) 	count.key_Dwn++; else count.key_Dwn =0;



	switch(keyState)
	{
		case keyState_Home :
			if(count.key_Mode == _2SEC)	keyState = keyState_Selection;	// Go to selection Page
			if(count.key_Up == _100MS) flag.key_Up = SET;				// Next parameter to show
			if(count.key_Dwn == _100MS) flag.key_Dwn = SET;			    // Prev parameter to show

			break;

		case keyState_Selection :
			if(key.Value > 0) count.exitMenu = _20SEC;
			count.exitMenu--;
			if(count.exitMenu == 0) keyState = keyState_Home;			// Automatic Back to Prev Page

			if(count.key_Up == _100MS)
				{
					flag.key_Up = SET;				// Next Menu
					indexMenu++;
					if(indexMenu > 8) indexMenu =0;
				}

			if(count.key_Dwn == _100MS)
				{
					flag.key_Dwn = SET;				// Prev Menu

					if(indexMenu > 0) indexMenu--;

				}

			if(count.key_Enter == _200MS) keyState = keyState_Setting;   // Go to Setting Page
			if(count.key_Mode == _100MS) keyState = keyState_Home;		// Back to Prev Page

			break;

		case keyState_Setting :
			if(key.Value > 0) count.exitMenu = _20SEC;
			count.exitMenu--;
			if(count.exitMenu == 0) keyState = keyState_Home;			// Automatic Back to Prev Page

			if(count.key_Up == _100MS)
				{
					flag.key_Up = SET;				// Increase Value
					indexValue++;
					if(indexValue > 8) indexValue =0;
				}

			if(count.key_Dwn == _100MS)
				{
					flag.key_Dwn = SET;				// Decrease Value
					if(indexValue > 0) indexValue--;

				}

			if(count.key_Enter == _100MS) keyState = keyState_Setting;   // Save Setting Value
			if(count.key_Mode == _100MS) keyState = keyState_Selection;	// Back to Prev Page

			break;
	}

}

//==============================================================
void execDISP(void)
{

#ifdef LCD

	if(keyState != keyState)
	{
		LCD_CLEAR_DISPLAY();
	}

	switch(keyState)
	{
		case keyState_Home :
			LCD_LOCATE(1, 1);
			LCD_printf("Curr = %4d A", invIinInst);
			LCD_LOCATE(2, 1);
			LCD_printf("Volt = %4d V", invVinInst);
			break;

		case keyState_Selection :
			LCD_LOCATE(1, 1);
			LCD_printf("Menu : %2d = %2d", indexMenu, indexValue);
			LCD_LOCATE(2, 1);
			LCD_printstring("Select >> Enter \n");
			break;

		case keyState_Setting :
			LCD_LOCATE(1, 1);
			LCD_printf("Value : %2d ", indexValue);
			LCD_LOCATE(2, 1);
			LCD_printstring("Adjust >> Save \n");
			break;
	}

	prv_keyState = keyState;

#endif

}

//==============================================================
void InitParameters(void)
{
	TIM1->CCR3 = ccr3; 	// Set ADCs Trigger point

	Reset_PI();
	Vout_Check();		// define analog value

}

//==============================================================
void InitPeripheral(void)
{

//	LD_FAULT_ACTIVE;
//	LD_BYPSS_ACTIVE;
//	LD_RUN_ACTIVE;
//	HAL_Delay(500);
//	LD_FAULT_INACTIVE;
//	LD_BYPSS_INACTIVE;
//	LD_RUN_INACTIVE;

	GPIOC->BSRR = (uint32_t)GPIO_PIN_13 << 16u;  // RESET
	GPIOC->BSRR = (uint32_t)GPIO_PIN_14 << 16u;  // RESET
	GPIOC->BSRR = (uint32_t)GPIO_PIN_15 << 16u;  // RESET
	HAL_Delay(1000);
	GPIOC->BSRR = GPIO_PIN_13;  // SET
	GPIOC->BSRR = GPIO_PIN_14;  // SET
	GPIOC->BSRR = GPIO_PIN_15;  // SET
	HAL_Delay(1000);
}

//==============================================================
void ClearParameters(void)
{

}

//==============================================================
static void Vout_Check(void)
{
	VoutT = (VOUT_TARGET * VOUT_RESISTOR_RATIO) / 10000;
	VoutT = (VoutT * 4096) / REAL_3V3;
	VoutRange = VOUT_TARGET / 1000;
	VoutDelta = VOUT_TARGET - (VoutRange * 1000);
}

//==============================================================
static void Reset_PI(void)
{
	/* Reset Integral term for PI */
	Int_term = 0;
	/* Reset Count Min and Max */
	CTMax = 0;
	CTMin = 0;
	/* Set Proportional and Integral constant terms */
	pi_control.ki = 0;  // 50/200 = 25% = 0.25
	pi_control.kp = 0;  // 50/200 = 25% = 0.25
	pi_control.ProportionalTerm =0;
}

//==============================================================
void EEPromWrite(void)
{
	if(flag_Write == SET)
	{
		flag_Write = RESET;
		HAL_I2C_Mem_Write(&hi2c1, EEPROM_ADDRESS, 0, 2, write_buffer, sizeof(write_buffer), 50);

	}
}

//==============================================================
void EEPromRead(void)
{
	if(flag_Read == SET)
	{
		flag_Read = RESET;
		HAL_I2C_Mem_Read(&hi2c1, EEPROM_ADDRESS, 0, 2, read_buffer, sizeof(read_buffer), 50);

	}
}

//==============================================================
void UartTransmit(void)
{
	if(flag_tx == SET)
	{
		flag_tx = RESET;
		HAL_UART_Transmit(&huart1, tx_text_buffer, sizeof(tx_text_buffer), 10);
	}
}

//==============================================================
//====================== Callback function
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
	/*
	 * 	Tconv = Sampling time + 12.5 cycles
	 * 	With an ADCCLK = 14 MHz and a sampling time of 1.5 cycles:
	 * 	Tconv = 1.5 + 12.5 = 14 cycles = 1 µs
	 * 	ADC Clock = 14 MHz
	 * 	Sample 7.5 Clock + Conversion 12.5 = 20 Clock = 1.43 uS/ Pair
	 *	Aprox = 4.28 uS
	 *
	 * 			  | Pair1 | Pair2 | Pair3 |
	 * 			  |   AN0 |   AN2 |   AN4 |
	 * 			  |   AN1 |   AN3 |   AN5 |
	 * 	_TIM1 TRIG|_______________________|EOC|___
	 */

	//HAL_GPIO_TogglePin(LD_FLT_GPIO_Port, LD_FLT_Pin);
	GPIOC->BSRR = GPIO_PIN_13;  // SET  // LED FLT

	adc1[0] = (adcPair[0] & 0xFFFF);
	adc2[0] = ((adcPair[0] >> 16) & 0xFFFF);

	adc1[1] = (adcPair[1] & 0xFFFF);
	adc2[1] = ((adcPair[1] >> 16) & 0xFFFF);

	adc1[2] = (adcPair[2] & 0xFFFF);
	adc2[2] = ((adcPair[2] >> 16) & 0xFFFF);

	convCompleted = 1;

}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(huart);
  /* NOTE: This function should not be modified, when the callback is needed,
           the HAL_UART_RxCpltCallback could be implemented in the user file
  */
  HAL_UART_Receive_IT(&huart1, tx_echo, sizeof(tx_echo));
  //HAL_UART_Transmit(&huart1, tx_echo , sizeof(tx_echo), 10);

}

void mainProgram(void)
{
	ntc.mosfet 		= convertTemp(adc1[2]);
	ntc.transformer = convertTemp(adc2[2]);

	EEPromWrite();
	EEPromRead();
	UartTransmit();
	execDISP();   // LCD

}


/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
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
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
