/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2019 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include <stdio.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

 // double timerG = 0;



/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

CAN_HandleTypeDef hcan1;

DAC_HandleTypeDef hdac;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim5;
TIM_HandleTypeDef htim10;

UART_HandleTypeDef huart6;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_CAN1_Init(void);
static void MX_USART6_UART_Init(void);
static void MX_TIM5_Init(void);
static void MX_DAC_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM10_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

volatile int proba = 0;
volatile int boza = 0;
volatile uint16_t adc[4];
float dac1_volt = 0.5;
float dac2_volt = 1;
uint8_t dac1_byte;
uint8_t dac2_byte;
uint16_t index_rot = 0;
uint16_t index_grip=0;
char rxData[30];
char txData[64] = {'m', 'a', 's', 'a', 'R', 0x00 };
uint8_t rxPointer = 0;
uint8_t rxBuffer;
char rxFlag;
int  encGrip; // Tim 2
int  encRot; // Tim 5
int  positionRot = 0;

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart);


int main(void) {
	/* USER CODE BEGIN 1 */
	int loop_counter = 0;
	uint16_t adc_history[2][10] = { { 0 } };
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
	MX_CAN1_Init();
	MX_USART6_UART_Init();
	MX_TIM5_Init();
	MX_DAC_Init();
	MX_ADC1_Init();
	MX_TIM1_Init();
	MX_TIM10_Init();
	MX_TIM2_Init();
	/* USER CODE BEGIN 2 */
	//----------------------------------- Config  PHF \/
	HAL_TIM_Base_Start_IT(&htim1); // Enable IRQ Tim 1
	HAL_DAC_Start(&hdac, DAC_CHANNEL_1);
	HAL_DAC_Start(&hdac, DAC_CHANNEL_2);
	HAL_ADC_Start_DMA(&hadc1, (uint32_t*) adc, 4);
	dac1_byte = (uint8_t) ((dac1_volt / 3.3) * 255);
	dac2_byte = (uint8_t) ((dac2_volt / 3.3) * 255);
	HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_8B_R, dac1_byte);
	HAL_DAC_SetValue(&hdac, DAC_CHANNEL_2, DAC_ALIGN_8B_R, dac2_byte);
	HAL_UART_Receive_IT(&huart6, &rxBuffer, 1);
	HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
	HAL_TIM_Encoder_Start(&htim5, TIM_CHANNEL_ALL);
	HAL_GPIO_WritePin(GPIOB, MS1_ROT_Pin, 1);
	HAL_GPIO_WritePin(GPIOB, MS2_ROT_Pin, 1);
	HAL_GPIO_WritePin(GPIOB, MS3_ROT_Pin, 1);
	HAL_GPIO_WritePin(GPIOB, MS1_GRIP_Pin, 1);
	HAL_GPIO_WritePin(GPIOB, MS2_GRIP_Pin, 1);
	HAL_GPIO_WritePin(GPIOA, MS3_GRIP_Pin, 1);
	proba = 1;
	boza = 400;
	//   void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)    -  vika podprograma pri napulnen uart bufer ????
	//-------------------------------- Config End   PHF
	/* USER CODE END 2 */
	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	for (;;) {
		//-------------------------------------------------------------------------------------- PHF
		//	  int a;
		//test1:
		//	a = 0;
		//	goto test1;
		if (adc[0] > 100) {
			TIM5->CNT = 0;
			positionRot = 0;
		}

		if (rxFlag == 1) {
			rxFlag = 0;
			HAL_UART_Receive_IT(&huart6, &rxBuffer, 1);
			rxData[rxPointer] = (char) rxBuffer;
			rxPointer++;
			if (rxPointer >= 29)
				rxPointer = 0;

			if (rxBuffer == 0x0D) {   // call - analyze
				if (!strncmp(rxData, FF_Open, strlen(FF_Open))) {
					HAL_GPIO_TogglePin(GPIOC, LED_Pin);
					memset(rxData, 0, sizeof(rxData));
					rxPointer = 0;
				} else if (!strncmp(rxData, FF_Close, strlen(FF_Close))) {
					memset(rxData, 0, sizeof(rxData));
					rxPointer = 0;
				} else if (!strncmp(rxData, FF_Init, strlen(FF_Init))) {
					memset(rxData, 0, sizeof(rxData));
					rxPointer = 0;
				} else if (!strncmp(rxData, FF_Left, strlen(FF_Left))) {
					int test1, test2;
					FF_parse_args(rxData, &test1, &test2);
					memset(rxData, 0, sizeof(rxData));
					rxPointer = 0;
				} else if (!strncmp(rxData, FF_Right, strlen(FF_Right))) {
					int test1, test2;
					FF_parse_args(rxData, &test1, &test2);
					memset(rxData, 0, sizeof(rxData));
					rxPointer = 0;
				} else if (!strncmp(rxData, FF_Help, strlen(FF_Help))) {
					memset(rxData, 0, sizeof(rxData));
					rxPointer = 0;
				} else if (!strncmp(rxData, FF_Status, strlen(FF_Status))) {
					memset(rxData, 0, sizeof(rxData));
					rxPointer = 0;
				} else {
					memset(rxData, 0, sizeof(rxData));
					rxPointer = 0;
				}
			}
		}

		int adc_h_it = 0;
		if (loop_counter > 2) {
			// do work
			adc_history[0][adc_h_it++ % 10] = adc[0];
			adc_history[1][adc_h_it++ % 10] = adc[1];
			loop_counter = 0;
		} else
			loop_counter++;

		//	  if (HAL_GPIO_ReadPin(GPIOC,ROT_INDEX_Pin) == 1)
		//	  		{HAL_GPIO_WritePin(GPIOC, LED_Pin, 1);}
		//	  else {HAL_GPIO_WritePin(GPIOC, LED_Pin, 0);}

		//	  	  if (HAL_GPIO_ReadPin(GPIOB,GRIP_INDEX_Pin) == 1)
		//	  	  		{HAL_GPIO_WritePin(GPIOC, LED_Pin, 1);}
		//	  	  else {HAL_GPIO_WritePin(GPIOC, LED_Pin, 0);}

		//	  if (encRot == 0)
		//	  		{HAL_GPIO_WritePin(GPIOC, LED_Pin, 0);}
		//	  else {HAL_GPIO_WritePin(GPIOC, LED_Pin, 1);}

		encRot = TIM5->CNT;
		encGrip = TIM2->CNT;

#if 0
		if (rxBuffer == 0x61) // malko "a" za 1 stupka
		{ // increment rot s 1 poziciq
			HAL_GPIO_WritePin(GPIOA, STEP_ROT_Pin, 1);// step
			for (int i = 0; i < 10; i++);
			HAL_GPIO_WritePin(GPIOA, STEP_ROT_Pin, 0);
			positionRot++;
			rxBuffer = 0;
		}

		if (rxBuffer == 0x64)  // malko "d"
		{	// disable motor power
			HAL_GPIO_WritePin(GPIOB, SLEEP_ROT_Pin, 0);// disable rotor power
			rxBuffer = 0;
		}

		if (rxBuffer == 0x65)  // malko "e"

		{	// enable motor power
			HAL_GPIO_WritePin(GPIOB, SLEEP_ROT_Pin, 1);// enable rotor power
			rxBuffer = 0;
		}

		if (rxBuffer == 0x63)  // malko "c"
		{	// nulira enkodera na rotora
			TIM5->CNT = 0;
			positionRot = 0;
			rxBuffer = 0;
		}
#endif

		//HAL_Delay(5);

		encRot = TIM5->CNT;
		encGrip = TIM2->CNT;

		if (encRot == 0)
			encRot = 1;

		double razlika;
		// razlika = (( (double) positionRot / encRot) * 1000000);
		razlika = ((double) positionRot - (encRot * 1.6) + 100);

		char c[64];
		// memset(txData, 0, 32);
		memset(c, 0, sizeof(c));
		snprintf(c, sizeof(c), "RotEnc %u Cnt %u razl %u\n\r", encRot,
				positionRot, (unsigned int) razlika);

		/*          ---- tova trqbva da stane funkciq

		 HAL_GPIO_WritePin(GPIOC, RS485SW_Pin, 1);
		 HAL_UART_Transmit(&huart6, (uint8_t *)c, strlen(c), 10);  // 5 e duljina na izprashtaniq paket, 10 e milisekundi timeout
		 HAL_GPIO_WritePin(GPIOC, RS485SW_Pin, 0);
		 */
#if 0
		HAL_GPIO_WritePin(GPIOC, RS485SW_Pin, 1);
		HAL_UART_Transmit_IT(&huart6, (uint8_t *)c, strlen(c)); // 5 e duljina na izprashtaniq paket, 10 e milisekundi timeout
		HAL_Delay(FF_DELAY_TIME);
#endif

		// HAL_GPIO_TogglePin (GPIOC, LED_Pin);

		//	  	  for (int i = 0; i < 100000; i++)
		//	  		  HAL_GPIO_WritePin(GPIOC, LED_Pin, 0);
		//		 for (int i = 0; i < 100000; i++)
		//		 HAL_GPIO_WritePin(GPIOC, LED_Pin, 1);
		//  for (int i = 0; i < 100000; i++)
		// if ( proba > 0 )
		// {
		//  HAL_GPIO_TogglePin (GPIOC, LED_Pin);
		//  HAL_GPIO_WritePin(GPIOC, LED_Pin, 0);}
		// else { HAL_GPIO_WritePin(GPIOC, LED_Pin, 1);}

		//--------------------------------------------------------------------------------------- PHF
		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */
	}
	//  -------   bi trqbvalo tuka da sa mi funkciite ????????
	//  if (!strncmp(rxBuffer, "init", 4))
	//  {
	//	  // tuka vikam funkciq inicializaciq
	//  }
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

  /**Configure the main internal regulator output voltage 
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /**Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 6;
  RCC_OscInitStruct.PLL.PLLN = 100;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /**Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
  /**Enables the Clock Security System 
  */
  HAL_RCC_EnableCSS();
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

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */
  /**Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion) 
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV8;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ENABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 4;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /**Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
  */
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_480CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /**Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
  */
  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = 2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /**Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
  */
  sConfig.Channel = ADC_CHANNEL_8;
  sConfig.Rank = 3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /**Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
  */
  sConfig.Channel = ADC_CHANNEL_TEMPSENSOR;
  sConfig.Rank = 4;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief CAN1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN1_Init(void)
{

  /* USER CODE BEGIN CAN1_Init 0 */

  /* USER CODE END CAN1_Init 0 */

  /* USER CODE BEGIN CAN1_Init 1 */

  /* USER CODE END CAN1_Init 1 */
  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 16;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_2TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_16TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_2TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = DISABLE;
  hcan1.Init.AutoWakeUp = DISABLE;
  hcan1.Init.AutoRetransmission = DISABLE;
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN1_Init 2 */

  /* USER CODE END CAN1_Init 2 */

}

/**
  * @brief DAC Initialization Function
  * @param None
  * @retval None
  */
static void MX_DAC_Init(void)
{

  /* USER CODE BEGIN DAC_Init 0 */

  /* USER CODE END DAC_Init 0 */

  DAC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN DAC_Init 1 */

  /* USER CODE END DAC_Init 1 */
  /**DAC Initialization 
  */
  hdac.Instance = DAC;
  if (HAL_DAC_Init(&hdac) != HAL_OK)
  {
    Error_Handler();
  }
  /**DAC channel OUT1 config 
  */
  sConfig.DAC_Trigger = DAC_TRIGGER_NONE;
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
  if (HAL_DAC_ConfigChannel(&hdac, &sConfig, DAC_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /**DAC channel OUT2 config 
  */
  if (HAL_DAC_ConfigChannel(&hdac, &sConfig, DAC_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DAC_Init 2 */

  /* USER CODE END DAC_Init 2 */

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

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 2;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 2500;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

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

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 4294967295;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM5_Init(void)
{

  /* USER CODE BEGIN TIM5_Init 0 */

  /* USER CODE END TIM5_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM5_Init 1 */

  /* USER CODE END TIM5_Init 1 */
  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 0;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 4294967295;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim5, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM5_Init 2 */

  /* USER CODE END TIM5_Init 2 */

}

/**
  * @brief TIM10 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM10_Init(void)
{

  /* USER CODE BEGIN TIM10_Init 0 */

  /* USER CODE END TIM10_Init 0 */

  /* USER CODE BEGIN TIM10_Init 1 */

  /* USER CODE END TIM10_Init 1 */
  htim10.Instance = TIM10;
  htim10.Init.Prescaler = 0;
  htim10.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim10.Init.Period = 0;
  htim10.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  if (HAL_TIM_Base_Init(&htim10) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM10_Init 2 */

  /* USER CODE END TIM10_Init 2 */

}

/**
  * @brief USART6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART6_UART_Init(void)
{

  /* USER CODE BEGIN USART6_Init 0 */

  /* USER CODE END USART6_Init 0 */

  /* USER CODE BEGIN USART6_Init 1 */

  /* USER CODE END USART6_Init 1 */
  huart6.Instance = USART6;
  huart6.Init.BaudRate = 115200;
  huart6.Init.WordLength = UART_WORDLENGTH_8B;
  huart6.Init.StopBits = UART_STOPBITS_1;
  huart6.Init.Parity = UART_PARITY_NONE;
  huart6.Init.Mode = UART_MODE_TX_RX;
  huart6.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart6.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart6) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART6_Init 2 */

  /* USER CODE END USART6_Init 2 */

}

/** 
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void) 
{
  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, RS485SW_Pin|LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, STEP_GRIP_Pin|STEP_ROT_Pin|MS3_GRIP_Pin|SLEEP_GRIP_Pin 
                          |DIR_GRIP_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, MS1_ROT_Pin|MS2_ROT_Pin|MS3_ROT_Pin|SLEEP_ROT_Pin 
                          |DIR_ROT_Pin|MS1_GRIP_Pin|MS2_GRIP_Pin|CAN_SILENT_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : RS485SW_Pin LED_Pin */
  GPIO_InitStruct.Pin = RS485SW_Pin|LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : ROT_INDEX_Pin */
  GPIO_InitStruct.Pin = ROT_INDEX_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(ROT_INDEX_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : STEP_GRIP_Pin STEP_ROT_Pin MS3_GRIP_Pin SLEEP_GRIP_Pin 
                           DIR_GRIP_Pin */
  GPIO_InitStruct.Pin = STEP_GRIP_Pin|STEP_ROT_Pin|MS3_GRIP_Pin|SLEEP_GRIP_Pin 
                          |DIR_GRIP_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : MS1_ROT_Pin MS2_ROT_Pin MS3_ROT_Pin SLEEP_ROT_Pin 
                           DIR_ROT_Pin MS1_GRIP_Pin MS2_GRIP_Pin CAN_SILENT_Pin */
  GPIO_InitStruct.Pin = MS1_ROT_Pin|MS2_ROT_Pin|MS3_ROT_Pin|SLEEP_ROT_Pin 
                          |DIR_ROT_Pin|MS1_GRIP_Pin|MS2_GRIP_Pin|CAN_SILENT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : GRIP_INDEX_Pin */
  GPIO_InitStruct.Pin = GRIP_INDEX_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GRIP_INDEX_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart == &huart6)
		HAL_GPIO_WritePin(GPIOC, RS485SW_Pin, 0);
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
