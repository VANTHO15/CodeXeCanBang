/* USER CODE BEGIN Header */


/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
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
#include "HuLaNRF24L01.h"
#include "string.h"
#include "MPU6050.h"
#include "stdio.h"
#include "BMP180.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim4;

/* USER CODE BEGIN PV */
uint8_t RxAddress[] = {0x00,0xDD,0xCC,0xBB,0xAA};
uint8_t RxData[32];
float DoCao, tocdo=100;
volatile int16_t TocDo=100,PWML,PWMR,PWM;
volatile float PitchPreE,PitchE,PitchViPhan,PitchTichPhan,PitchPwmPID,PitchKP=16,PitchKI=0,PitchKD= 0;  // 0.5 0.7
volatile float PitchMongMuon=-4.5, PitchHienTai=0;

MPU6050_t MPU6050;


void ReceiveDataFromNRF24L01();
void ReadDataFromMPU6050();
void ReadDataFromBMP180();
void KhoiDong();
void Tien(uint16_t TocDo);
void Lui(uint16_t TocDo);
void Trai(uint16_t TocDo);
void Phai(uint16_t TocDo);
void DungIm();
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM4_Init(void);
static void MX_ADC1_Init(void);
static void MX_I2C2_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
 {
		if(GPIO_Pin== A_LEFT_Pin)
		{

		}
		else
		if(GPIO_Pin== B_LEFT_Pin)
		{

		}
		else
		if(GPIO_Pin== A_RIGHT_Pin)
		{

		}
		else
		if(GPIO_Pin== B_RIGHT_Pin)
		{

		}

 }
void ReceiveDataFromNRF24L01()
{
	if(isDataAvailable(2)==1)
	  {
		  NRF24_Receive(RxData);
		  HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, 1);

			  if(RxData[1]==1) //  Trái Lên  / Nâng �?ộ Cao
			  {
				  tocdo =tocdo + 0.4;
				  if(tocdo > 250)
				  {
					  tocdo= 250;
				  }
				  TocDo = (int16_t)tocdo;
			  }
			  else
			  if(RxData[1]==0) // Trái Xuống / Hạ �?ộ Cao
			  {
				  tocdo =tocdo - 0.4;
				  if(tocdo < 0)
				  {
					  tocdo = 0;
				  }
				  TocDo = (int16_t)tocdo;
			  }
			  else
			  if(RxData[2]==1) // Phải Phải / �?ông Cơ �?i Phải
			  {

			  }
			  else
			  if(RxData[2]==0) // Phải Trái / �?ộng Cơ �?i Trái
			  {

			  }
			  else
			  if(RxData[3]==1) // Phải Lên  / �?ộng Cơ Tiến
			  {
				  Tien(TocDo);
			  }
			  else
			  if(RxData[3]==0) // Phải Xuống / �?ộng Cơ Lùi
			  {
				  Lui(TocDo);
			  }
			  else
			  if(RxData[4]==1) // Trái Phải / �?ộng Cơ Quay Phải
			  {
				  Phai(TocDo);
			  }
			  else
			  if(RxData[4]==0) // Trái Trái / �?ộng Cơ Quay Trái
			  {
				  Trai(TocDo);
			  }
			  else
			  {

//				__HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_3,0);  // A_LEFT
//				__HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_1,0);  // B_LEFT
//				__HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_4,0);	// A_RIGHT
//				__HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_2,0);  // B_RIGHT
			  }

	  }
	else
	{
		HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, 0);
	}
}
void ReadDataFromMPU6050()
{
	MPU6050_Read_All(&hi2c1, &MPU6050);
	PitchHienTai = MPU6050.KalmanAngleY ;
}
void ReadDataFromBMP180()
{
	 DoCao = BMP180_GetAlt(0);

}
void Tien(uint16_t TocDo)
{

//	__HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_1,100);  // INTB_LEFT
//	__HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_2,100);  // INTB_RIGHT
//	__HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_3,100);  // INTA_LEFT
//	__HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_4,100);	// INTA_RIGHT

	__HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_3,TocDo);    // INTA_LEFT
	__HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_1,0);    // INTB_LEFT

	__HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_4,0);  // INTA_RIGHT
	__HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_2,TocDo);  // INTB_RIGHT
}
void Lui(uint16_t TocDo)
{
	__HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_3,0);    // INTA_LEFT
	__HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_1,TocDo);    // INTB_LEFT

	__HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_4,TocDo);  // INTA_RIGHT
	__HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_2,0);  // INTB_RIGHT
}
void Trai(uint16_t TocDo)
{
	__HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_3,0);    // INTA_LEFT
	__HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_1,TocDo);    // INTB_LEFT

	__HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_4,0);  // INTA_RIGHT
	__HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_2,TocDo);  // INTB_RIGHT
}
void Phai(uint16_t TocDo)
{
	__HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_3,TocDo);    // INTA_LEFT
	__HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_1,0);    // INTB_LEFT

	__HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_4,TocDo);  // INTA_RIGHT
	__HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_2,0);  // INTB_RIGHT
}
void DungIm()
{

}
void KhoiDong()
{
	 HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
	 HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);
	 HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
	 HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4);
	 NRF24_Init();
	 NRF24_RxMode(RxAddress, 10);
	 while (MPU6050_Init(&hi2c1) == 1);
	 BMP180_Start();

	 for(int i=1;i<=6;i++)
	 {
		 HAL_GPIO_TogglePin(BUZZ_GPIO_Port, BUZZ_Pin);
		 HAL_Delay(500);
	 }
}
// timer 0.01s
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim->Instance == TIM2)
	{
		ReadDataFromMPU6050();
		PitchHienTai = MPU6050.KalmanAngleY;

		PitchE =  PitchMongMuon - PitchHienTai ;
		PitchViPhan = (PitchE-PitchPreE)/0.01;
		PitchTichPhan += PitchE * 0.01;

        PWM = PitchKP*PitchE  + PitchTichPhan*PitchKI + PitchKD*PitchViPhan;
        PitchPreE = PitchE;

        if(PWM > 250)  PWM = 250;
        else if(PWM < -250 )  PWM =-250;
        if(PWM <0 ) PWM = -PWM;


        if(PitchHienTai > -4)
        {
        	__HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_3,PWM);    // INTA_LEFT
			__HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_1,0);    // INTB_LEFT

			__HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_4,0);  // INTA_RIGHT
			__HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_2,PWM);  // INTB_RIGHT
        }
        else if((PitchHienTai > -5) && (PitchHienTai < -4))
        {
        	__HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_3,0);    // INTA_LEFT
			__HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_1,0);    // INTB_LEFT

			__HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_4,0);  // INTA_RIGHT
			__HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_2,0);  // INTB_RIGHT
        }
        else
        {
        	__HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_3,0);    // INTA_LEFT
			__HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_1,PWM);    // INTB_LEFT

			__HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_4,PWM);  // INTA_RIGHT
			__HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_2,0);  // INTB_RIGHT
        }

        if((PitchHienTai > 50) || (PitchHienTai < -50))
	   {
		__HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_3,0);  // A_LEFT
		__HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_1,0);  // B_LEFT
		__HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_4,0);	// A_RIGHT
		__HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_2,0);  // B_RIGHT
	   }
	}
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
  MX_SPI1_Init();
  MX_I2C1_Init();
  MX_TIM4_Init();
  MX_ADC1_Init();
  MX_I2C2_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */

  KhoiDong();
  // bắt đầu ngắt timer 2
   HAL_TIM_Base_Start_IT(&htim2);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  HAL_GPIO_TogglePin(LED2_GPIO_Port, LED2_Pin);
	  //ReceiveDataFromNRF24L01();
	  ReadDataFromMPU6050();
	  ReadDataFromBMP180();


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
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
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

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */
  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

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
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.ClockSpeed = 400000;
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

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
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 159;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 999;
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
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 639;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 249;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */
  HAL_TIM_MspPostInit(&htim4);

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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(NRF_CSN_GPIO_Port, NRF_CSN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, BUZZ_Pin|NRF_CE_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LED1_Pin|LED2_Pin|LED3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : NRF_CSN_Pin */
  GPIO_InitStruct.Pin = NRF_CSN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(NRF_CSN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : BUZZ_Pin NRF_CE_Pin */
  GPIO_InitStruct.Pin = BUZZ_Pin|NRF_CE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : LED1_Pin LED2_Pin LED3_Pin */
  GPIO_InitStruct.Pin = LED1_Pin|LED2_Pin|LED3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : A_LEFT_Pin B_LEFT_Pin A_RIGHT_Pin B_RIGHT_Pin */
  GPIO_InitStruct.Pin = A_LEFT_Pin|B_LEFT_Pin|A_RIGHT_Pin|B_RIGHT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

  HAL_NVIC_SetPriority(EXTI1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);

  HAL_NVIC_SetPriority(EXTI2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI2_IRQn);

  HAL_NVIC_SetPriority(EXTI3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI3_IRQn);

}

/* USER CODE BEGIN 4 */

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
