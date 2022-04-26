/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
#define ARM_MATH_CM4
#include "arm_math.h"
#define _USE_MATH_DEFINES
#include "math.h"
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
TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim11;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
float RealDegree = 0.0;
float Degree_Past = 0.0;
float Degree_offset = 0.0;
float AbsDegree = 0.0;
float AbsDegree_Past = 0.0;
float Velocity = 0.0;
float Velocity_Past = 0.0;
float Acceleration = 0.0;
float AbsDegree_kalman = 0.0;
float Velocity_kalman = 0.0;
float Acceleration_kalman = 0.0;

uint64_t TimeStamp = 0;
uint64_t TimeStamp_Past = 0;
uint64_t _micros = 0;

const float delta_t = 0.01;

float32_t A_data[9] =
{
	1.0,	delta_t,	delta_t*delta_t/2.0,
	0.0, 	1.0,			delta_t,
	0.0,	0.0,			1.0
};

float32_t B_data[3] =
{
	0.0,
	0.0,
	0.0
};

float32_t C_data[3] =
{
	1.0,	0.0,	0.0
};

float32_t D_data[1] =
{
	0.0
};

float32_t G_data[3] =
{
	delta_t*delta_t*delta_t/6.0,
	delta_t*delta_t/2.0,
	delta_t
};

float32_t U_data[1] =
{
	0.0
};

float32_t X_data[3] =
{
	0.0,
	0.0,
	0.0
};

float32_t Y_data[1] =
{
	0.0
};

float32_t I_data[9] =
{
	1.0,	0.0,	0.0,
	0.0,	1.0,	0.0,
	0.0,	0.0,	1.0
};

float32_t At_data[9];
float32_t Ct_data[3];
float32_t Gt_data[3];
float32_t Xp_data[3];
float32_t Xp1_data[3];
float32_t Xp2_data[3];
float32_t P_data[9] =
{
	0.0,	0.0,	0.0,
	0.0,	0.0,	0.0,
	0.0,	0.0,	0.0
};
float32_t Pp_data[9];
float32_t APAt_data[9];
float32_t Q_data[9];
float32_t Yp_data[1];
float32_t Yp1_data[1];
float32_t Yp2_data[1];
float32_t Yd_data[1];
float32_t S_data[1];
float32_t Si_data[1];
float32_t R_data[1] =
{
	0.0001
};
float32_t PpCt_data[3];
float32_t CPpCt_data[1];
float32_t K_data[3];
float32_t KYd_data[3];
float32_t KC_data[9];
float32_t IKC_data[9];

arm_matrix_instance_f32 A;
arm_matrix_instance_f32 At;
arm_matrix_instance_f32 B;
arm_matrix_instance_f32 C;
arm_matrix_instance_f32 Ct;
arm_matrix_instance_f32 D;
arm_matrix_instance_f32 G;
arm_matrix_instance_f32 Gt;
arm_matrix_instance_f32 U;
arm_matrix_instance_f32 X;
arm_matrix_instance_f32 Y;
arm_matrix_instance_f32 I;


arm_matrix_instance_f32 Xp;
arm_matrix_instance_f32 Xp1;
arm_matrix_instance_f32 Xp2;

arm_matrix_instance_f32 P;
arm_matrix_instance_f32 Pp;
arm_matrix_instance_f32 APAt;
arm_matrix_instance_f32 Q;

arm_matrix_instance_f32 Yp;
arm_matrix_instance_f32 Yp1;
arm_matrix_instance_f32 Yp2;
arm_matrix_instance_f32 Yd;

arm_matrix_instance_f32 S;
arm_matrix_instance_f32 Si;
arm_matrix_instance_f32 R;
arm_matrix_instance_f32 PpCt;
arm_matrix_instance_f32 CPpCt;
arm_matrix_instance_f32 K;
arm_matrix_instance_f32 KYd;
arm_matrix_instance_f32 KC;
arm_matrix_instance_f32 IKC;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM11_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */
void UpdatePosition();
void Calculate_Velocity();
void Calculate_Acceleration();
void Unwrapped();
void Kalman();
uint64_t micros();
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	arm_mat_init_f32(&A,3,3,A_data);
	arm_mat_init_f32(&At,3,3,At_data);
	arm_mat_init_f32(&B,3,1,B_data);
	arm_mat_init_f32(&C,1,3,C_data);
	arm_mat_init_f32(&Ct,3,1,Ct_data);
	arm_mat_init_f32(&D,1,1,D_data);
	arm_mat_init_f32(&G,3,1,G_data);
	arm_mat_init_f32(&Gt,1,3,Gt_data);
	arm_mat_init_f32(&U,1,1,U_data);
	arm_mat_init_f32(&X,3,1,X_data);
	arm_mat_init_f32(&Y,1,1,Y_data);
	arm_mat_init_f32(&I,3,3,I_data);

	arm_mat_init_f32(&Xp,3,1,Xp_data);
	arm_mat_init_f32(&Xp1,3,1,Xp1_data);
	arm_mat_init_f32(&Xp2,3,1,Xp2_data);

	arm_mat_init_f32(&P,3,3,P_data);
	arm_mat_init_f32(&Pp,3,3,Pp_data);
	arm_mat_init_f32(&APAt,3,3,APAt_data);
	arm_mat_init_f32(&Q,3,3,Q_data);

	arm_mat_init_f32(&Yp,1,1,Yp_data);
	arm_mat_init_f32(&Yp1,1,1,Yp1_data);
	arm_mat_init_f32(&Yp2,1,1,Yp2_data);
	arm_mat_init_f32(&Yd,1,1,Yd_data);

	arm_mat_init_f32(&S,1,1,S_data);
	arm_mat_init_f32(&Si,1,1,Si_data);
	arm_mat_init_f32(&R,1,1,R_data);
	arm_mat_init_f32(&PpCt,3,1,PpCt_data);
	arm_mat_init_f32(&CPpCt,1,1,CPpCt_data);
	arm_mat_init_f32(&K,3,1,K_data);
	arm_mat_init_f32(&KYd,3,1,KYd_data);
	arm_mat_init_f32(&KC,3,3,KC_data);
	arm_mat_init_f32(&IKC,3,3,IKC_data);
	//------------------------------------------------------
	arm_mat_trans_f32(&A,&At);
	arm_mat_trans_f32(&G, &Gt);
	arm_mat_mult_f32(&G,&Gt,&Q);
	arm_mat_scale_f32(&Q,0.5,&Q);
	arm_mat_trans_f32(&C, &Ct);


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
  MX_USART2_UART_Init();
  MX_TIM11_Init();
  MX_TIM1_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
  //start Microsec timer
  HAL_TIM_Base_Start_IT(&htim11);
  //start TIM for sampling time at 0.01 or 100 Hz
  HAL_TIM_Base_Start_IT(&htim3);
  //start Encoder TIM
  HAL_TIM_Encoder_Start(&htim1, TIM_CHANNEL_ALL);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  UpdatePosition();

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
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 100;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
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

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 3071;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim1, &sConfig) != HAL_OK)
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
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 99;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 9999;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief TIM11 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM11_Init(void)
{

  /* USER CODE BEGIN TIM11_Init 0 */

  /* USER CODE END TIM11_Init 0 */

  /* USER CODE BEGIN TIM11_Init 1 */

  /* USER CODE END TIM11_Init 1 */
  htim11.Instance = TIM11;
  htim11.Init.Prescaler = 0;
  htim11.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim11.Init.Period = 65535;
  htim11.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim11.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim11) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM11_Init 2 */

  /* USER CODE END TIM11_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

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
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void UpdatePosition()
{
	uint16_t RawRead = TIM1->CNT;
//	RealDegree = (RawRead/3072.0)*360.0;
	RealDegree = (RawRead/3072.0)*(2*M_PI);
}

uint64_t micros()
{
	return _micros + htim11.Instance->CNT;
}

void Kalman()
{
	arm_mat_mult_f32(&A,&X,&Xp1);
	arm_mat_mult_f32(&B,&U,&Xp2);
	arm_mat_add_f32(&Xp1,&Xp2,&Xp);

	arm_mat_mult_f32(&P,&At,&APAt);
	arm_mat_mult_f32(&A,&APAt,&APAt);
	arm_mat_add_f32(&APAt,&Q,&Pp);

	arm_mat_mult_f32(&C,&Xp,&Yp1);
	arm_mat_mult_f32(&D,&U,&Yp2);
	arm_mat_add_f32(&Yp1,&Yp2,&Yp);
	arm_mat_sub_f32(&Y,&Yp,&Yd);

	arm_mat_mult_f32(&Pp,&Ct,&PpCt);
	arm_mat_mult_f32(&C,&PpCt,&CPpCt);
	arm_mat_add_f32(&CPpCt,&R,&S);
	arm_mat_mult_f32(&Pp,&Ct,&K);
	arm_mat_inverse_f32(&S,&Si);
	arm_mat_mult_f32(&K,&Si,&K);

	arm_mat_mult_f32(&K,&Yd,&KYd);
	arm_mat_add_f32(&Xp,&KYd,&X);

	arm_mat_mult_f32(&K,&C,&KC);
	arm_mat_sub_f32(&I,&KC,&IKC);
	arm_mat_mult_f32(&IKC,&Pp,&P);
}

void Unwrapped()
{
	static uint8_t check = 0;
	if(RealDegree - Degree_Past <= -M_PI && check != 0)
	{
		Degree_offset = Degree_offset + (2*M_PI);
		AbsDegree = Degree_offset + RealDegree;
	}
	else if(RealDegree - Degree_Past >= M_PI && check != 0)
	{
		Degree_offset = Degree_offset - (2*M_PI);
		AbsDegree = Degree_offset + RealDegree;
	}
	else
	{
		check = 1;
		AbsDegree = Degree_offset + RealDegree;
	}
	Degree_Past = RealDegree;
}

void Calculate_Velocity()
{
//	Velocity = (AbsDegree - AbsDegree_Past)/((TimeStamp - TimeStamp_Past)/1000000);
	Velocity = (AbsDegree - AbsDegree_Past)/0.01;
	AbsDegree_Past = AbsDegree;
}

void Calculate_Acceleration()
{
//	Acceleration = (Velocity - Velocity_Past)/((TimeStamp - TimeStamp_Past)/1000000);
	Acceleration = (Velocity - Velocity_Past)/0.01;
	Velocity_Past = Velocity;
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
 if(htim == &htim11)
 {
	 _micros += 65535;
 }
 if(htim == &htim3)
 {
	 Unwrapped();
//	 TimeStamp = _micros;
	 Calculate_Velocity();
	 Calculate_Acceleration();
//	 TimeStamp_Past = TimeStamp;
	 Y_data[0] = AbsDegree;
	 Kalman();
	 AbsDegree_kalman = X_data[0];
	 Velocity_kalman = X_data[1];
	 Acceleration_kalman = X_data[2];
 }
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

