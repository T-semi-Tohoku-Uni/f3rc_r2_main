/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include <stdio.h>
#include <math.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct{
	volatile int16_t x;
	volatile int16_t y;
	volatile float theta;
	volatile float indx;
	volatile float indy;
	volatile float indt;
}purpose;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define PI 3.1415
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
const uint8_t mv_state = 2, kaishu_state = 3, reset_state = 4;
const int16_t state_id = 0x100, vel_id = 0x300;

FDCAN_HandleTypeDef hfdcan1;

TIM_HandleTypeDef htim6;
TIM_HandleTypeDef htim7;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
FDCAN_TxHeaderTypeDef TxHeader;
FDCAN_RxHeaderTypeDef RxHeader;
FDCAN_FilterTypeDef sFilterConfig;

uint8_t TxData[8] = {};
uint8_t RxData[8] = {};
uint32_t TxMailbox;

int16_t x = 0, y = 0;
float theta = 0;

float p_x = 0, p_y = 0, p_t = 0;
purpose mokuhyo[5] = {
		{0, 1360, 0, 0, 0, 0},//toppings 1
		{0, 0, PI/2, 0, 0, 0},//starting point
		{0, 0, 0, 0, 0, 0},//toppings 2
		{0, 0, 0, 0, 0, 0},//oke x
		{0, 0, 0, 0, 0, 0}//oke(dish)
};

volatile float vx = 0, vy = 0;//mm/ms
volatile float omega = 0;
uint8_t state = 2;
uint8_t sub_state = 0;
uint8_t hantei_4 = 0;

uint16_t t_1 = 0;
uint16_t t_3 = 0;
uint16_t t_6 = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_FDCAN1_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM6_Init(void);
static void MX_TIM7_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */


void HAL_FDCAN_RxFifo1Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo1ITs){
	if ((RxFifo1ITs & FDCAN_IT_RX_FIFO1_NEW_MESSAGE) != RESET) {

	        /* Retrieve Rx messages from RX FIFO0 */

		if (HAL_FDCAN_GetRxMessage(&hfdcan1, FDCAN_RX_FIFO1, &RxHeader, RxData) != HAL_OK) {
			printf("fdcan_getrxmessage is error\r\n");
			Error_Handler();
		}

		if (RxHeader.Identifier == 0x400) {
			//printf("canlive");
			x = (int16_t)((RxData[0] << 8) | RxData[1]);
			y = (int16_t)((RxData[2] << 8) | RxData[3]);
			theta = (int16_t)((RxData[4] << 8) | RxData[5]);
			hantei_4 = RxData[6];
//			float theta_syf = (int16_t)((RxData[4] << 8) | RxData[5]);
//			theta_syf /= 10000;
//			if (theta >= 0) {
//				theta += theta_syf;
//			}
//			else {
//				theta -= theta_syf;
//			}
			theta /= 400;
		}
	}
}

void FDCAN_RxTxSettings(void){
	FDCAN_FilterTypeDef FDCAN_Filter_settings;
	FDCAN_Filter_settings.IdType = FDCAN_STANDARD_ID;
	FDCAN_Filter_settings.FilterIndex = 0;
	FDCAN_Filter_settings.FilterType = FDCAN_FILTER_RANGE;
	FDCAN_Filter_settings.FilterConfig = FDCAN_FILTER_TO_RXFIFO1;
	FDCAN_Filter_settings.FilterID1 = 0x000;
	FDCAN_Filter_settings.FilterID2 = 0x600;

	TxHeader.Identifier = 0x000;
	TxHeader.IdType = FDCAN_STANDARD_ID;
	TxHeader.TxFrameType = FDCAN_DATA_FRAME;
	TxHeader.DataLength = FDCAN_DLC_BYTES_8;
	TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
	TxHeader.BitRateSwitch = FDCAN_BRS_ON;
	TxHeader.FDFormat = FDCAN_FD_CAN;
	TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
	TxHeader.MessageMarker = 0;


	if (HAL_FDCAN_ConfigFilter(&hfdcan1, &FDCAN_Filter_settings) != HAL_OK){
		printf("fdcan_configfilter is error\r\n");
		Error_Handler();
	}

	if (HAL_FDCAN_ConfigGlobalFilter(&hfdcan1, FDCAN_REJECT, FDCAN_FILTER_REJECT, FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE) != HAL_OK){
		printf("fdcan_configglobalfilter is error\r\n");
		Error_Handler();
	}

	if (HAL_FDCAN_Start(&hfdcan1) != HAL_OK) {
		printf("fdcan_start is error\r\n");
		Error_Handler();
	}

	if (HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO1_NEW_MESSAGE, 0) != HAL_OK){
		printf("fdcan_activatenotification is error\r\n");
		Error_Handler();
	}
}

void state_Rx(int8_t st, uint8_t sub_st){
	TxHeader.Identifier = state_id;
	uint8_t TxData_state[8] = {};
	TxData_state[0] = st;
	TxData_state[1] = sub_st;

	if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader, TxData_state) != HAL_OK) {
		printf("addmassage_state is error\r\n");
		Error_Handler();
	}
}

void vel_Rx(int16_t V_X, int16_t V_Y, int16_t Omega){
	TxHeader.Identifier = vel_id;
	uint8_t TxData_vel[8] = {};

//	uint16_t Omega_syi;
//	uint8_t Omega_se;
//	if (Omega > 0){
//		Omega_se = (int16_t)Omega;
//		float Omega_syf = Omega - (int16_t)Omega;
//		Omega_syi = (uint16_t)(Omega_syf*10000);
//	}
//	else{
//		Omega_se = (int16_t)Omega;
//		float Omega_k = -Omega;
//		float Omega_syf = Omega_k - (int16_t)Omega_k;
//		Omega_syi = (uint16_t)(Omega_syf*10000);
//
//	}


	TxData_vel[0] = (int16_t)(V_X) >> 8;
	TxData_vel[1] = (uint8_t)((int16_t)(V_X) & 0xff);
	TxData_vel[2] = (int16_t)(V_Y) >> 8;
	TxData_vel[3] = (uint8_t)((int16_t)(V_Y) & 0xff);
	TxData_vel[4] = (int16_t)(Omega) >> 8;
	TxData_vel[5] = (uint8_t)((int16_t)(Omega) & 0xff);
	//TxData_vel[6] = Omega_se;
	//printf("%d\r\n",TxData_vel[3]);

	if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader, TxData_vel) != HAL_OK){
		printf("add_message_vel is error\r\n");
		Error_Handler();
	}
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
	if (&htim6 == htim) {
		if (mv_state == state){
			uint8_t m_state = sub_state;
			float k_p = 0.001, k_i = 0, k_d = 0;
			float k_p_t = 1, k_i_t = 0, k_d_t = 0;
			float hensax = mokuhyo[m_state].x - x;
			float dx = (float)x - p_x;
			mokuhyo[m_state].indx += hensax;
			vx = (k_p*hensax + k_i*mokuhyo[m_state].indx + k_d*dx);

			p_x = x;

			float hensay = mokuhyo[m_state].y -y;
			float dy = (float)y - p_y;
			mokuhyo[m_state].indy += hensay;
			vy = (k_p*hensay + k_i*mokuhyo[m_state].indy + k_d*dy);
			//int Dy=vy*1000;
			//printf("dy:%d\r\n",Dy);

			p_y = y;

			float hensat = mokuhyo[m_state].theta - theta;
			float dt = theta - p_t;
			mokuhyo[m_state].indt += hensat;
			omega =(k_p_t*hensat + k_i_t*mokuhyo[m_state].indt + k_d_t*dt);

			p_t = theta;
			//printf("%d\r\n", (int)(omega*100));
		}
		else if (kaishu_state == state) {
			if (0 == sub_state){
			vx = 0;
			vy = 0.01;
			omega = 0;
			}
			else if (1 == sub_state) {
				vx = -0.05;
				vy = 0;
				omega = 0;
			}
		}
		else if (reset_state == state) {
			if (0 == sub_state || 2 == sub_state || 4 == sub_state || 6 == sub_state){
				vx = 0;
				vy = 0.05;
				omega = 0;
			}
			if (1 == sub_state || 3 == sub_state || 5 == sub_state) {
				vx = 0.05;
				vy = 0;
				omega = 0;
			}
		}

		else if (128 == state) {
			vx = 0;
			vy = 0;
			omega = 0;
		}
		else{
			vx = 0;
			vy = 0;
			omega = 0;
		}
		int16_t vx_tusin = (int16_t)(vx * 1000);
		int16_t vy_tusin = (int16_t)(vy * 1000);
		int16_t omega_tusin = (int16_t)(omega * 400);
		vel_Rx(vx_tusin, vy_tusin, omega_tusin);

	}

	if (&htim7 == htim) {
		if (0 == state) {
			if (NULL){
				//switch
				state = 1;
				sub_state = 0;
			}
		}
		else if (1 == state) {
			if (NULL) {
				if (1000 <= t_1){
					if (NULL){
						t_1 = 0;
						state = 2;
						sub_state = 0;
					}
				}
				else {
					t_1++;
				}
			}
			else {
				t_1++;
			}
		}
		else if (2 == state) {
			if (((fabsf(x-mokuhyo[sub_state].x) < 50) && (fabsf(y-mokuhyo[sub_state].y) < 50)) && (fabsf(theta-mokuhyo[sub_state].theta < 0.1))){
				if (0 == sub_state) {
					state = 3;
					sub_state = 0;
				}
				else if (1 == sub_state) {
					state = 4;
					sub_state = 1;
				}
				else if (2 == sub_state) {
					state = 6;
					sub_state = 0;
				}
				else if (3 == sub_state) {
					state = 2;
					sub_state = 4;
				}
				else if (4 == sub_state) {
					state = 4;
					sub_state = 2;
				}
				else if (5 == sub_state) {
					state = 3;
					sub_state = 1;
				}
				else if (6 == sub_state) {
					state = 4;
					sub_state = 4;
				}
				else if (7 == sub_state) {
					state = 6;
					sub_state = 1;
				}
				else {
					state = 128;
					sub_state = 128;
				}
			}
		}
			//else
		else if (3 == state) {
			if (0 == sub_state){
				if (t_3 > 10000) {
					t_3 = 0;
					state = 4;
					sub_state = 0;
				}
				else {
					t_3++;
				}
			}
			else if (1 == sub_state) {
				if (t_3 > 10000) {
					t_3 = 0;
					state = 4;
					sub_state = 3;
				}
				else {
					t_3++;
				}
			}
			else {
				state = 128;
				sub_state = 128;
			}
		}
		else if (4 == state) {
			if (0 == sub_state) {
				if (1 == hantei_4){
					state = 2;
					sub_state = 1;
				}
			}
			else if (1 == sub_state) {
				if (1 == hantei_4) {
					state = 2;
					sub_state = 2;
				}
			}
			else if (2 == sub_state) {
				if (1 == hantei_4) {
					state = 2;
					sub_state = 5;
				}
			}
			else if (3 == sub_state) {
				if (1 == hantei_4) {
					state = 2;
					sub_state = 6;
				}
			}
			else if (4 == sub_state) {
				if (1 == hantei_4) {
					state = 2;
					sub_state = 7;
				}
			}
/*			else if (5 == sub_state) {
				if (1 == hantei_4) {
					state = 2;
					sub_state = 3;
				}
			}
			else if (6 == sub_state) {
				if (1 == hantei_4) {
					state = 2;
					sub_state = 4;
				}
			}*/
			else {
				state = 128;
				sub_state = 128;
			}
		}
		else if (5 == state) {
			if (NULL) {//raspberry pi
				state = 6;
				sub_state = 0;
			}
		}
		else if (6 == state) {
			if (0 == sub_state) {
				if (t_6 < 5000) {
					t_6 = 0;
					state = 2;
					sub_state = 3;
				}
				else {
					t_6++;
				}
			}
			if (1 == sub_state) {
				if (t_6 < 5000) {
					t_6 = 0;
					state = 128;
					sub_state = 128;
				}
				else {
					t_6++;
				}
			}
			if (NULL) {//finish open
				state = 128;
				sub_state = 128;
			}
		}

		else if (128 == state) {
			HAL_GPIO_WritePin(Boad_LED_GPIO_Port, Boad_LED_Pin, GPIO_PIN_SET);
		}
		else {
			state = 128;
			sub_state = 128;
		}
		state_Rx(state, sub_state);
	}
}

int _write(int file, char *ptr, int len)
{
    HAL_UART_Transmit(&huart2,(uint8_t *)ptr,len,10);
    return len;
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
	setbuf(stdout, NULL);
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
  MX_FDCAN1_Init();
  MX_USART2_UART_Init();
  MX_TIM6_Init();
  MX_TIM7_Init();
  /* USER CODE BEGIN 2 */
  printf("start\r\n");
  FDCAN_RxTxSettings();
  printf("can_main_start\r\n");
  HAL_TIM_Base_Start_IT(&htim6);
  HAL_TIM_Base_Start_IT(&htim7);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  //printf("x: %d,y: %d\r\n",x,y);
	  //printf("dy: %f\r\n",dy);
	  //printf("%d.%d\r\n", (int)vy, (int)(100*(vy-(int)vy)));
	  //printf("%d\r\n", (int)(omega*100));
	  printf("%d, %d\r\n", state, sub_state);
	  HAL_Delay(1);
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
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV1;
  RCC_OscInitStruct.PLL.PLLN = 10;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief FDCAN1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_FDCAN1_Init(void)
{

  /* USER CODE BEGIN FDCAN1_Init 0 */

  /* USER CODE END FDCAN1_Init 0 */

  /* USER CODE BEGIN FDCAN1_Init 1 */

  /* USER CODE END FDCAN1_Init 1 */
  hfdcan1.Instance = FDCAN1;
  hfdcan1.Init.ClockDivider = FDCAN_CLOCK_DIV1;
  hfdcan1.Init.FrameFormat = FDCAN_FRAME_FD_BRS;
  hfdcan1.Init.Mode = FDCAN_MODE_NORMAL;
  hfdcan1.Init.AutoRetransmission = DISABLE;
  hfdcan1.Init.TransmitPause = DISABLE;
  hfdcan1.Init.ProtocolException = DISABLE;
  hfdcan1.Init.NominalPrescaler = 4;
  hfdcan1.Init.NominalSyncJumpWidth = 1;
  hfdcan1.Init.NominalTimeSeg1 = 15;
  hfdcan1.Init.NominalTimeSeg2 = 4;
  hfdcan1.Init.DataPrescaler = 2;
  hfdcan1.Init.DataSyncJumpWidth = 1;
  hfdcan1.Init.DataTimeSeg1 = 15;
  hfdcan1.Init.DataTimeSeg2 = 4;
  hfdcan1.Init.StdFiltersNbr = 1;
  hfdcan1.Init.ExtFiltersNbr = 0;
  hfdcan1.Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;
  if (HAL_FDCAN_Init(&hfdcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN FDCAN1_Init 2 */

  /* USER CODE END FDCAN1_Init 2 */

}

/**
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 9;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 7999;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

}

/**
  * @brief TIM7 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM7_Init(void)
{

  /* USER CODE BEGIN TIM7_Init 0 */

  /* USER CODE END TIM7_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM7_Init 1 */

  /* USER CODE END TIM7_Init 1 */
  htim7.Instance = TIM7;
  htim7.Init.Prescaler = 9;
  htim7.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim7.Init.Period = 7999;
  htim7.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim7) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim7, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM7_Init 2 */

  /* USER CODE END TIM7_Init 2 */

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
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart2, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart2, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart2) != HAL_OK)
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
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(Boad_LED_GPIO_Port, Boad_LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : Boad_LED_Pin */
  GPIO_InitStruct.Pin = Boad_LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(Boad_LED_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
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
  __disable_irq();
  printf("Error\r\n");
  HAL_GPIO_WritePin(Boad_LED_GPIO_Port, Boad_LED_Pin, GPIO_PIN_SET);
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
