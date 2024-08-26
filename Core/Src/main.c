/* USER CODE BEGIN Header */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
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
FDCAN_HandleTypeDef hfdcan1;
FDCAN_HandleTypeDef hfdcan2;

UART_HandleTypeDef hlpuart1;
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */
#define size_rx 64
uint8_t ubKeyNumber = 0x0;
FDCAN_RxHeaderTypeDef RxHeader;
uint8_t RxData[16];
uint8_t RxData2[16];
FDCAN_TxHeaderTypeDef TxHeader;
uint8_t TxData[8];
uint8_t TxData0[] = { 0x10, 0x32, 0x54, 0x76, 0x98, 0x00, 0x11, 0x22, 0x33,
		0x44, 0x55, 0x66, 0x77, 0x88, 0x99, 0x00, 0x55, 0x55, 0x55, 0x55, 0x55,
		0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55,
		0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55,
		0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55 };

uint8_t TxData1[] = { 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55,
		0x55, 0x55, 0x55 };
uint8_t TxData2[] = { 0xFF, 0x00, 0xFF, 0x00, 0xFF, 0x00, 0xFF, 0x00, 0xFF,
		0x00, 0x11, 0x22 };
uint8_t TxData4[];
uint8_t TxData5[];
uint8_t TxData6[];
uint8_t TxData7[];
uint8_t TxData8[];
uint8_t TxData9[];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_LPUART1_UART_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_FDCAN1_Init(void);
static void MX_FDCAN2_Init(void);
/* USER CODE BEGIN PFP */
static void FDCAN_Config(void);
static void FDCAN_Config2(void);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
typedef int (* putchar_func_t)(int);
static putchar_func_t uart_putchar=NULL;
int uart1_putchar(int ch){
	HAL_UART_Transmit(&hlpuart1, (uint8_t*) &ch, 1, HAL_MAX_DELAY);
	return  ch;
}
int uart3_putchar(int ch){
	HAL_UART_Transmit(&huart3, (uint8_t*) &ch, 1, HAL_MAX_DELAY);
	return  ch;
}


int __io_putchar(int ch) {
	if(uart_putchar != NULL) {
		return uart_putchar(ch);

	}
	return ch;

}


void Transmit_Data(FDCAN_TxHeaderTypeDef *TxHeader,uint32_t data_id) {
	TxHeader->Identifier = data_id;
	TxHeader->IdType = FDCAN_STANDARD_ID;
	TxHeader->TxFrameType = FDCAN_DATA_FRAME;
	TxHeader->DataLength = FDCAN_DLC_BYTES_64;
	TxHeader->ErrorStateIndicator = FDCAN_ESI_ACTIVE;
	TxHeader->BitRateSwitch = FDCAN_BRS_ON;
	TxHeader->FDFormat = FDCAN_FD_CAN;
	TxHeader->TxEventFifoControl = FDCAN_STORE_TX_EVENTS;
	TxHeader->MessageMarker = 0x0;
}
void Transmit_Data_EXTENDED_ID(FDCAN_TxHeaderTypeDef *TxHeader,uint32_t data_id) {
	TxHeader->Identifier = data_id;
	TxHeader->IdType = FDCAN_EXTENDED_ID;
	TxHeader->TxFrameType = FDCAN_DATA_FRAME;
	TxHeader->DataLength = FDCAN_DLC_BYTES_64;
	TxHeader->ErrorStateIndicator = FDCAN_ESI_ACTIVE;
	TxHeader->BitRateSwitch = FDCAN_BRS_ON;
	TxHeader->FDFormat = FDCAN_FD_CAN;
	TxHeader->TxEventFifoControl = FDCAN_STORE_TX_EVENTS;
	TxHeader->MessageMarker = 0x0;
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
  MX_LPUART1_UART_Init();
  MX_USART1_UART_Init();
  MX_USART3_UART_Init();
  MX_FDCAN1_Init();
  MX_FDCAN2_Init();
  /* USER CODE BEGIN 2 */
	FDCAN_Config();
	FDCAN_Config2();
	while (1) {
		uart_putchar=uart1_putchar;

		printf("lOW POWER UART1 CANFD  SIMULATION TOOL  DATA IS COMING FROM CANFD2 TO CANFD1  \n");

		uart_putchar=uart3_putchar;


		printf("UART3 CANFD  SIMULATION TOOL  DATA IS COMING FROM CANFD1 TO CANFD2  \n");




		Transmit_Data(&TxHeader,0x321);

		if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan2, &TxHeader, TxData0)!= HAL_OK) {
			/* Transmission request Error */
			Error_Handler();
		}


		HAL_Delay(100);
		for (int i = 0; i < 64; i++) {
			TxData4[i] = i;

		}
//		Transmit_Data(&TxHeader,0x322);
//
//		if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan2, &TxHeader, TxData4)!= HAL_OK) {
//			/* Transmission request Error */
//			Error_Handler();
//		}
//
//		if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader, TxData4)
//					!= HAL_OK) {
//				/* Transmission request Error */
//				Error_Handler();
//			}
		HAL_Delay(150);
//
		for (int i = 0; i < 64; i++) {
			TxData6[i] = 3 * i;

		}
		Transmit_Data(&TxHeader,0x323);

		if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan2, &TxHeader, TxData6)
				!= HAL_OK) {
			/* Transmission request Error */
			Error_Handler();
		}

		if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader, TxData6)
					!= HAL_OK) {
				/* Transmission request Error */
				Error_Handler();
			}
		HAL_Delay(200);
//
//		for (int i = 0; i < 64; i++) {
//			TxData5[i] = 2 * i;
//
//		}
//		Transmit_Data(&TxHeader,0x324);
//		if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan2, &TxHeader, TxData5)
//				!= HAL_OK) {
//			/* Transmission request Error */
//			Error_Handler();
//		}
////
//		if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader, TxData5)
//				!= HAL_OK) {
//			/* Transmission request Error */
//			Error_Handler();
//		}
//		HAL_Delay(250);
//
//		for (int i = 0; i < 64; i++) {
//				TxData9[i] = 8 * i;
//
//			}
//		Transmit_Data(&TxHeader,0x111);
//
//		if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader, TxData9)
//				!= HAL_OK) {
//			/* Transmission request Error */
//			Error_Handler();
//		}
//
//		HAL_Delay(350);
//
//		/* Add second message to Tx FIFO */
//		for (int i = 0; i < 64; i++) {
//			TxData7[i] = 5 * i;
//
//		}
//
//		Transmit_Data_EXTENDED_ID(&TxHeader,0x1111112);
//
//
//
//		if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan2, &TxHeader, TxData7)
//				!= HAL_OK) {
//			Error_Handler();
//		}
//		HAL_Delay(110);
//
//		for (int i = 0; i < 64; i++) {
//			TxData8[i] = 6 * i;
//
//		}
//		Transmit_Data_EXTENDED_ID(&TxHeader,0x1111113);
//
//		if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan2, &TxHeader, TxData8)
//				!= HAL_OK) {
//			Error_Handler();
//		}
//		HAL_Delay(110);
		/* Add third message to Tx FIFO */

	}
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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
  RCC_OscInitStruct.PLL.PLLN = 8;
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
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV4;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
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
  hfdcan1.Init.FrameFormat = FDCAN_FRAME_FD_NO_BRS;
  hfdcan1.Init.Mode = FDCAN_MODE_NORMAL;
  hfdcan1.Init.AutoRetransmission = ENABLE;
  hfdcan1.Init.TransmitPause = ENABLE;
  hfdcan1.Init.ProtocolException = ENABLE;
  hfdcan1.Init.NominalPrescaler = 8;
  hfdcan1.Init.NominalSyncJumpWidth = 16;
  hfdcan1.Init.NominalTimeSeg1 = 2;
  hfdcan1.Init.NominalTimeSeg2 = 5;
  hfdcan1.Init.DataPrescaler = 4;
  hfdcan1.Init.DataSyncJumpWidth = 5;
  hfdcan1.Init.DataTimeSeg1 = 14;
  hfdcan1.Init.DataTimeSeg2 = 5;
  hfdcan1.Init.StdFiltersNbr = 12;
  hfdcan1.Init.ExtFiltersNbr = 5;
  hfdcan1.Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;
  if (HAL_FDCAN_Init(&hfdcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN FDCAN1_Init 2 */
  /* USER CODE END FDCAN1_Init 2 */

}

/**
  * @brief FDCAN2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_FDCAN2_Init(void)
{

  /* USER CODE BEGIN FDCAN2_Init 0 */

  /* USER CODE END FDCAN2_Init 0 */

  /* USER CODE BEGIN FDCAN2_Init 1 */

  /* USER CODE END FDCAN2_Init 1 */
  hfdcan2.Instance = FDCAN2;
  hfdcan2.Init.ClockDivider = FDCAN_CLOCK_DIV1;
  hfdcan2.Init.FrameFormat = FDCAN_FRAME_FD_NO_BRS;
  hfdcan2.Init.Mode = FDCAN_MODE_NORMAL;
  hfdcan2.Init.AutoRetransmission = ENABLE;
  hfdcan2.Init.TransmitPause = ENABLE;
  hfdcan2.Init.ProtocolException = ENABLE;
  hfdcan2.Init.NominalPrescaler = 8;
  hfdcan2.Init.NominalSyncJumpWidth = 16;
  hfdcan2.Init.NominalTimeSeg1 = 2;
  hfdcan2.Init.NominalTimeSeg2 = 5;
  hfdcan2.Init.DataPrescaler = 4;
  hfdcan2.Init.DataSyncJumpWidth = 5;
  hfdcan2.Init.DataTimeSeg1 = 14;
  hfdcan2.Init.DataTimeSeg2 = 5;
  hfdcan2.Init.StdFiltersNbr = 12;
  hfdcan2.Init.ExtFiltersNbr = 5;
  hfdcan2.Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;
  if (HAL_FDCAN_Init(&hfdcan2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN FDCAN2_Init 2 */

  /* USER CODE END FDCAN2_Init 2 */

}

/**
  * @brief LPUART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_LPUART1_UART_Init(void)
{

  /* USER CODE BEGIN LPUART1_Init 0 */
  /* USER CODE END LPUART1_Init 0 */

  /* USER CODE BEGIN LPUART1_Init 1 */
  /* USER CODE END LPUART1_Init 1 */
  hlpuart1.Instance = LPUART1;
  hlpuart1.Init.BaudRate = 115200;
  hlpuart1.Init.WordLength = UART_WORDLENGTH_8B;
  hlpuart1.Init.StopBits = UART_STOPBITS_1;
  hlpuart1.Init.Parity = UART_PARITY_NONE;
  hlpuart1.Init.Mode = UART_MODE_TX_RX;
  hlpuart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  hlpuart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  hlpuart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  hlpuart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&hlpuart1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&hlpuart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&hlpuart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&hlpuart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN LPUART1_Init 2 */
  /* USER CODE END LPUART1_Init 2 */

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
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */
  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */
  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */
  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart3, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart3, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */
  /* USER CODE END USART3_Init 2 */

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
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PB14 */
  GPIO_InitStruct.Pin = GPIO_PIN_14;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

static void FDCAN_Config(void) {
	FDCAN_FilterTypeDef sFilterConfig;
	uint8_t filterIndex = 0;
	uint32_t filterIDs[4] = { 0x321, 0x322, 0x323, 0x324 };
	/* Configure Rx filter */

	sFilterConfig.FilterType = FDCAN_FILTER_DUAL;
	sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;

	for (uint8_t i = 0; i < 4; i++) {
		sFilterConfig.IdType = FDCAN_STANDARD_ID;
		sFilterConfig.FilterIndex = filterIndex++;
		sFilterConfig.FilterID1 = filterIDs[i];
		sFilterConfig.FilterID2 = filterIDs[i];
		if (HAL_FDCAN_ConfigFilter(&hfdcan1, &sFilterConfig) != HAL_OK) {
			Error_Handler();
		}

	}

	sFilterConfig.IdType = FDCAN_EXTENDED_ID;
	sFilterConfig.FilterIndex = filterIndex++;
	sFilterConfig.FilterID1 = 0x1111112;
	sFilterConfig.FilterID2 = 0x1111113;
	if (HAL_FDCAN_ConfigFilter(&hfdcan1, &sFilterConfig) != HAL_OK) {
		Error_Handler();
	}

	/* Configure global filter:
	 Filter all remote frames with STD and EXT ID
	 Reject non matching frames with STD ID and EXT ID */
	if (HAL_FDCAN_ConfigGlobalFilter(&hfdcan1, FDCAN_REJECT, FDCAN_REJECT,
	FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE) != HAL_OK) {
		Error_Handler();
	}

	/* Start the FDCAN module */
	if (HAL_FDCAN_Start(&hfdcan1) != HAL_OK) {
		Error_Handler();
	}

	if (HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE,
			0) != HAL_OK) {
		Error_Handler();
	}

	/* Prepare Tx Header */

}
static void FDCAN_Config2(void) {
	FDCAN_FilterTypeDef sFilterConfig;

	/* Configure Rx filter */
	sFilterConfig.IdType = FDCAN_STANDARD_ID;
	sFilterConfig.FilterIndex = 0;
	sFilterConfig.FilterType = FDCAN_FILTER_DUAL;
	sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
	sFilterConfig.FilterID1 = 0x111;
	sFilterConfig.FilterID2 = 0x324;
	if (HAL_FDCAN_ConfigFilter(&hfdcan2, &sFilterConfig) != HAL_OK) {
		Error_Handler();
	}

	/* Configure global filter:
	 Filter all remote frames with STD and EXT ID
	 Reject non matching frames with STD ID and EXT ID */
	if (HAL_FDCAN_ConfigGlobalFilter(&hfdcan2, FDCAN_REJECT, FDCAN_REJECT,
	FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE) != HAL_OK) {
		Error_Handler();
	}

	/* Start the FDCAN module */
	if (HAL_FDCAN_Start(&hfdcan2) != HAL_OK) {
		Error_Handler();
	}
//
	if (HAL_FDCAN_ActivateNotification(&hfdcan2, FDCAN_IT_RX_FIFO0_NEW_MESSAGE,
			0) != HAL_OK) {
		Error_Handler();
	}

	/* Prepare Tx Header */

}

/**
 * @brief  Rx FIFO 0 callback.
 * @param  hfdcan: pointer to an FDCAN_HandleTypeDef structure that contains
 *         the configuration information for the specified FDCAN.
 * @param  RxFifo0ITs: indicates which Rx FIFO 0 interrupts are signalled.
 *         This parameter can be any combination of @arg FDCAN_Rx_Fifo0_Interrupts.
 * @retval None
 */

void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs) {
	if (hfdcan == &hfdcan1) {
		printf(" DATA IS COMMING ON CANFD1 ---- \n");
		if ((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) != RESET) {
			/* Retrieve Rx messages from RX FIFO0 */
			if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &RxHeader,
					RxData) != HAL_OK) {
				/* Reception Error */
				uart_putchar=uart1_putchar;

				printf("error \n");

				Error_Handler();
			}

			/* Display LEDx */
			if ((RxHeader.Identifier == 0x321)
					&& (RxHeader.IdType == FDCAN_STANDARD_ID)) {
				uart_putchar=uart1_putchar;

				ubKeyNumber = RxData[0];
				printf("CAN1 receiving CAN ID IS 0x321 ----- ");
				for (int i = 0; i < size_rx; i++) {

					printf("%02X ", RxData[i]);

				}
				printf("\n");

			}
			if ((RxHeader.Identifier == 0x322)
					&& (RxHeader.IdType == FDCAN_STANDARD_ID)) {
				uart_putchar=uart1_putchar;

				printf("CAN1  receiving CAN ID IS 0x322 ----- ");
				for (int i = 0; i < size_rx; i++) {

					printf("%02X ", RxData[i]);

				}
				printf("\n");

			}
			if ((RxHeader.Identifier == 0x323)
					&& (RxHeader.IdType == FDCAN_STANDARD_ID)) {
				uart_putchar=uart1_putchar;

				printf("CAN1 receiving CAN ID IS 0x323 ----- ");
				for (int i = 0; i < size_rx; i++) {

					printf("%02X ", RxData[i]);

				}
				printf("\n");

			}
			if ((RxHeader.Identifier == 0x324)
					&& (RxHeader.IdType == FDCAN_STANDARD_ID)) {
				uart_putchar=uart1_putchar;

				printf("CAN1  receiving CAN ID IS 0x324 ----- ");
				for (int i = 0; i < size_rx; i++) {

					printf("%02X ", RxData[i]);

				}
				printf("\n");

			}
			if ((RxHeader.Identifier == 0x1111112)
					&& (RxHeader.IdType == FDCAN_EXTENDED_ID)) {
				uart_putchar=uart1_putchar;

				printf("CAN1 receiving CAN ID IS 0x1111112 ----- ");
				for (int i = 0; i < size_rx; i++) {

					printf("%02X ", RxData[i]);

				}
				printf("\n");

			}
			if ((RxHeader.Identifier == 0x1111113)
					&& (RxHeader.IdType == FDCAN_EXTENDED_ID)) {
				uart_putchar=uart1_putchar;

				printf("CAN1 receiving CAN ID IS 0x1111113 ----- ");
				for (int i = 0; i < size_rx; i++) {

					printf("%02X ", RxData[i]);

				}
				printf("\n");

			}
			if (HAL_FDCAN_ActivateNotification(hfdcan,
			FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0) != HAL_OK) {
				/* Notification Error */
				uart_putchar=uart1_putchar;

				printf("error 3 \n");

				Error_Handler();
			}
		}
	} else if (hfdcan == &hfdcan2) {

		uart_putchar=uart3_putchar;
		printf(" DATA IS COMMING ON CANFD2 ---- \n");

		if ((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) != RESET) {
			/* Retrieve Rx messages from RX FIFO0 */
			if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &RxHeader,
					RxData) != HAL_OK) {
				/* Reception Error */

				printf(" ERROOR IS COMMING ON CANFD2 ---- \n");


				Error_Handler();
			}

			/* Display LEDx */
		if ((RxHeader.Identifier == 0x111) && (RxHeader.IdType == FDCAN_STANDARD_ID)) {
			uart_putchar=uart3_putchar;



				char ch[50];
int array[64];
printf(" CAN2 receiving  CAN ID IS 0x111---- ");

				for (int i = 0; i < size_rx; i++) {

					printf("%02X ", RxData[i]);

				}
				printf("\n");

			}





			}

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
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
