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
#include "can.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "extern.h"
#include <string.h>
#include <stdio.h>
#include <stdint.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
CAN_RxHeaderTypeDef RxHeader;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint8_t rx_data;
uint8_t led_no=0;
volatile static uint8_t req_counter = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int _write(int file, char *ptr, int len)
{
  (void)file;
  HAL_UART_Transmit(&huart2, ptr, len, HAL_MAX_DELAY);
  return len;
}


void CAN1_Init(void);
void CAN1_Tx(void);
void CAN1_Rx(void);
void CAN_Filter_Config(void);
void Send_response(uint32_t StdId);
void LED_Manage_Output(uint8_t led_no);

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	int cnt = 0;
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
//  MX_CAN1_Init();
  MX_USART2_UART_Init();
  MX_TIM6_Init();
  /* USER CODE BEGIN 2 */
  HAL_UART_Receive_IT(&huart2, &rx_data,1); // size : 1byte
  CAN1_Init();
  CAN_Filter_Config();
  if(HAL_CAN_ActivateNotification(&hcan1, CAN_IT_TX_MAILBOX_EMPTY | CAN_IT_RX_FIFO0_MSG_PENDING | CAN_IT_BUSOFF)){
	  Error_Handler();
  }
  if(HAL_CAN_Start(&hcan1) != HAL_OK){
	  Error_Handler();
  }

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
//	  if(uart2RxIndex1 != uart2RxIndex2){
//		  printf("%c \r\n",uart2RxBuf[uart2RxIndex2++]);
//		  if(uart2RxIndex2 >= UART2_BUF_MAX){
//			  uart2RxIndex2 = 0;
//		  }
//	  }
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 50;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
	// N1 send remote frame to N2 for every 4sec
	CAN_TxHeaderTypeDef TxHeader;
	uint32_t TxMailbox;

	uint8_t message; // no meaning for data frame
	if(htim->Instance == TIM6){
		if(req_counter==4){
			//N1 sending Remote frame to N2
			TxHeader.DLC = 2; // N1 demanding 2bytes of reply
			TxHeader.StdId = 0x651;
			TxHeader.IDE = CAN_ID_STD;
			TxHeader.RTR = CAN_RTR_REMOTE; // request remote frame
			// &message : no meaning here
			if(HAL_CAN_AddTxMessage(&hcan1, &TxHeader, &message, &TxMailbox) != HAL_OK){
				Error_Handler();
			}
			req_counter = 0;
		}else{
			CAN1_Tx();
			req_counter++;
		}
	}
}
void CAN1_Tx(void){
	// create TxHeader variable
	CAN_TxHeaderTypeDef TxHeader;

	uint32_t TxMailbox;

	uint8_t message; // we gonna send LED number

	TxHeader.DLC = 1; // 1(we will send LED number)
	// use standard ID
	TxHeader.StdId = 0x65D;
	TxHeader.IDE = CAN_ID_STD;
	TxHeader.RTR = CAN_RTR_DATA; // data frame

	message = ++led_no;

	if(led_no == 4){
		led_no = 0;
	}

	HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);

	// send this message using CAN TX API
	if(HAL_CAN_AddTxMessage(&hcan1, &TxHeader, &message, &TxMailbox) != HAL_OK){
		// here, variable TxMailbox is updated by HAL_CAN_AddTxMessage API!
		Error_Handler();
	}
}

void CAN1_Init(void){
	hcan1.Instance = CAN1;
	hcan1.Init.Mode = CAN_MODE_NORMAL;
	hcan1.Init.AutoBusOff = DISABLE;
	hcan1.Init.AutoRetransmission = ENABLE;
	hcan1.Init.AutoWakeUp = DISABLE; // This is to do with low power modes
	hcan1.Init.ReceiveFifoLocked = DISABLE;
	hcan1.Init.TimeTriggeredMode = DISABLE;
	hcan1.Init.TransmitFifoPriority = DISABLE;// priority driven by the identifier of the msg

	// Settings related to CAN bit timings
	// Bit rate is 500kbps
	hcan1.Init.Prescaler = 5;
	hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
	hcan1.Init.TimeSeg1 = CAN_BS1_8TQ;
	hcan1.Init.TimeSeg2 = CAN_BS2_1TQ;

	if(HAL_CAN_Init(&hcan1) != HAL_OK){
		Error_Handler();
	}
}
void CAN1_Rx(void){
	char msg[50];
	uint8_t rcvd_msg[5];// to save received data

	// wait until some message comes to the FIFO( not euqal to 0)
	while(!HAL_CAN_GetRxFifoFillLevel(&hcan1,CAN_RX_FIFO0));

	if(HAL_CAN_GetRxMessage(&hcan1,CAN_RX_FIFO0
						,&RxHeader,rcvd_msg) != HAL_OK)
	{
		Error_Handler();
	}
	sprintf(msg,"Message Received : %s\r\n",rcvd_msg);
	HAL_UART_Transmit(&huart2,(uint8_t*)msg,strlen(msg),HAL_MAX_DELAY);
}
void CAN_Filter_Config(void){
	CAN_FilterTypeDef can1_filter_init;
	can1_filter_init.FilterActivation = ENABLE;
	can1_filter_init.FilterBank = 0;
	can1_filter_init.FilterFIFOAssignment = CAN_RX_FIFO0;
	can1_filter_init.FilterIdHigh = 0x000;
	can1_filter_init.FilterIdLow = 0x000;
	can1_filter_init.FilterMaskIdHigh = 0x000;
	can1_filter_init.FilterMaskIdLow = 0x000;
	// use MASK Mode
	can1_filter_init.FilterMode = CAN_FILTERMODE_IDMASK;
	can1_filter_init.FilterScale = CAN_FILTERSCALE_32BIT;

	if(HAL_CAN_ConfigFilter(&hcan1, &can1_filter_init) != HAL_OK){
		Error_Handler();
	}
}

void HAL_CAN_TxMailbox0CompleteCallback(CAN_HandleTypeDef *hcan){
	char msg[50];
	sprintf(msg,"Message Transmitted : M0\r\n");
	HAL_UART_Transmit(&huart2,(uint8_t*)msg,strlen(msg),HAL_MAX_DELAY);
}
void HAL_CAN_TxMailbox1CompleteCallback(CAN_HandleTypeDef *hcan){
	char msg[50];
	sprintf(msg,"Message Transmitted : M1\r\n");
	HAL_UART_Transmit(&huart2,(uint8_t*)msg,strlen(msg),HAL_MAX_DELAY);
}
void HAL_CAN_TxMailbox2CompleteCallback(CAN_HandleTypeDef *hcan){
	char msg[50];
	sprintf(msg,"Message Transmitted : M2\r\n");
	HAL_UART_Transmit(&huart2,(uint8_t*)msg,strlen(msg),HAL_MAX_DELAY);
}
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan){

	uint8_t rcvd_msg[8]; // maximum buf size = 8byte

	char msg[50];

	// we are waiting for at least one message in to the RX FIFO0
//	while(!HAL_CAN_GetRxFifoFillLevel(&hcan1, CAN_RX_FIFO0));

	if(HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0 , &RxHeader, rcvd_msg) != HAL_OK){
		Error_Handler();
	}

	if(RxHeader.StdId == 0x65D && RxHeader.RTR == 0){
		// when data frame, RTR = 0, dominant
		// if StdId is euqal to 0x65D, it's a Data frame which is for N2

		// This is data frame sent by n1 to n2
//		LED_Manage_Output(rcvd_msg[0]); // just pass only one byte(rcvd_msg[0])
		sprintf(msg,"Message Received : %#x\r\n",rcvd_msg[0]);
	}else if(RxHeader.StdId == 0x651 && RxHeader.RTR == CAN_RTR_REMOTE){
		// when remote frame, RTR =2(CAN_RTR_REMOTE), recessive
		// This is a remote frame sent by n1 to n2
		// then, N2 has to send back the response(function "Send_response")
		Send_response(RxHeader.StdId); //  send a data frame back to the Node1
		return;
	}else if(RxHeader.StdId == 0x651 && RxHeader.RTR == CAN_RTR_DATA){
		//it's a reply(data frame) by n2 to n1
		// N1 will just a display that reply that's it.
		sprintf(msg,"Reply Received : %#X\r\n",rcvd_msg[0] << 8 | rcvd_msg[1]);
	}
	HAL_UART_Transmit(&huart2,(uint8_t*)msg,strlen(msg),HAL_MAX_DELAY);
}
void HAL_CAN_ErrorCallback(CAN_HandleTypeDef *hcan){
	char msg[50];
	sprintf(msg,"CAN Error Detected\r\n");
	HAL_UART_Transmit(&huart2,(uint8_t*)msg,strlen(msg),HAL_MAX_DELAY);
}
void Send_response(uint32_t StdId){
	// send data frame
	CAN_TxHeaderTypeDef TxHeader;
	uint32_t TxMailbox;

	// send 2bytes as a response(it can be anything)
	uint8_t response[2] = {0xAB,0xCD};

	TxHeader.DLC = 2;
	TxHeader.StdId = StdId;
	TxHeader.IDE = CAN_ID_STD;
	TxHeader.RTR = CAN_RTR_DATA;

	if(HAL_CAN_AddTxMessage(&hcan1, &TxHeader, response, &TxMailbox) != HAL_OK){
		Error_Handler();
	}
}
void LED_Manage_Output(uint8_t led_no){
	switch(led_no){
	case 1:
		HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(LED4_GPIO_Port, LED4_Pin, GPIO_PIN_RESET);
		break;
	case 2:
		HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(LED4_GPIO_Port, LED4_Pin, GPIO_PIN_RESET);
		break;
	case 3:
		HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(LED4_GPIO_Port, LED4_Pin, GPIO_PIN_RESET);
		break;
	case 4:
		HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(LED4_GPIO_Port, LED4_Pin, GPIO_PIN_SET);
		break;
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
