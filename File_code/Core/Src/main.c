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

#include "gps_neo6.h"
#include "stdio.h"
#include "string.h"
//#include "GNSS.h"


#define SCALE_MAX 90
#define SCALE_MIN -90


#define X_MAX 2360
#define Y_MAX 2355
#define Z_MAX 2380

#define X_MIN 1554
#define Y_MIN 1549
#define Z_MIN 1566


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
DMA_HandleTypeDef hdma_adc1;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart6;

/* USER CODE BEGIN PV */

/* Private variables ---------------------------------------------------------*/
uint32_t data[3];
char Message1[160] = {'\0'};
char Message2[160] = {'\0'};
char Message3[160] = {'\0'};
char Message4[160] = {'\0'};
int32_t x_angle,y_angle,z_angle;
#define SIZEARRAY 100
char enter[2]={0x1A};
char Rx_Buffer[SIZEARRAY];
char Rx_Data[SIZEARRAY];
int flag1 = 0, flag2 = 0;
int UART6_count;
int UART6_time;
int Status = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART6_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc){
  	x_angle = (int32_t)((float)((SCALE_MAX-SCALE_MIN)*(data[0]-X_MIN))/(X_MAX-X_MIN))+SCALE_MIN;
  	y_angle = (int32_t)((float)((SCALE_MAX-SCALE_MIN)*(data[1]-Y_MIN))/(Y_MAX-Y_MIN))+SCALE_MIN;
  	z_angle = (int32_t)((float)(SCALE_MAX-SCALE_MIN)*(data[2]-Z_MIN)/(Z_MAX-Z_MIN))+SCALE_MIN;

  }
void delBuf(char* data)
{
	int len = strlen(data);
	for( i = 0; i < len; i++)
	{
		data[i] = 0;
	}
}
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	 if (huart -> Instance == USART1)
	 {
		 NEO6_ReceiveUartChar(&GpsState);
	 }
	 if (huart->Instance == USART6)// kiem tra ngat o bo UART6
		  {
			 if((HAL_GetTick() - UART6_time > 100)||(UART6_count == SIZEARRAY)){
				UART6_count = 0;
				delBuf(Rx_Buffer);// xoa bo dem nho
		  }
			 Rx_Buffer[UART6_count++]=Rx_Data[0];// tang count len 1 va lay byte data tu UART6_RXData
			 UART6_time = HAL_GetTick();


				if(Rx_Buffer[50]=='V' && Rx_Buffer[51]=='I' && Rx_Buffer[52]=='T' && Rx_Buffer[53]=='R' && Rx_Buffer[54]=='I' ){
					flag1=1;
				}
				if(Rx_Buffer[2]=='B' && Rx_Buffer[3]=='U' && Rx_Buffer[4]=='S' && Rx_Buffer[5]=='Y' ){
					flag2=1;
				}

		 } HAL_UART_Receive_IT(&huart6, (uint8_t *)Rx_Data,1);
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
  MX_USART1_UART_Init();
  MX_USART6_UART_Init();
  /* USER CODE BEGIN 2 */
  HAL_ADC_Start_DMA(&hadc1, (uint32_t*)data, 3);
  NEO6_Init(&GpsState, &huart1);
  HAL_UART_Receive_IT(&huart6, (uint8_t *)Rx_Data,1);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  while (1)
  {
		  if (x_angle < -15 || x_angle > 165 && flag1 == 0 ){
			  HAL_Delay(5000);
			  if (x_angle < -15 || x_angle > 165 ){
				  Status ++;
				 if (Status == 1){
					 for(int j = 0; j<100000; j++){
					 	  GetData();
					 }
					 SendSms();
					 while(flag2 == 0){
						 Call();
					 }
				 }
			 }
			  HAL_Delay(10000);
			 if (x_angle > -15 || x_angle < 165 ){
			  		flag2 = 0;
			  		Status = 0;
			 }
		  }
		  if( flag1 == 1){
			  for(int j = 0; j<100000; j++){
			  		GetData();
			  }
			  SendSms1();
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
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ENABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 3;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_480CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = 2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = 3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

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
  huart1.Init.BaudRate = 9600;
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
  huart6.Init.BaudRate = 9600;
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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_RESET);

  /*Configure GPIO pin : PA7 */
  GPIO_InitStruct.Pin = GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void GetData(){

	            NEO6_Task(&GpsState);
				if(NEO6_IsFix(&GpsState)){
						sprintf(Message1, "Time: %02d:%02d:%02d\n\r", GpsState.Hour, GpsState.Minute, GpsState.Second);
						sprintf(Message2, "Date: %02d.%02d.20%02d\n\r", GpsState.Day, GpsState.Month, GpsState.Year);
						sprintf(Message3, "Vi Tri Xe Hien Tai  https://maps.google.com/maps?q=%2f,%2f\n\r", GpsState.Latitude, GpsState.Longitude);
				}

				 else{
						sprintf(Message4, "No Fix\n\r");
				 }
}
  void SendSms(){
  	  		   	HAL_Delay(2000);
  	  			HAL_UART_Transmit(&huart6, (uint8_t *)"AT+CMGDA=\"DEL ALL\"\r\n", sizeof("AT+CMGDA=\"DEL ALL\"\r\n"),20);
  	  			HAL_Delay(1000);
  	  		   	HAL_UART_Transmit(&huart6, (uint8_t *)"AT+CMGS=\"0944022135\"\r\n", strlen("AT+CMGS=\"0944022135\"\r\n"), 20);
  	  		   	HAL_Delay(5000);
  	  		   	HAL_UART_Transmit(&huart6, (uint8_t *)"Xe Ban Bi Nga\r\n",strlen("Xe Ban Bi Nga\r\n"),20);
  	  			HAL_UART_Transmit(&huart6, (uint8_t *)(char *)Message3,strlen((char *)Message3),100);
  	  			HAL_UART_Transmit(&huart6, (uint8_t *)(char *)Message2,strlen((char *)Message2),20);
  	  			HAL_UART_Transmit(&huart6, (uint8_t *)(char *)Message1,strlen((char *)Message1),20);
  	  		   	HAL_UART_Transmit(&huart6, (uint8_t *)enter,1,10);
  }

  void SendSms1(){

  				HAL_Delay(1000);
  				HAL_UART_Transmit(&huart6, (uint8_t *)"AT+CMGDA=\"DEL ALL\"\r\n", sizeof("AT+CMGDA=\"DEL ALL\"\r\n"), 20);
  				HAL_Delay(1000);
  	  		    HAL_UART_Transmit(&huart6, (uint8_t *)"AT+CMGS=\"0944022135\"\r\n",strlen("AT+CMGS=\"0944022135\"\r\n"), 20);
  	  		  	HAL_Delay(1000);
  	  		  	HAL_UART_Transmit(&huart6, (uint8_t *)(char *)Message3,strlen((char *)Message3),100);
  	  		  	HAL_UART_Transmit(&huart6, (uint8_t *)(char *)Message2,strlen((char *)Message2),20);
  	  		  	HAL_UART_Transmit(&huart6, (uint8_t *)(char *)Message1,strlen((char *)Message1),20);
  	  			HAL_UART_Transmit(&huart6, (uint8_t *)enter,1,10);
  	            flag1=0;
  }

  void Call(){
  	HAL_UART_Transmit(&huart6, (uint8_t *)"ATD0944022135;\r\n",strlen("ATD0944022135;\r\n"), 20);
  	HAL_Delay(20000);
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
}
  /* USER CODE END Error_Handler_Debug */


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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
