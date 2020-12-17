/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
//#include <stdio.h>
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

CAN_HandleTypeDef hcan;

SPI_HandleTypeDef hspi1;
DMA_HandleTypeDef hdma_spi1_rx;
DMA_HandleTypeDef hdma_spi1_tx;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_CAN_Init(void);
static void MX_TIM3_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM2_Init(void);
static void MX_ADC1_Init(void);
/* USER CODE BEGIN PFP */
//uint8_t htm_data[80];
uint8_t mth_data[200]={};
uint8_t dma_complete;

//80 bytes out and 100 bytes back in (with offset of 8 bytes.
uint8_t htm_data_setup[80]={0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,4,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,4,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,4,0,25,0,0,0,0,0,0,0,128,0,0,0,128,0,0,0,37,1};
uint8_t htm_data[80]={0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,255,0,0,0,0,0,0,0,0,0};

uint8_t CalcMTHChecksum(void);
void CalcHTMChecksum(void);

float CalcTemp(uint32_t raw);
uint16_t PackHTMData(uint8_t *, uint8_t *, uint16_t );
uint16_t UnpackMTHData(uint8_t *, uint8_t *, uint16_t, uint16_t );

#define MG2MAXSPEED 10000


#if __GNUC__
  /* With GCC/RAISONANCE, small printf (option LD Linker->Libraries->Small printf
     set to 'Yes') calls __io_putchar() */
  #define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
  #define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint32_t adc_vals[3];
uint16_t counter;

typedef struct{
		uint8_t data:1;
	}bits_t;

union{
	bits_t bits[2048];
	uint8_t bytes[256];
	uint16_t words[128];

}tx_buff;
uint8_t uart1_rx_buffer[200], rx_buffer_count;

enum{
	PARK,
	REVERSE,
	NEUTRAL,
	DRIVE
};

uint8_t get_gear();

uint16_t get_torque(uint8_t gear);

long map(long x, long in_min, long in_max, long out_min, long out_max);

#if 0
uint8_t buffer[10]={0x55,0x55};

uint32_t byte_counter;

void HAL_USART_TxCpltCallback(USART_HandleTypeDef *husart)
{
	uint32_t tmp;


  /* Prevent unused argument(s) compilation warning */
	if ( husart->Instance == USART2 ){
		/* if its UART 2, shoot out the next byte! */

		//tmp = 0x30;
		//husart->Instance->DR = (tmp & (uint16_t)0x01FF);
		__HAL_USART_ENABLE_IT(&husart2, UART_IT_TC);
		HAL_USART_Transmit(husart, (uint8_t*)buffer, 1, 1000);


		//Toggle PIN
		byte_counter++;
		if(byte_counter>10){
			byte_counter = 0;
			HAL_GPIO_TogglePin(HTM_SYNC_GPIO_Port, HTM_SYNC_Pin );

		}
		//HAL_UART_Transmit(&huart2, (uint8_t*)buffer, 1, 1000);

	}

}

#endif



#if 1
#define MAX_COMMAND_SIZE 200





uint8_t serial1_in_buff[MAX_COMMAND_SIZE];
uint8_t serial1_in_buff_count = 0;
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){


	 if ( huart->Instance == USART2 ){

			if(serial1_in_buff_count<MAX_COMMAND_SIZE){

				mth_data[serial1_in_buff_count]=uart1_rx_buffer[0];
				serial1_in_buff_count++;
			}
			else{
				//Process frame



				}

			HAL_UART_Receive_IT(&huart2,uart1_rx_buffer,1);

	 }

}
#endif

/**
  * @brief  Retargets the C library printf function to the USART.
  * @param  None
  * @retval None
  */
PUTCHAR_PROTOTYPE
{
  /* Place your implementation of fputc here */
  /* e.g. write a character to the EVAL_COM1 and Loop until the end of transmission */
 // HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, 0xFFFF);

  return ch;
}
int _write(int file, char *ptr, int len)
{
	int DataIdx;

	for (DataIdx = 0; DataIdx < len; DataIdx++)
	{
		__io_putchar(*ptr++);
	}
	return len;
}

uint8_t txDat[256]={0};
uint8_t rxDat[256]={0};

static uint16_t htm_checksum;
static int16_t dc_bus_voltage,temp_inv_water, temp_inv_inductor, mg1_speed, mg2_speed;
static int16_t mg1_torque, mg2_torque, speedSum;
static uint8_t gear;
static uint8_t inv_status = 1;


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
  MX_USART1_UART_Init();
  MX_CAN_Init();
  MX_TIM3_Init();
  MX_USART2_UART_Init();
  MX_SPI1_Init();
  MX_TIM2_Init();
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */

 // HAL_ADC_Start_DMA(&hadc1, adc_vals, 3);
 // HAL_UART_Receive_IT(&husart1,uart1_rx_buffer,1);

  HAL_UART_Receive_IT(&huart2,uart1_rx_buffer,1);
  //__HAL_USART_ENABLE_IT(&husart2, UART_IT_TC);





  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  //HAL_USART_Transmit(&husart2, (uint8_t*)buffer, 1, 1000);
 // husart2.Instance->DR=0x34;

 // HAL_GPIO_WritePin(HTM_SYNC_GPIO_Port, HTM_SYNC_Pin,1);
 // HAL_Delay(500);

#if 0

  __HAL_TIM_ENABLE_OCxPRELOAD(&htim2, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);

  HAL_GPIO_WritePin(LED4_GPIO_Port, LED4_Pin,1);
  HAL_Delay(200);

  uint16_t delay_val=500;
  uint16_t buff[200];

  for(int i =0 ; i< 200; i++)
	  buff[i]=0x00;

  while(1){
	  serial1_in_buff_count=0;
	  HAL_GPIO_TogglePin(LED4_GPIO_Port, LED4_Pin);
	  HAL_Delay(1);
	  HAL_GPIO_TogglePin(LED4_GPIO_Port, LED4_Pin);


	  HAL_UART_Transmit_IT(&huart2, htm_data_setup, 80);

	  HAL_Delay(5);

	  if(CalcMTHChecksum()==0){
	  	  	HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, 1 );
	  	  	}
	  	  	else{
	  	  	//HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, 0 );
	  	  	}
	  mth_data[98]=0;
	  mth_data[99]=0;

	  //HAL_Delay(delay_val++);
	  if(counter>100){
		  HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin );
		  counter = 0;
	  }
	  else{
		  counter++;
	  }
  }
#else



  HAL_ADC_Start(&hadc1);

  __HAL_TIM_ENABLE_OCxPRELOAD(&htim2, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);

  HAL_GPIO_WritePin(LED4_GPIO_Port, LED4_Pin,1);
  HAL_Delay(200);

  uint16_t delay_val=500;
  uint16_t buff[200];

  for(int i =0 ; i< 200; i++)
	  buff[i]=0x00;




  while(1){
	  serial1_in_buff_count=0;
	  HAL_GPIO_TogglePin(LED4_GPIO_Port, LED4_Pin);
	  HAL_Delay(1);
	  HAL_GPIO_TogglePin(LED4_GPIO_Port, LED4_Pin);

	  if(inv_status==0){
		  HAL_UART_Transmit_IT(&huart2, htm_data, 80);
	  	  }
	  else {
		  HAL_UART_Transmit_IT(&huart2, htm_data_setup, 80);
		  if(mth_data[1]!=0)
			  	 inv_status--;
  	  	  }



	  HAL_Delay(4);

	  if(CalcMTHChecksum()==0){
	  	  	HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, 1 );
	  	  	}
	  	  	else{
	  	  	//HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, 0 );
	  	  	//exchange data and prepare next HTM frame
	  	  	dc_bus_voltage=(((mth_data[82]|mth_data[83]<<8)-5)/2);
	  	  	temp_inv_water=(mth_data[42]|mth_data[43]<<8);
	  	  	temp_inv_inductor=(mth_data[86]|mth_data[87]<<8);
	  	  	mg1_speed=mth_data[6]|mth_data[7]<<8;
	  	  	mg2_speed=mth_data[31]|mth_data[32]<<8;

	  	  	}

	    gear=get_gear();
	    mg2_torque=get_torque(gear); // -3500 (reverse) to 3500 (forward)
	    mg1_torque=((mg2_torque*5)/4);
	    if((mg2_speed>MG2MAXSPEED)||(mg2_speed<-MG2MAXSPEED))mg2_torque=0;
	    if(gear==REVERSE)mg1_torque=0;

	    //speed feedback
	    speedSum=mg2_speed+mg1_speed;
	    speedSum/=113;
	    htm_data[0]=(uint8_t)speedSum;
	    htm_data[75]=(mg1_torque*4)&0xFF;
	    htm_data[76]=((mg1_torque*4)>>8);

	    //mg1
	    htm_data[5]=(mg1_torque*-1)&0xFF;  //negative is forward
	    htm_data[6]=((mg1_torque*-1)>>8);
	    htm_data[11]=htm_data[5];
	    htm_data[12]=htm_data[6];

	    //mg2
	    htm_data[26]=(mg2_torque)&0xFF; //positive is forward
	    htm_data[27]=((mg2_torque)>>8);
	    htm_data[32]=htm_data[26];
	    htm_data[33]=htm_data[27];

	    //checksum
	    htm_checksum=0;
	    for(int i=0;i<78;i++)htm_checksum+=htm_data[i];
	    htm_data[78]=htm_checksum&0xFF;
	    htm_data[79]=htm_checksum>>8;


	  mth_data[98]=0;
	  mth_data[99]=0;

	  //HAL_Delay(delay_val++);
	  if(counter>100){
		  HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin );
		  counter = 0;
	  }
	  else{
		  counter++;
	  }
  }


#endif



#if 0
  HAL_GPIO_WritePin(HTM_SYNC_GPIO_Port, HTM_SYNC_Pin,0);
  HAL_Delay(500);

  //HAL_GPIO_WritePin(HTM_SYNC_GPIO_Port, HTM_SYNC_Pin,1);
  //PackHTMData(htm_data_setup ,txDat,80);
  //HAL_GPIO_WritePin(HTM_SYNC_GPIO_Port, HTM_SYNC_Pin,0);

  uint8_t data[10]={1,2,4,8,16,32,64,128 };

  HAL_GPIO_WritePin(HTM_SYNC_GPIO_Port, HTM_SYNC_Pin,1);

  uint16_t len = PackHTMData( htm_data_setup,txDat,80);
  HAL_SPI_TransmitReceive(&hspi1, &txDat, &rxDat, len, 50);
  UnpackMTHData(rxDat,  mth_data, len);

  HAL_GPIO_WritePin(HTM_SYNC_GPIO_Port, HTM_SYNC_Pin,1);

#endif

  uint8_t data[8]={0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0 };

#if 0
  HAL_GPIO_WritePin(LED4_GPIO_Port, LED4_Pin,1);
  HAL_Delay(200);


  for(int i =0 ; i < 256; i++)
	  txDat[i]=0xff;


  while(1){

	 // HAL_SPI_TransmitReceive(&hspi1, &data, &rxDat, 8, 50);
	  HAL_GPIO_TogglePin(LED4_GPIO_Port, LED4_Pin);
	  HAL_Delay(1);
	  HAL_GPIO_TogglePin(LED4_GPIO_Port, LED4_Pin);

	  HAL_SPI_TransmitReceive(&hspi1, &txDat, &rxDat, 127, 50);

	  UnpackMTHData(rxDat,  mth_data, 100, 0);


	  	if(CalcMTHChecksum()==0){
	  		HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, 1 );
	  	}
	  	else{
	  		HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, 0 );
	  	}

	  HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin );
	  HAL_Delay(100);
	  HAL_SPI_TransmitReceive(&hspi1, &data, &rxDat, 8, 50);
	  //HAL_SPI_TransmitReceive(&hspi1, &data, &rxDat, 8, 50);
	  MX_SPI1_Init();
  }

#endif

   uint16_t len, x;

   HAL_GPIO_WritePin(LED4_GPIO_Port, LED4_Pin,1);
   HAL_Delay(200);
   HAL_SPI_TransmitReceive(&hspi1, &data, &rxDat, 8, 50);

   for(int i =0 ; i < 256; i++)
   	  txDat[i]=0xff;


  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  if(x<20){
		  x++;
	  }
	  else{
		  HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin );
		  x=0;

		  HAL_GPIO_TogglePin(LED4_GPIO_Port, LED4_Pin);

		  HAL_Delay(1);

		  HAL_GPIO_TogglePin(LED4_GPIO_Port, LED4_Pin);
		  HAL_SPI_TransmitReceive_DMA(&hspi1, txDat, rxDat,150 ) ;
		 // HAL_Delay(1);



		  //HAL_SPI_Receive_DMA(&hspi1, rxDat, 150);
		  //HAL_SPI_Transmit_DMA(&hspi1, txDat, 150);

		  dma_complete = 0;
	  }
	  HAL_Delay(10);

	  if(dma_complete){


		  UnpackMTHData(rxDat,  mth_data, 100, 20);

			if(CalcMTHChecksum()==0){
			  		HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, 1 );
			  	}
			  	else{
			  		//HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, 0 );
			  	}

	  }

#if 0
	  len = PackHTMData( htm_data_setup,txDat,80);

	  HAL_GPIO_TogglePin(LED4_GPIO_Port, LED4_Pin);
	  HAL_SPI_TransmitReceive(&hspi1, &data, &rxDat, 2, 50);
	  HAL_GPIO_TogglePin(LED4_GPIO_Port, LED4_Pin);



	  HAL_SPI_TransmitReceive(&hspi1, &txDat, &rxDat, len+50, 50);
	  UnpackMTHData(rxDat,  mth_data, 100, 4);



	 // for(int i = 0; i<1000;i++)
	  HAL_SPI_TransmitReceive(&hspi1, &data, &rxDat, 8, 50);

	  HAL_Delay(1000);
#endif


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

  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL8;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV8;
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

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */
  /** Common config 
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel 
  */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief CAN Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN_Init(void)
{

  /* USER CODE BEGIN CAN_Init 0 */

  /* USER CODE END CAN_Init 0 */

  /* USER CODE BEGIN CAN_Init 1 */

  /* USER CODE END CAN_Init 1 */
  hcan.Instance = CAN1;
  hcan.Init.Prescaler = 16;
  hcan.Init.Mode = CAN_MODE_NORMAL;
  hcan.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan.Init.TimeSeg1 = CAN_BS1_1TQ;
  hcan.Init.TimeSeg2 = CAN_BS2_1TQ;
  hcan.Init.TimeTriggeredMode = DISABLE;
  hcan.Init.AutoBusOff = DISABLE;
  hcan.Init.AutoWakeUp = DISABLE;
  hcan.Init.AutoRetransmission = DISABLE;
  hcan.Init.ReceiveFifoLocked = DISABLE;
  hcan.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN_Init 2 */

  /* USER CODE END CAN_Init 2 */

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
  hspi1.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_LSB;
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
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 63;
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
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 32;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

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
  htim3.Init.Prescaler = 20000;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 1000;
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
  if (HAL_TIM_OnePulse_Init(&htim3, TIM_OPMODE_SINGLE) != HAL_OK)
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
  huart1.Init.BaudRate = 500000;
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
  huart2.Init.BaudRate = 500000;
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
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void) 
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel2_IRQn);
  /* DMA1_Channel3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel3_IRQn);

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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, s_Pin|UART1_SYNC_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LED1_Pin|LED2_Pin|LED3_Pin|LED4_Pin 
                          |HTM_SYNC_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : LED_GREEN_Pin */
  GPIO_InitStruct.Pin = LED_GREEN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_GREEN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : s_Pin UART1_SYNC_Pin */
  GPIO_InitStruct.Pin = s_Pin|UART1_SYNC_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : z_Pin */
  GPIO_InitStruct.Pin = z_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(z_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LED1_Pin LED2_Pin LED3_Pin LED4_Pin 
                           HTM_SYNC_Pin */
  GPIO_InitStruct.Pin = LED1_Pin|LED2_Pin|LED3_Pin|LED4_Pin 
                          |HTM_SYNC_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : FWD_IN_Pin REV_IN_Pin START_IN_Pin */
  GPIO_InitStruct.Pin = FWD_IN_Pin|REV_IN_Pin|START_IN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

float V25 = 1.43;
float Avg_Slope = 0.0043;
float step_size = 3.3/4096;

float CalcTemp(uint32_t raw){
	float raw_val = raw;

	float VSENSE = step_size * raw_val;
	return ((VSENSE-V25)/Avg_Slope)+25;

}



uint8_t CalcMTHChecksum(void){

	uint16_t mth_checksum=0;

	for(int i=0;i<98;i++)mth_checksum+=mth_data[i];


  if(mth_checksum==(mth_data[98]|(mth_data[99]<<8))) return 1;
  else return 0;

}

void CalcHTMChecksum(void){

uint16_t htm_checksum=0;

  for(int i=0;i<78;i++)htm_checksum+=htm_data[i];
  htm_data[78]=htm_checksum&0xFF;
  htm_data[79]=htm_checksum>>8;

}


uint16_t data[200]={0xFF,0xFF,0xFF,0xFF};
uint16_t tx_data[220];


#define SetBit(A,k)     ( A[(k)/8] |= (1 << ((k)%8)) )
#define ClearBit(A,k)   ( A[(k)/8] &= ~(1 << ((k)%8)) )
#define TestBit(A,k)    ( A[(k)/8] & (1 << ((k)%8)) )


uint16_t PackHTMData(uint8_t *data_in, uint8_t *data_out, uint16_t len){

	// prepare the out buffer, which is 20% bigger due to start and stop bits :-
	uint16_t out_len_bytes;
	uint16_t out_len_bits = len * 10;
	uint32_t *out_ptr;

	out_len_bytes = out_len_bits/8;
	if(out_len_bits%8){
		out_len_bytes++;
	}


	uint16_t bit_count=0;
	uint8_t mask;

	out_ptr= (uint32_t *)data_out;
	for(int i =0 ; i < out_len_bytes; i++){
		data_out[i]=0xff;
		//out_ptr=0xFFFFFFFF;
		//out_ptr++;
	}

	return out_len_bytes;

	 for(int i =0 ; i<len; i++){
	         ClearBit(data_out, bit_count);
	          bit_count++;

	         mask = 1;
	         for(int bit =0; bit < 8; bit++){
	             if(mask & data_in[i]){
	                SetBit(data_out, bit_count); //don't need to set bits as they are already set!
	                bit_count++;
	             }
	             else{
	                ClearBit(data_out, bit_count);
	                bit_count++;
	             }
	             mask<<=1;
	         }

	         SetBit(data_out, bit_count); //don't need to set bits as they are already set!
	         bit_count++;
	    }

	return out_len_bytes;
}

static uint16_t decode_errors =0;

uint16_t UnpackMTHData(uint8_t *data_in, uint8_t *data_out, uint16_t len, uint16_t offset){
	uint16_t out_bit_count = 0, bit_num;

	uint16_t out_len_bits = len * 10;
	//uint16_t out_len_bytes = out_len_bits/ 8;





	//data_out+= offset;

	//Extract out bit /  byte!
	out_bit_count=0;
	for(int i =offset ; i<out_len_bits; i++){

			bit_num=i-offset;

	        if(bit_num%10==0){ //start bit
	            if(TestBit(data_in,bit_num)!=0){
	                // we should not have got 1 here
	                decode_errors++;
	            }
	            continue;
	        }

	        if( (bit_num+1)%10==0){ //stop bit
	            if(TestBit(data_in,bit_num)==0){
	                //should not have got a 1 here
	                decode_errors++;
	            }
	            continue;
	        }


			if(TestBit(data_in,bit_num)){
			    SetBit(data_out,out_bit_count);
				out_bit_count++;
			}
			else{

				ClearBit(data_out,out_bit_count);
				out_bit_count++;
			}





	 }


	if(decode_errors>0){
		//HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, 1 );
	}
	else{
		//HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, 0 );
	}

	return len; //out_len_bytes;


}


uint8_t get_gear()
{
  if(HAL_GPIO_ReadPin(FWD_IN_GPIO_Port, FWD_IN_Pin))
  {
  return(DRIVE);
  }
  else
  {
  return(REVERSE);
  }
}

uint16_t get_torque(uint8_t gear)
{
  //accelerator pedal mapping to torque values here
  uint16_t ThrotVal=HAL_ADC_GetValue(&hadc1);
  if (ThrotVal<80) ThrotVal=75;//dead zone at start of throttle travel
 if(gear==DRIVE) ThrotVal = map(ThrotVal, 75, 4096, 0, 1000);
 if(gear==REVERSE) ThrotVal = map(ThrotVal, 75, 4096, 0, -1000);
  return ThrotVal; //return torque
}

long map(long x, long in_min, long in_max, long out_min, long out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}



/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM1 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM1) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

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
