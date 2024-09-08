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
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "rc522.h"
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define uc_num 4
#define dencity 5
#define us_wait 3000
#define time_usonic 10000
#define time_normal 8000
#define time_yellow 2000
#define time_emer 6000
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim1;

osThreadId NormalHandle;
osThreadId ultrasonicHandle;
osThreadId rfidHandle;
/* USER CODE BEGIN PV */
/*#define triggerPin0 GPIO_PIN_8
#define triggerPort0 GPIOB
#define echoPin0 GPIO_PIN_8
#define echoPort0 GPIOA*/

GPIO_TypeDef *triggerPorts[uc_num] = {Trigger0_GPIO_Port, Trigger1_GPIO_Port, Trigger2_GPIO_Port,Trigger3_GPIO_Port};
uint16_t triggerPins[uc_num] = {Trigger0_Pin, Trigger1_Pin, Trigger2_Pin, Trigger3_Pin};
GPIO_TypeDef *echoPorts[uc_num] = {Echo0_GPIO_Port, Echo1_GPIO_Port, Echo2_GPIO_Port, Echo3_GPIO_Port};
uint16_t echoPins[uc_num] = {Echo0_Pin, Echo1_Pin, Echo2_Pin,Echo3_Pin};

uint8_t	str[MFRC522_MAX_LEN];
uint8_t sNum[5];// MFRC522_MAX_LEN = 16
uint8_t status;

uint16_t distancesInCm[uc_num];
uint32_t pMillis;
uint32_t Value1 = 0;
uint32_t Value2 = 0;
uint16_t Distance  = 0;  // cm
uint8_t sensor=0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM1_Init(void);
static void MX_SPI1_Init(void);
void NormalApp(void const * argument);
void UltrasonicApp(void const * argument);
void rfidApp(void const * argument);

/* USER CODE BEGIN PFP */
uint8_t MFRC522_Check(uint8_t* id);
uint8_t MFRC522_Compare(uint8_t* CardID, uint8_t* CompareID);
void MFRC522_WriteRegister(uint8_t addr, uint8_t val);
uint8_t MFRC522_ReadRegister(uint8_t addr);
void MFRC522_SetBitMask(uint8_t reg, uint8_t mask);
void MFRC522_ClearBitMask(uint8_t reg, uint8_t mask);
uint8_t MFRC522_Request(uint8_t reqMode, uint8_t* TagType);
uint8_t MFRC522_ToCard(uint8_t command, uint8_t* sendData, uint8_t sendLen, uint8_t* backData, uint16_t* backLen);
uint8_t MFRC522_Anticoll(uint8_t* serNum);
void MFRC522_CalulateCRC(uint8_t* pIndata, uint8_t len, uint8_t* pOutData);
uint8_t MFRC522_SelectTag(uint8_t* serNum);
uint8_t MFRC522_Auth(uint8_t authMode, uint8_t BlockAddr, uint8_t* Sectorkey, uint8_t* serNum);
uint8_t MFRC522_Read(uint8_t blockAddr, uint8_t* recvData);
uint8_t MFRC522_Write(uint8_t blockAddr, uint8_t* writeData);
void MFRC522_Init(void);
void MFRC522_Reset(void);
void MFRC522_AntennaOn(void);
void MFRC522_AntennaOff(void);
void MFRC522_Halt(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint16_t measureDistance(GPIO_TypeDef *triggerPort, uint16_t triggerPin, GPIO_TypeDef *echoPort, uint16_t echoPin){
	if(!HAL_GPIO_ReadPin(echoPort, echoPin))//skip sensor if ECHO pin is still busy
	{
		  HAL_GPIO_WritePin(triggerPort, triggerPin, GPIO_PIN_RESET);
		  HAL_GPIO_WritePin(triggerPort, triggerPin, GPIO_PIN_SET);  // pull the TRIG pin HIGH
		  __HAL_TIM_SET_COUNTER(&htim1, 0);
		  while (__HAL_TIM_GET_COUNTER (&htim1) < 10);  // wait for 10 us
		  HAL_GPIO_WritePin(triggerPort, triggerPin, GPIO_PIN_RESET);  // pull the TRIG pin low

		  pMillis = HAL_GetTick(); // used this to avoid infinite while loop  (for timeout)
		  // wait for the echo pin to go high
		  while (!(HAL_GPIO_ReadPin (echoPort, echoPin)) && pMillis + 10 >  HAL_GetTick());
		  Value1 = __HAL_TIM_GET_COUNTER (&htim1);

		  pMillis = HAL_GetTick(); // used this to avoid infinite while loop (for timeout)
		  // wait for the echo pin to go low
		  while ((HAL_GPIO_ReadPin (echoPort, echoPin)) && pMillis + 50 > HAL_GetTick());
		  Value2 = __HAL_TIM_GET_COUNTER (&htim1);

		  Distance = (Value2-Value1)* 0.034/2;
		  HAL_Delay(50);
	}
	else//give max distance if ECHO pin is still busy
	{
		Distance = 500;
	}
	return Distance;
}

void up_road(uint8_t R_state,uint8_t Y_state,uint8_t G_state){
	HAL_GPIO_WritePin(LED_ULR_GPIO_Port, LED_ULR_Pin, R_state);
	HAL_GPIO_WritePin(LED_ULY_GPIO_Port, LED_ULY_Pin, Y_state);
	HAL_GPIO_WritePin(LED_ULG_GPIO_Port, LED_ULG_Pin, G_state);
}
void Down_road(uint8_t R_state,uint8_t Y_state,uint8_t G_state){
	HAL_GPIO_WritePin(LED_DLR_GPIO_Port, LED_DLR_Pin, R_state);
	HAL_GPIO_WritePin(LED_DLY_GPIO_Port, LED_DLY_Pin, Y_state);
	HAL_GPIO_WritePin(LED_DLG_GPIO_Port, LED_DLG_Pin, G_state);
}
void Right_road(uint8_t R_state,uint8_t Y_state,uint8_t G_state){
	HAL_GPIO_WritePin(LED_RLR_GPIO_Port, LED_RLR_Pin, R_state);
	HAL_GPIO_WritePin(LED_RLY_GPIO_Port, LED_RLY_Pin, Y_state);
	HAL_GPIO_WritePin(LED_RLG_GPIO_Port, LED_RLG_Pin, G_state);
}
void Left_road(uint8_t R_state,uint8_t Y_state,uint8_t G_state){
	HAL_GPIO_WritePin(LED_LLR_GPIO_Port, LED_LLR_Pin, R_state);
	HAL_GPIO_WritePin(LED_LLY_GPIO_Port, LED_LLY_Pin, Y_state);
	HAL_GPIO_WritePin(LED_LLG_GPIO_Port, LED_LLG_Pin, G_state);
}
void up_down_green (void){
	Right_road(1,0,0);
	Left_road(1,0,0);
	up_road(0,0,1);
	Down_road(0,0,1);
}
void right_left_green (void){
	Right_road(0,0,1);
	Left_road(0,0,1);
	up_road(1,0,0);
	Down_road(1,0,0);
}
void left_green (void){
	Right_road(1,0,0);
	Left_road(0,0,1);
	up_road(1,0,0);
	Down_road(1,0,0);
}
void all_on (void){
	Right_road(1, 1,1);
	Left_road(1, 1,1);
	up_road(1, 1,1);
	Down_road(1, 1,1);
}
void all_off (void){
	Right_road(0, 0,0);
	Left_road(0, 0,0);
	up_road(0,0,0);
	Down_road(0, 0,0);
}
void right_green (void){
	Right_road(0,0,1);
	Left_road(1,0,0);
	up_road(1,0,0);
	Down_road(1,0,0);
}
void up_green (void){
	Right_road(1,0,0);
	Left_road(1,0,0);
	up_road(0,0,1);
	Down_road(1,0,0);
}
void down_green (void){
	Right_road(1,0,0);
	Left_road(1,0,0);
	up_road(1,0,0);
	Down_road(0,0,1);
}
void up_yellow(void){
	up_road(0,1,0);
}
void down_yellow(void){
	Down_road(0,1,0);
}
void right_yellow(void){
	Right_road(0,1,0);
}
void left_yellow (void){
	Left_road(0,1,0);
}
void all_yellow (void){
	Left_road(0,1,0);
	up_road(0,1,0);
	Right_road(0,1,0);
	Down_road(0,1,0);
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
  MX_TIM1_Init();
  MX_SPI1_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start(&htim1);
  MFRC522_Init();

  /* USER CODE END 2 */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of Normal */
  osThreadDef(Normal, NormalApp, osPriorityHigh, 0, 256);
  NormalHandle = osThreadCreate(osThread(Normal), NULL);

  /* definition and creation of ultrasonic */
  osThreadDef(ultrasonic, UltrasonicApp, osPriorityAboveNormal, 0, 256);
  ultrasonicHandle = osThreadCreate(osThread(ultrasonic), NULL);

  /* definition and creation of rfid */
  osThreadDef(rfid, rfidApp, osPriorityRealtime, 0, 256);
  rfidHandle = osThreadCreate(osThread(rfid), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	/*  for(uint16_t i=0; i<4; i++) {
	  		 sensor = i;//update sensor index in timer2
	  		 distancesInCm[i] = measureDistance(triggerPorts[i], triggerPins[i], echoPorts[i], echoPins[i]);
	  	 }*/


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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 84;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
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
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
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
  htim1.Init.Prescaler = 84;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_LLG_GPIO_Port, LED_LLG_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LED_LLY_Pin|LED_ULR_Pin|LED_ULY_Pin|LED_ULG_Pin
                          |GPIO_PIN_4, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LED_RLR_Pin|LED_RLY_Pin|LED_RLG_Pin|LED_DLG_Pin
                          |LED_LLR_Pin|Trigger1_Pin|Trigger2_Pin|LED_DLR_Pin
                          |Trigger3_Pin|Trigger0_Pin|LED_DLY_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : LED_LLG_Pin */
  GPIO_InitStruct.Pin = LED_LLG_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_LLG_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LED_LLY_Pin LED_ULR_Pin LED_ULY_Pin LED_ULG_Pin
                           PA4 */
  GPIO_InitStruct.Pin = LED_LLY_Pin|LED_ULR_Pin|LED_ULY_Pin|LED_ULG_Pin
                          |GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : LED_RLR_Pin LED_RLY_Pin LED_RLG_Pin LED_DLG_Pin
                           LED_LLR_Pin Trigger1_Pin Trigger2_Pin LED_DLR_Pin
                           Trigger3_Pin Trigger0_Pin LED_DLY_Pin */
  GPIO_InitStruct.Pin = LED_RLR_Pin|LED_RLY_Pin|LED_RLG_Pin|LED_DLG_Pin
                          |LED_LLR_Pin|Trigger1_Pin|Trigger2_Pin|LED_DLR_Pin
                          |Trigger3_Pin|Trigger0_Pin|LED_DLY_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : Echo1_Pin Echo2_Pin Echo3_Pin */
  GPIO_InitStruct.Pin = Echo1_Pin|Echo2_Pin|Echo3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : Echo0_Pin */
  GPIO_InitStruct.Pin = Echo0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(Echo0_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* USER CODE BEGIN Header_NormalApp */
/**
  * @brief  Function implementing the Normal thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_NormalApp */
void NormalApp(void const * argument)
{
  /* USER CODE BEGIN 5 */
	uint8_t normal=0;
  /* Infinite loop */
  for(;;)
  {
	  	 //osSemaphoreWait(myBinarySem01Handle, time_emer);
	  	  if (normal==0){
	  	 		  up_yellow();
	  	 		  down_yellow();
	  	 		  osDelay(time_yellow);
	  	 		  up_down_green();
	  	 		  normal=1;
	  	 		  osDelay(time_normal);
	  	 	  }
	  	 	  else if (normal==1){
	  	 		  right_yellow();
	  	 		  left_yellow();
	  	 		  osDelay(time_yellow);
	  	 		  right_left_green();
	  	 		  normal=0;
	  	 		  osDelay(time_normal);
	  	 	  }
  	 	  else {
	  	 		  osDelay(500);
	  	 	  }
    osDelay(1);
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_UltrasonicApp */
/**
* @brief Function implementing the ultrasonic thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_UltrasonicApp */
void UltrasonicApp(void const * argument)
{
  /* USER CODE BEGIN UltrasonicApp */

  /* Infinite loop */
  while(1)
  {
	  for(uint16_t i=0; i<4; i++) {
	  	  		 sensor = i;//update sensor index in timer2
	  	  		 distancesInCm[i] = measureDistance(triggerPorts[i], triggerPins[i], echoPorts[i], echoPins[i]);
	  }
	  if (distancesInCm[0]<=dencity){
		  osDelay(us_wait);
		  Distance=measureDistance(Trigger0_GPIO_Port, Trigger0_Pin, Echo0_GPIO_Port, Echo0_Pin);
		  if (Distance<=dencity){
			  osThreadSuspend(NormalHandle);
			  right_green();
			  osDelay(time_usonic);
			  osThreadResume(NormalHandle);
		  }
	  }
	  	 else if (distancesInCm[1]<=dencity){
	  		 osDelay(us_wait);
	  		 Distance=measureDistance(Trigger1_GPIO_Port, Trigger1_Pin, Echo1_GPIO_Port, Echo1_Pin);
	  		 if (Distance<=dencity){
	  			 osThreadSuspend(NormalHandle);
	  			 up_green();
	  			 osDelay(time_usonic);
	  			 osThreadResume(NormalHandle);
	  		 }
	  	 }
	  	 else if (distancesInCm[2]<=dencity){
	  		 osDelay(us_wait);
	  		 Distance=measureDistance(Trigger2_GPIO_Port, Trigger2_Pin, Echo2_GPIO_Port, Echo2_Pin);
	  		 if (Distance<=dencity){
	  			 osThreadSuspend(NormalHandle);
	  			 down_green();
	  			 osDelay(time_usonic);
	  			 osThreadResume(NormalHandle);
	  		 }
	  	 }
	  	 else if (distancesInCm[3]<=dencity){
	  		 osDelay(us_wait);
	  		 Distance=measureDistance(Trigger3_GPIO_Port, Trigger3_Pin, Echo3_GPIO_Port, Echo3_Pin);
	  		 if (Distance<=dencity){
	  			 osThreadSuspend(NormalHandle);
	  			left_green();
	  			 osDelay(time_usonic);
	  			 osThreadResume(NormalHandle);
	  		 }
	  	 }
	  	 else {

	  	 }
    osDelay(1);
  }
  /* USER CODE END UltrasonicApp */
}

/* USER CODE BEGIN Header_rfidApp */
/**
* @brief Function implementing the rfid thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_rfidApp */
void rfidApp(void const * argument)
{
  /* USER CODE BEGIN rfidApp */
  /* Infinite loop */
  for(;;)
  {
	  status = MFRC522_Request(PICC_REQIDL, str);
	  status = MFRC522_Anticoll(str);
	  memcpy(sNum, str, 5);
	  if((str[0]==19) && (str[1]==60) && (str[2]==14) && (str[3]==42) && (str[4]==11) ){
		osThreadSuspend(NormalHandle);
		  up_green();
		  osDelay(time_emer);
		  osThreadResume(NormalHandle);
	  }
	  else if((str[0]==195) && (str[1]==162) && (str[2]==2) && (str[3]==41) && (str[4]==74) ){
		  osThreadSuspend(NormalHandle);
		  down_green();
		  osDelay(time_emer);
		  osThreadResume(NormalHandle);
	  }
	  else if((str[0]==83) && (str[1]==253) && (str[2]==27) && (str[3]==248) && (str[4]==77) ){
		  osThreadSuspend(NormalHandle);
		  right_green();
		  osDelay(time_emer);
		  osThreadResume(NormalHandle);
	  }
	  else if((str[0]==83) && (str[1]==216) && (str[2]==29) && (str[3]==45) && (str[4]==187) ){
		  osThreadSuspend(NormalHandle);
		  left_green();
		  osDelay(time_emer);
		  osThreadResume(NormalHandle);
	  }
	  else {
		  osDelay(500);
	  }
	  osThreadYield();

  }
  /* USER CODE END rfidApp */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM11 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM11) {
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
