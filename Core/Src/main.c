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
#include "cmsis_os.h"
#include "message_buffer.h"
#include "portmacro.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include "Usart.h"
#include "Retarget.h"
#include "Gpio.h"
#include "Adc.h"
#include "Rcc.h"
#include "Nvic.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
#define	VREFPLUS 			3.3
#define	MAX_BUFFER_SIZE		512
#define	COMMANDBYTES		3
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
MessageBufferHandle_t msgForSlave, msgForMaster, msgForInterpreter;
MessageBufferHandle_t msgFromMaster, msgFromInterpreter;
MessageBufferHandle_t msgAdc;
BaseType_t xHigherPriorityTaskWoken = pdFALSE;

char masterCommand[256];
char slaveResponse[256];
uint8_t hostCommand[256];
/* Definitions for slave */
osThreadId_t slaveHandle;
const osThreadAttr_t slave_attributes = {
  .name = "slave",
  .stack_size = 128 * 8,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for master */
osThreadId_t masterHandle;
const osThreadAttr_t master_attributes = {
  .name = "master",
  .stack_size = 128 * 8,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for commInterpreter */
osThreadId_t commInterpreterHandle;
const osThreadAttr_t commInterpreter_attributes = {
  .name = "commInterpreter",
  .stack_size = 128 * 8,
  .priority = (osPriority_t) osPriorityNormal,
};
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
void StartSlave(void *argument);
void StartMaster(void *argument);
void StartInterpret(void *argument);
void gpioConfig();
void usartConfig();
void adcConfig();
void clearBuffer(char *buffer, size_t size);
/* USER CODE BEGIN PFP */

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
  gpioConfig();
  usartConfig();
  adcConfig();
  MX_GPIO_Init();
  /* USER CODE BEGIN 2 */
  //Create message buffers
  msgForSlave = xMessageBufferCreate(MAX_BUFFER_SIZE);
  msgForMaster = xMessageBufferCreate(MAX_BUFFER_SIZE);
  msgForInterpreter = xMessageBufferCreate(MAX_BUFFER_SIZE);
  msgFromMaster = xMessageBufferCreate(MAX_BUFFER_SIZE);
  msgFromInterpreter = xMessageBufferCreate(MAX_BUFFER_SIZE);
  msgAdc = xMessageBufferCreate(MAX_BUFFER_SIZE);
  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

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
  /* creation of slave */
  slaveHandle = osThreadNew(StartSlave, NULL, &slave_attributes);

  /* creation of master */
  masterHandle = osThreadNew(StartMaster, NULL, &master_attributes);

  /* creation of commInterpreter */
  commInterpreterHandle = osThreadNew(StartInterpret, NULL, &commInterpreter_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
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
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();

}

/* USER CODE BEGIN 4 */
void gpioConfig() {
	rccUnresetAndEnableDevice(RCC_GPIOA);
	rccUnresetAndEnableDevice(RCC_GPIOC);

	//USART 2 --> Tx : PA2 ; Rx : PA3
	gpioConfigurePin(gpioA, PIN2, GPIO_ALT_FUNC | GPIO_HIGH_SPEED | AF_7 | GPIO_PUSH_PULL);
	gpioConfigurePin(gpioA, PIN3, GPIO_ALT_FUNC | AF_7);

	//USART 6 --> Tx : PC6 ; Rx : PC7
	gpioConfigurePin(gpioC, PIN6, GPIO_ALT_FUNC | GPIO_HIGH_SPEED | AF_8 | GPIO_PUSH_PULL);
	gpioConfigurePin(gpioC, PIN7, GPIO_ALT_FUNC | AF_8);

	//USART 1 --> Tx : PA9 ; Rx : PA10
	gpioConfigurePin(gpioA, PIN9, GPIO_ALT_FUNC | GPIO_HIGH_SPEED | AF_7 | GPIO_PUSH_PULL);
	gpioConfigurePin(gpioA, PIN10, GPIO_ALT_FUNC | AF_7);

	//ADC Analog input AIN0 --> PA0
	gpioConfigurePin(gpioA, PIN0, GPIO_ANALOG_IN);

	//LED Pin --> PC10
	gpioConfigurePin(gpioC, PIN10, GPIO_MED_SPEED | GPIO_PUSH_PULL | GPIO_OUTPUT);
}

void usartConfig() {
	rccUnresetAndEnableDevice(RCC_USART1);
	rccUnresetAndEnableDevice(RCC_USART2);
	rccUnresetAndEnableDevice(RCC_USART6);

	nvicEnableIrq(USART1_IRQ);
	nvicSetIrqPriority(USART1_IRQ, 6);
	nvicEnableIrq(USART2_IRQ);
	nvicSetIrqPriority(USART2_IRQ, 6);
	nvicEnableIrq(USART6_IRQ);
	nvicSetIrqPriority(USART6_IRQ, 6);

	usartSetBaudRate(usart1, 115200);
	usartConfigure(usart1, USART_TX_EN | USART_RX_EN | USART_9_DATA_BIT | USART_STOPB_1 | USART_ODD_PARITY | USART_RXNE_IE | USART_EN);
	usartSetBaudRate(usart2, 115200);
	usartConfigure(usart2, USART_TX_EN | USART_RX_EN | USART_9_DATA_BIT | USART_STOPB_1 | USART_ODD_PARITY | USART_RXNE_IE | USART_EN);
	usartSetBaudRate(usart6, 115200);
	usartConfigure(usart6, USART_TX_EN | USART_RX_EN | USART_9_DATA_BIT | USART_STOPB_1 | USART_ODD_PARITY | USART_RXNE_IE | USART_EN);
}

void adcConfig() {
	int channels[1] = {0};
	rccUnresetAndEnableDevice(RCC_ADC1);
	adcSetSampleTime(adc1, 0, SAMPLE_3_CYCLES);			//channel 0, sampling time of 3 cycles
	adcCommonConfigure(ADC_PSC_2);						//ADC clock is half of PCLK2
	adcSetChannelSequence(adc1, channels, 1);			//Scan one channel, channel 0
	nvicEnableIrq(ADC1_IRQ);							//Enable ADC interrupt
	nvicSetIrqPriority(ADC1_IRQ, 6);
	configureADC(adc1, ADC_SINGLE_MODE | ADC_RIGHT_ALIGN | ADC_RESOLUTION_12 | ADC_EOC_INT_EN | ADC_EN);
}

void ADC_IRQHandler(void) {
	int adcResult;
	if(adc1->SR & ADC_END_OF_CONVERSION) {
		adc1->SR &= ~ADC_END_OF_CONVERSION;
		adcResult = adc1->DR;
		xMessageBufferSendFromISR(msgAdc, &adcResult, sizeof(int), &xHigherPriorityTaskWoken);
	}
}

//master
void USART6_IRQHandler(void) {
	static int i = 0;
	uint8_t temp;
	if(usart6->SR & USART_RXNE) {
		temp = usart6->DR;
		slaveResponse[i++] = temp;
		usart6->SR &= ~USART_RXNE;
		if(i == COMMANDBYTES) {
			i = 0;
			xMessageBufferSendFromISR(msgForMaster, slaveResponse, sizeof(slaveResponse), &xHigherPriorityTaskWoken);
		}
	}
}

//slave
void USART1_IRQHandler(void) {
	static int i = 0;
	uint8_t temp;
	if(usart1->SR & USART_RXNE) {
		temp = usart1->DR;
		masterCommand[i++] = temp;
		usart1->SR &= ~USART_RXNE;
		if(i == COMMANDBYTES) {
			i = 0;
			xMessageBufferSendFromISR(msgForSlave, masterCommand, sizeof(hostCommand), &xHigherPriorityTaskWoken);
		}
	}
}

//command interpreter
void USART2_IRQHandler(void) {
	static int i = 0;
	uint8_t temp;
	if(usart2->SR & USART_RXNE) {
		temp = usart2->DR;
		usart2->SR &= ~USART_RXNE;
		if(temp != '\r')
			hostCommand[i++] = temp;
		else {
			i = 0;
			xMessageBufferSendFromISR(msgForInterpreter, hostCommand, sizeof(hostCommand), &xHigherPriorityTaskWoken);
		}
	}
}

void clearBuffer(char *buffer, size_t size) {
	for(int i = 0; i < size; i++)
		buffer[i] = '\0';
}
/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartSlave */
/**
  * @brief  Function implementing the slave thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartSlave */
void StartSlave(void *argument)
{
  /* USER CODE BEGIN 5 */
  /*
   * Legend
   * 'S' = Start ADC Conversion
   * 'R' = ADC result ready to be read (for master)
   * 'L' = LED operation ('1' to turn on LED, '0' to turn off)
   * 'A' = Acknowledge (to notify master that slave has finished the requested LED operation)
   */
  char buffer[256] = {'\0'};
  char data[256] = {'\0'};
  int command, dataLen, adcResult;
  /* Infinite loop */
  for(;;)
  {
	  xMessageBufferReceive(msgForSlave, buffer, sizeof(buffer), portMAX_DELAY);
	  command = buffer[0];
	  dataLen = buffer[1];
	  for(int i = 0; i < dataLen; i++)
		  data[i] = buffer[i + 2];
	  clearBuffer(masterCommand, sizeof(masterCommand));	//clear global buffer after finish fetching the command
	  if(command == 'S') {									//'S' = 0x53 indicating master request to start ADC conversion
		  adcConfigureRegularChannel(adc1, ADC_REG_CHN_START);
		  xMessageBufferReceive(msgAdc, &adcResult, sizeof(int), portMAX_DELAY);
		  sprintf(buffer, "R%c%c", adcResult >> 8, adcResult & 0xFF);
	  }
	  if(command == 'L') {
		  if(data[0] == '1')
			  gpioWritePin(gpioC, PIN10, 1);
		  else
			  gpioWritePin(gpioC, PIN10, 0);
		  sprintf(buffer, "A");
	  }
	  usartSendString(usart1, buffer, COMMANDBYTES);
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_StartMaster */
/**
* @brief Function implementing the master thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartMaster */
void StartMaster(void *argument)
{
  /* USER CODE BEGIN StartMaster */
  int buffer;
  int j = 0;
  char result[256] = {'\0'};
  char bufferInString[256] = {'\0'};
  int resultInBinary = 0;
  /* Infinite loop */
  for(;;)
  {
	  xMessageBufferReceive(msgFromInterpreter, &buffer, sizeof(int), portMAX_DELAY);
	  for(int i = COMMANDBYTES - 1; i >= 0; i--)
		  bufferInString[j++] = (((buffer >> (i * 8)) & 0xFF));		//convert to string to send through usart
	  j = 0;
	  usartSendString(usart6, bufferInString, COMMANDBYTES);
	  clearBuffer((char *)bufferInString, sizeof(bufferInString));
	  xMessageBufferReceive(msgForMaster, result, sizeof(result), portMAX_DELAY);
	  resultInBinary = 0;
	  for(int i = 0; i < COMMANDBYTES; i++)
		  resultInBinary = (resultInBinary << 8) | result[i];		//convert to binary to send through message buffer
	  clearBuffer((char *)slaveResponse, sizeof(slaveResponse));
	  xMessageBufferSend(msgFromMaster, &resultInBinary, sizeof(int), 0);
  }
  /* USER CODE END StartMaster */
}

/* USER CODE BEGIN Header_StartInterpret */
/**
* @brief Function implementing the commInterpreter thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartInterpret */
void StartInterpret(void *argument)
{
  /* USER CODE BEGIN StartInterpret */
  //Command : 0xAABBcccccc.... (AA is the command, BB is no of bytes of data, cc... is the data)
  char buffer[256] = {'\0'};
  int led_on = 0x4C0131;
  int led_off = 0x4C0130;
  int adc_start = 0x530000;
  int result, adc_result, response;
  double voltage;
  /* Infinite loop */
  for(;;)
  {
	  xMessageBufferReceive(msgForInterpreter, buffer, sizeof(buffer), portMAX_DELAY);
	  if(!strcasecmp(buffer, "TURN ON LED"))
		  xMessageBufferSend(msgFromInterpreter, &led_on, sizeof(int), 0);
	  else if(!strcasecmp(buffer, "TURN OFF LED"))
		  xMessageBufferSend(msgFromInterpreter, &led_off, sizeof(int), 0);
	  else if(!strcasecmp(buffer, "START ADC CONVERSION"))
		  xMessageBufferSend(msgFromInterpreter, &adc_start, sizeof(int), 0);
	  else {
		  printf("Invalid instruction!\r\n");
		  clearBuffer((char *)hostCommand, sizeof(hostCommand));
		  continue;
	  }
	  clearBuffer((char *)hostCommand, sizeof(hostCommand));
	  xMessageBufferReceive(msgFromMaster, &result, sizeof(int), portMAX_DELAY);
	  response = result >> ((COMMANDBYTES - 1) * 8);
	  if(response == 0x41)
		  printf("LED operation succeeded!\r\n");
	  if(response == 0x52) {
		  adc_result = result & 0xFFFF;
		  voltage = (adc_result / 4096.0000) * VREFPLUS;
	  	  printf("ADC Value: %d, voltage: %.4lfV\r\n", adc_result, voltage);
	  }
  }
  /* USER CODE END StartInterpret */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM9 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM9) {
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
