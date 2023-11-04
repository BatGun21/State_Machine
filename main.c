/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

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

/* USER CODE BEGIN PV */
char msg[250];
volatile uint32_t counter = 0;
char invalid[15] = "Invalid Input ";
int UART_Receive_signal = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void SystemClock_Config(void);
void SysTick_Init(uint32_t ticks);
void DelayMS(unsigned int time);
void UART_Tx(char chat);
char UART_Rx(void);
void UART_Send(char str[]);
void UART_Receive(char str[]);
void str_empty (char str[]);
int debounceSwitch(int pin);
int detectedRisingEdge(int currPin, int *prevousPin);
void intToString(uint32_t value, char str[]);
void reverseString(char str[], int length);
void UART_ISR(int Rx, int LED, char strrx[]);
void LED_Control (char str[]);
int UART_MessageAvailable(void);
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
  SysTick_Init(8000);
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  /* USER CODE BEGIN 2 */

  /*------ GPIOC INIT -----*/
  // Enabling Clock for Port C
  RCC->AHBENR |= RCC_AHBENR_GPIOCEN;

  GPIOC->MODER |= GPIO_MODER_MODER6_0; //Set bit 0 to 1 Red
  GPIOC->MODER |= GPIO_MODER_MODER7_0; //Set bit 0 to 1 Blue
  GPIOC->MODER |= GPIO_MODER_MODER8_0; //Set bit 0 to 1 Orange
  GPIOC->MODER |= GPIO_MODER_MODER9_0; //Set bit 0 to 1 Green

  /*----- GPIOA INIT -----*/

  // Enabling Clock for Port A
  RCC->AHBENR |= RCC_AHBENR_GPIOAEN;
  // Enabling Input for PA0
  GPIOA->MODER &= 0xfffffffc;
  // Setting the speed of the pin PA0 to High Speed
  GPIOA->OSPEEDR |= 0x00000003;
  // Enabling Pull Up for PA0
  GPIOA->PUPDR |= 0x00000002;


  /* ----- USART1 INIT ----- */

  // Enabling Clock USART1
  __disable_irq();

  RCC->APB2ENR |= RCC_APB2ENR_USART1EN;
  // Setting the speed of the PA9 and PA10 to High Speed
  GPIOA->OSPEEDR |= 0x003c0000;
  // PA9 is TX and PA10 is RX
  GPIOA->AFR[1] |= 0x00000110;
  // Setting up Pins PA9 and PA10 for Alternate Function
  GPIOA->MODER |= 0x00280000;


  // Setting up Baud Rate to 9600 bps, Oversampling by 16;
  USART1->BRR = 0x0341;


  // Enabling UART Transmit
  USART1->CR1 |= USART_CR1_TE ;
  // Enabling UART Receive
  USART1->CR1 |= USART_CR1_RXNEIE | USART_CR1_RE;
  // Enabling UART
  USART1->CR1 |= USART_CR1_UE;
//   Generating Interupt when data is ready to read;
  USART1->CR1 |= USART_CR1_RXNEIE;
  //Enabling the Interupt Requests.
  NVIC_EnableIRQ(USART1_IRQn);
  __enable_irq();


  /* USER CODE END 2 */

  int risingEdge = 0;
  int pin = 0;
  int previousPin = 0;
  char systickStr[11];
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  while (1)
  {
	/* USER CODE END 3 */

	  // Reading if Switch is pressed and debouncing

	  pin = GPIOA->IDR & 0x00000001;
	  pin = debounceSwitch(pin);

	  // Sending SysTick Value
	  risingEdge = detectedRisingEdge(pin, &previousPin);
	  if (risingEdge){
		  uint32_t systickValue = counter;
		  intToString(systickValue, systickStr);
		  UART_Send(systickStr);
	  }else{
	  	  str_empty(systickStr);
	  }
	  str_empty(systickStr);



    /* USER CODE BEGIN 3 */
  }

  /* USER CODE END WHILE */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

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
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;


  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */


void USART1_IRQHandler(void){
	UART_ISR(1,1,msg);
}

void UART_ISR(int Rx, int LED, char strrx[]){
	if (Rx){
		UART_Receive(strrx);
	}
	if (LED){
		LED_Control(strrx);
	}

}

void LED_Control (char str[]){

		// Compare the string and do the Tasks accordingly
		if (strcmp(str, "REDSR\0") == 0){
			GPIOC->ODR |= 0x00000040; //Red LED on
		}else if (strcmp(str, "REDST\0") == 0){
			GPIOC->ODR &= 0xffffffbf; //Red LED off
		}else if (strcmp(str, "GRNSR\0") == 0){
			GPIOC->ODR |= 0x00000200; //Green LED on
		}else if (strcmp(str, "GRNST\0") == 0){
			GPIOC->ODR &= 0xfffffdff; //Green LED off
		}else if (strcmp(str, "BLINK\0") == 0){
			//Blinking all the LEDs Together
			int i = 0;
			GPIOC->ODR &= 0xffffffbf; //Red LED off
			GPIOC->ODR &= 0xfffffdff; //Green LED off
			while (i<20){
				GPIOC->ODR ^= 0x000003c0;
				DelayMS(100);
				i++;
			}
		 }else{
			UART_Send(invalid);
		 }
		  str_empty(str);
}

void SysTick_Handler(void) {
    if (counter == 0xffffffff) {
        counter = 0; // Reset the counter if the maximum value is reached
    } else {
        counter++; // Increment the counter
    }
}

void intToString(uint32_t value, char str[]) {
    int i = 0;

    if (value == 0) {
        str[i++] = '0';
    } else {
        while (value) {
            str[i++] = '0' + (value % 10);  // Convert digit to character
            value /= 10;
        }
    }

    str[i] = '\0';  // Null-terminate the string
    reverseString(str, i);
}

void reverseString(char str[], int length) {
    int start = 0;
    int end = length - 1;
    while (start < end) {
        char temp = str[start];
        str[start] = str[end];
        str[end] = temp;
        start++;
        end--;
    }
}

int debounceSwitch(int pin){
	int currPin = 0;
	int temp = 0;
	temp = pin;
	DelayMS(1);
	if (pin==temp){

		DelayMS(1);
		if (pin==temp){
		  	currPin = temp;
		}
	}else{
		currPin = pin;
	}
	return currPin;

}

int detectedRisingEdge(int currPin, int *prevousPin){
  	int RE = 0;
	if (currPin-*prevousPin>0){
  		RE = 1;
  		*prevousPin = currPin;
  	}else{
  		RE=0;
  		*prevousPin = currPin;
  	}
	return RE;
}

void str_empty (char str[]){
	int i = 0;
	while (str[i] != '\0'){
		str[i] = '\0';
		i++;
	}
}

void UART_Tx(char c){

	while ((USART1->ISR & USART_ISR_TXE) != USART_ISR_TXE){
		__NOP();
	}
	USART1->TDR = c;
}

char UART_Rx(void){

	char c;

	while ((USART1->ISR & USART_ISR_RXNE) != USART_ISR_RXNE){
		__NOP();
	}
	c = USART1->RDR;

	return c;
}

void UART_Send(char str[]){
	int i = 0;
	while (str[i] != '\0'){
		UART_Tx(str[i]);
		i++;
	}
}

void UART_Receive(char str[]) {
    int counter = 0;
    int signal = 0;

    while (signal == 0) {
        char receivedChar = UART_Rx();
        if (receivedChar == '\n' || receivedChar == '\r' || receivedChar == '\0') {
        	str[counter] = '\0';
            signal = 1;
        } else {
            str[counter] = receivedChar;
            counter++;
        }
    }
}

void DelayMS(unsigned int time){

	for(int i=0; i<=time; i++){
		while ((SysTick->CTRL & 0x00010000) == 0){
				//Wait for 1 millisec.
		}
	}
}

void SysTick_Init(uint32_t ticks){

	SysTick->CTRL = 0; // Disable SysTick

	SysTick->LOAD = ticks-1; // Set Reload Register

	// Setting Interrupt Priority to the highest
	NVIC_SetPriority(SysTick_IRQn, (1<<__NVIC_PRIO_BITS)-1);

	SysTick->VAL = 0; // Reset the SysTick counter value

	SysTick->CTRL |= SysTick_CTRL_CLKSOURCE_Msk; // Selecting internal clock source
	SysTick->CTRL |= SysTick_CTRL_TICKINT_Msk; // Enabling SysTick exception Request when 0


	SysTick->CTRL |= SysTick_CTRL_ENABLE_Msk; // Enable SysTick
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