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
#include <string.h>
#include <stdio.h>
#include <time.h>
#include <stdlib.h>
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

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim5;

UART_HandleTypeDef huart2;
UART_HandleTypeDef huart6;

/* USER CODE BEGIN PV */
volatile uint8_t clock_hours = 0;
volatile uint8_t clock_mins = 0;
volatile uint8_t clock_secs = 0;
volatile uint8_t wall_clock_hr_update_flag = 0;

volatile uint8_t hcsr04_Rx_flag = 0; // Reset the interrupt flag for the HCSR04
volatile uint8_t first_edge = 0; // Set it back to false
volatile uint16_t time_edge1 = 0;
volatile uint16_t time_edge2 = 0;

volatile uint16_t rpm_tick_count = 0;

uint8_t empty = 0;
uint8_t byte[2];
uint8_t rcv_intpt_flag = 0;
uint8_t txd_msg_buffer[64] = {0};

uint16_t time_diff = 0;
int filled = 0;
int distance = 0;

uint8_t counter_tens = 0;
uint8_t counter_ones = 0;
uint8_t bottom_distance = 65; //distance to bottom in cm
uint8_t top_distance = 10; //distance to top in cm
uint8_t rpm = 0;

int TIM2_CH1_DCVAL = 500;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART6_UART_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM5_Init(void);
/* USER CODE BEGIN PFP */
static void ADC_Select_CH(int CH);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */


void ADC_Select_CH(int CH)
{
    ADC_ChannelConfTypeDef sConfig = {0};
    switch(CH)
    {
        case 0:
            sConfig.Channel = ADC_CHANNEL_0;
            sConfig.Rank = 1;
            if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
            {
                Error_Handler();
            }
            break;
        case 1:
            sConfig.Channel = ADC_CHANNEL_1;
            sConfig.Rank = 1;
            if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
            {
                Error_Handler();
            }
            break;
        case 2:
            sConfig.Channel = ADC_CHANNEL_2;
            sConfig.Rank = 1;
            if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
            {
                Error_Handler();
            }
            break;
        case 3:
            sConfig.Channel = ADC_CHANNEL_3;
            sConfig.Rank = 1;
            if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
            {
                Error_Handler();
            }
            break;
        case 4:
            sConfig.Channel = ADC_CHANNEL_4;
            sConfig.Rank = 1;
            if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
            {
                Error_Handler();
            }
            break;
        case 5:
            sConfig.Channel = ADC_CHANNEL_5;
            sConfig.Rank = 1;
            if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
            {
                Error_Handler();
            }
            break;
        case 6:
            sConfig.Channel = ADC_CHANNEL_6;
            sConfig.Rank = 1;
            if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
            {
                Error_Handler();
            }
            break;
        case 7:
            sConfig.Channel = ADC_CHANNEL_7;
            sConfig.Rank = 1;
            if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
            {
                Error_Handler();
            }
            break;
        case 8:
            sConfig.Channel = ADC_CHANNEL_8;
            sConfig.Rank = 1;
            if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
            {
                Error_Handler();
            }
            break;
        case 9:
            sConfig.Channel = ADC_CHANNEL_9;
            sConfig.Rank = 1;
            if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
            {
                Error_Handler();
            }
            break;
        case 10:
            sConfig.Channel = ADC_CHANNEL_10;
            sConfig.Rank = 1;
            if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
            {
                Error_Handler();
            }
            break;
        case 11:
            sConfig.Channel = ADC_CHANNEL_11;
            sConfig.Rank = 1;
            if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
            {
                Error_Handler();
            }
            break;
        case 12:
            sConfig.Channel = ADC_CHANNEL_12;
            sConfig.Rank = 1;
            if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
            {
                Error_Handler();
            }
            break;
        case 13:
            sConfig.Channel = ADC_CHANNEL_13;
            sConfig.Rank = 1;
            if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
            {
                Error_Handler();
            }
            break;
        case 14:
            sConfig.Channel = ADC_CHANNEL_14;
            sConfig.Rank = 1;
            if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
            {
                Error_Handler();
            }
            break;
        case 15:
            sConfig.Channel = ADC_CHANNEL_15;
            sConfig.Rank = 1;
            if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
            {
                Error_Handler();
            }
            break;
    }
}


void DIGITS_Display(uint8_t DIGIT)
{
	uint8_t DIGIT_B = DIGIT % 10;
	uint8_t DIGIT_A = (DIGIT - DIGIT_B) / 10;

	uint8_t DIGITA_VAL = 0x0F & DIGIT_A; //mask off higher4 bits
	 int Abit0 = (DIGITA_VAL ) & 1;  	// extract Abit0 of the 4-bit value
	 int Abit1 = (DIGITA_VAL >> 1) & 1;  // extract Abit1 of the 4-bit value
	 int Abit2 = (DIGITA_VAL >> 2) & 1;  // extract Abit2 of the 4-bit value
	 int Abit3 = (DIGITA_VAL >> 3) & 1;  // extract Abit3 of the 4-bit value

	 uint8_t DIGITB_VAL = 0x0F & DIGIT_B; //mask off higher4 bits
	 int Bbit0 = (DIGITB_VAL ) & 1;  	// extract Bbit0 of the 4-bit value
	 int Bbit1 = (DIGITB_VAL >> 1) & 1;  // extract Bbit1 of the 4-bit value
	 int Bbit2 = (DIGITB_VAL >> 2) & 1;  // extract Bbit2 of the 4-bit value
	 int Bbit3 = (DIGITB_VAL >> 3) & 1;  // extract Bbit3 of the 4-bit value

	 if (Abit0 == (0))
	 {
		 HAL_GPIO_WritePin(GPIOC, DIGIT_A0_Pin, GPIO_PIN_RESET);
	 }
	 else
	 {
		 HAL_GPIO_WritePin(GPIOC, DIGIT_A0_Pin, GPIO_PIN_SET);

	 }
	 if (Abit1 == (0))
	 {
		 HAL_GPIO_WritePin(GPIOC, DIGIT_A1_Pin, GPIO_PIN_RESET);
	 }
	 else
	 {
		 HAL_GPIO_WritePin(GPIOC, DIGIT_A1_Pin, GPIO_PIN_SET);

	 }
	 if (Abit2 == (0))
	 {
		 HAL_GPIO_WritePin(GPIOC, DIGIT_A2_Pin, GPIO_PIN_RESET);
	 }
	 else
	 {
		 HAL_GPIO_WritePin(GPIOC, DIGIT_A2_Pin, GPIO_PIN_SET);

	 }
	 if (Abit3 == (0))
	 {
		 HAL_GPIO_WritePin(GPIOC, DIGIT_A3_Pin, GPIO_PIN_RESET);
	 }
	 else
	 {
		 HAL_GPIO_WritePin(GPIOC, DIGIT_A3_Pin, GPIO_PIN_SET);

	 }


	 if (Bbit0 == (0))
	 {
		 HAL_GPIO_WritePin(GPIOA, DIGIT_B0_Pin, GPIO_PIN_RESET);
	 }
	 else
	 {
		 HAL_GPIO_WritePin(GPIOA, DIGIT_B0_Pin, GPIO_PIN_SET);

	 }
	 if (Bbit1 == (0))
	 {
		 HAL_GPIO_WritePin(GPIOB, DIGIT_B1_Pin, GPIO_PIN_RESET);
	 }
	 else
	 {
		 HAL_GPIO_WritePin(GPIOB, DIGIT_B1_Pin, GPIO_PIN_SET);

	 }
	 if (Bbit2 == (0))
	 {
		 HAL_GPIO_WritePin(GPIOA, DIGIT_B2_Pin, GPIO_PIN_RESET);
	 }
	 else
	 {
		 HAL_GPIO_WritePin(GPIOA, DIGIT_B2_Pin, GPIO_PIN_SET);

	 }
	 if (Bbit3 == (0))
	 {
		 HAL_GPIO_WritePin(GPIOA, DIGIT_B3_Pin, GPIO_PIN_RESET);
	 }
	 else
	 {
		 HAL_GPIO_WritePin(GPIOA, DIGIT_B3_Pin, GPIO_PIN_SET);

	 }
}

void HCSR04_TRIG_PULSE(void)
{
	HAL_GPIO_WritePin(HCSR04_TRIG_GPIO_Port, HCSR04_TRIG_Pin, GPIO_PIN_SET);
	for (int j = 0; j != 48; j = j + 1) {}
	HAL_GPIO_WritePin(HCSR04_TRIG_GPIO_Port, HCSR04_TRIG_Pin, GPIO_PIN_RESET);
	HAL_Delay(30);
}

void MOTOR_Run(int speed)
{
	if(speed >= 0) {
		TIM3->CCR3 = ((float)speed/100)*60000;
		TIM3->CCR1 = 0;
	} else {
		TIM3->CCR1 = ((float)speed/-100)*60000;
		TIM3->CCR3 = 0;
	}
}

void SERVO_Run(int position)
{
	int TIM2_CH1_STEP = 20;
	while(TIM2_CH1_DCVAL != position) {
		if(TIM2_CH1_DCVAL < position){
			TIM2_CH1_DCVAL += TIM2_CH1_STEP;
		}else if (TIM2_CH1_DCVAL > position){
			TIM2_CH1_DCVAL -= TIM2_CH1_STEP;
		}
		TIM2->CCR1 = TIM2_CH1_DCVAL;
		HAL_Delay(5);
	}
}

void RGBLED_Run(uint8_t colour)
{
	switch(colour)
	{
	case 0: // purple
		HAL_GPIO_WritePin(GPIOB, BLU_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOB, GRN_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOB, RED_Pin, GPIO_PIN_SET);
		break;
	case 1: // red
		HAL_GPIO_WritePin(GPIOB, BLU_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOB, GRN_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOB, RED_Pin, GPIO_PIN_SET);
		break;
	case 2: // green
		HAL_GPIO_WritePin(GPIOB, BLU_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOB, GRN_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOB, RED_Pin, GPIO_PIN_RESET);
		break;
	case 3: // blue
		HAL_GPIO_WritePin(GPIOB, BLU_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOB, GRN_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOB, RED_Pin, GPIO_PIN_RESET);
		break;
	case 4: // none
		HAL_GPIO_WritePin(GPIOB, BLU_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOB, GRN_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOB, RED_Pin, GPIO_PIN_RESET);
		break;
	case 5: // white
		HAL_GPIO_WritePin(GPIOB, BLU_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOB, GRN_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOB, RED_Pin, GPIO_PIN_SET);
		break;
	}
}

int generate_random(int min, int max){
	return (rand() % (max-min+1))+ min;
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
  MX_USART2_UART_Init();
  MX_USART6_UART_Init();
  MX_ADC1_Init();
  MX_TIM3_Init();
  MX_TIM2_Init();
  MX_TIM4_Init();
  MX_TIM5_Init();
  /* USER CODE BEGIN 2 */

  HAL_TIM_Base_Start(&htim2);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
  TIM3->PSC = 16-1;
  TIM3->ARR = 20000-1;
  TIM3->CCR1 = TIM2_CH1_DCVAL;

  //HAL_TIM_Base_Init(&htim3);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
  TIM3->PSC = 0;
  TIM3->ARR = 60000;
  TIM3->CCR1 = 0;
  TIM3->CCR3 = 0;

  HAL_TIM_Base_Init(&htim4);
  HAL_TIM_Base_Start(&htim4);
  HAL_TIM_IC_Start_IT(&htim4, TIM_CHANNEL_1);

  HAL_TIM_Base_Start_IT(&htim5);
  clock_hours = 0;
  clock_mins = 0;
  clock_secs = 0;
  /* USER CODE END 2 */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  uint8_t pipe_connection[4] = {0};
  uint8_t pipe_pwm_mode[4] = {0};
  uint8_t current_time = 0;
  uint8_t start_time[4] = {0};
  uint8_t end_time[4] = {0};

  sprintf((char*)txd_msg_buffer, "\r\n\nSETUP MODE");
  HAL_UART_Transmit(&huart6, txd_msg_buffer, strlen((char*)txd_msg_buffer), 1000);
  HAL_Delay(500);

  sprintf((char*)txd_msg_buffer,"\r\n\nFIRST PIPELINE CHOICE FOR CONNECTION: ");
  HAL_UART_Transmit(&huart6, txd_msg_buffer, strlen((char*)txd_msg_buffer),1000);
  rcv_intpt_flag = 00;
  HAL_UART_Receive_IT(&huart6, &byte, 1);
  while(rcv_intpt_flag == (00)) {}
  pipe_connection[0] = (byte[0]-'0');
  sprintf((char*)txd_msg_buffer,"\r\nFIRST PIPELINE CHOICE FOR MOTOR PWM: ");
  HAL_UART_Transmit(&huart6, txd_msg_buffer, strlen((char*)txd_msg_buffer),1000);
  rcv_intpt_flag = 00;
  HAL_UART_Receive_IT(&huart6, &byte, 2);
  while(rcv_intpt_flag == (00)) {}
  pipe_pwm_mode[0] = (byte[0]-'0');

  sprintf((char*)txd_msg_buffer,"\r\nSECOND ZONE CHOICE FOR CONNECTION: ");
  HAL_UART_Transmit(&huart6, txd_msg_buffer, strlen((char*)txd_msg_buffer),1000);
  rcv_intpt_flag = 00;
  HAL_UART_Receive_IT(&huart6, &byte, 1);
  while(rcv_intpt_flag == (00)) {}
  pipe_connection[1] = (byte[0]-'0');
  sprintf((char*)txd_msg_buffer,"\r\nSECOND ZONE CHOICE FOR MOTOR PWM: ");
  HAL_UART_Transmit(&huart6, txd_msg_buffer, strlen((char*)txd_msg_buffer),1000);
  rcv_intpt_flag = 00;
  HAL_UART_Receive_IT(&huart6, &byte, 1);
  while(rcv_intpt_flag == (00)) {}
  pipe_pwm_mode[1] = (byte[0]-'0');

  sprintf((char*)txd_msg_buffer,"\r\nTHIRD ZONE CHOICE FOR CONNECTION: ");
  HAL_UART_Transmit(&huart6, txd_msg_buffer, strlen((char*)txd_msg_buffer),1000);
  rcv_intpt_flag = 00;
  HAL_UART_Receive_IT(&huart6, &byte, 1);
  while(rcv_intpt_flag == (00)) {}
  pipe_connection[2] = (byte[0]-'0');
  sprintf((char*)txd_msg_buffer,"\r\nTHIRD ZONE CHOICE FOR MOTOR PWM: ");
  HAL_UART_Transmit(&huart6, txd_msg_buffer, strlen((char*)txd_msg_buffer),1000);
  rcv_intpt_flag = 00;
  HAL_UART_Receive_IT(&huart6, &byte, 1);
  while(rcv_intpt_flag == (00)) {}
  pipe_pwm_mode[2] = (byte[0]-'0');

  sprintf((char*)txd_msg_buffer,"\r\nFOURTH ZONE CHOICE FOR CONNECTION: ");
  HAL_UART_Transmit(&huart6, txd_msg_buffer, strlen((char*)txd_msg_buffer),1000);
  rcv_intpt_flag = 00;
  HAL_UART_Receive_IT(&huart6, &byte, 1);
  while(rcv_intpt_flag == (00)) {}
  pipe_connection[3] = (byte[0]-'0');
  sprintf((char*)txd_msg_buffer,"\r\nFOURTH ZONE CHOICE FOR MOTOR PWM: ");
  HAL_UART_Transmit(&huart6, txd_msg_buffer, strlen((char*)txd_msg_buffer),1000);
  rcv_intpt_flag = 00;
  HAL_UART_Receive_IT(&huart6, &byte, 1);
  while(rcv_intpt_flag == (00)) {}
  pipe_pwm_mode[3] = (byte[0]-'0');


  sprintf((char*)txd_msg_buffer,"\r\n\nCURRENT WALL CLOCK TIME: ");
  HAL_UART_Transmit(&huart6, txd_msg_buffer, strlen((char*)txd_msg_buffer),1000);
  rcv_intpt_flag = 00;
  HAL_UART_Receive_IT(&huart6, &byte, 2);
  while(rcv_intpt_flag == (00)) {}
  current_time = (byte[0] - '0') * 10 + (byte[1] - '0');

  sprintf((char*)txd_msg_buffer,"\r\nINLET WALL CLOCK START TIME: ");
  HAL_UART_Transmit(&huart6, txd_msg_buffer, strlen((char*)txd_msg_buffer),1000);
  rcv_intpt_flag = 00;
  HAL_UART_Receive_IT(&huart6, &byte, 2);
  while(rcv_intpt_flag == (00)) {}
  start_time[0] = (byte[0] - '0') * 10 + (byte[1] - '0');
  sprintf((char*)txd_msg_buffer,"\r\nINLET WALL CLOCK STOP TIME: ");
  HAL_UART_Transmit(&huart6, txd_msg_buffer, strlen((char*)txd_msg_buffer),1000);
  rcv_intpt_flag = 00;
  HAL_UART_Receive_IT(&huart6, &byte, 2);
  while(rcv_intpt_flag == (00)) {}
  end_time[0] = (byte[0] - '0') * 10 + (byte[1] - '0');

  sprintf((char*)txd_msg_buffer,"\r\nFIRST ZONE CHOICE WALL CLOCK START TIME: ");
  HAL_UART_Transmit(&huart6, txd_msg_buffer, strlen((char*)txd_msg_buffer),1000);
  rcv_intpt_flag = 00;
  HAL_UART_Receive_IT(&huart6, &byte, 2);
  while(rcv_intpt_flag == (00)) {}
  start_time[1] = (byte[0] - '0') * 10 + (byte[1] - '0');
  sprintf((char*)txd_msg_buffer,"\r\nFIRST ZONE CHOICE WALL CLOCK STOP TIME: ");
  HAL_UART_Transmit(&huart6, txd_msg_buffer, strlen((char*)txd_msg_buffer),1000);
  rcv_intpt_flag = 00;
  HAL_UART_Receive_IT(&huart6, &byte, 2);
  while(rcv_intpt_flag == (00)) {}
  end_time[1] = (byte[0] - '0') * 10 + (byte[1] - '0');

  sprintf((char*)txd_msg_buffer,"\r\nSECOND ZONE CHOICE WALL CLOCK START TIME: ");
  HAL_UART_Transmit(&huart6, txd_msg_buffer, strlen((char*)txd_msg_buffer),1000);
  rcv_intpt_flag = 00;
  HAL_UART_Receive_IT(&huart6, &byte, 2);
  while(rcv_intpt_flag == (00)) {}
  start_time[2] = (byte[0] - '0') * 10 + (byte[1] - '0');
  sprintf((char*)txd_msg_buffer,"\r\nSECOND ZONE CHOICE WALL CLOCK STOP TIME: ");
  HAL_UART_Transmit(&huart6, txd_msg_buffer, strlen((char*)txd_msg_buffer),1000);
  rcv_intpt_flag = 00;
  HAL_UART_Receive_IT(&huart6, &byte, 2);
  while(rcv_intpt_flag == (00)) {}
  end_time[2] = (byte[0] - '0') * 10 + (byte[1] - '0');

  sprintf((char*)txd_msg_buffer,"\r\nTHIRD ZONE CHOICE WALL CLOCK START TIME: ");
  HAL_UART_Transmit(&huart6, txd_msg_buffer, strlen((char*)txd_msg_buffer),1000);
  rcv_intpt_flag = 00;
  HAL_UART_Receive_IT(&huart6, &byte, 2);
  while(rcv_intpt_flag == (00)) {}
  start_time[3] = (byte[0] - '0') * 10 + (byte[1] - '0');

  sprintf((char*)txd_msg_buffer,"\r\nTHIRD ZONE CHOICE WALL CLOCK STOP TIME: ");
  HAL_UART_Transmit(&huart6, txd_msg_buffer, strlen((char*)txd_msg_buffer),1000);
  rcv_intpt_flag = 00;
  HAL_UART_Receive_IT(&huart6, &byte, 2);
  while(rcv_intpt_flag == (00)) {}
  end_time[3] = (byte[0] - '0') * 10 + (byte[1] - '0');

  /*
  pipe_connection[0] = 1;
  pipe_connection[1] = 0;
  pipe_connection[2] = 2;
  pipe_connection[3] = 3;
  pipe_pwm_mode[0] = 0;
  pipe_pwm_mode[1] = 0;
  pipe_pwm_mode[2] = 0;
  pipe_pwm_mode[3] = 0;
  start_time[0] = 0;
  start_time[1] = 2;
  start_time[2] = 4;
  start_time[3] = 6;
  end_time[0] = 2;
  end_time[1] = 4;
  end_time[2] = 6;
  end_time[3] = 8;
  current_time = 0;
  */

  sprintf((char*)txd_msg_buffer,"\r\n\nPRESS BLUE BUTTON TO PRECEED TO RUN MODE");
  HAL_UART_Transmit(&huart6, txd_msg_buffer, strlen((char*)txd_msg_buffer),1000);
  while(HAL_GPIO_ReadPin(GPIOC, B1_Pin)){
	  HAL_GPIO_WritePin(GPIOA, LD2_Pin, GPIO_PIN_SET);
	  HAL_Delay(500);
	  HAL_GPIO_WritePin(GPIOA, LD2_Pin, GPIO_PIN_RESET);
	  HAL_Delay(500);
  }

  sprintf((char*)txd_msg_buffer,"\r\n\nRUN MODE");
  HAL_UART_Transmit(&huart6, txd_msg_buffer, strlen((char*)txd_msg_buffer),1000);
  HAL_GPIO_WritePin(GPIOA, LD2_Pin, GPIO_PIN_SET);


  clock_hours = current_time;
  clock_mins = 0;
  clock_secs = 0;
  wall_clock_hr_update_flag = 1;

  int zone = 0;
  int motor_speed = 0;
  int servo_position[4] = {1000, 1500, 2000, 2500};


    while (1)
    {
    	//potentiometer code
    	ADC_Select_CH(9);
    	HAL_ADC_Start(&hadc1);
    	HAL_ADC_PollForConversion(&hadc1, 1000);
    	uint8_t ADC_CH9 = HAL_ADC_GetValue(&hadc1);
    	HAL_ADC_Stop(&hadc1);

    	current_time = clock_hours;

    	hcsr04_Rx_flag = 0;
    	first_edge = 0;
    	time_edge1 = 0;
    	time_edge2 = 0;
    	time_diff = 0;

    	HCSR04_TRIG_PULSE();
    	while(hcsr04_Rx_flag == 0){};
    	time_diff = time_edge2-time_edge1;
    	distance = (float)time_diff/54;
    	filled = 100-((((float)distance-top_distance)/(bottom_distance-top_distance))*100);
    	sprintf((char*)txd_msg_buffer,"\r\n %d and %d", distance, filled);
    	    			HAL_UART_Transmit(&huart6, txd_msg_buffer, strlen((char*)txd_msg_buffer),1000);


    	if (filled>99) {
    		filled = 99;
    	} else if (filled<0) {
    		filled = 0;
    		empty = 1;
    	}
    	DIGITS_Display(filled);

    	//maybe need to wait until sensor shows 99

    		if(current_time >= start_time[zone] && current_time < end_time[zone]){
    			RGBLED_Run(pipe_connection[zone]);
    			SERVO_Run(servo_position[pipe_connection[zone]]);
    		    switch(pipe_pwm_mode[zone])
    		    {
    		    case 0:
    		    	motor_speed = (((float)ADC_CH9/255)*100);
    		    	rpm = generate_random(((float)ADC_CH9/255)*400,((float)ADC_CH9/255)*500);
    		    	break;
    		    case 1:
    		    	motor_speed = 70;
    		    	rpm = generate_random(325,375);
    		    	break;
    		    case 2:
    		    	motor_speed = 85;
    		    	rpm = generate_random(375,425);
    		    	break;
    		    case 3:
    		    	motor_speed = 99;
    		    	rpm = generate_random(425,475);
    		    	break;
    		    }
    		    if(pipe_connection[zone] == 0){
    		        //motor_speed *= -1;
    		    }
    		    MOTOR_Run(motor_speed);
    		} else if(current_time >= end_time[zone]){
    			zone++;
    		}

    	if(wall_clock_hr_update_flag ==1){


    		if(empty == 1 || zone > 3){
    			sprintf((char*)txd_msg_buffer,"\r\nRESERVOIR EMPTY");
    			HAL_UART_Transmit(&huart6, txd_msg_buffer, strlen((char*)txd_msg_buffer),1000);
    			MOTOR_Run(0);
    			TIM3->CCR1 = 0;
    			TIM3->CCR3 = 0;
    			HAL_GPIO_WritePin(GPIOA, LD2_Pin,GPIO_PIN_SET);
    			while(1)
    			{
    				RGBLED_Run(5);
    				HAL_Delay(500);
    				RGBLED_Run(4);
    				HAL_Delay(500);
    			}
    		}



			sprintf((char*)txd_msg_buffer,"\r\nHour: %d | Zone %d | Motor Speed: %d | Motor RPM: %d | Water Reservoir Depth %d |", clock_hours, zone, motor_speed, rpm, filled);
			HAL_UART_Transmit(&huart6, txd_msg_buffer, strlen((char*)txd_msg_buffer),1000);
			wall_clock_hr_update_flag = 0;
    	}
      /*

  	  // Potentiometer with dc motor code
  	  TIM3_DCVAL = ((float)ADC_CH9/255)*60000;
  	  TIM3->CCR1 = (int)TIM3_DCVAL;

  	  sprintf((char*)txd_msg_buffer, "RPM COUNT: %d \r \n", rpm_tick_count);
  	  HAL_UART_Transmit(&huart6,txd_msg_buffer, strlen((char*)txd_msg_buffer), 1000);


	  //dc motor code
	  TIM3->CCR1 = (int)TIM3_DCVAL;
	  rcv_intpt_flag = 00;
	  HAL_UART_Receive_IT(&huart6, &byte, 1);
	  HAL_UART_Transmit(&huart6,txd_msg_buffer, strlen((char*)txd_msg_buffer), 1000);
	  while(rcv_intpt_flag == (00)) {}
	  */
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
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
  hadc1.Init.Resolution = ADC_RESOLUTION_8B;
  hadc1.Init.ScanConvMode = ENABLE;
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
  sConfig.Channel = ADC_CHANNEL_9;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_15CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

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
  htim2.Init.Prescaler = 16-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 20000-1;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
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
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 500-1;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
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

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 16-1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 2000-1;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 1200-1;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.Pulse = 0;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

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
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 16-1;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 65536-1;
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
  if (HAL_TIM_IC_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_BOTHEDGE;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim4, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

}

/**
  * @brief TIM5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM5_Init(void)
{

  /* USER CODE BEGIN TIM5_Init 0 */

  /* USER CODE END TIM5_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM5_Init 1 */

  /* USER CODE END TIM5_Init 1 */
  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 53;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 1000-1;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim5) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim5, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM5_Init 2 */

  /* USER CODE END TIM5_Init 2 */

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
  /* This callback is called by the HAL_UART_IRQHandler when the given number of bytes are received */

  /* USER CODE END USART6_Init 2 */

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
  HAL_GPIO_WritePin(GPIOC, DIGIT_A3_Pin|DIGIT_A2_Pin|DIGIT_A1_Pin|DIGIT_A0_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, DIGIT_B3_Pin|DIGIT_B2_Pin|DIGIT_B0_Pin|LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, DIGIT_B1_Pin|BLU_Pin|HCSR04_TRIG_Pin|GRN_Pin
                          |RED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : DIGIT_A3_Pin DIGIT_A2_Pin DIGIT_A1_Pin DIGIT_A0_Pin */
  GPIO_InitStruct.Pin = DIGIT_A3_Pin|DIGIT_A2_Pin|DIGIT_A1_Pin|DIGIT_A0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : DIGIT_B3_Pin DIGIT_B2_Pin DIGIT_B0_Pin LD2_Pin */
  GPIO_InitStruct.Pin = DIGIT_B3_Pin|DIGIT_B2_Pin|DIGIT_B0_Pin|LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : DIGIT_B1_Pin BLU_Pin HCSR04_TRIG_Pin GRN_Pin
                           RED_Pin */
  GPIO_InitStruct.Pin = DIGIT_B1_Pin|BLU_Pin|HCSR04_TRIG_Pin|GRN_Pin
                          |RED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : RPM_Tick_Pin */
  GPIO_InitStruct.Pin = RPM_Tick_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(RPM_Tick_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI2_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART6)
    {
        /* Transmit one byte with 100 ms timeout. Transmit the character that was received into the variable called "byte" */
        HAL_UART_Transmit(&huart6, &byte, 1, 100);
        rcv_intpt_flag = 1;
    }
}

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
if (htim->Instance == TIM4)
    {
        if (htim->Channel == 1) // if the interrupt source is channel 1
        {
            if (first_edge == 0) // if the first value is not captured
            {
                time_edge1 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1); // read the first value
                first_edge = 1; // set the first captured as true
            }
   else // if the first is already captured
            {
                time_edge2 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1); // read second value
                __HAL_TIM_SET_COUNTER(htim, 0); // reset the counter
                hcsr04_Rx_flag = 1; // set the interrupt flag for result done
            }
        }
    }
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == RPM_Tick_Pin)
    {
        rpm_tick_count += 1;
    }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if ((htim->Instance == TIM5))
    {
        wall_clock_hr_update_flag = 0; // screen updates occur hourly on the half-hour
        clock_secs += 1;

        if ((clock_secs == 60))
        {
            clock_mins += 1;
            clock_secs = 0;
        }

        if ((clock_mins == 60))
        {
            clock_hours += 1;
            clock_mins = 0;
        }

        wall_clock_hr_update_flag = 1; // screen updates occur hourly on the hour
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
