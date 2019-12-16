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
#include "stdio.h"
#include "stdlib.h"
#include "StepperMot.h"
#include "Display.h"
#include "u8g2.h"
#include "SPI_parallel.h"
#include "can_handler.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum
{
	DISP_M0, //Default
	DISP_M1,
	DISP_M2,
	DISP_Mcharge,
	DISP_Mlast
}DISP_MODE;

typedef enum
{
	INST_INIT,
	INST_DRIVE,
	INST_CHARGE,
	INST_OFF,
	INST_ERR
}INSTRUMENT_STATE;

char drive_mode_char[] = "PNRDSEU";


/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/*DISPLAY dimensions*/
#define DISP_W 128
#define DISP_H 64

#define DISP_MOD_W 20
#define DISP_MOD_H 22

#define DISP_LINE1_Y 42
#define DISP_ODO_size 10
#define DISP_LINE2_Y 53
#define DISP_TRIP_size 10

/*LED control definitions*/

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;
CAN_HandleTypeDef hcan1;
I2C_HandleTypeDef hi2c1;
SPI_HandleTypeDef hspi1;
/* USER CODE BEGIN PV */


s_STEPPER stepper1, stepper2, stepper3;
u8g2_t u8g2;
uint64_t baseTimer = 0;
uint8_t period_1, period_10, period_100, period_1000;
uint8_t sw1_pressed = 0, sw2_pressed = 0;
uint32_t odo_dist = 12340, trip_dist = 2500;
uint8_t drive_mode = 0;
DISP_MODE display_mode = DISP_M0;

uint16_t hp_batt_tem_C = 0;
uint16_t hp_batt_volt = 0;
uint16_t hp_batt_perc = 0;

uint16_t lp_batt_volt = 0; /*OK 12-15V 0.1V*/

char univ_string4[] = "1234";
char univ_string6[] = "123456";

char char_mode[] = "P";
char char_odo[] = "000000";
char char_trip[] = "0000.0";

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_DMA_Init(void);
static void MX_CAN1_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void L_button()
{
	trip_dist = 0;
	if(sw2_pressed)	HAL_GPIO_TogglePin(Backlight_GPIO_Port, Backlight_Pin);
}
void R_button() /*OK*/
{
	display_mode++;
	if(display_mode > DISP_M2) display_mode = DISP_M0;
}

#define OUT_BAT400 	2
#define OUT_REL400 	1
#define OUT_FAN_BAT	5
#define OUT_FAN_MOT 4
#define OUT_PUMPS	3


void set_out(uint8_t out_nr, uint8_t out_enable)
{
	switch (out_nr) {
		case 1:
			HAL_GPIO_WritePin(OUT1_GPIO_Port, OUT1_Pin, out_enable);
			break;
		case 2:
			HAL_GPIO_WritePin(OUT2_GPIO_Port, OUT2_Pin, out_enable);
			break;
		case 3:
			HAL_GPIO_WritePin(OUT3_GPIO_Port, OUT3_Pin, out_enable);
			break;
		case 4:
			HAL_GPIO_WritePin(OUT4_GPIO_Port, OUT4_Pin, out_enable);
			break;
		case 5:
			HAL_GPIO_WritePin(OUT5_GPIO_Port, OUT5_Pin, out_enable);
			break;
		case 6:
			HAL_GPIO_WritePin(OUT6_GPIO_Port, OUT6_Pin, out_enable);
			break;
	}
}

typedef enum
{
	LED_PARKING, /*OK*/
	LED_BRAKE_PEDAL,
	LED_LOW_BATT, /*OK*/
	LED_CHARGING_ERROR, /*OK*/
	LED_OVERHEAT,
	LED_CHECK_ENGINE
}LED_INDICATOR;

void set_indicator(LED_INDICATOR indicator, uint8_t ind_enable)
{
	switch (indicator) {
		case LED_PARKING:
			HAL_GPIO_WritePin(D1_R_GPIO_Port,D1_R_Pin,ind_enable);
			break;
		case LED_BRAKE_PEDAL:
			HAL_GPIO_WritePin(D6_G_GPIO_Port,D6_G_Pin,ind_enable);
			break;
		case LED_LOW_BATT:
			HAL_GPIO_WritePin(D5_Y_GPIO_Port,D5_Y_Pin,ind_enable);
			break;
		case LED_CHARGING_ERROR:
			HAL_GPIO_WritePin(D2_R_GPIO_Port, D2_R_Pin ,ind_enable);
			break;
		case LED_OVERHEAT:
			HAL_GPIO_WritePin(D3_R_GPIO_Port,D3_R_Pin,ind_enable);
			break;
		case LED_CHECK_ENGINE:
			HAL_GPIO_WritePin(D4_Y_GPIO_Port,D4_Y_Pin,ind_enable);
			break;
	}
}

void get_lp_voltage()
{
	if(HAL_ADC_GetValue(&hadc1) < 1900) lp_batt_volt = 0;
	else
		lp_batt_volt = (10 * HAL_ADC_GetValue(&hadc1) - 18120)/104;
}


void int2str(char* array, int32_t number, uint32_t a_length, char fill_char) /*OK*/
{
	for(uint8_t i = a_length; i > 0; i--)
	{
		if(number == 0 && i < a_length ) array[i-1] = fill_char;
		else
		{
			array[i-1] = (number % 10) + '0';
			number = number/10;
		}
	}
}

void fp2str(char* array, int32_t number, uint8_t point_pos, uint32_t a_length, char fill_char)/*OK*/
{
	for(uint8_t i = a_length; i > 0; i--)
	{
		if(i == a_length-point_pos && i < a_length)
		{
			array[i-1] = '.';
		}
		else if(number == 0 && i < a_length ) array[i-1] = fill_char;
		else
		{
			array[i-1] = (number % 10) + '0';
			number = number/10;
		}
	}
}

void draw_bargraph(uint8_t y_line, uint16_t percent, char* b_min, char* b_max)/*OK*/
{
	u8g2_SetFont(&u8g2, u8g2_font_ncenR08_tr);
	u8g2_DrawStr(&u8g2, 1, y_line + 8, b_min);
	u8g2_DrawStr(&u8g2, 105, y_line + 8, b_max);
	u8g2_DrawFrame(&u8g2, 24, y_line, 80, 8);
	u8g2_DrawBox(&u8g2, 24, y_line, (percent<<3)/10, 8);
}

void display_redraw()
{
	u8g2_ClearBuffer(&u8g2);


	switch(display_mode)
	{
		// default
		case DISP_M0:
			u8g2_SetFont(&u8g2, u8g2_font_ncenR08_tr);
			u8g2_DrawStr(&u8g2, 30, 20, "Mode1");
			draw_bargraph(30,hp_batt_tem_C,"  0%","100%");
			break;
		// Mode1 - battery
		case DISP_M1:
			u8g2_SetFont(&u8g2, u8g2_font_ncenR08_tr);
			u8g2_DrawStr(&u8g2, 30, 20, "Mode2");

			u8g2_DrawStr(&u8g2, 60, 38, "V");
			fp2str(univ_string6, lp_batt_volt, 1, 6, ' ');
			u8g2_DrawStr(&u8g2, 30, 38, univ_string6);

			break;
		// Mode2 - power settings
		case DISP_M2:
			u8g2_SetFont(&u8g2, u8g2_font_ncenR08_tr);
			u8g2_DrawStr(&u8g2, 30, 20, "Mode3");
			break;
		case DISP_Mcharge: /*OK*/
			u8g2_SetFont(&u8g2, u8g2_font_ncenR08_tr);
			u8g2_DrawStr(&u8g2, 35, 20, "CHARGING");
			draw_bargraph(30,hp_batt_perc,"  0%","100%");
			break;
		case DISP_Mlast:
		default:
			break;
	}
	//MODE
	u8g2_DrawFrame(&u8g2, DISP_W-DISP_MOD_W, DISP_H-DISP_MOD_H, DISP_MOD_W, DISP_MOD_H);
	u8g2_SetFont(&u8g2, u8g2_font_timB18_tr);
	char_mode[0] = drive_mode_char[drive_mode];
	u8g2_DrawStr(&u8g2, DISP_W - DISP_MOD_W + 2, DISP_H-2, char_mode);
	//ODO + TRIP
	u8g2_DrawLine(&u8g2, 0, DISP_LINE1_Y, DISP_W - DISP_MOD_W , DISP_LINE1_Y);
	u8g2_DrawLine(&u8g2, 0, DISP_LINE2_Y, DISP_W - DISP_MOD_W, DISP_LINE2_Y);
	u8g2_SetFont(&u8g2, u8g2_font_ncenB08_tr);
	u8g2_DrawStr(&u8g2, 02, DISP_LINE1_Y + DISP_TRIP_size, "TRIP");
	u8g2_DrawStr(&u8g2, 02, DISP_LINE2_Y + DISP_ODO_size, "ODO");
	u8g2_SetFont(&u8g2, u8g2_font_ncenR08_tr);
	fp2str(char_trip, trip_dist, 1, 6, '0');
	u8g2_DrawStr(&u8g2, 60, DISP_LINE1_Y + DISP_TRIP_size, char_trip);
	int2str(char_odo, odo_dist, 6, '0');
	u8g2_DrawStr(&u8g2, 60, DISP_LINE2_Y + DISP_ODO_size, char_odo);
	u8g2_UpdateDisplay(&u8g2);
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */

HAL_CAN_StateTypeDef CAN_state1;
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */
  

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
  INSTRUMENT_STATE inst_state = INST_INIT;

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  SysTick_Config(168000);

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_ADC1_Init();
  MX_DMA_Init();
  MX_CAN1_Init();
  MX_I2C1_Init();
  MX_SPI1_Init();
  /* USER CODE BEGIN 2 */
  HAL_Delay(100);
  get_lp_voltage();

  u8g2_Setup_ks0108_128x64_f(&u8g2, U8G2_R0, u8x8_byte_parallel_hw_spi, u8x8_gpio_delay_parallel);
  u8g2_InitDisplay(&u8g2);
  u8g2_SetPowerSave(&u8g2, 0);

  if(lp_batt_volt < 50) inst_state = INST_CHARGE;

  if(inst_state == INST_INIT)
  {
	  //init STEPPERS
	  stepper1.port[0] = GPIOE;
	  stepper1.pin[0] = GPIO_PIN_9;
	  stepper1.port[1] = GPIOE;
	  stepper1.pin[1] = GPIO_PIN_11;
	  stepper1.port[2] = GPIOE;
	  stepper1.pin[2] = GPIO_PIN_8;
	  stepper1.port[3] = GPIOB;
	  stepper1.pin[3] = GPIO_PIN_0;
	  stepper1.direction = FORWARD;
	  StepperMot_init(&stepper1, 20);

	  stepper2.port[0] = GPIOE;
	  stepper2.pin[0] = GPIO_PIN_13;
	  stepper2.port[1] = GPIOC;
	  stepper2.pin[1] = GPIO_PIN_6;
	  stepper2.port[2] = GPIOB;
	  stepper2.pin[2] = GPIO_PIN_1;
	  stepper2.port[3] = GPIOA;
	  stepper2.pin[3] = GPIO_PIN_7;
	  stepper2.direction = FORWARD;
	  StepperMot_init(&stepper2, 100);

	  stepper3.port[0] = GPIOC;
	  stepper3.pin[0] = GPIO_PIN_7;
	  stepper3.port[1] = GPIOC;
	  stepper3.pin[1] = GPIO_PIN_8;
	  stepper3.port[2] = GPIOB;
	  stepper3.pin[2] = GPIO_PIN_14;
	  stepper3.port[3] = GPIOB;
	  stepper3.pin[3] = GPIO_PIN_15;
	  stepper3.direction = BACKWARD;
	  StepperMot_init(&stepper3, 20);

		  //set LEDS ON
		  HAL_GPIO_WritePin(D1_R_GPIO_Port,D1_R_Pin,GPIO_PIN_SET);
		  HAL_GPIO_WritePin(D2_R_GPIO_Port, D2_R_Pin ,GPIO_PIN_SET);
		  HAL_GPIO_WritePin(D3_R_GPIO_Port,D3_R_Pin,GPIO_PIN_SET);
		  HAL_GPIO_WritePin(D4_Y_GPIO_Port,D4_Y_Pin,GPIO_PIN_SET);
		  HAL_GPIO_WritePin(D5_Y_GPIO_Port,D5_Y_Pin,GPIO_PIN_SET);
		  HAL_GPIO_WritePin(D6_G_GPIO_Port,D6_G_Pin,GPIO_PIN_SET);
		  HAL_GPIO_WritePin(Backlight_GPIO_Port, Backlight_Pin, GPIO_PIN_SET); //backlight
  }

  HAL_GPIO_WritePin(Disp_backlight_GPIO_Port,Disp_backlight_Pin,GPIO_PIN_SET); //LCD backlight


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  switch (inst_state)
	  {
	  	  case INST_INIT :
			  if(stepper1.at_position && stepper2.at_position && stepper3.at_position)
			  {
				  inst_state = INST_DRIVE;
				  HAL_GPIO_WritePin(D1_R_GPIO_Port,D1_R_Pin,GPIO_PIN_RESET);
				  HAL_GPIO_WritePin(D2_R_GPIO_Port, D2_R_Pin ,GPIO_PIN_RESET);
				  HAL_GPIO_WritePin(D3_R_GPIO_Port,D3_R_Pin,GPIO_PIN_RESET);
				  HAL_GPIO_WritePin(D4_Y_GPIO_Port,D4_Y_Pin,GPIO_PIN_RESET);
				  HAL_GPIO_WritePin(D5_Y_GPIO_Port,D5_Y_Pin,GPIO_PIN_RESET);
				  HAL_GPIO_WritePin(D6_G_GPIO_Port,D6_G_Pin,GPIO_PIN_RESET);
			  }
	  		  break;
	  	  case INST_DRIVE:
	  		  /*TODO DRIVE MODE*/
	  		  /*LED control*/


	  		  set_out(OUT_REL400, 1);  /*motor relay ON*/
	  		  set_out(OUT_BAT400, 1); /*HP battery switch ON*/
	  		  set_out(OUT_PUMPS, 1);  /*OUT control - cooling*/

	  		  if(lp_batt_volt < 125) set_indicator(LED_CHARGING_ERROR, 1);
	  		  else set_indicator(LED_CHARGING_ERROR, 0);

	  		  if(lp_batt_volt < 50) inst_state = INST_OFF;

	  		  break;
	  	  case INST_CHARGE:
	  		  display_mode = DISP_Mcharge;
	  		  /*HP battery switch ON*/
	  		  set_out(OUT_BAT400, 1);
	  		  /*TODO CHARGE MODE*/
	  		  break;
	  	  case INST_OFF:
	  		  HAL_GPIO_WritePin(D1_R_GPIO_Port,D1_R_Pin,GPIO_PIN_RESET);
	  		  HAL_GPIO_WritePin(D2_R_GPIO_Port, D2_R_Pin ,GPIO_PIN_RESET);
	  		  HAL_GPIO_WritePin(D3_R_GPIO_Port,D3_R_Pin,GPIO_PIN_RESET);
	  		  HAL_GPIO_WritePin(D4_Y_GPIO_Port,D4_Y_Pin,GPIO_PIN_RESET);
	  		  HAL_GPIO_WritePin(D5_Y_GPIO_Port,D5_Y_Pin,GPIO_PIN_RESET);
	  		  HAL_GPIO_WritePin(D6_G_GPIO_Port,D6_G_Pin,GPIO_PIN_RESET);
	  		  HAL_GPIO_WritePin(Disp_backlight_GPIO_Port,Disp_backlight_Pin,GPIO_PIN_RESET); //LCD backlight
	  		  //HAL_GPIO_WritePin(Backlight_GPIO_Port, Backlight_Pin, GPIO_PIN_RESET); //backlight

	  		  /*steppers to 0 */
	  		  stepper1.wPosition = 0;
	  		  stepper2.wPosition = 0;
	  		  stepper3.wPosition = 0;

	  		  set_out(OUT_REL400, 0);/*motor relay OFF*/
			  set_out(OUT_BAT400, 0);/*HP battery switch OFF*/
			  set_out(OUT_PUMPS, 0);
			  set_out(OUT_FAN_BAT, 0);
			  set_out(OUT_FAN_MOT, 0);

	  		  /*OUT control OFF*/

	  		  /*TODO save distance to EEPROM */
	  		  break;
	  	  case INST_ERR:
	  	  default:
	  		  break;
	  }
	  if(period_1)
	  {
		  get_lp_voltage();
		  StepperMot_step(&stepper2);
		  period_1 = 0;


		  CAN_state1 = HAL_CAN_GetState(&hcan1);
	  }
	  if(period_10)
	  {
		  StepperMot_step(&stepper1);
		  StepperMot_step(&stepper3);

		  if(sw1_pressed == 0 && HAL_GPIO_ReadPin(SW1_GPIO_Port, SW1_Pin))
		  {
			  sw1_pressed = 1;
			  L_button();
		  }
		  else if(HAL_GPIO_ReadPin(SW1_GPIO_Port, SW1_Pin) == 0)sw1_pressed = 0;

		  if(sw2_pressed == 0 && HAL_GPIO_ReadPin(SW2_GPIO_Port, SW2_Pin))
		  {
		  	  sw2_pressed = 1;
		  	  R_button();
		  }
		  else if(HAL_GPIO_ReadPin(SW2_GPIO_Port, SW2_Pin) == 0) sw2_pressed = 0;

		  period_10 = 0;
	  }
	  if(period_100)
{

		  period_100 = 0;
	  }
	  if(period_1000)
	  {
		  display_redraw();
		  period_1000 = 0;
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
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV4;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
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
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ENABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SEQ_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
  */
  sConfig.Channel = ADC_CHANNEL_11;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_28CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
  */
  /*sConfig.Channel = ADC_CHANNEL_12;
  sConfig.Rank = 2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }*/
  /* USER CODE BEGIN ADC1_Init 2 */
  HAL_ADC_Start(&hadc1);

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief CAN1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN1_Init(void)
{

  /* USER CODE BEGIN CAN1_Init 0 */

  /* USER CODE END CAN1_Init 0 */

  /* USER CODE BEGIN CAN1_Init 1 */

  /* USER CODE END CAN1_Init 1 */
  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 12;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_3TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_3TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = DISABLE;
  hcan1.Init.AutoWakeUp = DISABLE;
  hcan1.Init.AutoRetransmission = DISABLE;
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN1_Init 2 */
  HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
  /* USER CODE END CAN1_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */
  HAL_SPI_DeInit(&hspi1);
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
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOF, Disp_backlight_Pin|Backlight_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, D5_Y_Pin|D3_R_Pin|D2_R_Pin|DISP_CS_Pin 
                          |SM2__Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, DISP_RST_Pin|DISP_CD_Pin|SM2B_Pin|SM3A_Pin 
                          |SM3B_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, SM1B__Pin|SM2A__Pin|D4_Y_Pin|SM3A__Pin 
                          |SM3B__Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, SM1A__Pin|SM1A_Pin|SM1B_Pin|SM2A_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOG, D1_R_Pin|D6_G_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, OUT6_Pin|OUT5_Pin|OUT4_Pin|OUT3_Pin 
                          |OUT2_Pin|OUT1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : Disp_backlight_Pin Backlight_Pin */
  GPIO_InitStruct.Pin = Disp_backlight_Pin|Backlight_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pins : D5_Y_Pin D3_R_Pin D2_R_Pin DISP_CS_Pin 
                           SM2__Pin */
  GPIO_InitStruct.Pin = D5_Y_Pin|D3_R_Pin|D2_R_Pin|DISP_CS_Pin 
                          |SM2__Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : DISP_RST_Pin DISP_CD_Pin SM2B_Pin SM3A_Pin 
                           SM3B_Pin */
  GPIO_InitStruct.Pin = DISP_RST_Pin|DISP_CD_Pin|SM2B_Pin|SM3A_Pin 
                          |SM3B_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : SM1B__Pin SM2A__Pin D4_Y_Pin SM3A__Pin 
                           SM3B__Pin */
  GPIO_InitStruct.Pin = SM1B__Pin|SM2A__Pin|D4_Y_Pin|SM3A__Pin 
                          |SM3B__Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : SW1_Pin SW2_Pin */
  GPIO_InitStruct.Pin = SW1_Pin|SW2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pins : SM1A__Pin SM1A_Pin SM1B_Pin SM2A_Pin */
  GPIO_InitStruct.Pin = SM1A__Pin|SM1A_Pin|SM1B_Pin|SM2A_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : D1_R_Pin D6_G_Pin */
  GPIO_InitStruct.Pin = D1_R_Pin|D6_G_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /*Configure GPIO pins : OUT6_Pin OUT5_Pin OUT4_Pin OUT3_Pin 
                           OUT2_Pin OUT1_Pin */
  GPIO_InitStruct.Pin = OUT6_Pin|OUT5_Pin|OUT4_Pin|OUT3_Pin 
                          |OUT2_Pin|OUT1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void BaseTimer()
{
	baseTimer++;
	period_1 = 1;
	 if (baseTimer%10 == 3)		period_10 = 1;
	 if (baseTimer%100 == 33)	period_100 = 1;
	 if (baseTimer%1000 == 333)	period_1000 = 1;

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
