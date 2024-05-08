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
#include <string.h>
#include <math.h>
#include "fonts.h"
#include "ssd1306.h"
#include "stm32l0xx_hal.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define BUTTON GPIO_PIN_13
#define BUTTON_GPIO_Port GPIOC

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc;
DMA_HandleTypeDef hdma_adc;

I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim6;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
//#define Samples 100
int Samples = 10;

volatile uint8_t uart_data_ready = 0;

uint16_t adc_values[3]; //Dvieju LDR ADC vertės ir Mikrafono ADC
float lux1, lux2; 		// Kiekvieno kanalo ver ?ių kintamieji
float min_lux = 100000; // Initialize with maximum float value
float max_lux = -100000; // Initialize with minimum float value
float vid = 0.0; 		// Initialize range variable
float RMS; 				// Mikrofono signalas
char msg[100];			// Kompiuteriui isvedama žinutė
float dB;

uint32_t lux_reset_time = 0; // Timer to reset min/max values periodically
uint32_t lux_reset_interval = 120; // Reset interval in milliseconds

uint32_t up_time2 = 0; // Kintamasis OLED LUX atnaujimui 2s
uint32_t up_time1 = 0;  // Kintamasis OLED RMS atnaujinimui 1s


uint8_t flag1,flag2; //flags oled display update time
uint8_t display_mode = 0; // atvaizduoti LUX ir RMS paspaudus mygtuka
uint32_t last_button_press_time = 0; // Variable to track the time of the last button press

float convertToLux(uint16_t adc_value); //Konvertuoti 0-1 kanalo ADC vertę foto varzai 1
float convertRMS(uint16_t adc_value, int Sample); // Kovertuoti 2 kanalo ADC mikrafono verte i RMS


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM6_Init(void);
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
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC_Init();
  MX_USART2_UART_Init();
  MX_I2C1_Init();
  MX_TIM6_Init();
  /* USER CODE BEGIN 2 */

  // Start timer with interrupt
  HAL_TIM_Base_Start_IT(&htim6);

  // OLED initialization
  SSD1306_Init();
  // Clear OLED
  SSD1306_Fill(SSD1306_COLOR_BLACK);

  // Display some text on the screen
  //SSD1306_GotoXY(0,20);
  //SSD1306_Puts("Hello, OLED!", &Font_11x18, SSD1306_COLOR_WHITE);
  //SSD1306_UpdateScreen();


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  // Start ADC nuskaitymus
	  HAL_ADC_Start_DMA(&hadc, (uint32_t *)adc_values, 3);

      // ADC verciu konvertavimas
      lux1 = convertToLux(adc_values[0]);
      lux2 = convertToLux(adc_values[1]);
      RMS = convertRMS(adc_values[2], Samples);


      // Reset minimum and maximum lux values periodically
       if (HAL_GetTick() - lux_reset_time >= lux_reset_interval) {
           min_lux = 100000;
           max_lux = -100000;
           lux_reset_time = HAL_GetTick();
       }

      if (lux1 < min_lux)
          min_lux = lux1;
      else if (lux1 <= 1)
    	  min_lux = 1;

      if (lux1 > max_lux)
          max_lux = lux1;
      else if (lux1 > 5)
    	  max_lux = 5;

      if (lux2 < min_lux)
          min_lux = lux2;
      else if (lux2 <= 1)
    	  min_lux = 1;

      if (lux2 > max_lux)
          max_lux = lux2;
      else if (lux2 > 5)
          max_lux = 5;

          // Vidurkis
          vid = (lux1+lux2)/2;


	  // UART siuntimas i kompiuteri
      if (uart_data_ready)
          {
              sprintf(msg, "LDR ADC: %d, %d | Lux: %.2f, %.2f |MIN: %.2f, MAX: %.2f, Vidurkis: %.2f |\r\n"
            		  "MIC ADC: %d | RMS: %.2f\r\n",adc_values[0], adc_values[1],lux1,lux2,min_lux,max_lux,vid,adc_values[2],RMS);
    	  	  //sprintf(msg, "Oras ADC: %d | PPM: %.0f \r\nDregme ADC: %d | Dregme %: %.0f \r\n", adc_values[0], oras, adc_values[1], dregme);
              // UART transmission
              HAL_UART_Transmit(&huart2, (uint8_t *)msg, strlen(msg), HAL_MAX_DELAY);

              flag1++;
              flag2++;
              uart_data_ready = 0; // Reset flag

          }

      if (flag1>=4)
      	flag1=0;
      if (flag2>=8)
      	flag2=0;

	    // Mygtuko paspaudimas
	    ToggleDisplay();
	    //OLED LUX ir RMS su mygtuku
	    DisplayOLED();



	    //OLED
	   /* if (HAL_GetTick() - lux_update_time >= 2000)
	        {
	          lux_update_time = HAL_GetTick();

	          // Clear the OLED screen
	          SSD1306_Fill(SSD1306_COLOR_BLACK);

	          // Display LUX1 and LUX2 values on the OLED screen
	          char lux1_str[20];
	          char lux2_str[20];
	          snprintf(lux1_str, sizeof(lux1_str), "%.2f", lux1);
	          snprintf(lux2_str, sizeof(lux2_str), "%.2f", lux2);

	          // Display LUX1 and LUX2 values on the OLED screen
	          SSD1306_GotoXY(0, 0);
	          SSD1306_Puts("LUX1: ", &Font_11x18, SSD1306_COLOR_WHITE);
	          SSD1306_Puts(lux1_str, &Font_11x18, SSD1306_COLOR_WHITE);
	          SSD1306_GotoXY(0, 20);
	          SSD1306_Puts("LUX2: ", &Font_11x18, SSD1306_COLOR_WHITE);
	          SSD1306_Puts(lux2_str, &Font_11x18, SSD1306_COLOR_WHITE);

	          // Update the OLED screen
	          SSD1306_UpdateScreen();
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLLMUL_4;
  RCC_OscInitStruct.PLL.PLLDIV = RCC_PLLDIV_2;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2|RCC_PERIPHCLK_I2C1;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC_Init(void)
{

  /* USER CODE BEGIN ADC_Init 0 */

  /* USER CODE END ADC_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC_Init 1 */

  /* USER CODE END ADC_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc.Instance = ADC1;
  hadc.Init.OversamplingMode = DISABLE;
  hadc.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc.Init.Resolution = ADC_RESOLUTION_12B;
  hadc.Init.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  hadc.Init.ScanConvMode = ADC_SCAN_DIRECTION_FORWARD;
  hadc.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc.Init.ContinuousConvMode = ENABLE;
  hadc.Init.DiscontinuousConvMode = DISABLE;
  hadc.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc.Init.DMAContinuousRequests = DISABLE;
  hadc.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc.Init.LowPowerAutoWait = DISABLE;
  hadc.Init.LowPowerFrequencyMode = DISABLE;
  hadc.Init.LowPowerAutoPowerOff = DISABLE;
  if (HAL_ADC_Init(&hadc) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel to be converted.
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_RANK_CHANNEL_NUMBER;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel to be converted.
  */
  sConfig.Channel = ADC_CHANNEL_1;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel to be converted.
  */
  sConfig.Channel = ADC_CHANNEL_4;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC_Init 2 */

  /* USER CODE END ADC_Init 2 */

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
  hi2c1.Init.Timing = 0x00300F38;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

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
  htim6.Init.Prescaler = 40000;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 160;
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
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
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
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

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
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
// UART transmission complete callback
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim == &htim6)
    {
        uart_data_ready = 1; // Set flag to indicate data is ready to be sent
    }
}


void ToggleDisplay()
{
	static uint8_t button_state = GPIO_PIN_SET; // Kintamasis tikrina paskutinę būseną

	    // Skaityti dabartinę mygtuko būseną
	    uint8_t current_state = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13);

	    // Tikrina ar 'buton state' pasikeitė nuo atleidimo iki įspausto
	    if (button_state == GPIO_PIN_SET && current_state == GPIO_PIN_RESET &&
	        HAL_GetTick() - last_button_press_time >= 50) {
	        // Toggle display mode
	        display_mode = (display_mode == 0) ? 1 : 0;
	        //display_mode != display_mode;

	        // Naujinti paskutini paspaudimo laiką
	        last_button_press_time = HAL_GetTick();
	    }

	    // Atnaujinit 'buton state' kitai iteracijai
	    button_state = current_state;
}


void DisplayOLED(void) //OLED atvaizdavimas
{
	// LUX atvaizdavimas
    if (display_mode == 0 && HAL_GetTick() - up_time2 >= 2000)
    {

        SSD1306_Fill(SSD1306_COLOR_BLACK); // Clear OLED

        char lux_str[20];
        char min_str[20];
        char max_str[20];
        snprintf(lux_str, sizeof(lux_str), "%.2f", vid);
        snprintf(min_str, sizeof(min_str), "%.2f", min_lux);
        snprintf(max_str, sizeof(max_str), "%.2f", max_lux);

        SSD1306_GotoXY(0, 0);
        SSD1306_Puts("VID: ", &Font_11x18, SSD1306_COLOR_WHITE);
        SSD1306_Puts(lux_str, &Font_11x18, SSD1306_COLOR_WHITE);
        SSD1306_GotoXY(0, 20);
        SSD1306_Puts("MIN: ", &Font_11x18, SSD1306_COLOR_WHITE);
        SSD1306_Puts(min_str, &Font_11x18, SSD1306_COLOR_WHITE);
        SSD1306_GotoXY(0, 40);
        SSD1306_Puts("MAX: ", &Font_11x18, SSD1306_COLOR_WHITE);
        SSD1306_Puts(max_str, &Font_11x18, SSD1306_COLOR_WHITE);

        SSD1306_UpdateScreen();

        up_time2 = HAL_GetTick();

    }
    // RMS atvaizdavimas
    else if (display_mode == 1 && HAL_GetTick() - up_time1 >= 1000)
    {

        SSD1306_Fill(SSD1306_COLOR_BLACK); // Clear OLED

        char rms_str[20];
        snprintf(rms_str, sizeof(rms_str), "%.2f", RMS);

        SSD1306_GotoXY(0, 20);
        SSD1306_Puts("RMS: ", &Font_11x18, SSD1306_COLOR_WHITE);
        SSD1306_Puts(rms_str, &Font_11x18, SSD1306_COLOR_WHITE);

        SSD1306_UpdateScreen();

        up_time1 = HAL_GetTick();
    }
}

/*
void DisplayOLED(void) //OLED atvaizdavimas
{
    if (display_mode == 0 && flag2==0)
    {

    	//up_time2 = HAL_GetTick();

        SSD1306_Fill(SSD1306_COLOR_BLACK); // Clear OLED

        char lux_str[20];
        char min_str[20];
        char max_str[20];
        snprintf(lux_str, sizeof(lux_str), "%.2f", vid);
        snprintf(min_str, sizeof(min_str), "%.2f", min_lux);
        snprintf(max_str, sizeof(max_str), "%.2f", max_lux);

        SSD1306_GotoXY(0, 0);
        SSD1306_Puts("VID: ", &Font_11x18, SSD1306_COLOR_WHITE);
        SSD1306_Puts(lux_str, &Font_11x18, SSD1306_COLOR_WHITE);
        SSD1306_GotoXY(0, 20);
        SSD1306_Puts("MIN: ", &Font_11x18, SSD1306_COLOR_WHITE);
        SSD1306_Puts(min_str, &Font_11x18, SSD1306_COLOR_WHITE);
        SSD1306_GotoXY(0, 40);
        SSD1306_Puts("MAX: ", &Font_11x18, SSD1306_COLOR_WHITE);
        SSD1306_Puts(max_str, &Font_11x18, SSD1306_COLOR_WHITE);

        SSD1306_UpdateScreen();

    }
    else if (display_mode == 1 && flag1==0)
    {
        //up_time1 = HAL_GetTick();

        SSD1306_Fill(SSD1306_COLOR_BLACK); // Clear OLED

        char rms_str[20];
        snprintf(rms_str, sizeof(rms_str), "%.2f", RMS);

        SSD1306_GotoXY(0, 20);
        SSD1306_Puts("RMS: ", &Font_11x18, SSD1306_COLOR_WHITE);
        SSD1306_Puts(rms_str, &Font_11x18, SSD1306_COLOR_WHITE);

        SSD1306_UpdateScreen();
    }
}*/

/* Funkcija konvertuoti ADC vertes į
 * tikrąsias apšvietimo vertes */


float convertToLux(uint16_t adc_value) {

	float voltage = (adc_value / 4095.0) * 3.3; // Konvertuoti ASK vertę į įtampą, prie Vref-3v3, 12bit
	float r_ldr = (5000.0 * voltage) / (3.3 - voltage); // KOnvertuoti foto varzos vertę pagal įtampos daliklio formulę
	float r10 = 7500.0; // didžiausia foto varžos vertė  Resistance of LDR at 10 lux (from datasheet)
	float y = 0.6; // Gamma charakteristika Gamma characteristic (from datasheet)
	float lux_value = pow(10, y * log10(r_ldr / r10)); // Apskaiciuoja Lux vertę pagal gamma charakteristika (datasheet)
	return lux_value;

}


float convertRMS(uint16_t adc_value, int Samples)
{
	float sum_square = 0.0;

	 // Kvadratinamos vertes
	    for (int i = 0; i < Samples; i++) {
	        float voltage = adc_values[2] * 3.3 / 4096; // paverčiama į įtampą
	        float voltage_squared = voltage * voltage;
	        sum_square += voltage_squared;
	    }

	// Vidurkinam
	float mean_square = sum_square / Samples;

	return RMS= sqrt(mean_square);
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
