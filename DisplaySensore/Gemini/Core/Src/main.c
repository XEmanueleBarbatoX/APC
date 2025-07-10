/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "ssd1306/ssd1306.h"
#include "ssd1306/ssd1306_fonts.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define REG_TEMP		   0x00
#define REG_CONF           0x01
#define REG_THYST          0x02
#define REG_TOS            0x03
#define LM75A_TEMP_TO_REG(T) \
    ((uint16_t)((int16_t)roundf((T) * 2.0f)) << 7)

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef hlpuart1;
UART_HandleTypeDef huart5;
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
static const uint8_t LM75A = 0x48 << 1;


// Stati del sistema
typedef enum {
    STATE_NORMAL,
    STATE_WAIT_RESPONSE,
    STATE_RESPONSE_RECEIVED,
    STATE_POST_RESPONSE_WAIT
} SystemState_t;

volatile SystemState_t system_state = STATE_NORMAL;
volatile uint8_t rx_data_ready = 1;
uint8_t  rxBuf[8];
uint32_t postResponseTick;

static const char alarmMsg[] = "FRALRM\r\n";
static const uint8_t alarmLen = sizeof(alarmMsg) - 1;  // 13

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_UART5_Init(void);
static void MX_LPUART1_UART_Init(void);
/* USER CODE BEGIN PFP */


/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

// Routine per scrivere un byte
static HAL_StatusTypeDef LM75A_WriteByte(uint8_t reg, uint8_t data) {
    return HAL_I2C_Mem_Write(
        &hi2c1,
        LM75A,
        reg,
        I2C_MEMADD_SIZE_8BIT,
        &data,
        1,
        100  /* timeout ms */
    );
}

// Routine per scrivere due byte
static HAL_StatusTypeDef LM75A_WriteWord(uint8_t reg, uint16_t value) {
    uint8_t buf[2];
    buf[0] = (value >> 8) & 0xFF;  // MSB
    buf[1] = value & 0xFF;         // LSB (“don’t care” bits = 0)
    return HAL_I2C_Mem_Write(
        &hi2c1,
        LM75A,
        reg,
        I2C_MEMADD_SIZE_8BIT,
        buf,
        2,
        100
    );
}

// Routine di configurazione LM75A
void LM75A_Init(void) {
    HAL_StatusTypeDef ret;

    /* 1) Config register:
         *    OS_F_QUE = 10 (4 conversions) → B4=1, B3=0
         *    OS_POL   = 1 (active-high)   → B2=1
         *    OS_COMP_INT = 1 (interrupt)  → B1=1
         *    SHUTDOWN    = 0 (normal)     → B0=0
         *    byte = 0b0001_0110 = 0x16
    */

	ret = LM75A_WriteByte(REG_CONF, 0x16);
	if (ret != HAL_OK) { Error_Handler(); }

    /* 2) TOS = 35°C ⇒ 35/0.5 = 70 = 0x46 */
    /*    value register = 0x46 << 8 = 0x4600 */

	// Temperatura per prova: 32 °C / 0.5 °C = 64  → 0x40
	uint16_t temp_max = LM75A_TEMP_TO_REG(30.0f);
    ret = LM75A_WriteWord(REG_TOS, temp_max);
    if (ret != HAL_OK) { Error_Handler(); }

    /* 3) THYST = 30°C ⇒ 30/0.5 = 60 = 0x3C ⇒ 0x3C00 */
    uint16_t temp_min = LM75A_TEMP_TO_REG(28.0f);
    ret = LM75A_WriteWord(REG_THYST, temp_min);
    if (ret != HAL_OK) { Error_Handler(); }
}

/*
// Ridefinizione funzione di CallBack interrupt su EXTI0
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
    if (GPIO_Pin == GPIO_PIN_0) {
        // Togliere l’interrupt leggendo un registro qualunque (ad es. Temp)
    	char temp_s[10];
    	uint8_t msg[32];
        uint8_t temp_raw[2];
        int len;
        if (HAL_I2C_Mem_Read(&hi2c1, LM75A, 0x00, I2C_MEMADD_SIZE_8BIT, temp_raw, 2, 100) == HAL_OK) {
            int16_t raw = (temp_raw[0] << 8) | temp_raw[1];
            // float temp = raw * 0.0078125f;  // 1/128
            raw = raw >> 5;
            float temp = (float)raw * 0.125f;

            // gestisci l’allerta…
            len = snprintf((char*)msg, sizeof(msg), "Interruzione: %.2f C\r\n", temp);
            HAL_UART_Transmit(&huart2, msg, len, HAL_MAX_DELAY);
            // HAL_Delay(1000);

            sprintf(temp_s, "%.2f", temp);
            ssd1306_SetCursor(2, 32);
			ssd1306_WriteString(temp_s, Font_6x8, Black);
			ssd1306_UpdateScreen();

        }
    }
}*/

// Ridefinizione Callback di ricezione UART (interrupt)
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    // if (huart->Instance == USART1 && system_state == STATE_WAIT_RESPONSE) {
	// if (huart->Instance == USART1) {
	if (huart->Instance == USART1) {
        // terminatore stringa
        //rxBuf[8] = '\0';
        system_state = STATE_RESPONSE_RECEIVED;
        rx_data_ready = 1;

        // Preparazione USART prossima ricezione
        HAL_UART_Receive_IT(&huart1, rxBuf, 8);
    }
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
  uint8_t buf[12];
  HAL_StatusTypeDef ret, check;
  int16_t val;
  float temp_c;
  uint8_t msg[32];
  int len;
  uint8_t data_read[2];
  char temp_s[10];
  int cont = 0;
  float soglia = 30.0f;
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
  MX_I2C1_Init();
  MX_USART2_UART_Init();
  MX_USART1_UART_Init();
  MX_UART5_Init();
  MX_LPUART1_UART_Init();
  /* USER CODE BEGIN 2 */
  HAL_NVIC_SetPriority(USART1_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(USART1_IRQn);

  // Preparazione USART
  // __HAL_UART_ENABLE_IT(&huart1,UART_IT_RXNE);
  HAL_UART_Receive_IT(&huart1, rxBuf, 8);
  LM75A_Init();
  ssd1306_Init();

  /* USER CODE END 2 */

  /* Initialize led */
  BSP_LED_Init(LED_GREEN);

  /* Initialize USER push-button, will be used to trigger an interrupt each time it's pressed.*/
  BSP_PB_Init(BUTTON_USER, BUTTON_MODE_EXTI);

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  //HAL_UART_Transmit(&huart1, (uint8_t*)alarmMsg, 9, HAL_MAX_DELAY);

	  if (rx_data_ready == 1){

		  rx_data_ready = 0;

		  if (system_state == STATE_WAIT_RESPONSE){
			  system_state = STATE_RESPONSE_RECEIVED;
		  }
	  }

	  switch(system_state) {

	              case STATE_NORMAL: {
	                  // Modalità normale: leggo e stampo la temperatura
	                  uint8_t cmd = REG_TEMP;
	                  if (HAL_I2C_Master_Transmit(&hi2c1, LM75A, &cmd, 1, HAL_MAX_DELAY) == HAL_OK) {
	                      uint8_t data[2];
	                      if (HAL_I2C_Master_Receive(&hi2c1, LM75A, data, 2, HAL_MAX_DELAY) == HAL_OK) {
	                          int16_t raw = ((int16_t)data[0] << 8) | data[1];
	                          raw >>= 5;
	                          float temp = raw * 0.125f;
	                          char temp_s[16];
	                          snprintf(temp_s, sizeof(temp_s), "T: %.2f C", temp);

	                          ssd1306_Fill(Black);
	                          ssd1306_SetCursor(2, 2);
	                          ssd1306_WriteString("Temperatura corrente:", Font_6x8, White);
	                          ssd1306_SetCursor(2, 20);
	                          ssd1306_WriteString(temp_s, Font_7x10, Black);
	                          ssd1306_UpdateScreen();

	                          if (temp > soglia){
	                        	  cont ++;
	                        	  if (cont == 4){
	                        		  cont = 0;
	                        		  // Invio l'allarme e lo stampo
									  // const char alarmMsg[] = "ALLARME Fratm\r\n";



	                        		  HAL_UART_Transmit(&huart1, (uint8_t*)alarmMsg, 8, HAL_MAX_DELAY);

									  ssd1306_Fill(Black);
									  ssd1306_SetCursor(2, 20);
									  ssd1306_WriteString("ALLARME INCENDIO", Font_7x10, White);
									  ssd1306_UpdateScreen();

									  // passo a stato di attesa risposta
									  system_state = STATE_WAIT_RESPONSE;
	                        	  }
	                          }
	                          else{
	                        	  cont = 0;
	                          }
	                      }
	                  }
	                  HAL_Delay(100);
	                  break;
	              }

	              /*
	              case STATE_ALARM_SENT: {
	                  // Invio l'allarme e lo stampo
	                  // const char alarmMsg[] = "ALLARME INCENDIO\r\n";


	                  HAL_UART_Transmit(&huart1, (uint8_t*)alarmMsg, 9, HAL_MAX_DELAY);
	                  // HAL_UART_Receive_IT(&huart1, rxBuf, 9);


	                  ssd1306_Fill(Black);
	                  ssd1306_SetCursor(2, 20);
	                  ssd1306_WriteString("ALLARME INCENDIO", Font_7x10, White);
	                  ssd1306_UpdateScreen();

	                  // passo a stato di attesa risposta
	                  // system_state = STATE_WAIT_RESPONSE;
	                  break;
	              }
	  	  	  	  */


	              case STATE_WAIT_RESPONSE: {

	                  uint8_t cmd = REG_TEMP;
					  if (HAL_I2C_Master_Transmit(&hi2c1, LM75A, &cmd, 1, HAL_MAX_DELAY) == HAL_OK) {
						  uint8_t data[2];
						  if (HAL_I2C_Master_Receive(&hi2c1, LM75A, data, 2, HAL_MAX_DELAY) == HAL_OK) {
							  int16_t raw = ((int16_t)data[0] << 8) | data[1];
							  raw >>= 5;
							  float temp = raw * 0.125f;
							  char temp_s[16];
							  snprintf(temp_s, sizeof(temp_s), "T: %.2f C", temp);

							  ssd1306_SetCursor(40, 45);
							  ssd1306_WriteString(temp_s, Font_7x10, Black);
							  ssd1306_UpdateScreen();

						  }
					  }
					  HAL_Delay(100);
					  break;
	              }

	              case STATE_RESPONSE_RECEIVED: {
	                  // Appena arriva la risposta UART (gestita in RxCpltCallback)
	                  ssd1306_Fill(Black);
	                  ssd1306_SetCursor(2, 12);
	                  ssd1306_WriteString((char*)rxBuf, Font_6x8, Black);
	                  ssd1306_UpdateScreen();

	                  // inizio conteggio 5 secondi
	                  postResponseTick = HAL_GetTick();
	                  system_state = STATE_POST_RESPONSE_WAIT;
	                  break;
	              }

	              case STATE_POST_RESPONSE_WAIT: {
	                  // Aspetto 5 secondi
	                  if ((HAL_GetTick() - postResponseTick) >= 5000) {
	                      system_state = STATE_NORMAL;
	                  }
	                  break;
	              }

	              default: {
	            	  break;
	              }

	          }

/*
	  ssd1306_SetCursor(2, 2);
	  ssd1306_WriteString("Temperatura corrente:", Font_6x8, White);
	  ssd1306_UpdateScreen();

	  // ssd1306_Fill(White);
	  // ssd1306_UpdateScreen();

	  buf[0] = REG_TEMP;
	  check = HAL_I2C_IsDeviceReady(&hi2c1, LM75A, 5, HAL_MAX_DELAY);
	  ret = HAL_I2C_Master_Transmit(&hi2c1, LM75A, buf, 1, HAL_MAX_DELAY);
	  if( ret != HAL_OK){
		 strcpy((char*)buf, "Error Tx\r\n");
		} else{
			ret = HAL_I2C_Master_Receive(&hi2c1, LM75A, data_read, 2, HAL_MAX_DELAY); // <--- AGGIUNTA FONDAMENTALE QUI!

			if (ret != HAL_OK) {
				strcpy((char*)buf, "Error Rx\r\n"); // Gestione errore di ricezione
			} else {
				// I dati di temperatura sono in data_read[0] (MSB) e data_read[1] (LSB)
				val = (int16_t)((data_read[0] << 8) | data_read[1]);

				// Shift a destra di 5 per ottenere i 11 bit significativi e l'estensione del segno
				val = val >> 5;

				// Calcolo della temperatura (1 LSB = 0.125°C)
				temp_c = (float)val * 0.125f;

				// Ora 'temp_c' contiene la temperatura corretta.
				// Formatta la stringa per la UART
				len = snprintf((char*)msg, sizeof(msg), "Temp: %.2f C\r\n", temp_c);
				HAL_UART_Transmit(&huart2, msg, len, HAL_MAX_DELAY);
				HAL_Delay(100);

				sprintf(temp_s, "%.2f", temp_c);
				ssd1306_SetCursor(2, 30);
				ssd1306_WriteString(temp_s, Font_7x10, Black);
				ssd1306_UpdateScreen();

				if (temp_c > soglia) {
					cont++;
					if (cont == 4){
						len = snprintf((char*)msg, sizeof(msg), "INTERRUZIONE: %.2f C\r\n", temp_c);
						HAL_UART_Transmit(&huart2, msg, len, HAL_MAX_DELAY);
						cont = 0;

						ssd1306_Fill(Black);
						ssd1306_UpdateScreen();
						ssd1306_SetCursor(2, 20);
						ssd1306_WriteString("ALLARME INCENDIO", Font_7x10, White);
						ssd1306_UpdateScreen();
						HAL_Delay(10000);
						ssd1306_Fill(Black);
						ssd1306_UpdateScreen();
					}
				}
				else{
					cont = 0;
				}

			}
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
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV4;
  RCC_OscInitStruct.PLL.PLLN = 85;
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
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
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
  hi2c1.Init.Timing = 0x40B285C2;
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
  hlpuart1.Init.BaudRate = 209700;
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
  * @brief UART5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART5_Init(void)
{

  /* USER CODE BEGIN UART5_Init 0 */

  /* USER CODE END UART5_Init 0 */

  /* USER CODE BEGIN UART5_Init 1 */

  /* USER CODE END UART5_Init 1 */
  huart5.Instance = UART5;
  huart5.Init.BaudRate = 115200;
  huart5.Init.WordLength = UART_WORDLENGTH_8B;
  huart5.Init.StopBits = UART_STOPBITS_1;
  huart5.Init.Parity = UART_PARITY_NONE;
  huart5.Init.Mode = UART_MODE_TX_RX;
  huart5.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart5.Init.OverSampling = UART_OVERSAMPLING_16;
  huart5.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart5.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart5.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart5) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart5, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart5, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart5) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART5_Init 2 */

  /* USER CODE END UART5_Init 2 */

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
  huart2.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart2, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart2, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

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
#ifdef USE_FULL_ASSERT
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
