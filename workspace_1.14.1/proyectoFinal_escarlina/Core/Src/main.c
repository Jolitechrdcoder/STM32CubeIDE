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
#include"matriztarea1.h"
#include"lcdjorge.h"
#include <stdlib.h>
#include <time.h>
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
I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C1_Init(void);
void mostrar_pieza(uint8_t piezas[8], int fila_inicial, int columna_inicial);
void mover_pieza_abajo(const uint8_t *pieza, int fila, int columna);
void verificar_filas_llenas();
void reiniciar_juego();
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
// Define las formas de las piezas
uint8_t pieza_1[8] = { 0b00001111 };
uint8_t pieza_2[8] = { 0b00001000, 0b00001000, 0b00001110 };
uint8_t pieza_3[8] = { 0b00000100, 0b00000100, 0b00001110 };
uint8_t pieza_4[8] = { 0b00001100, 0b00001100 };
uint8_t pieza_5[8] = { 0b00001100, 0b00011000 };
uint8_t pieza_6[8] = { 0b00001100, 0b00011100 };
uint8_t pieza_7[8] = { 0b00011000, 0b00001100 };

// Matriz de juego
uint8_t matriz_tablero[8][8] = { 0 };

// Variables globales
int movimiento_piezas = 0;
int score = 0;
// Función para mostrar la pieza en la matriz de juego
void mostrar_pieza(uint8_t piezas[8], int fila_inicial, int columna_inicial) {
    for (int i = 0; i < 8; i++) {
        setrow(fila_inicial - i, piezas[i] << columna_inicial);
    }
}

// Función para mover la pieza hacia abajo en la matriz de juego
void mover_pieza_abajo(const uint8_t *pieza, int fila, int columna) {
    for (int i = 0; i < 8; i++) {
        matriz_tablero[fila][columna] |= pieza[i] << columna;
        fila--;
    }
}
int puntos =0;
int fila_llena = 0;
int filas_completas = 0;
// Función para verificar si alguna fila está completa y aumentar la puntuación
void verificar_filas_llenas() {


    for (int i = 0; i < 8; i++) {
        fila_llena = 1;
        for (int j = 0; j < 8; j++) {
            if (matriz_tablero[i][j] == 0) {
                fila_llena = 0;
                puntos++;
                break;
            }
        }
        if (fila_llena) {
            filas_completas++; // Incrementa el contador de filas completas
            // Borra la fila llena
            for (int j = 0; j < 8; j++) {
                matriz_tablero[i][j] = 0;
            }
        }
    }

    if (filas_completas > 0) {

        reiniciar_juego(); // Reinicia el juego si alguna fila está llena
    }
}


// Función para reiniciar el juego
void reiniciar_juego() {
    // Muestra "Game Over" en el LCD

    lcd_enviar("Game Over         ", 0, 0);
    max_clear();
    write_char('X', 1);
    HAL_Delay(500);

    write_char(' ', 1);
    HAL_Delay(500);

    write_char('X', 1);
    HAL_Delay(500);

    write_char(' ', 1);
   HAL_Delay(500);
    max_clear();
    HAL_Delay(2000);
    lcd_clear();

    // Reinicia la matriz
    for (int k = 0; k < 8; k++) {
        for (int l = 0; l < 8; l++) {
            matriz_tablero[k][l] = 0;
        }
    }
    // Reinicia la puntuación
    puntos = 0;
    // Muestra la puntuación reiniciada en el LCD
    lcd_enviar("Tetris Escarlina", 0, 0);
    lcd_enviar("Score:", 1, 0);
    lcd_enviar_int(puntos, 1, 7);
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	srand((unsigned int)time(NULL));



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
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */
   max_init(0x02);

  lcd_init();
  lcd_enviar("Tetris Escarlina", 0, 0);
  lcd_enviar("Score:", 1, 0);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1) {
      // Generación aleatoria de piezas, fila inicial y columna inicial
      int piezas_ramdom = rand() % 7;
      const uint8_t *piezas[] = { pieza_1, pieza_2, pieza_3, pieza_4, pieza_5, pieza_6, pieza_7 };
      const uint8_t *sel_piezas = piezas[piezas_ramdom];
      int fila_inicial = 0;
      int columna_inicial = rand() % 6;

      while (fila_inicial < 8) {
          // Muestra la pieza en la posición actual del tablero
          mostrar_pieza(sel_piezas, fila_inicial, columna_inicial);
          HAL_Delay(800);

          // Verifica si la pieza ha alcanzado el fondo o una posición ocupada en el tablero
          if (fila_inicial == 7 || matriz_tablero[fila_inicial + 1][columna_inicial] != 0) {
              // Agrega la pieza al tablero en la fila actual
              mover_pieza_abajo(sel_piezas, fila_inicial, columna_inicial);

              // Verifica si alguna fila está completa y aumenta la puntuación en consecuencia
              verificar_filas_llenas();

              // Muestra la puntuación actualizada en el LCD
              lcd_enviar_int(puntos, 1, 7);

              // Verifica si el juego debe reiniciarse
              if (matriz_tablero[0][columna_inicial] != 0) {
                  reiniciar_juego();
              }

              break; // Sale del bucle
          } else {
              fila_inicial++; // Mueve la pieza hacia abajo
          }

          // Verifica si se ha presionado un botón para mover la pieza hacia la derecha
          if (movimiento_piezas > 0) {
              columna_inicial++;
              movimiento_piezas = 0; // Reinicia el movimiento
          }
          // Verifica si se ha presionado un botón para mover la pieza hacia la izquierda
          else if (movimiento_piezas < 0) {
              columna_inicial--;
              movimiento_piezas = 0; // Reinicia el movimiento
          }
      }

      HAL_Delay(500); // Retardo entre piezas
  }
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_I2C1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_HSI;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
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
  hi2c1.Init.Timing = 0x2000090E;
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
  huart2.Init.BaudRate = 38400;
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
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_8, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : btn1_Pin btn2_Pin */
  GPIO_InitStruct.Pin = btn1_Pin|btn2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PC5 PC6 PC8 */
  GPIO_InitStruct.Pin = GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : LED_Pin */
  GPIO_InitStruct.Pin = LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if (GPIO_Pin == btn1_Pin){
		movimiento_piezas = 1;

	}
	if (GPIO_Pin == btn2_Pin){
		movimiento_piezas = -1;

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
