/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2019 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

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
UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
uint8_t command[2];
uint8_t rece[1];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */
void work(uint8_t* command){
  switch (command[0]) {
    case '1':
      switch (command[1]) {
        case 1:
          HAL_GPIO_WritePin(LED_A_1_GPIO_Port,LED_A_1_Pin,GPIO_PIN_SET);
          HAL_GPIO_WritePin(LED_A_2_GPIO_Port,LED_A_2_Pin,GPIO_PIN_RESET);
          HAL_GPIO_WritePin(LED_A_3_GPIO_Port,LED_A_3_Pin,GPIO_PIN_RESET);
          HAL_GPIO_WritePin(LED_A_4_GPIO_Port,LED_A_4_Pin,GPIO_PIN_RESET);
          HAL_GPIO_WritePin(LED_A_5_GPIO_Port,LED_A_5_Pin,GPIO_PIN_RESET);
          HAL_GPIO_WritePin(LED_A_6_GPIO_Port,LED_A_6_Pin,GPIO_PIN_RESET);
          break;

        case 2:
          HAL_GPIO_WritePin(LED_A_1_GPIO_Port,LED_A_1_Pin,GPIO_PIN_SET);
          HAL_GPIO_WritePin(LED_A_2_GPIO_Port,LED_A_2_Pin,GPIO_PIN_SET);
          HAL_GPIO_WritePin(LED_A_3_GPIO_Port,LED_A_3_Pin,GPIO_PIN_RESET);
          HAL_GPIO_WritePin(LED_A_4_GPIO_Port,LED_A_4_Pin,GPIO_PIN_RESET);
          HAL_GPIO_WritePin(LED_A_5_GPIO_Port,LED_A_5_Pin,GPIO_PIN_RESET);
          HAL_GPIO_WritePin(LED_A_6_GPIO_Port,LED_A_6_Pin,GPIO_PIN_RESET);
          break;

        case 3:
          HAL_GPIO_WritePin(LED_A_1_GPIO_Port,LED_A_1_Pin,GPIO_PIN_SET);
          HAL_GPIO_WritePin(LED_A_2_GPIO_Port,LED_A_2_Pin,GPIO_PIN_SET);
          HAL_GPIO_WritePin(LED_A_3_GPIO_Port,LED_A_3_Pin,GPIO_PIN_SET);
          HAL_GPIO_WritePin(LED_A_4_GPIO_Port,LED_A_4_Pin,GPIO_PIN_RESET);
          HAL_GPIO_WritePin(LED_A_5_GPIO_Port,LED_A_5_Pin,GPIO_PIN_RESET);
          HAL_GPIO_WritePin(LED_A_6_GPIO_Port,LED_A_6_Pin,GPIO_PIN_RESET);
          break;

        case 4:
          HAL_GPIO_WritePin(LED_A_1_GPIO_Port,LED_A_1_Pin,GPIO_PIN_SET);
          HAL_GPIO_WritePin(LED_A_2_GPIO_Port,LED_A_2_Pin,GPIO_PIN_SET);
          HAL_GPIO_WritePin(LED_A_3_GPIO_Port,LED_A_3_Pin,GPIO_PIN_SET);
          HAL_GPIO_WritePin(LED_A_4_GPIO_Port,LED_A_4_Pin,GPIO_PIN_SET);
          HAL_GPIO_WritePin(LED_A_5_GPIO_Port,LED_A_5_Pin,GPIO_PIN_RESET);
          HAL_GPIO_WritePin(LED_A_6_GPIO_Port,LED_A_6_Pin,GPIO_PIN_RESET);
          break;

        case 5:
          HAL_GPIO_WritePin(LED_A_1_GPIO_Port,LED_A_1_Pin,GPIO_PIN_SET);
          HAL_GPIO_WritePin(LED_A_2_GPIO_Port,LED_A_2_Pin,GPIO_PIN_SET);
          HAL_GPIO_WritePin(LED_A_3_GPIO_Port,LED_A_3_Pin,GPIO_PIN_SET);
          HAL_GPIO_WritePin(LED_A_4_GPIO_Port,LED_A_4_Pin,GPIO_PIN_SET);
          HAL_GPIO_WritePin(LED_A_5_GPIO_Port,LED_A_5_Pin,GPIO_PIN_SET);
          HAL_GPIO_WritePin(LED_A_6_GPIO_Port,LED_A_6_Pin,GPIO_PIN_RESET);
          break;

        case 6:
          HAL_GPIO_WritePin(LED_A_1_GPIO_Port,LED_A_1_Pin,GPIO_PIN_SET);
          HAL_GPIO_WritePin(LED_A_2_GPIO_Port,LED_A_2_Pin,GPIO_PIN_SET);
          HAL_GPIO_WritePin(LED_A_3_GPIO_Port,LED_A_3_Pin,GPIO_PIN_SET);
          HAL_GPIO_WritePin(LED_A_4_GPIO_Port,LED_A_4_Pin,GPIO_PIN_SET);
          HAL_GPIO_WritePin(LED_A_5_GPIO_Port,LED_A_5_Pin,GPIO_PIN_SET);
          HAL_GPIO_WritePin(LED_A_6_GPIO_Port,LED_A_6_Pin,GPIO_PIN_SET);
          break;

        default:
          HAL_GPIO_WritePin(LED_A_1_GPIO_Port,LED_A_1_Pin,GPIO_PIN_RESET);
          HAL_GPIO_WritePin(LED_A_2_GPIO_Port,LED_A_2_Pin,GPIO_PIN_RESET);
          HAL_GPIO_WritePin(LED_A_3_GPIO_Port,LED_A_3_Pin,GPIO_PIN_RESET);
          HAL_GPIO_WritePin(LED_A_4_GPIO_Port,LED_A_4_Pin,GPIO_PIN_RESET);
          HAL_GPIO_WritePin(LED_A_5_GPIO_Port,LED_A_5_Pin,GPIO_PIN_RESET);
          HAL_GPIO_WritePin(LED_A_6_GPIO_Port,LED_A_6_Pin,GPIO_PIN_RESET);
          break;
      }
      break;

    case '2':
      switch (command[1]) {
        case 1:
          HAL_GPIO_WritePin(LED_B_1_GPIO_Port,LED_B_1_Pin,GPIO_PIN_SET);
          HAL_GPIO_WritePin(LED_B_2_GPIO_Port,LED_B_2_Pin,GPIO_PIN_RESET);
          HAL_GPIO_WritePin(LED_B_3_GPIO_Port,LED_B_3_Pin,GPIO_PIN_RESET);
          HAL_GPIO_WritePin(LED_B_4_GPIO_Port,LED_B_4_Pin,GPIO_PIN_RESET);
          HAL_GPIO_WritePin(LED_B_5_GPIO_Port,LED_B_5_Pin,GPIO_PIN_RESET);
          HAL_GPIO_WritePin(LED_B_6_GPIO_Port,LED_B_6_Pin,GPIO_PIN_RESET);
          break;

        case 2:
          HAL_GPIO_WritePin(LED_B_1_GPIO_Port,LED_B_1_Pin,GPIO_PIN_SET);
          HAL_GPIO_WritePin(LED_B_2_GPIO_Port,LED_B_2_Pin,GPIO_PIN_SET);
          HAL_GPIO_WritePin(LED_B_3_GPIO_Port,LED_B_3_Pin,GPIO_PIN_RESET);
          HAL_GPIO_WritePin(LED_B_4_GPIO_Port,LED_B_4_Pin,GPIO_PIN_RESET);
          HAL_GPIO_WritePin(LED_B_5_GPIO_Port,LED_B_5_Pin,GPIO_PIN_RESET);
          HAL_GPIO_WritePin(LED_B_6_GPIO_Port,LED_B_6_Pin,GPIO_PIN_RESET);
          break;

        case 3:
          HAL_GPIO_WritePin(LED_B_1_GPIO_Port,LED_B_1_Pin,GPIO_PIN_SET);
          HAL_GPIO_WritePin(LED_B_2_GPIO_Port,LED_B_2_Pin,GPIO_PIN_SET);
          HAL_GPIO_WritePin(LED_B_3_GPIO_Port,LED_B_3_Pin,GPIO_PIN_SET);
          HAL_GPIO_WritePin(LED_B_4_GPIO_Port,LED_B_4_Pin,GPIO_PIN_RESET);
          HAL_GPIO_WritePin(LED_B_5_GPIO_Port,LED_B_5_Pin,GPIO_PIN_RESET);
          HAL_GPIO_WritePin(LED_B_6_GPIO_Port,LED_B_6_Pin,GPIO_PIN_RESET);
          break;

        case 4:
          HAL_GPIO_WritePin(LED_B_1_GPIO_Port,LED_B_1_Pin,GPIO_PIN_SET);
          HAL_GPIO_WritePin(LED_B_2_GPIO_Port,LED_B_2_Pin,GPIO_PIN_SET);
          HAL_GPIO_WritePin(LED_B_3_GPIO_Port,LED_B_3_Pin,GPIO_PIN_SET);
          HAL_GPIO_WritePin(LED_B_4_GPIO_Port,LED_B_4_Pin,GPIO_PIN_SET);
          HAL_GPIO_WritePin(LED_B_5_GPIO_Port,LED_B_5_Pin,GPIO_PIN_RESET);
          HAL_GPIO_WritePin(LED_B_6_GPIO_Port,LED_B_6_Pin,GPIO_PIN_RESET);
          break;

        case 5:
          HAL_GPIO_WritePin(LED_B_1_GPIO_Port,LED_B_1_Pin,GPIO_PIN_SET);
          HAL_GPIO_WritePin(LED_B_2_GPIO_Port,LED_B_2_Pin,GPIO_PIN_SET);
          HAL_GPIO_WritePin(LED_B_3_GPIO_Port,LED_B_3_Pin,GPIO_PIN_SET);
          HAL_GPIO_WritePin(LED_B_4_GPIO_Port,LED_B_4_Pin,GPIO_PIN_SET);
          HAL_GPIO_WritePin(LED_B_5_GPIO_Port,LED_B_5_Pin,GPIO_PIN_SET);
          HAL_GPIO_WritePin(LED_B_6_GPIO_Port,LED_B_6_Pin,GPIO_PIN_RESET);
          break;

        case 6:
          HAL_GPIO_WritePin(LED_B_1_GPIO_Port,LED_B_1_Pin,GPIO_PIN_SET);
          HAL_GPIO_WritePin(LED_B_2_GPIO_Port,LED_B_2_Pin,GPIO_PIN_SET);
          HAL_GPIO_WritePin(LED_B_3_GPIO_Port,LED_B_3_Pin,GPIO_PIN_SET);
          HAL_GPIO_WritePin(LED_B_4_GPIO_Port,LED_B_4_Pin,GPIO_PIN_SET);
          HAL_GPIO_WritePin(LED_B_5_GPIO_Port,LED_B_5_Pin,GPIO_PIN_SET);
          HAL_GPIO_WritePin(LED_B_6_GPIO_Port,LED_B_6_Pin,GPIO_PIN_SET);
          break;

        default:
          HAL_GPIO_WritePin(LED_B_1_GPIO_Port,LED_B_1_Pin,GPIO_PIN_RESET);
          HAL_GPIO_WritePin(LED_B_2_GPIO_Port,LED_B_2_Pin,GPIO_PIN_RESET);
          HAL_GPIO_WritePin(LED_B_3_GPIO_Port,LED_B_3_Pin,GPIO_PIN_RESET);
          HAL_GPIO_WritePin(LED_B_4_GPIO_Port,LED_B_4_Pin,GPIO_PIN_RESET);
          HAL_GPIO_WritePin(LED_B_5_GPIO_Port,LED_B_5_Pin,GPIO_PIN_RESET);
          HAL_GPIO_WritePin(LED_B_6_GPIO_Port,LED_B_6_Pin,GPIO_PIN_RESET);
          break;
      }
      break;

    case 'M':
      READ_BIT(command[1],1)?HAL_GPIO_TogglePin(Mag_1_GPIO_Port,Mag_1_Pin):HAL_GPIO_WritePin(Mag_1_GPIO_Port,Mag_1_Pin,GPIO_PIN_RESET);
      break;
  }
}
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
  switch (rece[0]) {
    case '1':
    case '2':
    case 'M':
      command[0] = rece[0];
      break;

    default:
      command[1] = rece[0];
      work(command);
      break;
  }
  HAL_UART_Receive_IT(&huart1,rece,1);
}
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
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  HAL_UART_Receive_IT(&huart1,rece,1);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    HAL_GPIO_TogglePin(LED_work_GPIO_Port,LED_work_Pin);
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

  /**Initializes the CPU, AHB and APB busses clocks
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /**Initializes the CPU, AHB and APB busses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
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
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LED_A_6_Pin|LED_A_5_Pin|LED_A_4_Pin|LED_B_6_Pin
                          |LED_B_5_Pin|LED_B_4_Pin|LED_A_1_Pin|LED_B_3_Pin
                          |LED_B_2_Pin|LED_B_1_Pin|Mag_1_Pin|LED_work_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LED_A_3_Pin|LED_A_2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : LED_A_6_Pin LED_A_5_Pin LED_A_4_Pin LED_B_6_Pin
                           LED_B_5_Pin LED_B_4_Pin LED_A_1_Pin LED_B_3_Pin
                           LED_B_2_Pin LED_B_1_Pin Mag_1_Pin LED_work_Pin */
  GPIO_InitStruct.Pin = LED_A_6_Pin|LED_A_5_Pin|LED_A_4_Pin|LED_B_6_Pin
                          |LED_B_5_Pin|LED_B_4_Pin|LED_A_1_Pin|LED_B_3_Pin
                          |LED_B_2_Pin|LED_B_1_Pin|Mag_1_Pin|LED_work_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : LED_A_3_Pin LED_A_2_Pin */
  GPIO_InitStruct.Pin = LED_A_3_Pin|LED_A_2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

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
void assert_failed(char *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
