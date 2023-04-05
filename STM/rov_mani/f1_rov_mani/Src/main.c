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
#include "register_map.h"
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

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */
uint8_t rece[1];
uint8_t rece_i2c[1];
uint8_t command[2];
uint8_t registers[register_count];
uint8_t operate[2];

TIM_OC_InitTypeDef m1,m2,m3,m4;

uint8_t addressed = 0;
uint8_t addressed_i2c = 0;
uint8_t i2c_check_flag = 0;
uint8_t command_flag = 0;
uint8_t led_light_1;
uint8_t led_light_2;
uint8_t send[2];
uint32_t n_pulse;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM1_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM3_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void operate_handle(uint8_t* todo){
	switch (todo[0]) {
		case '1':
			switch (todo[1]) {
				case 'F':
					n_pulse = registers[motor_1_speed - 0x80] * motor_pulse_max;
					n_pulse /= 100;
					htim1.Instance->CCR1 = n_pulse;
					HAL_GPIO_WritePin(Motor_1_B_GPIO_Port,Motor_1_B_Pin,GPIO_PIN_SET);
					HAL_GPIO_WritePin(Motor_1_A_GPIO_Port,Motor_1_A_Pin,GPIO_PIN_RESET);
					HAL_GPIO_WritePin(Motor_EN_A_GPIO_Port,Motor_EN_A_Pin,GPIO_PIN_SET);
					HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_1);
					break;

				case 'B':
					n_pulse = registers[motor_1_speed - 0x80] * motor_pulse_max;
					n_pulse /= 100;
					htim1.Instance->CCR1 = n_pulse;
					HAL_GPIO_WritePin(Motor_1_B_GPIO_Port,Motor_1_B_Pin,GPIO_PIN_RESET);
					HAL_GPIO_WritePin(Motor_1_A_GPIO_Port,Motor_1_A_Pin,GPIO_PIN_SET);
					HAL_GPIO_WritePin(Motor_EN_A_GPIO_Port,Motor_EN_A_Pin,GPIO_PIN_SET);
					HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_1);
					break;

				case 'S':
					HAL_GPIO_WritePin(Motor_EN_A_GPIO_Port,Motor_EN_A_Pin,GPIO_PIN_RESET);
					break;
			}
			break;

		case '2':
			switch (todo[1]) {
				case 'F':
					n_pulse = registers[motor_2_speed - 0x80] * motor_pulse_max;
					n_pulse /= 100;
					htim1.Instance->CCR2 = n_pulse;
					HAL_GPIO_WritePin(Motor_2_A_GPIO_Port,Motor_2_A_Pin,GPIO_PIN_SET);
					HAL_GPIO_WritePin(Motor_2_B_GPIO_Port,Motor_2_B_Pin,GPIO_PIN_RESET);
					HAL_GPIO_WritePin(Motor_EN_A_GPIO_Port,Motor_EN_A_Pin,GPIO_PIN_SET);
					HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_2);
					break;

				case 'B':
					n_pulse = registers[motor_2_speed - 0x80] * motor_pulse_max;
					n_pulse /= 100;
					htim1.Instance->CCR2 = n_pulse;
					HAL_GPIO_WritePin(Motor_2_A_GPIO_Port,Motor_2_A_Pin,GPIO_PIN_RESET);
					HAL_GPIO_WritePin(Motor_2_B_GPIO_Port,Motor_2_B_Pin,GPIO_PIN_SET);
					HAL_GPIO_WritePin(Motor_EN_A_GPIO_Port,Motor_EN_A_Pin,GPIO_PIN_SET);
					HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_2);
					break;

				case 'S':
					HAL_GPIO_WritePin(Motor_EN_A_GPIO_Port,Motor_EN_A_Pin,GPIO_PIN_RESET);
					break;
			}
			break;

		case '3':
			switch (todo[1]) {
				case 'F':
					n_pulse = registers[motor_3_speed - 0x80] * motor_pulse_max;
					n_pulse /= 100;
					htim1.Instance->CCR3 = n_pulse;
					HAL_GPIO_WritePin(Motor_3_B_GPIO_Port,Motor_3_B_Pin,GPIO_PIN_SET);
					HAL_GPIO_WritePin(Motor_3_A_GPIO_Port,Motor_3_A_Pin,GPIO_PIN_RESET);
					HAL_GPIO_WritePin(Motor_EN_B_GPIO_Port,Motor_EN_B_Pin,GPIO_PIN_SET);
					HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_3);
					break;

				case 'B':
					n_pulse = registers[motor_3_speed - 0x80] * motor_pulse_max;
					n_pulse /= 100;
					htim1.Instance->CCR3 = n_pulse;
					HAL_GPIO_WritePin(Motor_3_B_GPIO_Port,Motor_3_B_Pin,GPIO_PIN_RESET);
					HAL_GPIO_WritePin(Motor_3_A_GPIO_Port,Motor_3_A_Pin,GPIO_PIN_SET);
					HAL_GPIO_WritePin(Motor_EN_B_GPIO_Port,Motor_EN_B_Pin,GPIO_PIN_SET);
					HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_3);
					break;

				case 'S':
					HAL_GPIO_WritePin(Motor_EN_B_GPIO_Port,Motor_EN_B_Pin,GPIO_PIN_RESET);
					break;
			}
			break;

		case '4':
			switch (todo[1]) {
				case 'F':
					n_pulse = registers[motor_4_speed - 0x80] * motor_pulse_max;
					n_pulse /= 100;
					htim1.Instance->CCR4 = n_pulse;
					HAL_GPIO_WritePin(Motor_4_A_GPIO_Port,Motor_4_A_Pin,GPIO_PIN_SET);
					HAL_GPIO_WritePin(Motor_4_B_GPIO_Port,Motor_4_B_Pin,GPIO_PIN_RESET);
					HAL_GPIO_WritePin(Motor_EN_B_GPIO_Port,Motor_EN_B_Pin,GPIO_PIN_SET);
					HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_4);
					break;

				case 'B':
					n_pulse = registers[motor_4_speed - 0x80] * motor_pulse_max;
					n_pulse /= 100;
					htim1.Instance->CCR4 = n_pulse;
					HAL_GPIO_WritePin(Motor_4_A_GPIO_Port,Motor_4_A_Pin,GPIO_PIN_RESET);
					HAL_GPIO_WritePin(Motor_4_B_GPIO_Port,Motor_4_B_Pin,GPIO_PIN_SET);
					HAL_GPIO_WritePin(Motor_EN_B_GPIO_Port,Motor_EN_B_Pin,GPIO_PIN_SET);
					HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_4);
					break;

				case 'S':
					HAL_GPIO_WritePin(Motor_EN_B_GPIO_Port,Motor_EN_B_Pin,GPIO_PIN_RESET);
					break;
			}
			break;

		case 'H':
			switch (todo[1]) {
				case '0':
					send[0] = '1';
					send[1] = 0;
					HAL_UART_Transmit(&huart3,send,2,10);
					break;

				case '1':
					led_light_1 = registers[led_light - 0x80] & 0x07;
					send[0] = '1';
					send[1] = led_light_1;
					HAL_UART_Transmit(&huart3,send,2,10);
					break;
				
				case 'B':
					led_light_1 = registers[led_light - 0x80] & 0x07;
					send[0] = '1';
					if (led_light_1 != 6){
						registers[led_light - 0x80] += 1;
						led_light_1 += 1;
					}
					send[1] = led_light_1;
					HAL_UART_Transmit(&huart3,send,2,10);
					break;
					
				case 'D':
					led_light_1 = registers[led_light - 0x80] & 0x07;
					send[0] = '1';
					if (led_light_1 != 0){
						registers[led_light - 0x80] -= 1;
						led_light_1 -= 1;
					}
					send[1] = led_light_1;
					HAL_UART_Transmit(&huart3,send,2,10);
					break;			
			}
			break;

		case 'B':
			switch (todo[1]) {
				case '0':
					send[0] = '2';
					send[1] = 0;
					HAL_UART_Transmit(&huart3,send,2,10);
					break;

				case '1':
					led_light_2 = registers[led_light - 0x80] >> 3;
					led_light_2 &= 0x07;
					send[0] = '2';
					send[1] = led_light_2;
					HAL_UART_Transmit(&huart3,send,2,10);
					break;
				
				case 'B':
					led_light_2 = registers[led_light - 0x80] >> 3;
					led_light_2 &= 0x07;
					send[0] = '2';
					if (led_light_2 != 6){
						registers[led_light - 0x80] += 8;
						led_light_2 += 1;
					}
					send[1] = led_light_2;
					HAL_UART_Transmit(&huart3,send,2,10);
					break;
					
				case 'D':
					led_light_2 = registers[led_light - 0x80] >> 3;
					led_light_2 &= 0x07;
					send[0] = '2';
					if (led_light_2 != 0){
						registers[led_light - 0x80] -= 8;
						led_light_2 -= 1;
					}
					send[1] = led_light_2;
					HAL_UART_Transmit(&huart3,send,2,10);
					break;			

			}
			break;

		case 'F':
			switch (todo[1]) {
				case '0':
					READ_BIT(registers[mosfet_1_reg - 0x80],mos_0)?HAL_GPIO_TogglePin(MOS_0_GPIO_Port,MOS_0_Pin):HAL_GPIO_WritePin(MOS_0_GPIO_Port,MOS_0_Pin,GPIO_PIN_SET);
					break;

				case '1':
					READ_BIT(registers[mosfet_1_reg - 0x80],mos_1)?HAL_GPIO_TogglePin(MOS_1_GPIO_Port,MOS_1_Pin):HAL_GPIO_WritePin(MOS_1_GPIO_Port,MOS_1_Pin,GPIO_PIN_SET);
					break;

				case '2':
					READ_BIT(registers[mosfet_1_reg - 0x80],mos_2)?HAL_GPIO_TogglePin(MOS_2_GPIO_Port,MOS_2_Pin):HAL_GPIO_WritePin(MOS_2_GPIO_Port,MOS_2_Pin,GPIO_PIN_SET);
					break;

				case '3':
					READ_BIT(registers[mosfet_1_reg - 0x80],mos_3)?HAL_GPIO_TogglePin(MOS_3_GPIO_Port,MOS_3_Pin):HAL_GPIO_WritePin(MOS_3_GPIO_Port,MOS_3_Pin,GPIO_PIN_SET);
					break;

				case '4':
					READ_BIT(registers[mosfet_1_reg - 0x80],mos_4)?HAL_GPIO_TogglePin(MOS_4_GPIO_Port,MOS_4_Pin):HAL_GPIO_WritePin(MOS_4_GPIO_Port,MOS_4_Pin,GPIO_PIN_SET);
					break;

				case '5':
					READ_BIT(registers[mosfet_1_reg - 0x80],mos_5)?HAL_GPIO_TogglePin(MOS_5_GPIO_Port,MOS_5_Pin):HAL_GPIO_WritePin(MOS_5_GPIO_Port,MOS_5_Pin,GPIO_PIN_SET);
					break;

				case '6':
					READ_BIT(registers[mosfet_1_reg - 0x80],mos_6)?HAL_GPIO_TogglePin(MOS_6_GPIO_Port,MOS_6_Pin):HAL_GPIO_WritePin(MOS_6_GPIO_Port,MOS_6_Pin,GPIO_PIN_SET);
					break;

				case '7':
					READ_BIT(registers[mosfet_2_reg - 0x80],mos_7)?HAL_GPIO_TogglePin(MOS_7_GPIO_Port,MOS_7_Pin):HAL_GPIO_WritePin(MOS_7_GPIO_Port,MOS_7_Pin,GPIO_PIN_SET);
					break;

				case '8':
					READ_BIT(registers[mosfet_2_reg - 0x80],mos_8)?HAL_GPIO_TogglePin(MOS_8_GPIO_Port,MOS_8_Pin):HAL_GPIO_WritePin(MOS_8_GPIO_Port,MOS_8_Pin,GPIO_PIN_SET);
					break;

				case '9':
					READ_BIT(registers[mosfet_2_reg - 0x80],mos_9)?HAL_GPIO_TogglePin(MOS_9_GPIO_Port,MOS_9_Pin):HAL_GPIO_WritePin(MOS_9_GPIO_Port,MOS_9_Pin,GPIO_PIN_SET);
					break;
			}
			HAL_TIM_Base_Start_IT(&htim2);
			break;

		case 'M':
			send[0] = 'M';
			send[1] = 1;
			HAL_UART_Transmit(&huart3,send,2,10);
			HAL_TIM_Base_Start_IT(&htim2);
			break;
	}
}

void change_reg(void){
	uint8_t tmp = command[0] + 0x80;

	switch (tmp) {
		case servo_1_deg:
			htim3.Instance->CCR1 = servo_0 + registers[servo_1_deg - 0x80];
			HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_1);
			break;

		case servo_2_deg:
			htim3.Instance->CCR2 = servo_0 + registers[servo_2_deg - 0x80];
			HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_2);
			break;

		case led_light:
			led_light_1 = registers[led_light - 0x80] & 0x07;
			send[0] = '1';
			send[1] = led_light_1;
			HAL_UART_Transmit(&huart3,send,2,10);
			led_light_2 = registers[led_light - 0x80] >> 3;
			led_light_2 &= 0x07;
			send[0] = '2';
			send[1] = led_light_2;
			HAL_UART_Transmit(&huart3,send,2,10);
			break;

		case motor_1_speed:
			n_pulse = registers[motor_1_speed - 0x80] * motor_pulse_max;
			n_pulse /= 100;
			htim1.Instance->CCR1 = n_pulse;
			HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_1);
			break;

		case motor_2_speed:
			n_pulse = registers[motor_2_speed - 0x80] * motor_pulse_max;
			n_pulse /= 100;
			htim1.Instance->CCR2 = n_pulse;
			HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_2);
			break;

		case motor_3_speed:
			n_pulse = registers[motor_3_speed - 0x80] * motor_pulse_max;
			n_pulse /= 100;
			htim1.Instance->CCR3 = n_pulse;
			HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_3);
			break;

		case motor_4_speed:
			n_pulse = registers[motor_4_speed - 0x80] * motor_pulse_max;
			n_pulse /= 100;
			htim1.Instance->CCR4 = n_pulse;
			HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_4);
			break;
	}
}
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
	HAL_GPIO_TogglePin(LED_work_GPIO_Port,LED_work_Pin);

	if (!command_flag){
		if (rece[0] == 0x8A){
			command_flag = 2;
		}else if (READ_BIT(rece[0],0x80) && !addressed){    // set register
			command[0] = rece[0] - 0x80;
			addressed++;
		} else if (command[0]){                         		// set value
			command[1] = rece[0];
			registers[command[0]] = rece[0];
			change_reg();
			addressed--;
			command[0] = 0;
		}
	} else {
		if (command_flag == 2){
			operate[0] = rece[0];
		} else if (command_flag == 1){
			operate[1] = rece[0];
			operate_handle(operate);
		}
		command_flag--;
	}
  HAL_UART_Receive_IT(&huart1,rece,1);
}

/*void HAL_I2C_SlaveRxCpltCallback(I2C_HandleTypeDef *hi2c){
	if (READ_BIT(rece_i2c[0],0x80) && !addressed_i2c){    // set register
    command[0] = rece_i2c[0] - 0x80;
		addressed_i2c++;
	} else if (command[0]){                         // set value
    command[1] = rece_i2c[0];
    registers[command[0]] = rece_i2c[0];
    work(registers);
		addressed_i2c--;
		command[0] = 0;
  }
	HAL_I2C_Slave_Receive_IT(&hi2c1,rece_i2c,1);
}*/

void nth(void){};
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
	if (htim == &htim4){
		addressed = 0;
		addressed_i2c = 0;
	}
	if (htim == &htim2){
		!READ_BIT(registers[mosfet_1_reg - 0x80],mos_0)?HAL_GPIO_WritePin(MOS_0_GPIO_Port,MOS_0_Pin,GPIO_PIN_RESET):nth();
		!READ_BIT(registers[mosfet_1_reg - 0x80],mos_1)?HAL_GPIO_WritePin(MOS_1_GPIO_Port,MOS_1_Pin,GPIO_PIN_RESET):nth();
		!READ_BIT(registers[mosfet_1_reg - 0x80],mos_3)?HAL_GPIO_WritePin(MOS_3_GPIO_Port,MOS_3_Pin,GPIO_PIN_RESET):nth();
		!READ_BIT(registers[mosfet_1_reg - 0x80],mos_2)?HAL_GPIO_WritePin(MOS_2_GPIO_Port,MOS_2_Pin,GPIO_PIN_RESET):nth();
		!READ_BIT(registers[mosfet_1_reg - 0x80],mos_4)?HAL_GPIO_WritePin(MOS_4_GPIO_Port,MOS_4_Pin,GPIO_PIN_RESET):nth();
		!READ_BIT(registers[mosfet_1_reg - 0x80],mos_5)?HAL_GPIO_WritePin(MOS_5_GPIO_Port,MOS_5_Pin,GPIO_PIN_RESET):nth();
		!READ_BIT(registers[mosfet_1_reg - 0x80],mos_6)?HAL_GPIO_WritePin(MOS_6_GPIO_Port,MOS_6_Pin,GPIO_PIN_RESET):nth();
		!READ_BIT(registers[mosfet_2_reg - 0x80],mos_7)?HAL_GPIO_WritePin(MOS_7_GPIO_Port,MOS_7_Pin,GPIO_PIN_RESET):nth();
		!READ_BIT(registers[mosfet_2_reg - 0x80],mos_8)?HAL_GPIO_WritePin(MOS_8_GPIO_Port,MOS_8_Pin,GPIO_PIN_RESET):nth();
		!READ_BIT(registers[mosfet_2_reg - 0x80],mos_9)?HAL_GPIO_WritePin(MOS_9_GPIO_Port,MOS_9_Pin,GPIO_PIN_RESET):nth();
		if (!READ_BIT(registers[mosfet_2_reg - 0x80],magnet)){
			send[0] = 'M';
			send[1] = 0;
			HAL_UART_Transmit(&huart3,send,2,10);
		}
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
  MX_I2C1_Init();
  MX_USART1_UART_Init();
  MX_TIM3_Init();
  MX_USART3_UART_Init();
  MX_TIM4_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
  registers[state_reg - 0x80] = 0x7F;
	registers[mosfet_2_reg - 0x80] = 0x08;
	registers[servo_1_deg - 0x80] = 90;
	registers[servo_2_deg - 0x80] = 90;
	registers[motor_1_speed - 0x80] = 10;
	registers[motor_2_speed - 0x80] = 10;
	registers[motor_3_speed - 0x80] = 10;
	registers[motor_4_speed - 0x80] = 10;
	registers[led_light - 0x80] = 0x1B;
	//HAL_I2C_Slave_Receive_IT(&hi2c1,rece_i2c,1);

	uint8_t init[2];
	init[0] = '1';
	init[1] = 3;
	HAL_UART_Transmit(&huart3,init,2,20);
	init[0] = '2';
	HAL_UART_Transmit(&huart3,init,2,20);
	HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_2);
  HAL_UART_Receive_IT(&huart1,rece,1);
	HAL_TIM_Base_Start_IT(&htim4);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
//		HAL_Delay(500);
//		READ_BIT(registers[mosfet_2_reg - 0x80],mos_9)?HAL_GPIO_TogglePin(MOS_9_GPIO_Port,MOS_9_Pin):HAL_GPIO_WritePin(MOS_9_GPIO_Port,MOS_9_Pin,GPIO_PIN_SET);
//		HAL_TIM_Base_Start_IT(&htim2);
		/*HAL_GPIO_WritePin(MOS_5_GPIO_Port,MOS_5_Pin,1);
		HAL_GPIO_WritePin(MOS_7_GPIO_Port,MOS_7_Pin,1);
		HAL_GPIO_WritePin(MOS_8_GPIO_Port,MOS_8_Pin,1);
		HAL_GPIO_WritePin(MOS_9_GPIO_Port,MOS_9_Pin,1);*/
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
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
  hi2c1.Init.OwnAddress1 = 8;
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
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 720;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

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

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 7199;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 2000;
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
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

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
  htim3.Init.Prescaler = 399;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 3600;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
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
  sConfigOC.Pulse = 270;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
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

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 7199;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 10000;
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
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

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
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 9600;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

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
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, MOS_7_Pin|MOS_8_Pin|MOS_9_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, Motor_EN_A_Pin|Motor_EN_B_Pin|MOS_0_Pin|MOS_1_Pin
                          |MOS_2_Pin|MOS_3_Pin|Motor_3_A_Pin|Motor_2_B_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, MOS_4_Pin|MOS_5_Pin|MOS_6_Pin|LED_work_Pin
                          |Motor_4_B_Pin|Motor_4_A_Pin|Motor_3_B_Pin|Motor_2_A_Pin
                          |Motor_1_B_Pin|Motor_1_A_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : MOS_7_Pin MOS_8_Pin MOS_9_Pin */
  GPIO_InitStruct.Pin = MOS_7_Pin|MOS_8_Pin|MOS_9_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : Motor_EN_A_Pin Motor_EN_B_Pin MOS_0_Pin MOS_1_Pin
                           MOS_2_Pin MOS_3_Pin Motor_3_A_Pin Motor_2_B_Pin */
  GPIO_InitStruct.Pin = Motor_EN_A_Pin|Motor_EN_B_Pin|MOS_0_Pin|MOS_1_Pin
                          |MOS_2_Pin|MOS_3_Pin|Motor_3_A_Pin|Motor_2_B_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : MOS_4_Pin MOS_5_Pin MOS_6_Pin LED_work_Pin
                           Motor_4_B_Pin Motor_4_A_Pin Motor_3_B_Pin Motor_2_A_Pin
                           Motor_1_B_Pin Motor_1_A_Pin */
  GPIO_InitStruct.Pin = MOS_4_Pin|MOS_5_Pin|MOS_6_Pin|LED_work_Pin
                          |Motor_4_B_Pin|Motor_4_A_Pin|Motor_3_B_Pin|Motor_2_A_Pin
                          |Motor_1_B_Pin|Motor_1_A_Pin;
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
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
