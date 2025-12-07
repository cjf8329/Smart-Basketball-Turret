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
#include <stdbool.h>
#include <math.h>
extern void
initialise_monitor_handles(void);
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
bool detected = false;
float curr_angle = 0; //0 degrees, pointed to the right
float step_size = 1.8; //1.8 degrees per nema 17 step without microstepping
int step_delay = 1; //delay between steps in ms
bool active[5] = { false, false, false, false, false };
int active_len = 5;
//int kick_time = 1000; //number of milliseconds between player being detected (IR) and kicker activating
uint32_t prescaler = 71; //1 MHz timer clock
uint32_t timer_clk = 72000000; //72 MHz APB1
//int steps_per_rev = 200;
//float kicker_rpm = 60;
//int kick_duration = 500; //number of milliseconds where the nema 23 is enabled when kicking
float steps_per_degree = 12.19; //experimentally determined value for turret motor steps per degree of turret rotation
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
/* USER CODE BEGIN PFP */
void rotate_to_angle(float);
//void kick();
//void kicker_init();
//void kicker_SetRPM(float);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
typedef struct {
	GPIO_TypeDef *port;
	uint16_t pin;
} GpioPin;

//turret stepper pin structs
GpioPin turret_dir_pin = { GPIOA, GPIO_PIN_1 };
GpioPin turret_step_pin = { GPIOA, GPIO_PIN_0 };
GpioPin turret_en_pin = { GPIOA, GPIO_PIN_2 };

//kicker stepper pin structs
//GpioPin kicker_dir_pin = { GPIOA, GPIO_PIN_5 };
//GpioPin kicker_step_pin = { GPIOA, GPIO_PIN_3 };
//GpioPin kicker_en_pin = { GPIOA, GPIO_PIN_4 };

//receiver pin structs
GpioPin receivers[5] = { { GPIOB, GPIO_PIN_1 }, { GPIOB, GPIO_PIN_4 }, { GPIOB,
		GPIO_PIN_5 }, { GPIOB, GPIO_PIN_3 }, { GPIOB, GPIO_PIN_0 }

};

//gpio helpers
void gpio_write(GpioPin pin, GPIO_PinState state) {
	HAL_GPIO_WritePin(pin.port, pin.pin, state);
}

void gpio_toggle(GpioPin pin) {
	HAL_GPIO_TogglePin(pin.port, pin.pin);
}

void rotate_to_angle(float angle) {
	float offset = angle - curr_angle;

	if (offset >= 0) {
		gpio_write(turret_dir_pin, GPIO_PIN_SET); // forwards
	} else {
		gpio_write(turret_dir_pin, GPIO_PIN_RESET); // backwards
		offset = -offset; // make offset positive
	}

	int steps = (offset / step_size) * steps_per_degree;

	for (int i = 0; i < steps; i++) {
		gpio_write(turret_step_pin, GPIO_PIN_SET);
		HAL_Delay(step_delay);
		gpio_write(turret_step_pin, GPIO_PIN_RESET);
		HAL_Delay(step_delay);
	}

	curr_angle = angle;
}

//void kick() {
//	//HAL_GPIO_WritePin(kicker_en_pin.port, kicker_en_pin.pin, GPIO_PIN_SET);
//	//HAL_Delay(kick_duration);
//	//HAL_GPIO_WritePin(kicker_dir_pin.port, kicker_dir_pin.pin, GPIO_PIN_RESET);
//}

//void kicker_init()
//{
//	// Configure TIM2_CH1 PWM @ 50% duty
//    TIM_OC_InitTypeDef sConfigOC = {0};
//    sConfigOC.OCMode = TIM_OCMODE_PWM1;
//    sConfigOC.Pulse = 50;
//    HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1);
//
//    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
//}
//
//void kicker_SetRPM(float rpm)
//{
//    float freq = (rpm * steps_per_rev) / 60.0f; // Hz
//    uint32_t arr = (1000000 / freq) - 1;
//
//    __HAL_TIM_SET_AUTORELOAD(&htim2, arr);
//    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, arr/2); // 50% duty
//}

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {

	/* USER CODE BEGIN 1 */

	/* USER CODE END 1 */

	/* MCU Configuration--------------------------------------------------------*/

	/* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	HAL_Init();

	/* USER CODE BEGIN Init */
	initialise_monitor_handles();
	/* USER CODE END Init */

	/* Configure the system clock */
	SystemClock_Config();

	/* USER CODE BEGIN SysInit */

	/* USER CODE END SysInit */

	/* Initialize all configured peripherals */
	MX_GPIO_Init();
	/* USER CODE BEGIN 2 */
	//kicker_init();
	//kicker_SetRPM(kicker_rpm);
	//HAL_GPIO_WritePin(kicker_dir_pin.port, kicker_dir_pin.pin, GPIO_PIN_SET);
	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1) {
		/* USER CODE END WHILE */

		//rotate_to_angle(45 * steps_per_degree);

		//rotate_to_angle(0 * steps_per_degree);

		/* USER CODE BEGIN 3 */
		if (detected) {
			float angles_deg[5] = { 180, 0, -1, 90, -1 };
			int angles_deg_len = sizeof(angles_deg) / sizeof(angles_deg[0]);
			float sum_x = 0, sum_y = 0;
			int count = 0;

			for (int i = 0; i < angles_deg_len; i++) {
				if (active[i] && angles_deg[i] >= 0) {
					float rad = angles_deg[i] * (3.14159265f / 180.0f);
					sum_x += cosf(rad);
					sum_y += sinf(rad);
					count++;
				}
			}

			float angle = 0;
			if (count > 0) {
				angle = atan2f(sum_y / count, sum_x / count)
						* (180.0f / 3.14159265f);
				if (angle < 0)
					angle += 360.0f;
			}

			rotate_to_angle(angle);

			//HAL_Delay(kick_time);

			//kick();

			// Reset flag
			detected = false;

			// Re-enable interrupts
			HAL_NVIC_EnableIRQ(EXTI0_IRQn);
			HAL_NVIC_EnableIRQ(EXTI1_IRQn);
			HAL_NVIC_EnableIRQ(EXTI2_IRQn);
			HAL_NVIC_EnableIRQ(EXTI3_IRQn);
			HAL_NVIC_EnableIRQ(EXTI4_IRQn);
		}
	}
}
/* USER CODE END 3 */

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
	RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };

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
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}

	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) {
		Error_Handler();
	}
}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void) {
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };
	/* USER CODE BEGIN MX_GPIO_Init_1 */

	/* USER CODE END MX_GPIO_Init_1 */

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOD_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOA,
	GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 /*| GPIO_PIN_4 | GPIO_PIN_5*/,
			GPIO_PIN_RESET);

	/*Configure GPIO pins : PA0 PA1 PA2
	 PA5 */
//	GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_4
//			| GPIO_PIN_5;
	GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/*Configure GPIO pin : PA3 */
//	GPIO_InitStruct.Pin = GPIO_PIN_3;
//	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
//	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
//	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/*Configure GPIO pins : PB0 PB1 PB2 PB3
	 PB4 */
	GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3
			| GPIO_PIN_4;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/* EXTI interrupt init*/
	HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(EXTI0_IRQn);

	HAL_NVIC_SetPriority(EXTI1_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(EXTI1_IRQn);

	HAL_NVIC_SetPriority(EXTI2_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(EXTI2_IRQn);

	HAL_NVIC_SetPriority(EXTI3_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(EXTI3_IRQn);

	HAL_NVIC_SetPriority(EXTI4_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(EXTI4_IRQn);

	/* USER CODE BEGIN MX_GPIO_Init_2 */

	/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
// boolean to keep track of whether the turret is actively returning a ball, or if no actions are queued.

	detected = true;
	for (int i = 0; i < active_len; i++) {
		if (GPIO_Pin == receivers[i].pin) {
			active[i] = true;
		} else {
			active[i] = false;
		}
	}
}
/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	while (1) {
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
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
