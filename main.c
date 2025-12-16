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

// Enable semihosting
extern void initialise_monitor_handles(void);
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
// Set true by EXTI callback when any IR receiver triggers; main loop consumes it.
bool detected = false;

// Current turret angle estimate (deg). Convention in this project: 90° = forward,
// 0° = left. (Caution: other code below uses 0–360° conventions too.)
float curr_angle = 90;

// Motor step angle (deg/step) for a typical NEMA 17 at full-step (no microstepping).
float step_size = 1.8; //1.8 degrees per nema 17 step without microstepping

// Blocking delay between step pulses (ms). Controls speed/torque crudely.
int step_delay = 1;

// One-hot-ish array indicating which receiver was last triggered (set in EXTI callback).
bool active[5] = { false, false, false, false, false };
int active_len = 5;

// Delay between detection and firing the kicker (ms). Gives turret time to rotate first.
int kick_time = 1000;

// Timer settings intended for PWM-based step generation (currently unused/commented out).
uint32_t prescaler = 71;        // Intended to yield ~1 MHz timer tick (72 MHz / (71+1)).
uint32_t timer_clk = 72000000;  // APB clock assumption (Hz).

// Stepper/kicker parameters (some currently unused since kicker code is commented).
int steps_per_rev = 200;     // Full steps per revolution for typical 1.8° motor.
float kicker_rpm = 60;       // Target kicker motor speed.
int kick_duration = 500;     // How long to enable the kicker motor (ms).

// Empirical calibration: how many driver "steps" correspond to 1° of turret rotation.
// This folds in gear/belt ratio and any microstepping configuration.
float steps_per_degree = 22;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
/* USER CODE BEGIN PFP */
// Rotate turret to an absolute angle (degrees) using open-loop stepping.
void rotate_to_angle(float);

// Trigger the kicker mechanism (currently stubbed).
void kick();

// Setup and control of PWM-driven kicker stepping (currently commented out).
void kicker_init();
void kicker_SetRPM(float);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
// Wrapper to treat a GPIO pin as a (port, pin) pair.
typedef struct {
	GPIO_TypeDef *port;
	uint16_t pin;
} GpioPin;

// Turret stepper driver pins (DIR/STEP/EN).
GpioPin turret_dir_pin  = { GPIOA, GPIO_PIN_1 };
GpioPin turret_step_pin = { GPIOA, GPIO_PIN_0 };
GpioPin turret_en_pin   = { GPIOA, GPIO_PIN_2 };

// Kicker stepper driver pins (DIR/STEP/EN).
GpioPin kicker_dir_pin  = { GPIOA, GPIO_PIN_5 };
GpioPin kicker_step_pin = { GPIOA, GPIO_PIN_3 };
GpioPin kicker_en_pin   = { GPIOA, GPIO_PIN_4 };

// IR receiver inputs. These correspond to EXTI-capable pins configured as falling-edge
// interrupts with pull-ups (active-low TSOP receivers).
GpioPin receivers[5] = {
	{ GPIOB, GPIO_PIN_1 },
	{ GPIOB, GPIO_PIN_4 },
	{ GPIOB, GPIO_PIN_5 },
	{ GPIOB, GPIO_PIN_3 },
	{ GPIOB, GPIO_PIN_0 }
};

//GPIO helpers
void gpio_write(GpioPin pin, GPIO_PinState state) {
	HAL_GPIO_WritePin(pin.port, pin.pin, state);
}

void gpio_toggle(GpioPin pin) {
	HAL_GPIO_TogglePin(pin.port, pin.pin);
}

// Rotate from curr_angle to target angle using open-loop stepping.
// Notes/assumptions:
// - No acceleration profile: this is a simple constant-delay step loop.
// - No homing/encoder feedback: curr_angle is purely an estimate.
// - Direction convention is controlled by how DIR is wired and how angle is defined.
void rotate_to_angle(float angle) {
	float offset = angle - curr_angle;

	// Select direction based on sign of requested offset.
	if (offset >= 0) {
		gpio_write(turret_dir_pin, GPIO_PIN_SET);   // "forwards"
	} else {
		gpio_write(turret_dir_pin, GPIO_PIN_RESET); // "backwards"
		offset = -offset; // make magnitude positive for step count
	}

	// Convert requested angular motion into step pulses.
	// step_size is motor deg/step; steps_per_degree accounts for drivetrain ratio
	// and any microstepping/driver scaling you empirically baked in.
	int steps = (offset / step_size) * steps_per_degree;

	// Generate STEP pulses with blocking delays.
	for (int i = 0; i < steps; i++) {
		gpio_write(turret_step_pin, GPIO_PIN_SET);
		HAL_Delay(step_delay);
		gpio_write(turret_step_pin, GPIO_PIN_RESET);
		HAL_Delay(step_delay);
	}

	// Update the internal open-loop angle estimate.
	curr_angle = angle;
}

// The functions below were intended to drive the kicker via TIM2 PWM (step pulses),
// but are currently commented out. If you re-enable them, ensure:
// - TIM2 is initialized and htim2 exists
// - PA3 AF mapping matches TIM2_CH1 on your MCU/package
// - ARR calculations don’t underflow at high RPM

void kick() {
	//HAL_GPIO_WritePin(kicker_en_pin.port, kicker_en_pin.pin, GPIO_PIN_SET);
	//HAL_Delay(kick_duration);
	//HAL_GPIO_WritePin(kicker_dir_pin.port, kicker_dir_pin.pin, GPIO_PIN_RESET);
}


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
		/* USER CODE BEGIN 3 */
		// When an IR receiver interrupt fires, detected becomes true and active[]
		// indicates which sensor triggered. This block:
		// 1) Computes a target angle from the active sensors
		// 2) Rotates turret
		// 3) Waits kick_time
		// 4) Fires kicker
		// 5) Clears state and re-enables EXTI interrupts
		if (detected) {
			// Map each receiver index to a turret angle in degrees.
			// -1 marks an unused/unmapped receiver position.
			// NOTE: These angles assume a 0–360° polar convention for averaging.
			// SECOND NOTE: Angle averaging logic is currently useless since each interrupt is
			// authoritative and clears the others
			float angles_deg[5] = { 180, 0, -1, 90, -1 };
			int angles_deg_len = sizeof(angles_deg) / sizeof(angles_deg[0]);

			// Vector-average in the unit circle to handle wrap-around cleanly
			// (e.g., averaging near 359° and 1°).
			float sum_x = 0, sum_y = 0;
			int count = 0;

			for (int i = 0; i < angles_deg_len; i++) {
				// Only include receivers that are currently active and mapped.
				if (active[i] && angles_deg[i] >= 0) {
					float rad = angles_deg[i] * (3.14159265f / 180.0f);
					sum_x += cosf(rad);
					sum_y += sinf(rad);
					count++;
				}
			}

			// Convert averaged vector back to a compass angle.
			float angle = 0;
			if (count > 0) {
				angle = atan2f(sum_y / count, sum_x / count) * (180.0f / 3.14159265f);
				if (angle < 0) angle += 360.0f; // normalize to [0, 360)
			}

			// Rotate turret to the computed target.
			rotate_to_angle(angle);

			// Wait before kicking (lets the turret settle / gives player time).
			HAL_Delay(kick_time);

			// Fire the ball return.
			kick();

			// Clear the event so we don't retrigger without a new interrupt.
			detected = false;

			// Re-enable EXTI interrupts (implies you expect them to be disabled
			// elsewhere to avoid re-entrancy while processing a detection).
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
	GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_4 | GPIO_PIN_5,
			GPIO_PIN_RESET);

	/*Configure GPIO pins : PA0 PA1 PA2 PA4
	 PA5 */
	GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_4
			| GPIO_PIN_5;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/*Configure GPIO pin : PA3 */
	GPIO_InitStruct.Pin = GPIO_PIN_3;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

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
// Called by HAL on an EXTI interrupt for any configured GPIO pin.
// This implementation:
// - Flags that a detection occurred
// - Sets exactly one receiver as "active" (the one that triggered) and clears the rest
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
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
