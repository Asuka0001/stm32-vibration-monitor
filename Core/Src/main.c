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
#include "dma.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "adxl345.h"
#include <stdio.h>   
#include "string.h"
#include <math.h>
#include "vibration_monitor.h"
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
// ADXL345 device handle
ADXL345_HandleTypeDef adxl345;

// Data structures
ADXL345_Acceleration_t acceleration;
ADXL345_Velocity_t velocity;
ADXL345_Angles_t angles;

VIBMON_Handle_t vibmon;
VIBMON_Stats_t vibStats;
// Timer interrupt flag
volatile uint8_t timer2_flag = 0;

// Performance monitoring
static uint32_t sample_count = 0;
static uint32_t error_count = 0;
static uint32_t last_print_time = 0;
static uint32_t data_ready_count = 0;

// DMA buffers
#define UART_BUFFER_SIZE 1024
static char uart_tx_buffer1[UART_BUFFER_SIZE];
static char uart_tx_buffer2[UART_BUFFER_SIZE];
static char *current_tx_buffer = uart_tx_buffer1;
static char *next_tx_buffer = uart_tx_buffer2;

// DMA status flags
volatile uint8_t uart_tx_busy = 0;
volatile uint8_t uart_data_pending = 0;
static char msg[1024];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/**
  * @brief Timer2 update interrupt callback
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM2) {
        timer2_flag = 1;  // Set flag for main loop
    }
}

// UART DMA complete callback
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1) {
        uart_tx_busy = 0;
        
        if (uart_data_pending) {
            uart_data_pending = 0;
            
            // Swap buffers
            char *temp = current_tx_buffer;
            current_tx_buffer = next_tx_buffer;
            next_tx_buffer = temp;
            
            // Start DMA transfer
            uart_tx_busy = 1;
            HAL_UART_Transmit_DMA(&huart1, (uint8_t*)next_tx_buffer, strlen(next_tx_buffer));
        }
    }
}

// Async UART send function
uint8_t UART_SendAsync(const char *data, uint16_t len)
{
    if (len >= UART_BUFFER_SIZE) return 1;
    
    if (!uart_tx_busy) {
        // DMA idle, send directly
        memcpy(current_tx_buffer, data, len);
        current_tx_buffer[len] = '\0';
        uart_tx_busy = 1;
        HAL_UART_Transmit_DMA(&huart1, (uint8_t*)current_tx_buffer, len);
    } else {
        // DMA busy, queue to next buffer
        if (uart_data_pending) return 1;
        memcpy(next_tx_buffer, data, len);
        next_tx_buffer[len] = '\0';
        uart_data_pending = 1;
    }
    
    return 0;
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
  MX_DMA_Init();
  MX_SPI1_Init();
  MX_USART1_UART_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
  
    //char msg[1024];
    ADXL_StatusTypeDef status;

    // Send startup message via UART
    HAL_UART_Transmit(&huart1, (uint8_t*)"\r\n======================================\r\n", 40, 100);
    HAL_UART_Transmit(&huart1, (uint8_t*)"ADXL345 1000Hz Sampling Test\r\n", 29, 100);
    HAL_UART_Transmit(&huart1, (uint8_t*)"Timer2-based interrupt sampling\r\n", 33, 100);
    HAL_UART_Transmit(&huart1, (uint8_t*)"======================================\r\n\r\n", 42, 100);
    
    HAL_Delay(100);

    // Initialize ADXL345 sensor (SPI1, CS pin on GPIOA Pin 4)
    status = ADXL345_Init(&adxl345, &hspi1, GPIOA, GPIO_PIN_4);
    
		//VIBMON_Init(&vibmon, 1000);  // 1s window @ 1000Hz
		VIBMON_Init(&vibmon, 1000, 1000.0f);
		
    // Check if initialization was successful
    if (status != ADXL_OK) {
        // If init failed, print error and stop
        snprintf(msg, sizeof(msg), "[ERROR] ADXL345 Init Failed: %d\r\n", status);
        HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), 100);
        
        while(1) {
            HAL_Delay(1000);  // Infinite loop - system halted
        }
    } else {
        // Init successful
        HAL_UART_Transmit(&huart1, (uint8_t*)"[OK] ADXL345 Init Successful\r\n", 31, 100);
    }

    // Apply custom configuration
    HAL_UART_Transmit(&huart1, (uint8_t*)"Applying configuration...\r\n", 28, 100);

    
    if (status == ADXL_OK) {
        HAL_UART_Transmit(&huart1, (uint8_t*)"[OK] Configuration applied\r\n", 30, 100);
        
        // Run auto-calibration with 50 samples
        HAL_UART_Transmit(&huart1, (uint8_t*)"Running auto-calibration...\r\n", 30, 100);
        //status = ADXL345_AutoCalibrate(&adxl345, 50);
        if (status == ADXL_OK) {
            HAL_UART_Transmit(&huart1, (uint8_t*)"[OK] Auto-calibration complete\r\n", 33, 100);
        } else {
            snprintf(msg, sizeof(msg), "[WARN] Calibration failed: %d\r\n", status);
            HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), 100);
        }
    } else {
        // Configuration failed
        snprintf(msg, sizeof(msg), "[ERROR] Configuration failed: %d\r\n", status);
        HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), 100);
    }

    // Configure for 1000Hz sampling
    HAL_UART_Transmit(&huart1, (uint8_t*)"Setting up 1000Hz sampling...\r\n", 32, 100);
    ADXL345_SetSampleInterval(&adxl345, 1);  // 1ms = 1000Hz
    
    // Enable velocity tracking
    HAL_UART_Transmit(&huart1, (uint8_t*)"Enabling velocity tracking...\r\n", 32, 100);
    ADXL345_EnableVelocityTracking(&adxl345, 1);
    

    // Start Timer2 for 1000Hz sampling
    HAL_UART_Transmit(&huart1, (uint8_t*)"Starting Timer2...\r\n", 21, 100);
    HAL_TIM_Base_Start_IT(&htim2);
    
    // Reset system time tracking
    last_print_time = HAL_GetTick();
    
    HAL_UART_Transmit(&huart1, (uint8_t*)"\r\nSystem Ready! Starting 1000Hz sampling...\r\n", 45, 100);
    HAL_UART_Transmit(&huart1, (uint8_t*)"Data will be printed every 1000ms.\r\n\r\n", 39, 100);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
        
        // ====================================================================
        // NEW: Timer-based sampling approach (1000Hz)
        // ====================================================================
        
        // Check if timer interrupt occurred
        if (timer2_flag) {
            timer2_flag = 0;
            sample_count++;
            
            // Perform timed sample (reads acceleration and updates velocity)
            status = ADXL345_TimedSample(&adxl345);
            if (status != ADXL_OK) {
                error_count++;
            }
        }
        
				// Check if new data is available from the ADXL345
				if (ADXL345_IsDataReady(&adxl345)) {
						data_ready_count++;

						/* 1. raw accel ? vibration */
						VIBMON_Update(&vibmon, &adxl345.lastRawAccel);

						/* 2. linear accel ? angle / velocity */
						ADXL345_GetLastAcceleration(&adxl345, &acceleration);

						// Calculate angles from acceleration
						ADXL345_CalculateAnglesFromAccel(&acceleration, &angles);

						// Get velocity if tracking is enabled
						ADXL345_GetVelocity(&adxl345, &velocity);
				}

				uint32_t current_time = HAL_GetTick();
				if ((current_time - last_print_time) >= 1000) {
						last_print_time = current_time;
						
						float actual_rate = sample_count;
						float mag = sqrtf(acceleration.x * acceleration.x + 
														 acceleration.y * acceleration.y + 
														 acceleration.z * acceleration.z);
						
						// ?????? mm/s(??1000)
						float speed_mm = sqrtf(velocity.x * velocity.x + 
																	velocity.y * velocity.y + 
																	velocity.z * velocity.z) * 1000.0f;
						
						float vx_mm = velocity.x * 1000.0f;
						float vy_mm = velocity.y * 1000.0f;
						float vz_mm = velocity.z * 1000.0f;
						
						VIBMON_Stats_t vib_stats;
						VIBMON_GetStats(&vibmon, &vib_stats);
						
						int len = snprintf(msg, sizeof(msg),
										"\r\n=== System Status ===\r\n"
										"Rate: %.1f Hz | Ready: %u | Errors: %u\r\n"
										"\r\n--- Acceleration (g) ---\r\n"
										"X:%7.3f | Y:%7.3f | Z:%7.3f | Mag:%.3f\r\n"
										"\r\n--- Orientation (deg) ---\r\n"
										"Roll:%6.2f | Pitch:%6.2f\r\n"
										"\r\n--- Velocity (mm/s) ---\r\n"
										"X:%7.1f | Y:%7.1f | Z:%7.1f | Speed:%.1f\r\n"
										"Bias X:%.4f Y:%.4f Z:%.4f\r\n"
										"\r\n--- ZUPT ---\r\n"
										"Static:%s | Buffer:%s | Candidate:%s\r\n"
										"Window:%u | MinDur:%u ms\r\n"
										"\r\n--- Vibration ---\r\n"
										"RMS:%.4f g | Peak:%.4f g | Samples:%u\r\n"
										"Freq:%.2f Hz (%.0f RPM)\r\n"  // ????
										"===============================\r\n",
										actual_rate, (unsigned int)data_ready_count, (unsigned int)error_count,
										acceleration.x, acceleration.y, acceleration.z, mag,
										angles.roll, angles.pitch,
										vx_mm, vy_mm, vz_mm, speed_mm,
										adxl345.accelBias[0], adxl345.accelBias[1], adxl345.accelBias[2],
										adxl345.velState.isStatic ? "YES" : "NO",
										adxl345.velState.bufferFilled ? "OK" : "...",
										adxl345.velState.staticCandidate ? "YES" : "NO",
										(unsigned int)adxl345.zuptConfig.windowSize,
										(unsigned int)adxl345.zuptConfig.minStaticDuration,
										vib_stats.rms, vib_stats.peak, (unsigned int)vib_stats.sampleCount,
										vib_stats.frequency, vib_stats.frequency * 60.0f  // ????
						);
						
						UART_SendAsync(msg, len);
						
						sample_count = 0;
						error_count = 0;
						data_ready_count = 0;
									}}
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
