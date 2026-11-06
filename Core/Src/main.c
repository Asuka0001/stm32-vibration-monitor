/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include "icm42670.h"
#include "motion_estimator.h"
#include "velocity_estimator.h"
#include "signal_preprocess.h"
#include "fft_analyzer.h" 
#include "uart_debug.h"

#include "string.h"
#include <stdio.h> 
#include "arm_math.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define RAD2DEG 57.2957795f
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
ICM42670_Handle_t imu;
MotionEstimator_Handle_t me;
VelocityEstimator_Handle_t ve;
FFT_TriAxis_t fft_tri;
SignalPreprocess_t preprocess[3];

static uint32_t me_count = 0;
static uint32_t last_report = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
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
  MX_SPI1_Init();
  MX_USART1_UART_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */
	ICM42670_Data_t raw;
	float accel[3], gyro[3];
	char debug_buf[128];
	uint16_t print_counter = 0;
	
	//HAL_TIM_PWM_Start
	Debug_Init(&huart1);

	imu.hspi    = &hspi1;
	imu.cs_port = GPIOA;
	imu.cs_pin  = GPIO_PIN_4;

	ICM42670_Config_t cfg = {
			.accel_fs   = ICM42670_ACCEL_FS_4G,
			.gyro_fs    = ICM42670_GYRO_FS_500DPS,
			.accel_odr  = ICM42670_ACCEL_ODR_1600HZ,
			.gyro_odr   = ICM42670_GYRO_ODR_1600HZ,
			.accel_mode = ICM42670_MODE_LOW_NOISE,
			.gyro_mode  = ICM42670_MODE_LOW_NOISE
	};
	
	ICM42670_Status_t status = ICM42670_Init(&imu, &cfg);
	if (status != ICM42670_OK) {
			Debug_Printf("[ICM42670] Init failed: %d\r\n", status);
			Error_Handler();
	}
	Debug_Print("[ICM42670] Init OK\r\n");
	
	ICM42670_DMA_Init(&imu);
	ME_Init(&me, &imu, 4);
	VE_Init(&ve, 400.0f);

	
	for (int i = 0; i < 3; i++) {
			Preprocess_InitDefault(&preprocess[i]);
			Preprocess_Warmup(&preprocess[i], 0.0f, 200);
	}
	
	if (!FFT_TriAxis_Init(&fft_tri)) {
			Debug_Print("[ERR] FFT init failed!\r\n");
			Error_Handler();
	} else {
			Debug_Printf("[OK] FFT TriAxis ready (%dpt, %.2fHz resolution)\r\n", 
									 FFT_SIZE, FFT_SAMPLE_RATE / FFT_SIZE);
	}
	
	FFT_TriAxis_SetAveraging(&fft_tri, 2);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

    uint32_t now = HAL_GetTick();
    
		if (now - last_report >= 1000)
		{
				last_report = now;
				Debug_Printf("[%lus] dma=%lu me=%lu\r\n", now/1000, ICM42670_GetDMACount(), me_count);
				
				if (ICM42670_HealthCheck()) {
						Debug_Printf("[WARN] DMA recovered, total=%lu\r\n", ICM42670_GetRecoveryCount());
				}
		}
				
		
    if (ICM42670_DMA_GetData(&raw))
			{ 
          ICM42670_ConvertToPhysical(&imu, &raw, accel, gyro);
				
					float filtered[3];
					for (int i = 0; i < 3; i++) {
							filtered[i] = Preprocess_Sample(&preprocess[i], accel[i]);
					}
					FFT_TriAxis_AddSamples(&fft_tri, filtered);
            
					if (ME_Update(&me, accel[0], accel[1], accel[2], gyro[0], gyro[1], gyro[2]))
					{
							me_count++;
							VE_Update(&ve, me.linear_accel[0], me.linear_accel[1], me.linear_accel[2], me.is_stationary);

							if (++print_counter >= 400)
							{
									print_counter = 0;
									
									float roll, pitch, yaw;
									ME_GetEuler(&me, &roll, &pitch, &yaw);
                    
                  Debug_Printf("-------- Motion Estimator --------\r\n");
                  Debug_Printf("Raw Accel (g):       x=%.3f  y=%.3f  z=%.3f\r\n", accel[0], accel[1], accel[2]);
								
                  Debug_Printf("Raw Gyro (dps):      x=%.2f  y=%.2f  z=%.2f\r\n", gyro[0], gyro[1], gyro[2]);
								
                  Debug_Printf("Linear Accel (m/s2): x=%.3f  y=%.3f  z=%.3f\r\n", me.linear_accel[0], me.linear_accel[1], me.linear_accel[2]);
								
									Debug_Printf("Vel(time) (mm/s): x=%.2f y=%.2f z=%.2f  RMS(T)=%.2f\r\n", ve.velocity[0], ve.velocity[1], ve.velocity[2], ve.velocity_rms);
								
									Debug_Printf("ZUPT: %s  acc=%.6f(thr=%.4f)  gyro=%.3f(thr=%.2f)\r\n", 
															 me.is_stationary ? "STAT" : "MOVE", 
															 me.zupt_accel_var, ZUPT_ACCEL_VAR_THRESH,
															 me.zupt_gyro_var, ZUPT_GYRO_VAR_THRESH);
                  
									const char *axis_name[] = {"X", "Y", "Z"};
									for (uint8_t axis = 0; axis < FFT_AXIS_COUNT; axis++) {
											if (FFT_TriAxis_IsReady(&fft_tri, axis)) {
													FFT_Peak_t peaks[FFT_NUM_PEAKS];
													FFT_TriAxis_GetPeaks(&fft_tri, axis, peaks, FFT_NUM_PEAKS);
													
													Debug_Printf("FFT[%s]: ", axis_name[axis]);
													for (uint8_t i = 0; i < 3; i++) {
															if (peaks[i].magnitude > 0) {
																	Debug_Printf("%.1fHz(%.4fg) ", peaks[i].freq, peaks[i].magnitude);
															}
													}
													Debug_Printf("\r\n");
											}
									}
									
//									Debug_Printf("Vel(freq) (mm/s): x=%.2f y=%.2f z=%.2f  RMS(F)=%.2f\r\n",
//															 FFT_TriAxis_GetVelocityRMS(&fft_tri, FFT_AXIS_X),
//															 FFT_TriAxis_GetVelocityRMS(&fft_tri, FFT_AXIS_Y),
//															 FFT_TriAxis_GetVelocityRMS(&fft_tri, FFT_AXIS_Z),
//															 FFT_TriAxis_GetTotalVelocityRMS(&fft_tri));
									Debug_Printf("Vel(freq) (mm/s): x=%.2f y=%.2f z=%.2f  RMS(F)=%.2f [avg=%d]\r\n",
									 FFT_TriAxis_GetAveragedVelocityRMS(&fft_tri, FFT_AXIS_X),
									 FFT_TriAxis_GetAveragedVelocityRMS(&fft_tri, FFT_AXIS_Y),
									 FFT_TriAxis_GetAveragedVelocityRMS(&fft_tri, FFT_AXIS_Z),
									 FFT_TriAxis_GetTotalAveragedVelocityRMS(&fft_tri),
									 fft_tri.axis[0].avg_target);
									Debug_Printf("======================================\r\n\r\n");
              }
          }
      }

			FFT_TriAxis_Process(&fft_tri);

      Debug_Process();
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 200;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
		Debug_TxCpltCallback(huart);
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == GPIO_PIN_0)
    {
        ICM42670_StartDMACapture();
    }
}

void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
{
    if (hspi == &hspi1)
    {
        ICM42670_DMAComplete();
    }
}

void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi)
{
    if (hspi == &hspi1)
    {
        Debug_Print("[ERR] SPI Error!\r\n");
    }
}

void HAL_DMA_ErrorCallback(DMA_HandleTypeDef *hdma)
{
    Debug_Print("[ERR] DMA Error!\r\n");
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
		  HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);//LED_BLUE
      for (volatile uint32_t i = 0; i < 50000; i++);
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
