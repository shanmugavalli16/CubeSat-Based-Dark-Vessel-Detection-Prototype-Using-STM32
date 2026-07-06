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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <stdint.h>
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
// MPU6050 I2C address
#define MPU6050_ADDR (0x68 << 1)

// Registers
#define WHO_AM_I_REG     0x75
#define PWR_MGMT_1_REG   0x6B
#define ACCEL_XOUT_H     0x3B
#define GYRO_XOUT_H      0x43

uint8_t check;
uint8_t data;
uint8_t Rec_Data[6];

int16_t Accel_X_RAW;
int16_t Accel_Y_RAW;
int16_t Accel_Z_RAW;

int16_t Gyro_X_RAW;
int16_t Gyro_Y_RAW;
int16_t Gyro_Z_RAW;

float Ax, Ay, Az;
float Gx, Gy, Gz;

// Dynamic Calibration Offsets
int32_t accX_offset = 0, accY_offset = 0, accZ_offset = 0;
int32_t gyroX_offset = 0, gyroY_offset = 0, gyroZ_offset = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/* printf redirection to UART1 */
int __io_putchar(int ch)
  {
      HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
      return ch;
  }

/* Run automatic sensor calibration calibration */
void MPU6050_Calibrate(void)
{
    int32_t sumAccX = 0, sumAccY = 0, sumAccZ = 0;
    int32_t sumGyroX = 0, sumGyroY = 0, sumGyroZ = 0;

    printf("Calibrating MPU6050... DO NOT MOVE THE SENSOR!\r\n");

    for (int i = 0; i < 200; i++)
    {
        // Read Raw Accelerometer
        if (HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, ACCEL_XOUT_H, 1, Rec_Data, 6, 100) == HAL_OK)
        {
            sumAccX += (int16_t)(Rec_Data[0] << 8 | Rec_Data[1]);
            sumAccY += (int16_t)(Rec_Data[2] << 8 | Rec_Data[3]);
            sumAccZ += (int16_t)(Rec_Data[4] << 8 | Rec_Data[5]);
        }

        // Read Raw Gyroscope
        if (HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, GYRO_XOUT_H, 1, Rec_Data, 6, 100) == HAL_OK)
        {
            sumGyroX += (int16_t)(Rec_Data[0] << 8 | Rec_Data[1]);
            sumGyroY += (int16_t)(Rec_Data[2] << 8 | Rec_Data[3]);
            sumGyroZ += (int16_t)(Rec_Data[4] << 8 | Rec_Data[5]);
        }

        HAL_Delay(5);
    }

    // Calculate Averages
    accX_offset = sumAccX / 200;
    accY_offset = sumAccY / 200;
    accZ_offset = (sumAccZ / 200) - 16384; // Account for Earth's gravity (+1g) on Z axis

    gyroX_offset = sumGyroX / 200;
    gyroY_offset = sumGyroY / 200;
    gyroZ_offset = sumGyroZ / 200;

    printf("Calibration complete!\r\n");
    printf("Offsets -> AccX:%ld AccY:%ld AccZ:%ld | GyroX:%ld GyroY:%ld GyroZ:%ld\r\n",
            accX_offset, accY_offset, accZ_offset, gyroX_offset, gyroY_offset, gyroZ_offset);
    printf("\nStarting Clean Readings:\r\n\n");
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
  MX_I2C1_Init();
  MX_USART2_UART_Init();
  HAL_StatusTypeDef status;

  status = HAL_I2C_Mem_Read(&hi2c1,
                            MPU6050_ADDR,
                            ACCEL_XOUT_H,
                            I2C_MEMADD_SIZE_8BIT,
                            Rec_Data,
                            6,
                            1000);

  printf("Read status = %d\r\n", status);

  if(status == HAL_OK)
  {
      printf("%02X %02X %02X %02X %02X %02X\r\n",
             Rec_Data[0], Rec_Data[1],
             Rec_Data[2], Rec_Data[3],
             Rec_Data[4], Rec_Data[5]);
  }
  else
  {
      printf("Read Failed\r\n");
  }

  HAL_Delay(500);

  /* USER CODE BEGIN 2 */
  printf("\r\n");
    printf("=================================\r\n");
    printf("STM32 MPU6050 Test Starting...\r\n");
    printf("=================================\r\n");

    /* Check MPU6050 */
    HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, WHO_AM_I_REG, 1, &check, 1, 1000);

    if(check == 0x68  || check==0x70)
    {
        printf("MPU6050 Detected!\r\n");

        data = 0;
        /* Wake up MPU6050 */
        HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, PWR_MGMT_1_REG, 1, &data, 1, 1000);
        printf("MPU6050 Initialized\r\n");
        HAL_I2C_Mem_Read(&hi2c1,
                         MPU6050_ADDR,
                         PWR_MGMT_1_REG,
                         I2C_MEMADD_SIZE_8BIT,
                         &data,
                         6,
                         1000);

        printf("PWR_MGMT_1 = 0x%02X\r\n", data);

        HAL_Delay(500);

        /* Run the Calibration Sequence */
        MPU6050_Calibrate();
    }
    else
    {
        printf("MPU6050 NOT FOUND! Blocked execution.\r\n");
        while(1);
    }
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
	  printf("I2C State = %d\r\n", HAL_I2C_GetState(&hi2c1));
	  if (HAL_I2C_GetState(&hi2c1) != HAL_I2C_STATE_READY)
	  {
	      printf("I2C Busy... Reinitializing\r\n");

	      HAL_I2C_DeInit(&hi2c1);
	      HAL_Delay(5);
	      MX_I2C1_Init();
	      HAL_Delay(5);
	  }
	  /* Read Accelerometer */
	  HAL_StatusTypeDef status;

	  status = HAL_I2C_Mem_Read(&hi2c1,
	                            MPU6050_ADDR,
	                            ACCEL_XOUT_H,
	                            I2C_MEMADD_SIZE_8BIT,
	                            Rec_Data,
	                            6,
	                            1000);

	  if(status == HAL_OK)
	  {
	      Accel_X_RAW = (int16_t)((Rec_Data[0] << 8) | Rec_Data[1]) - accX_offset;
	      Accel_Y_RAW = (int16_t)((Rec_Data[2] << 8) | Rec_Data[3]) - accY_offset;
	      Accel_Z_RAW = (int16_t)((Rec_Data[4] << 8) | Rec_Data[5]) - accZ_offset;

	      printf("RAW %d %d %d\r\n",
	             Accel_X_RAW,
	             Accel_Y_RAW,
	             Accel_Z_RAW);
	  }

	     /* Read Gyroscope */
	     if (HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, GYRO_XOUT_H, 1, Rec_Data, 6, 1000) == HAL_OK)
	     {
	         Gyro_X_RAW = (int16_t)(Rec_Data[0] << 8 | Rec_Data[1]) - gyroX_offset;
	         Gyro_Y_RAW = (int16_t)(Rec_Data[2] << 8 | Rec_Data[3]) - gyroY_offset;
	         Gyro_Z_RAW = (int16_t)(Rec_Data[4] << 8 | Rec_Data[5]) - gyroZ_offset;

	         Gx = Gyro_X_RAW / 131.0f;
	         Gy = Gyro_Y_RAW / 131.0f;
	         Gz = Gyro_Z_RAW / 131.0f;
	     }

	     /* Print out structured data to Serial Monitor */
	     printf("Raw AX=%d AY=%d AZ=%d | GX=%d GY=%d GZ=%d\r\n",
	            Accel_X_RAW,
	            Accel_Y_RAW,
	            Accel_Z_RAW,
	            Gyro_X_RAW,
	            Gyro_Y_RAW,
	            Gyro_Z_RAW); // @suppress("Float formatting support")

	     HAL_Delay(200);
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
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
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
