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
#include <stdio.h>
#include <string.h>
#include <math.h>
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
char rxData[32];
uint8_t vesselStatus = 0;
uint32_t startTime;
// NRF24 Commands
#define R_REGISTER         0x00
#define W_REGISTER         0x20
#define W_TX_PAYLOAD       0xA0
#define FLUSH_TX           0xE1

// NRF24 Registers
#define CONFIG             0x00
#define EN_AA              0x01
#define EN_RXADDR          0x02
#define SETUP_AW           0x03
#define SETUP_RETR         0x04
#define RF_CH              0x05
#define RF_SETUP           0x06
#define STATUS             0x07
#define RX_ADDR_P0         0x0A
#define TX_ADDR            0x10
#define RX_PW_P0           0x11
/* USER CODE END Includes */
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

float pitch = 0;
float roll = 0;
float yaw = 0;

uint32_t lastTime = 0;

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
SPI_HandleTypeDef hspi2;

TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart2;

I2C_HandleTypeDef hi2c1;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM2_Init(void);
static void MX_SPI2_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */
uint8_t NRF_ReadReg(uint8_t reg);
void NRF_WriteReg(uint8_t reg, uint8_t value);
void NRF_WriteBuf(uint8_t reg, uint8_t *buf, uint8_t len);
void NRF_Send(uint8_t *data, uint8_t len);
void NRF_Init_TX(void);
void NRF_Init_RX(void);
void NRF_ReadPayload(uint8_t *data, uint8_t len);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int __io_putchar(int ch)
{
    HAL_UART_Transmit(&huart2,
                      (uint8_t *)&ch,
                      1,
                      HAL_MAX_DELAY);
    return ch;
}

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

void MPU6050_Init(void)
{
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
}



void MPU6050_Read(void)
{
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
}

uint8_t NRF_ReadReg(uint8_t reg)
{
    uint8_t tx[2]={reg,0xFF};
    uint8_t rx[2];

    HAL_GPIO_WritePin(GPIOA,GPIO_PIN_4,GPIO_PIN_RESET);
    HAL_SPI_TransmitReceive(&hspi2,tx,rx,2,100);
    HAL_GPIO_WritePin(GPIOA,GPIO_PIN_4,GPIO_PIN_SET);

    return rx[1];
}

void NRF_WriteReg(uint8_t reg,uint8_t value)
{
    uint8_t tx[2];

    tx[0]=W_REGISTER|reg;
    tx[1]=value;

    HAL_GPIO_WritePin(GPIOA,GPIO_PIN_4,GPIO_PIN_RESET);
    HAL_SPI_Transmit(&hspi2,tx,2,100);
    HAL_GPIO_WritePin(GPIOA,GPIO_PIN_4,GPIO_PIN_SET);
}

void NRF_WriteBuf(uint8_t reg,uint8_t *buf,uint8_t len)
{
    uint8_t cmd=W_REGISTER|reg;

    HAL_GPIO_WritePin(GPIOA,GPIO_PIN_4,GPIO_PIN_RESET);

    HAL_SPI_Transmit(&hspi2,&cmd,1,100);
    HAL_SPI_Transmit(&hspi2,buf,len,100);

    HAL_GPIO_WritePin(GPIOA,GPIO_PIN_4,GPIO_PIN_SET);
}

void NRF_ReadPayload(uint8_t *data, uint8_t len)
{
    uint8_t cmd = 0x61;      // R_RX_PAYLOAD

    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);

    HAL_SPI_Transmit(&hspi2, &cmd, 1, 100);
    HAL_SPI_Receive(&hspi2, data, len, 100);

    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);

    // Clear RX_DR interrupt
    NRF_WriteReg(0x07, 0x40);
}

void NRF_Send(uint8_t *data, uint8_t len)
{
    uint8_t cmd;

    // Flush TX FIFO
    cmd = FLUSH_TX;

    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
    HAL_SPI_Transmit(&hspi2, &cmd, 1, 100);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);

    // Write payload
    cmd = W_TX_PAYLOAD;

    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
    HAL_SPI_Transmit(&hspi2, &cmd, 1, 100);
    HAL_SPI_Transmit(&hspi2, data, len, 100);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);

    // Pulse CE
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_SET);
    HAL_Delay(1);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_RESET);
}

void NRF_Init_TX(void)
{
    uint8_t addr[5]={'0','0','0','0','2'};

    HAL_GPIO_WritePin(GPIOA,GPIO_PIN_0,GPIO_PIN_RESET);
    HAL_Delay(5);

    NRF_WriteReg(CONFIG,0x0E);
    NRF_WriteReg(EN_AA,0x00);
    NRF_WriteReg(EN_RXADDR,0x01);
    NRF_WriteReg(SETUP_AW,0x03);
    NRF_WriteReg(SETUP_RETR,0x2F);
    NRF_WriteReg(RF_CH,76);
    NRF_WriteReg(RF_SETUP,0x06);

    NRF_WriteBuf(TX_ADDR,addr,5);
    NRF_WriteBuf(RX_ADDR_P0,addr,5);

    NRF_WriteReg(RX_PW_P0,32);

    printf("NRF TX Ready\r\n");
}

void NRF_Init_RX(void)
{
    uint8_t addr[5] = {'0','0','0','0','1'};

    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_RESET);
    HAL_Delay(5);

    NRF_WriteReg(0x00, 0x0F);      // CONFIG: CRC + PWR_UP + PRIM_RX
    NRF_WriteReg(0x01, 0x01);      // EN_AA
    NRF_WriteReg(0x02, 0x01);      // Enable Pipe0
    NRF_WriteReg(0x03, 0x03);      // 5-byte address
    NRF_WriteReg(0x04, 0x2F);      // Auto Retransmit
    NRF_WriteReg(0x05, 76);        // Channel
    NRF_WriteReg(0x06, 0x06);      // 1 Mbps

    NRF_WriteBuf(0x0A, addr, 5);   // RX_ADDR_P0
    NRF_WriteReg(0x11, 32);        // RX Payload = 32 bytes

    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_SET);

    printf("NRF RX Ready\r\n");
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
  MX_TIM2_Init();
  MX_SPI2_Init();
  MX_I2C1_Init();
  MX_USART2_UART_Init();





  MPU6050_Init();

  /* USER CODE BEGIN 2 */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  uint8_t rxData[32];

  while (1)
  {
      /***********************
        STEP 1 : RECEIVE
      ************************/
      NRF_Init_RX();

      HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
         uint8_t cmd = 0xE2;        // FLUSH_RX
         HAL_SPI_Transmit(&hspi2, &cmd, 1, 100);
         HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);

         // Clear all IRQ flags
            NRF_WriteReg(STATUS, 0x70);


      vesselStatus = 0;
      memset(rxData, 0, sizeof(rxData));

      uint32_t start = HAL_GetTick();

      while ((HAL_GetTick() - start) < 5000)
      {
          uint8_t status = NRF_ReadReg(STATUS);

          if (status & 0x40)        // RX_DR
          {
              memset(rxData,0,32);

              NRF_ReadPayload((uint8_t *)rxData,32);

              printf("Received : %s\r\n",rxData);

              if(strncmp(rxData,"HB:",3)==0)
              {
                  vesselStatus = 1;
              }

              NRF_WriteReg(STATUS,0x40);     // Clear RX_DR
          }
      }

      if(vesselStatus)
          printf("VESSEL DETECTED\r\n");
      else
          printf("DARK VESSEL DETECTED\r\n");

      HAL_Delay(1000);


      /***********************
        STEP 2 : MPU6050
      ************************/
      MPU6050_Read();

      float ax = Accel_X_RAW / 16384.0f;
      float ay = Accel_Y_RAW / 16384.0f;
      float az = Accel_Z_RAW / 16384.0f;

      pitch = atan2(-ax,sqrt(ay*ay+az*az))*180.0f/3.14159f;
      roll  = atan2(ay,az)*180.0f/3.14159f;

      uint32_t now = HAL_GetTick();
      float dt=(now-lastTime)/1000.0f;
      lastTime=now;

      yaw += (Gyro_Z_RAW/131.0f)*dt;


      /***********************
        STEP 3 : PREPARE DATA
      ************************/
      char txData[32];

      memset(txData,0,32);

      snprintf(txData,
               sizeof(txData),
               "V:%d,P:%.1f,R:%.1f,Y:%.1f",
               vesselStatus,
               pitch,
               roll,
               yaw);

      printf("Sending : %s\r\n",txData);


      /***********************
        STEP 4 : TRANSMIT
      ************************/
      NRF_Init_TX();

      NRF_WriteReg(STATUS,0x70);

      NRF_Send((uint8_t *)txData,32);

      uint32_t timeout=HAL_GetTick();

      while(1)
      {
          uint8_t status=NRF_ReadReg(STATUS);

          if(status & 0x20)
          {
              printf("Telemetry Sent\r\n");
              break;
          }

          if(status & 0x10)
          {
              printf("TX Failed\r\n");
              break;
          }

          if(HAL_GetTick()-timeout>100)
          {
              printf("TX Timeout\r\n");
              break;
          }
      }

      NRF_WriteReg(STATUS,0x70);

      HAL_Delay(5);

      /***********************
        LOOP REPEATS
      ************************/
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
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

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
  htim2.Init.Prescaler = 71;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 65535;
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
  huart2.Init.BaudRate = 9600;
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
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  // CE LOW
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_RESET);

  // CSN HIGH
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);

  /*Configure GPIO pins : PA0 PA4 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

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
