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

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define REG_FIFO                0x00
#define REG_OP_MODE             0x01
#define REG_FRF_MSB             0x06
#define REG_FRF_MID             0x07
#define REG_FRF_LSB             0x08
#define REG_PA_CONFIG           0x09
#define REG_FIFO_ADDR_PTR       0x0D
#define REG_FIFO_TX_BASE_ADDR   0x0E
#define REG_PAYLOAD_LENGTH      0x22

#define MODE_SLEEP              0x00
#define MODE_STDBY              0x01
#define MODE_TX                 0x03
#define LONG_RANGE_MODE         0x80

#define REG_FIFO_RX_BASE_ADDR   0x0F
#define REG_IRQ_FLAGS           0x12
#define REG_FIFO_RX_CURRENT     0x10

#define REG_DIO_MAPPING_1       0x40

#define REG_PAYLOAD_LENGTH      0x22

#define TX_DONE_MASK            0x08

#define REG_MODEM_CONFIG_1   0x1D
#define REG_MODEM_CONFIG_2   0x1E
#define REG_MODEM_CONFIG_3   0x26

#define REG_PREAMBLE_MSB      0x20
#define REG_PREAMBLE_LSB      0x21
#define REG_LNA              0x0C
#define REG_FIFO_RX_BASE_ADDR 0x0F


/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi1;

/* USER CODE BEGIN PV */
uint8_t version;
uint8_t opmode;

uint8_t SX1278_ReadRegister(uint8_t addr);
void SX1278_WriteRegister(uint8_t addr, uint8_t value);
void SX1278_SendPacket(uint8_t *data, uint8_t len);

uint8_t frf_msb;
uint8_t frf_mid;
uint8_t frf_lsb;
uint8_t pa_cfg;

uint8_t modem1;
uint8_t modem2;
uint8_t modem3;

uint8_t txmode;
uint8_t irq;

uint8_t paylen;
uint8_t fifoptr;
uint8_t txbase;
uint8_t sync;

uint8_t dio1;
uint8_t dio2;

uint32_t tx_count = 0;
uint8_t timeout_occured = 0;
uint8_t version2;
uint8_t version3;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
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
  MX_SPI1_Init();
  /* USER CODE BEGIN 2 */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_RESET);
  HAL_Delay(10);
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_SET);
  HAL_Delay(10);


  version  = SX1278_ReadRegister(0x42);
  version2 = SX1278_ReadRegister(0x42);
  version3 = SX1278_ReadRegister(0x42);

  if(version2 == version3)
  {
      version = version2;
  }
  else
  {
      version = version3;
  }
  SX1278_WriteRegister(REG_OP_MODE,
                       LONG_RANGE_MODE | MODE_SLEEP);

  HAL_Delay(10);

  uint8_t op1 = SX1278_ReadRegister(REG_OP_MODE);

  SX1278_WriteRegister(REG_OP_MODE,
                       LONG_RANGE_MODE | MODE_STDBY);

  HAL_Delay(10);

  uint8_t op2 = SX1278_ReadRegister(REG_OP_MODE);

  opmode = SX1278_ReadRegister(REG_OP_MODE);

  SX1278_WriteRegister(REG_PA_CONFIG, 0x8F);
  pa_cfg = SX1278_ReadRegister(REG_PA_CONFIG);

  // Over-Current Protection (OCP) configuration
  SX1278_WriteRegister(0x0B, 0x2B); // Default 100mA current limit for 17dBm output

  // Optional: Enable +20dBm maximum output power on PA_BOOST pin
  // (Uncomment the two lines below for maximum forest range, draws ~120-150mA)
  // SX1278_WriteRegister(0x0B, 0x3B); // Raise OCP limit to 150mA
  // SX1278_WriteRegister(0x4D, 0x87); // Enable +20dBm DAC mode

  SX1278_WriteRegister(REG_FRF_MSB, 0x6C);
  SX1278_WriteRegister(REG_FRF_MID, 0x40);
  SX1278_WriteRegister(REG_FRF_LSB, 0x00);

  frf_msb = SX1278_ReadRegister(REG_FRF_MSB);
  frf_mid = SX1278_ReadRegister(REG_FRF_MID);
  frf_lsb = SX1278_ReadRegister(REG_FRF_LSB);

  SX1278_WriteRegister(REG_MODEM_CONFIG_1, 0x72);
  SX1278_WriteRegister(REG_MODEM_CONFIG_2, 0x74);
  SX1278_WriteRegister(REG_MODEM_CONFIG_3, 0x04);

  SX1278_WriteRegister(REG_FIFO_TX_BASE_ADDR, 0x00);
  SX1278_WriteRegister(REG_FIFO_RX_BASE_ADDR, 0x00);

  SX1278_WriteRegister(REG_PREAMBLE_MSB, 0x00);
  SX1278_WriteRegister(REG_PREAMBLE_LSB, 0x08);

  SX1278_WriteRegister(REG_LNA, 0x23);

  SX1278_WriteRegister(0x39, 0x12);   // Sync Word

  modem1 = SX1278_ReadRegister(REG_MODEM_CONFIG_1);
  modem2 = SX1278_ReadRegister(REG_MODEM_CONFIG_2);
  modem3 = SX1278_ReadRegister(REG_MODEM_CONFIG_3);

  sync = SX1278_ReadRegister(0x39);





  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  char msg_buf[32];
  uint32_t packet_num = 0;
  SX1278_WriteRegister(REG_DIO_MAPPING_1, 0x40);
  while (1)
  {
    /* USER CODE END WHILE */

        int len = snprintf(msg_buf, sizeof(msg_buf), "BEACON_%04lu", (unsigned long)packet_num++);
        SX1278_SendPacket((uint8_t *)msg_buf, len);

        HAL_Delay(2000);

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
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 50;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV8;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3|GPIO_PIN_4, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_RESET);

  /*Configure GPIO pin : PA2 */
  GPIO_InitStruct.Pin = GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PA3 PA4 */
  GPIO_InitStruct.Pin = GPIO_PIN_3|GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PD12 (Green LED) */
  GPIO_InitStruct.Pin = GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
uint8_t SX1278_ReadRegister(uint8_t addr)
{
    uint8_t tx[2];
    uint8_t rx[2];

    tx[0] = addr & 0x7F;
    tx[1] = 0x00;

    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);

    HAL_SPI_TransmitReceive(&hspi1, tx, rx, 2, HAL_MAX_DELAY);

    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);

    return rx[1];
}


void SX1278_WriteRegister(uint8_t addr, uint8_t value)
{
    uint8_t tx[2] = {addr | 0x80, value};

    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);

    HAL_SPI_Transmit(&hspi1, tx, 2, HAL_MAX_DELAY);

    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
}

void SX1278_SendPacket(uint8_t *data, uint8_t len)
{
    // Turn ON Green LED (PD12) to indicate active transmission start
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_SET);

    SX1278_WriteRegister(REG_FIFO_TX_BASE_ADDR, 0x00);
    SX1278_WriteRegister(REG_FIFO_ADDR_PTR, 0x00);

    for(uint8_t i = 0; i < len; i++)
    {
        SX1278_WriteRegister(REG_FIFO, data[i]);
    }

    SX1278_WriteRegister(REG_PAYLOAD_LENGTH, len);

    paylen  = SX1278_ReadRegister(REG_PAYLOAD_LENGTH);
    fifoptr = SX1278_ReadRegister(REG_FIFO_ADDR_PTR);
    txbase  = SX1278_ReadRegister(REG_FIFO_TX_BASE_ADDR);

    SX1278_WriteRegister(REG_IRQ_FLAGS, 0xFF);

    SX1278_WriteRegister(REG_OP_MODE,
                         LONG_RANGE_MODE | MODE_TX);
    dio1 = SX1278_ReadRegister(0x40);
    dio2 = SX1278_ReadRegister(0x41);

    txmode = SX1278_ReadRegister(REG_OP_MODE);

    uint32_t timeout = HAL_GetTick();

    while (1)
    {
        irq = SX1278_ReadRegister(REG_IRQ_FLAGS);

        if (irq & TX_DONE_MASK)
            break;

        if ((HAL_GetTick() - timeout) > 1000)
        {
        	timeout_occured = 1;
            break;   // 1 second timeout
        }
    }

    SX1278_WriteRegister(REG_IRQ_FLAGS, 0xFF);

    SX1278_WriteRegister(REG_OP_MODE,
                         LONG_RANGE_MODE | MODE_STDBY);

    // Turn OFF Green LED (PD12) to indicate transmission finished
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_RESET);
    
    tx_count++;
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
