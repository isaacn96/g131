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
#include <stdlib.h>
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
DMA_HandleTypeDef hdma_usart2_rx;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

//Stuff for Bluetooth
uint8_t dataRX[6];
uint8_t dataTX[100];

//Stuff for MPU
uint8_t i2cBuff[15];
uint16_t mpu6050Address = 0xD0;
int16_t gx,gy,gz,ax,ay,az;
int32_t gxOff, gyOff, gzOff;
int32_t axOff, ayOff, azOff;
uint16_t loop;
double Xang, Yang, Zang, ZangVel;
double XangVel, YangVel, Xacc, Yacc, Zacc;
double XgyroAng, YgyroAng, ZgyroAng, XaccAng, YaccAng, ZaccAng;
float alpha = 0.02;
uint8_t outstrX[10];
uint8_t outstrY[10];
uint8_t outstrZ[10];

//For printing the angles
char *XGyrSign, *YGyrSign, *ZGyrSign;
float XGyrVal, YGyrVal, ZGyrVal;
int XGyrInt1, YGyrInt1, ZGyrInt1;
float XGyrFrac, YGyrFrac, ZGyrFrac;
int XGyrInt2, YGyrInt2, ZGyrInt2;

char *XAccSign, *YAccSign, *ZAccSign;
float XAccVal, YAccVal, ZAccVal;
int XAccInt1, YAccInt1, ZAccInt1;
float XAccFrac, YAccFrac, ZAccFrac;
int XAccInt2, YAccInt2, ZAccInt2;

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
  MX_I2C1_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */

  //Wake up MPU
  i2cBuff[0] = 0x6B;
  i2cBuff[1] = 0x00;
  HAL_I2C_Master_Transmit(&hi2c1, mpu6050Address, i2cBuff, 2, 100);

  //Set Gyro Range
  i2cBuff[0] = 0x27;
  i2cBuff[1] = 0x00;
  HAL_I2C_Master_Transmit(&hi2c1, mpu6050Address, i2cBuff, 2, 100);

  //Get data from Bluetooth
  sprintf(dataTX, "\nPress any Button to Start.\r\n");
  HAL_UART_Transmit(&huart2, dataTX, strlen(dataTX), 1000);
  sprintf(dataRX, "______");
  HAL_UART_Receive_DMA(&huart2, dataRX, 6);

  gxOff = 0; //X-Offset
  gyOff = 0; //Y-Offset
  gzOff = 0; //Z-Offset
  axOff = 0; //X-Offset
  ayOff = 0; //Y-Offset
  azOff = 0; //Z-Offset

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	  //For testing
	  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, 1);

	  //Get gyroscope info
	  i2cBuff[0]= 0x3B; //Accelerometer addresses
	  HAL_I2C_Master_Transmit(&hi2c1, mpu6050Address, i2cBuff, 1, 20);
	  i2cBuff[1] = 0x00;
	  HAL_I2C_Master_Receive(&hi2c1, mpu6050Address, &i2cBuff[1], 14, 20);

	  ax = (i2cBuff[1]<<8 | i2cBuff[2]);
	  ay = (i2cBuff[3]<<8 | i2cBuff[4]);
	  az = (i2cBuff[5]<<8 | i2cBuff[6]);

	  gx = (i2cBuff[9]<<8  | i2cBuff[10]);
	  gy = (i2cBuff[11]<<8 | i2cBuff[12]);
	  gz = (i2cBuff[13]<<8 | i2cBuff[14]);

	  //Add offsets
	  gx +=  gxOff; //X-Offset
	  gy +=  gyOff; //Y-Offset
	  gz +=  gzOff; //Z-Offset
	  ax +=  axOff; //X-Offset
	  ay +=  ayOff; //Y-Offset
	  az +=  azOff; //Z-Offset

	  //Convert angular velocities to deg/sec
	  XangVel = ((double)gx / 32768) * 250;
	  YangVel = ((double)gy / 32768) * 250;
	  ZangVel = ((double)gz / 32768) * 250;

	  //Integrate angular velocities
	  XgyroAng += XangVel * 0.01; //Using a sampling rate of 10 ms
	  YgyroAng += YangVel * 0.01;
	  ZgyroAng += ZangVel * 0.01;

	  //Convert accelerations to g's
	  Xacc = ((double)ax / 32768) * 2;
	  Yacc = ((double)ay / 32768) * 2;
	  Zacc = ((double)az / 32768) * 2;

	  //Find angle from direction of gravitational acceleration
	  /*XaccAng = atan2(Xacc, sqrt((Yacc * Yacc) + (Zacc * Zacc))) * 57.2958;
	  YaccAng = atan2(Yacc, sqrt((Xacc * Xacc) + (Zacc * Zacc))) * 57.2958;
	  ZaccAng = atan2(sqrt((Xacc * Xacc) + (Yacc * Yacc)), Zacc) * 57.2958;*/

	  //Find angle from direction of gravitational acceleration using other formula
	  XaccAng = atan2(Yacc, Zacc) * 57.2958;
	  YaccAng = atan2(Xacc, Zacc) * 57.2958;
	  ZaccAng = atan2(Xacc, Yacc) * 57.2958;

	  //Get the filtered angles
	  Xang = ((1.0 - alpha) * XgyroAng) + (alpha * XaccAng);
	  Yang = ((1.0 - alpha) * YgyroAng) + (alpha * YaccAng);
	  Zang = ((1.0 - alpha) * ZgyroAng) + (alpha * ZaccAng);

	  //Calibrate
	  if (strcmp(dataRX, "CALB\r\n") == 0) {

		  //Send confirmation to Bluetooth
		  sprintf(dataTX, "Calibration Starting....\r\n");
		  HAL_UART_Transmit(&huart2, dataTX, strlen(dataTX), 1000);
		  sprintf(dataRX, "______");

		  gxOff = 480; //X-Offset
		  gyOff = -100; //Y-Offset
		  gzOff = -100; //Z-Offset
		  axOff = -700; //X-Offset
		  ayOff = 650; //Y-Offset
		  azOff = 1800; //Z-Offset

		  XgyroAng = 0;
		  YgyroAng = 0;
		  ZgyroAng = 0;


		  //Burn 100 read cycles
		  for (loop = 0; loop < 100; loop++){
			  i2cBuff[0]= 0x3B; //Accelerometer addresses
			  HAL_I2C_Master_Transmit(&hi2c1, mpu6050Address, i2cBuff, 1, 20);
			  i2cBuff[1] = 0x00;
			  HAL_I2C_Master_Receive(&hi2c1, mpu6050Address, &i2cBuff[1], 14, 20);
		  }

		  //Average the next 1000 reads
		  for (loop = 0; loop < 1000; loop++){
		  	  i2cBuff[0]= 0x3B; //Accelerometer addresses
		  	  HAL_I2C_Master_Transmit(&hi2c1, mpu6050Address, i2cBuff, 1, 20);
		      i2cBuff[1] = 0x00;
		      HAL_I2C_Master_Receive(&hi2c1, mpu6050Address, &i2cBuff[1], 14, 20);

		      ax = (i2cBuff[1]<<8  |  i2cBuff[2]);
		      ay = (i2cBuff[3]<<8  |  i2cBuff[4]);
		      az = (i2cBuff[5]<<8  |  i2cBuff[6]);
		      gx = (i2cBuff[9]<<8  | i2cBuff[10]);
		      gy = (i2cBuff[11]<<8 | i2cBuff[12]);
		      gz = (i2cBuff[13]<<8 | i2cBuff[14]);

		      //Find running count
		      axOff += ax;
		      ayOff += ay;
		      azOff += (16384 - az);
		      gxOff += gx;
		      gyOff += gy;
		      gzOff += gz;
		  }

		  //Find the averages
		  axOff /= 1000;
		  ayOff /= 1000;
		  azOff /= 1000;
		  gxOff /= 1000;
		  gyOff /= 1000;
		  gzOff /= 1000;

		  //Send confirmation of calibration to Bluetooth
		  sprintf(dataTX, "Finished!\nCalibration Offsets are: axOff=%d, ayOff=%d, azOff=%d, gxOff=%d, gyOff=%d, gzOff=%d\r\n",
				  axOff, ayOff, azOff, gxOff, gyOff, gzOff);
		  HAL_UART_Transmit(&huart2, dataTX, strlen(dataTX), 1000);
		  sprintf(dataRX, "______");
	  }


	  //Send angle info to Bluetooth for testing purposes
	  if (strcmp(dataRX, "SEND\r\n") == 0) {
		  //Clear RX buffer
		  //sprintf(dataRX, "______");

		  //This converts the floats to a series of three ints,
		  //As there is no support for printing floats in the STM32
		  XGyrSign = (XgyroAng < 0) ? "-" : " ";
		  XGyrVal = (XgyroAng < 0) ? -XgyroAng : XgyroAng;
		  XGyrInt1 = XGyrVal;                      // Get the integer
		  XGyrFrac = XGyrVal - XGyrInt1;            // Get fraction
		  XGyrInt2 = trunc(XGyrFrac * 100);        // Turn into integer

		  YGyrSign = (YgyroAng < 0) ? "-" : " ";
		  YGyrVal = (YgyroAng < 0) ? -YgyroAng : YgyroAng;
		  YGyrInt1 = YGyrVal;                      // Get the integer
		  YGyrFrac = YGyrVal - YGyrInt1;            // Get fraction
		  YGyrInt2 = trunc(YGyrFrac * 100);        // Turn into integer

		  ZGyrSign = (ZgyroAng < 0) ? "-" : " ";
		  ZGyrVal = (ZgyroAng < 0) ? -ZgyroAng : ZgyroAng;
		  ZGyrInt1 = ZGyrVal;                      // Get the integer
		  ZGyrFrac = ZGyrVal - ZGyrInt1;            // Get fraction
		  ZGyrInt2 = trunc(ZGyrFrac * 100);        // Turn into integer

		  //sprintf(dataTX, "%s%d.%02d, %s%d.%02d, %s%d.%02d, \r\n",
			// 	  XGyrSign, XGyrInt1, XGyrInt2, YGyrSign, YGyrInt1, YGyrInt2, ZGyrSign, ZGyrInt1, ZGyrInt2);

		  //sprintf(dataTX, "Xacc = %d, Yacc = %d, Zacc = %d\r\n", ax, ay, az);
		  //Send Bluetooth
		  //HAL_UART_Transmit(&huart2, dataTX, strlen(dataTX), 1000);

		  //This converts the floats to a series of three ints
		  XAccSign = (XaccAng < 0) ? "-" : " ";
		  XAccVal = (XaccAng < 0) ? -XaccAng : XaccAng;
		  XAccInt1 = XAccVal;                      // Get the integer
		  XAccFrac = XAccVal - XAccInt1;              // Get fraction
		  XAccInt2 = trunc(XAccFrac * 100);        // Turn into integer

		  YAccSign = (YaccAng < 0) ? "-" : " ";
		  YAccVal = (YaccAng < 0) ? -YaccAng : YaccAng;
		  YAccInt1 = YAccVal;                      // Get the integer
		  YAccFrac = YAccVal - YAccInt1;              // Get fraction
		  YAccInt2 = trunc(YAccFrac * 100);        // Turn into integer

		  ZAccSign = (ZaccAng < 0) ? "-" : " ";
		  ZAccVal = (ZaccAng < 0) ? -ZaccAng : ZaccAng;
		  ZAccInt1 = ZAccVal;                      // Get the integer
		  ZAccFrac = ZAccVal - ZAccInt1;              // Get fraction
		  ZAccInt2 = trunc(ZAccFrac * 100);        // Turn into integer

		  sprintf(dataTX, "%s%d.%02d,%s%d.%02d,%s%d.%02d, %s%d.%02d,%s%d.%02d,%s%d.%02d\r\n",
				  XGyrSign, XGyrInt1, XGyrInt2, YGyrSign, YGyrInt1, YGyrInt2, ZGyrSign, ZGyrInt1, ZGyrInt2,
				  XAccSign, XAccInt1, XAccInt2, YAccSign, YAccInt1, YAccInt2, ZAccSign, ZAccInt1, ZAccInt2);

		  //sprintf(dataTX, "Xang = %d, Yang = %d, Zang = %d\r\n", gx, gy, gz);
		  //Send Bluetooth
		  HAL_UART_Transmit(&huart2, dataTX, strlen(dataTX), 1000);

		  //This converts the floats to a series of three ints
		  /*XSign = (Xang < 0) ? "-" : " ";
		  XVal = (Xang < 0) ? -Xang : Xang;
		  XInt1 = XVal;                      // Get the integer
		  XFrac = XVal - XInt1;              // Get fraction
		  XInt2 = trunc(XFrac * 100);        // Turn into integer

		  YSign = (Yang < 0) ? "-" : " ";
		  YVal = (Yang < 0) ? -Yang : Yang;
		  YInt1 = YVal;                      // Get the integer
		  YFrac = YVal - YInt1;              // Get fraction
		  YInt2 = trunc(YFrac * 100);        // Turn into integer

		  ZSign = (Zang < 0) ? "-" : " ";
		  ZVal = (Zang < 0) ? -Zang : Zang;
		  ZInt1 = ZVal;                      // Get the integer
		  ZFrac = ZVal - ZInt1;              // Get fraction
		  ZInt2 = trunc(ZFrac * 100);        // Turn into integer

		  sprintf(dataTX, "Xang = %s%d.%02d, Yang = %s%d.%02d, Zang = %s%d.%02d\r\n",
		   		  XSign, XInt1, XInt2, YSign, YInt1, YInt2, ZSign, ZInt1, ZInt2);

		  //Send Bluetooth
		  HAL_UART_Transmit(&huart2, dataTX, strlen(dataTX), 1000);*/
	  }

	  //For testing
	  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, 0);
	  HAL_Delay(10);
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

  /**Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /**Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
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
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void) 
{
  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel6_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel6_IRQn);

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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14, GPIO_PIN_RESET);

  /*Configure GPIO pins : PB12 PB13 PB14 */
  GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	__NOP();
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
