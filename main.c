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
#include "quaternionFilters.h"


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

UART_HandleTypeDef huart1;
DMA_HandleTypeDef hdma_usart1_rx;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

//Stuff for cube flip
int pwm_esc1 = 0;
int pwm_esc2 = 0;
int pwm_esc3 = 0;

int pwm_servo1 = 0;
int pwm_servo2 = 0;
int pwm_servo3 = 0;
int dummy = 0;

//Stuff for loop time
uint8_t sendCount;
uint32_t mainCountPrev, mainCountCurr, mainCount;

//Stuff for Bluetooth
uint8_t dataRX[6];
uint8_t dataTX[100];

//Stuff for MPU
uint8_t i2cBuff[15];
uint16_t mpu6050Address = 0xD0;
uint16_t ak8963Address = 0x18;
int16_t gx, gy, gz, ax, ay, az, mx, my, mz;
int32_t gxOff, gyOff, gzOff;
int32_t axOff, ayOff, azOff;
int32_t mxOff, myOff, mzOff;
float mxScale, myScale, mzScale;
uint16_t loop;
double Xang, Yang, Zang;
double XangVel, YangVel, ZangVel, Xacc, Yacc, Zacc;
double XDegree, YDegree, ZDegree;
double XgyroAng, YgyroAng, ZgyroAng, XaccAng, YaccAng, ZaccAng;
float alpha = 0.02;
uint8_t outstrX[10];
uint8_t outstrY[10];
uint8_t outstrZ[10];

//Stuff for printing the angles
char *XSign, *YSign, *ZSign;
float XVal, YVal, ZVal;
int XInt1, YInt1, ZInt1;
float XFrac, YFrac, ZFrac;
int XInt2, YInt2, ZInt2;

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
  MX_USART1_UART_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */

  //Begin PWM
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
  HAL_Delay(3000);
  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, 350);
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 700); // 680 implies the brake is not active

  //Enable i2c bypass on mpu9250
  i2cBuff[0] = 0x37;
  i2cBuff[1] = 0x02;
  HAL_I2C_Master_Transmit(&hi2c1, mpu6050Address, i2cBuff, 2, 100);

  //Wake up MPU
  i2cBuff[0] = 0x6B;
  i2cBuff[1] = 0x00;
  HAL_I2C_Master_Transmit(&hi2c1, mpu6050Address, i2cBuff, 2, 100);

  //Set Gyro Range to +/- 250 deg/sec
  i2cBuff[0] = 0x27;
  i2cBuff[1] = 0x00;
  HAL_I2C_Master_Transmit(&hi2c1, mpu6050Address, i2cBuff, 2, 100);

  //Set Accel Range to +/- 2g
  i2cBuff[0] = 0x1C;
  i2cBuff[1] = 0x00;
  HAL_I2C_Master_Transmit(&hi2c1, mpu6050Address, i2cBuff, 2, 100);

  //Set Mag range to 16 bits and mode to continuous 100Hz measurement
  i2cBuff[0] = 0x0A;
  i2cBuff[1] = 0x16;
  HAL_I2C_Master_Transmit(&hi2c1, ak8963Address, i2cBuff, 2, 100);

  //Get data from Bluetooth
  sprintf(dataTX, "Press any Button to Start.\r\n");
  HAL_UART_Transmit(&huart1, dataTX, strlen(dataTX), 1000);
  sprintf(dataRX, "______");
  HAL_UART_Receive_DMA(&huart1, dataRX, 6);

  //Set initial offsets
  gxOff = -480; //X-Offset
  gyOff = 100; //Y-Offset
  gzOff = 100; //Z-Offset
  axOff = 700; //X-Offset
  ayOff = -650; //Y-Offset
  azOff = -1800; //Z-Offset

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	  //See if mag data is ready
	  i2cBuff[0]= 0x02; //Data is ready register
	  HAL_I2C_Master_Transmit(&hi2c1, ak8963Address, i2cBuff, 1, 20);
	  i2cBuff[1] = 0x00;
	  HAL_I2C_Master_Receive(&hi2c1, ak8963Address, &i2cBuff[1], 1, 20);
	  if (i2cBuff[1] & 0x01){
		  //Get mag data
		  i2cBuff[0]= 0x03; //Magnetometer address
		  HAL_I2C_Master_Transmit(&hi2c1, ak8963Address, i2cBuff, 1, 20);
		  i2cBuff[1] = 0x00;
		  HAL_I2C_Master_Receive(&hi2c1, ak8963Address, &i2cBuff[1], 7, 20);
		  //See if magnetometer has overflowed
		  if (!(i2cBuff[7] & 0x08)){
			  mx = (i2cBuff[2]<<8  |  i2cBuff[1]);
			  my = (i2cBuff[4]<<8  |  i2cBuff[3]);
			  mz = (i2cBuff[6]<<8  |  i2cBuff[5]);
			  //sprintf(dataTX, "Xmag = %d, Ymag = %d, Zmag = %d\r\n", mx, my, mz);
			  //HAL_UART_Transmit(&huart1, dataTX, strlen(dataTX), 1000);
		  }
	  }

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

	  //Subtract offsets
	  gx -=  gxOff; //X-Offset
	  gy -=  gyOff; //Y-Offset
	  gz -=  gzOff; //Z-Offset
	  ax -=  axOff; //X-Offset
	  ay -=  ayOff; //Y-Offset
	  az -=  azOff; //Z-Offset

	  //Convert accelerations to g's
	  Xacc = ((double)ax / 32768) * 2;
	  Yacc = ((double)ay / 32768) * 2;
	  Zacc = ((double)az / 32768) * 2;

	  //Find angle from direction of gravitational acceleration
	  XaccAng = atan2(Yacc, sqrt((Xacc * Xacc) + (Zacc * Zacc))) * 57.2958;
	  YaccAng = -atan2(Xacc, sqrt((Yacc * Yacc) + (Zacc * Zacc))) * 57.2958;
	  ZaccAng = atan2(Zacc, sqrt((Xacc * Xacc) + (Yacc * Yacc))) * 57.2958;

	  //Find angle from direction of gravitational acceleration using other formula
	  //XaccAng = atan2(Yacc, Zacc) * 57.2958;
	  //YaccAng = atan2(Xacc, Zacc) * 57.2958;
	  //ZaccAng = atan2(Xacc, Yacc) * 57.2958;

	  //Convert angular velocities to deg/sec
	  XangVel = ((double)gx / 32768) * 250;
	  YangVel = ((double)gy / 32768) * 250;
	  ZangVel = ((double)gz / 32768) * 250;

	  //Get the time since the last MPU read
	  mainCountCurr = (uint32_t)__HAL_TIM_GET_COUNTER(&htim1); //Using timer 1 for testing
	  mainCount = mainCountCurr - mainCountPrev;
	  if (mainCount > 66000){
		  mainCount = mainCount + 65535; //Make mainCount positive if the timer rolled over
	  }
	  mainCountPrev = mainCountCurr;
	  mainCount = (mainCount * 64) / 8; //Main loop time in us

	  //Find degrees turned since last read
	  /*XDegree = XangVel * ((float)mainCount / 1000000); //Using a sampling rate of 10 ms
	  YDegree = YangVel * ((float)mainCount / 1000000); //Main loop seems to take an extra 4 ms
	  ZDegree = ZangVel * ((float)mainCount / 1000000);*/

	    XDegree = XangVel * 0.013; //Using a sampling rate of 10 ms
	    YDegree = YangVel * 0.013; //Main loop seems to take an extra 4 ms
	  	ZDegree = ZangVel * 0.013;

	  //Integrate angular velocities
	  XgyroAng += XDegree;
	  YgyroAng += YDegree;
	  ZgyroAng += ZDegree;

	  //Get the filtered angles
	  Xang = ((1.0 - alpha) * (Xang + XDegree)) + (alpha * XaccAng);
	  Yang = ((1.0 - alpha) * (Yang + YDegree)) + (alpha * YaccAng);
	  Zang = ((1.0 - alpha) * (Zang + ZDegree)) + (alpha * ZaccAng);

	  //Test motor
	  if (strcmp(dataRX, "TEST\r\n") == 0) {
		  sprintf(dataRX, "______");
	  	  sprintf(dataTX, "Testing Motors...\r\n");
	  	  HAL_UART_Transmit(&huart1, dataTX, strlen(dataTX), 1000);

		  //Motor Test Code
		  HAL_Delay(3000);
		  for(pwm_esc1 = 350; pwm_esc1 <= 2100; pwm_esc1++ ){
			  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, pwm_esc1);
			  HAL_Delay(30);
		  }

		  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, 350);
		  sprintf(dataTX, "Test Finished!\r\n");
		  HAL_UART_Transmit(&huart1, dataTX, strlen(dataTX), 1000);
	  }

	  //Test servo
      if (strcmp(dataRX, "SERV\r\n") == 0) {
	  	  sprintf(dataRX, "______");
	   	  sprintf(dataTX, "Testing Servo...\r\n");
	   	  HAL_UART_Transmit(&huart1, dataTX, strlen(dataTX), 1000);

	  	  //Motor Test Code
	  	  HAL_Delay(3000);
	  	  for(pwm_servo1 = 700; pwm_servo1 >= 580; pwm_servo1-- ){
	  		  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, pwm_servo1);
	  		  HAL_Delay(100);
	  	  }

  		  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 680);
  		  sprintf(dataTX, "Test Finished!\r\n");
  		  HAL_UART_Transmit(&huart1, dataTX, strlen(dataTX), 1000);
  	  }

	  //Test cube flip
      if (strcmp(dataRX, "FLIP\r\n") == 0) {
	  	  sprintf(dataRX, "______");
	   	  sprintf(dataTX, "Flipping Cube...\r\n");
	   	  HAL_UART_Transmit(&huart1, dataTX, strlen(dataTX), 1000);

	  	  //Cube Flip Test Code
		  HAL_Delay(5000);                                    //Small delay before flipping program begins
		  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 700);  //Deactivate brake
		  HAL_Delay(500);                                     //Small delay for timing purposes
	   	  sprintf(dataTX, "Ramping up motor...\r\n");
	   	  HAL_UART_Transmit(&huart1, dataTX, strlen(dataTX), 1000);
		  for (pwm_esc1 = 350; pwm_esc1 <= 1800; pwm_esc1++ ){
			  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, pwm_esc1); //Slowly ramp wheel to max speed
			  HAL_Delay(25);
		  }
		  //__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 1500); //Set wheel to max speed
	   	  sprintf(dataTX, "About to flip...\r\n");
	   	  HAL_UART_Transmit(&huart1, dataTX, strlen(dataTX), 1000);
		  HAL_Delay(12000);                                   //Gives 12 seconds for wheel to spin up
		  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, 0);    //Cuts power to wheel
		  HAL_Delay(20);                                      //Small delay so events don't overlap
		  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 560);  //Activate brake
		  HAL_Delay(1000);                                    //Waits 1 second for timing purposes
		  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 700);  //Deactivate brake
		  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, 0);  //Reset motor to normal running speed

  		  sprintf(dataTX, "Test Finished!\r\n");
  		  HAL_UART_Transmit(&huart1, dataTX, strlen(dataTX), 1000);
  	  }

	  //Get loop time
	  if (strcmp(dataRX, "TIME\r\n") == 0) {
		  sprintf(dataTX, "%lu\r\n", mainCount);
		  HAL_UART_Transmit(&huart1, dataTX, strlen(dataTX), 1000);
	  }

	  //Calibrate
	  if (strcmp(dataRX, "CALB\r\n") == 0) {
		  //Send confirmation to Bluetooth
		  sprintf(dataRX, "______");
		  sprintf(dataTX, "Calibration Starting....\r\n");
		  HAL_UART_Transmit(&huart1, dataTX, strlen(dataTX), 1000);

		  gxOff = 0; //X-Offset
		  gyOff = 0; //Y-Offset
		  gzOff = 0; //Z-Offset
		  axOff = 0; //X-Offset
		  ayOff = 0; //Y-Offset
		  azOff = 0; //Z-Offset

		  //Reset gyro angles
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

		  //Add the next 1000 reads
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

		      axOff += ax;
		      ayOff += ay;
		      azOff += (az - 16384);
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



/*
		  sprintf(dataTX, "Wave the device in a figure-eight pattern.\r\n");
		  HAL_UART_Transmit(&huart1, dataTX, strlen(dataTX), 1000);

		  //Calibrate magnetometer
		  uint16_t ii = 0;
		  int32_t mag_bias[3]  = {0, 0, 0},
		          mag_scale[3] = {0, 0, 0};
		  int16_t mag_max[3]   = {-32768, -32768, -32768},
		          mag_min[3]   = {32767, 32767, 32767},
		          mag_temp[3]  = {0, 0, 0};

		  for (ii = 0; ii < 1500; ii++){



			i2cBuff[0]= 0x03;
			HAL_I2C_Master_Transmit(&hi2c1, ak8963Address, i2cBuff, 1, 20);
			i2cBuff[1] = 0x00;
			HAL_I2C_Master_Receive(&hi2c1, ak8963Address, &i2cBuff[1], 6, 20);
			mag_temp[0] = (i2cBuff[2]<<8  |  i2cBuff[1]);
			mag_temp[1] = (i2cBuff[4]<<8  |  i2cBuff[3]);
			mag_temp[2] = (i2cBuff[6]<<8  |  i2cBuff[5]);



		    for (int jj = 0; jj < 3; jj++){
		      if (mag_temp[jj] > mag_max[jj]){
		        mag_max[jj] = mag_temp[jj];
		      }
		      if (mag_temp[jj] < mag_min[jj]){
		        mag_min[jj] = mag_temp[jj];
		      }
		    }

		    HAL_Delay(12);
		  }

		  // Get hard iron correction
		  // Get 'average' x mag bias in counts
		  mag_bias[0] = (mag_max[0] + mag_min[0]) / 2;
		  // Get 'average' y mag bias in counts
		  mag_bias[1] = (mag_max[1] + mag_min[1]) / 2;
		  // Get 'average' z mag bias in counts
		  mag_bias[2] = (mag_max[2] + mag_min[2]) / 2;


		  mxOff = mag_bias[0];
		  myOff = mag_bias[1];
		  mzOff = mag_bias[2];


		  // Get soft iron correction estimate
		  // Get average x axis max chord length in counts
		  mag_scale[0] = (mag_max[0] - mag_min[0]) / 2;
		  // Get average y axis max chord length in counts
		  mag_scale[1] = (mag_max[1] - mag_min[1]) / 2;
		  // Get average z axis max chord length in counts
		  mag_scale[2] = (mag_max[2] - mag_min[2]) / 2;

		  float avg_rad = mag_scale[0] + mag_scale[1] + mag_scale[2];
		  avg_rad /= 3.0;

		  mxScale = avg_rad / ((float)mag_scale[0]);
		  myScale = avg_rad / ((float)mag_scale[1]);
		  mzScale = avg_rad / ((float)mag_scale[2]);








		  //Send confirmation of calibration to Bluetooth
		  sprintf(dataTX, "Finished!\nCalibration Offsets are:\naxOff=%d, ayOff=%d, azOff=%d,\ngxOff=%d, gyOff=%d, gzOff=%d,"
				  "\nmxOff=%d, myOff=%d, mzOff=%d,\nmxScale=%d, myScale=%d, mzScale=%d\r\n",
				  axOff, ayOff, azOff, gxOff, gyOff, gzOff,
				  mxOff, myOff, mzOff, mxScale, myScale, mzScale);
		  HAL_UART_Transmit(&huart1, dataTX, strlen(dataTX), 1000);
		  */
	  }

	  //Send angle info to Bluetooth for testing purposes
	  if ((strcmp(dataRX, "SEND\r\n") == 0) && ((sendCount % 16) == 0)) {
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
		  //sprintf(dataTX, "Xacc = %d, Yacc = %d, Zacc = %d\r\n", ax, ay, az);

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
		  //sprintf(dataTX, "Xang = %d, Yang = %d, Zang = %d\r\n", gx, gy, gz);

		  //This converts the floats to a series of three ints
		  XSign = (Xang < 0) ? "-" : " ";
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

		  sprintf(dataTX, "%s%d.%02d,%s%d.%02d, %s%d.%02d,%s%d.%02d, %s%d.%02d,%s%d.%02d\r\n",
		  		  XGyrSign, XGyrInt1, XGyrInt2, YGyrSign, YGyrInt1, YGyrInt2,
		  		  XAccSign, XAccInt1, XAccInt2, YAccSign, YAccInt1, YAccInt2,
				  XSign, XInt1, XInt2, YSign, YInt1, YInt2);

		  //Send Bluetooth
		  HAL_UART_Transmit(&huart1, dataTX, strlen(dataTX), 1000);
	  }
	  sendCount++;

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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
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
  htim1.Init.Prescaler = 64;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
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
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
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
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 3;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 2500;
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
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

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
  htim3.Init.Prescaler = 7;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 20000;
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
  sConfigOC.Pulse = 0;
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
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

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
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/** 
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void) 
{
  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel5_IRQn);

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
  HAL_GPIO_WritePin(GPIOA, DIR_3_Pin|DIR_2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, DIR_1_Pin|BOOT_1_Pin|LED_1_Pin|LED_2_Pin 
                          |LED_3_Pin|LED_4_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : DIR_3_Pin DIR_2_Pin */
  GPIO_InitStruct.Pin = DIR_3_Pin|DIR_2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : DIR_1_Pin */
  GPIO_InitStruct.Pin = DIR_1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(DIR_1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : BOOT_1_Pin LED_1_Pin LED_2_Pin LED_3_Pin 
                           LED_4_Pin */
  GPIO_InitStruct.Pin = BOOT_1_Pin|LED_1_Pin|LED_2_Pin|LED_3_Pin 
                          |LED_4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : TACH_1_Pin TACH_2_Pin TACH_3_Pin */
  GPIO_InitStruct.Pin = TACH_1_Pin|TACH_2_Pin|TACH_3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

//UART Callback for testing purposes
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
