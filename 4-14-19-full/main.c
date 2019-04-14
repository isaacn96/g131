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
#include <math.h>
#include <string.h>
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
TIM_HandleTypeDef htim4;

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
static void MX_TIM4_Init(void);
static void MX_TIM1_Init(void);
/* USER CODE BEGIN PFP */

void SendInterruptMessage();
void UpdatePID();
void UpdateGyro();
void MahonyQuaternionUpdate(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz, float deltat);
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);
void GetSpeed();
void UpdateMotorSpeed();
void Increase_Overflow();
float median(uint8_t n, float x[]);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

// Stuff for cube flip
int pwm_esc1 = 0;
int pwm_esc2 = 0;
int pwm_esc3 = 0;

int pwm_servo1 = 0;
int pwm_servo2 = 0;
int pwm_servo3 = 0;
int dummy = 0;

uint16_t ServoMax = 870; //brake value
uint16_t ServoMin = 750; //slack value

uint16_t MotorMax = 500;
volatile uint8_t dummy_count = 0;
uint8_t timer1_Overflow = 0;
uint16_t motorCount1, lastMotorCount;

float freq1_Buff[5];
float freq1_Buff_Copy[5];
uint8_t sendCount = 0;

// Stuff for PID
uint8_t balancing = 0;
float ki, kp, kd;
float kp_servo, kd_servo, ki_servo;
float yawError, pitchError, rollError;
float yawErrorPrev, pitchErrorPrev, rollErrorPrev;
float pitchErrorStep, rollErrorStep, yawErrorStep;

//Stuff for finding wheel speed
uint32_t count1 = 2;
uint32_t countPrev1 = 1;
uint32_t dummy1;
int32_t stepCount1 = 600;
float freq1 = 0;

// Stuff for loop time
uint8_t PID_sendCount;
uint32_t mainCountPrev, mainCountCurr, mainCount;

// Stuff for Bluetooth
uint8_t dataRX[6];
uint8_t dataTX[100];

// Stuff for MPU and AK
uint8_t i2cBuff[15];
uint16_t mpu6050Address = 0xD0;
uint16_t ak8963Address = 0x18;
uint16_t loop;
int16_t gX, gY, gZ, aX, aY, aZ, mX, mY, mZ;
int32_t gxOff, gyOff, gzOff;
int32_t axOff, ayOff, azOff;
int32_t mxOff, myOff, mzOff;
float mxScale, myScale, mzScale;
int16_t mx_max, my_max, mz_max;
int16_t mx_min, my_min, mz_min;
int16_t mx_temp, my_temp, mz_temp;
float mxSens, mySens, mzSens;
float avg_rad;
float Xang, Yang, Zang;
float XangVel, YangVel, ZangVel;
float Xacc, Yacc, Zacc;
float Xmag, Ymag, Zmag;
float XDegree, YDegree, ZDegree;
float XgyroAng, YgyroAng, ZgyroAng, XaccAng, YaccAng, ZaccAng;
float alpha = 0.02;
float yaw, pitch, roll;
uint8_t outstrX[10];
uint8_t outstrY[10];
uint8_t outstrZ[10];

// Stuff for printing the angles
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

// Stuff for Mahony Quaternion Filter
// These are the free parameters in the Mahony filter and fusion scheme, Kp
// for proportional feedback, Ki for integral
float Kp = 2.0f * 5.0f;
float Ki = 0.0f;

float GyroMeasError = 3.141592 * (40.0f / 180.0f);
// gyroscope measurement drift in rad/s/s (start at 0.0 deg/s/s)
float GyroMeasDrift = 3.141592 * (0.0f  / 180.0f);
// There is a tradeoff in the beta parameter between accuracy and response
// speed. In the original Madgwick study, beta of 0.041 (corresponding to
// GyroMeasError of 2.7 degrees/s) was found to give optimal accuracy.
// However, with this value, the LSM9SD0 response time is about 10 seconds
// to a stable initial quaternion. Subsequent changes also require a
// longish lag time to a stable output, not fast enough for a quadcopter or
// robot car! By increasing beta (GyroMeasError) by about a factor of
// fifteen, the response time constant is reduced to ~2 sec. I haven't
// noticed any reduction in solution accuracy. This is essentially the I
// coefficient in a PID control sense; the bigger the feedback coefficient,
// the faster the solution converges, usually at the expense of accuracy.
// In any case, this is the free parameter in the Madgwick filtering and
// fusion scheme.

// Beta and zeta are not needed for the Mahony Filter
// float beta = sqrt(3.0f / 4.0f) * GyroMeasError;   // Compute beta
// Compute zeta, the other free parameter in the Madgwick scheme usually
// set to a small or zero value
// float zeta = sqrt(3.0f / 4.0f) * GyroMeasDrift;

// Vector to hold integral error for Mahony method
float eInt[3] = {0.0f, 0.0f, 0.0f};
// Vector to hold quaternion
float q[4] = {1.0f, 0.0f, 0.0f, 0.0f};


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
  MX_TIM4_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */

  //Initialize timers and interrupts
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4); // Start wheel speed measurements
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3); // Start wheel motor 1 PWM
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2); // Start wheel motor 2 PWM
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1); // Start brake servo 1 PWM
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2); // Start brake servo 2 PWM
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3); // Start brake servo 3 PWM
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4); // Start interrupt for gyroscope measurements
  HAL_Delay(1000);
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 32767);
  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, 0); // 0 is implied that the wheel is not active
  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 0); // 0 is implied that the wheel is not active
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, ServoMin); // 700 implies the brake is not active
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, ServoMin); // 700 implies the brake is not active
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, ServoMin); // 700 implies the brake is not active
  __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, 0); // 1500 creates an interrupt every 12 ms

  // Wake up MPU
  i2cBuff[0] = 0x6B;
  i2cBuff[1] = 0x00;
  HAL_I2C_Master_Transmit(&hi2c1, mpu6050Address, i2cBuff, 2, 100);

  // Set Gyro Range to +/- 250 deg/sec
  i2cBuff[0] = 0x27;
  i2cBuff[1] = 0x00;
  HAL_I2C_Master_Transmit(&hi2c1, mpu6050Address, i2cBuff, 2, 100);

  // Set Accel Range to +/- 2g
  i2cBuff[0] = 0x1C;
  i2cBuff[1] = 0x00;
  HAL_I2C_Master_Transmit(&hi2c1, mpu6050Address, i2cBuff, 2, 100);

  // Enable i2c bypass on mpu9250
  i2cBuff[0] = 0x37;
  i2cBuff[1] = 0x02;
  HAL_I2C_Master_Transmit(&hi2c1, mpu6050Address, i2cBuff, 2, 100);

  // Find the factory-measured magnetometer sensitivity values
  // Enter Fuse ROM Access Mode
  i2cBuff[0] = 0x0A;
  i2cBuff[1] = 0x0F;
  HAL_I2C_Master_Transmit(&hi2c1, ak8963Address, i2cBuff, 2, 100);
  // Get sensitivity values
  i2cBuff[0] = 0x10;
  HAL_I2C_Master_Transmit(&hi2c1, mpu6050Address, i2cBuff, 1, 20);
  i2cBuff[1] = 0x00;
  HAL_I2C_Master_Receive(&hi2c1, mpu6050Address, &i2cBuff[1], 3, 20);

  // Calculate sensitivities from the raw values
  mxSens = ((float)(i2cBuff[1] - 128) / 256.0) + 1.0;
  mySens = ((float)(i2cBuff[2] - 128) / 256.0) + 1.0;
  mzSens = ((float)(i2cBuff[3] - 128) / 256.0) + 1.0;

  // For testing
  /*XSign = (mxSens < 0) ? "-" : " ";
  XVal = (mxSens < 0) ? -mxSens : mxSens;
  XInt1 = XVal;                      // Get the integer
  XFrac = XVal - XInt1;              // Get fraction
  XInt2 = trunc(XFrac * 100);        // Turn into integer

  YSign = (mySens < 0) ? "-" : " ";
  YVal = (mySens < 0) ? -mySens : mySens;
  YInt1 = YVal;                      // Get the integer
  YFrac = YVal - YInt1;              // Get fraction
  YInt2 = trunc(YFrac * 100);        // Turn into integer

  ZSign = (mzSens < 0) ? "-" : " ";
  ZVal = (mzSens < 0) ? -mzSens : mzSens;
  ZInt1 = ZVal;                      // Get the integer
  ZFrac = ZVal - ZInt1;              // Get fraction
  ZInt2 = trunc(ZFrac * 100);        // Turn into integer

  sprintf(dataTX, "mxSens=%s%d.%02d, mySens=%s%d.%02d, mzSens=%s%d.%02d\r\n",
		  XSign, XInt1, XInt2, YSign, YInt1, YInt2, ZSign, ZInt1, ZInt2);
  HAL_UART_Transmit(&huart1, dataTX, strlen(dataTX), 1000);*/




  //Set Mag range to 16 bits and measurement mode to continuous 100Hz
  i2cBuff[0] = 0x0A;
  i2cBuff[1] = 0x16;
  HAL_I2C_Master_Transmit(&hi2c1, ak8963Address, i2cBuff, 2, 100);

  //Get data from Bluetooth
  sprintf(dataTX, "\nPress any Button to Start.\r\n");
  HAL_UART_Transmit(&huart1, dataTX, strlen(dataTX), 1000);
  sprintf(dataRX, "______");
  HAL_UART_Receive_DMA(&huart1, dataRX, 6);

  //Set initial offsets
  gxOff = -480;
  gyOff = 100;
  gzOff = 100;
  axOff = 700;
  ayOff = -650;
  azOff = -1800;
  mxOff = 0;
  myOff = 0;
  mzOff = 0;
  mxScale = 1.06;
  myScale = 0.90;
  mzScale = 1.06;

  //Set interrupts
  __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, 625); // 1500 creates an interrupt every 12 ms

  HAL_NVIC_SetPriority(TIM4_IRQn, 2, 2);
  HAL_TIM_Base_Start_IT(&htim4);

  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, 0);
  HAL_TIM_Base_Start_IT(&htim1);
  HAL_NVIC_SetPriority(TIM1_UP_IRQn, 0, 0);

  // EXT interrupt init
  HAL_NVIC_SetPriority(EXTI3_IRQn, 1, 1);
  HAL_NVIC_EnableIRQ(EXTI3_IRQn);

  HAL_NVIC_SetPriority(EXTI4_IRQn, 1, 1);
  HAL_NVIC_EnableIRQ(EXTI4_IRQn);

  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 1, 1);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	  //UpdateGyro();

	  //For testing
	  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, 1);

	  //Change motor top speed
	  if (strcmp(dataRX, "MSET\r\n") == 0) {
		  sprintf(dataRX, "______");
		  HAL_Delay(100);
	  	  sprintf(dataTX, "Set the motor frequency value.\r\n");
	  	  HAL_UART_Transmit(&huart1, dataTX, strlen(dataTX), 1000);
	  	  while(strcmp(dataRX, "______") == 0){
	  		 dummy_count++;
	  	  }
  		  MotorMax = ((dataRX[0] - 48) * 1000) + ((dataRX[1] - 48) * 100) +
  				     ((dataRX[2] - 48) * 10) + ((dataRX[3] - 48) * 1);
		  sprintf(dataTX, "Motor frequency set to %d\r\n", MotorMax);
		  HAL_UART_Transmit(&huart1, dataTX, strlen(dataTX), 1000);
	  }

	  //Test Balancing
	  if (strcmp(dataRX, "BLNC\r\n") == 0) {
		  sprintf(dataRX, "______");
		  sprintf(dataTX, "Starting balance test...\r\n");
		  HAL_UART_Transmit(&huart1, dataTX, strlen(dataTX), 1000);

	   	  pwm_esc1 = 400;
		  //__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, pwm_esc1);

		  pitchErrorPrev = 0;
		  balancing = 1;

	  }

	  //Test motor
	  if (strcmp(dataRX, "TEST\r\n") == 0) {
		  sprintf(dataRX, "______");
	  	  sprintf(dataTX, "Testing Motor 1...\r\n");
	  	  HAL_UART_Transmit(&huart1, dataTX, strlen(dataTX), 1000);

		  //Motor Test Code
		  HAL_Delay(1000);
		  for(pwm_esc1 = 350; pwm_esc1 <= MotorMax; pwm_esc1++ ){
			  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, pwm_esc1);
			  HAL_Delay(30);
		  }
		  pwm_esc1 = 0;
		  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, pwm_esc1);

	  	  sprintf(dataTX, "Testing Motor 2...\r\n");
	  	  HAL_UART_Transmit(&huart1, dataTX, strlen(dataTX), 1000);

		  //Motor Test Code
		  HAL_Delay(1000);
		  for(pwm_esc2 = 350; pwm_esc2 <= MotorMax; pwm_esc2++ ){
			  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, pwm_esc2);
			  HAL_Delay(30);
		  }
		  pwm_esc2 = 0;
		  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, pwm_esc2);

	   	  /*if ((sendCount % 16) == 0){
	   		sprintf(dataTX, "Frequency = %d\r\n", (int)freq1);
	   		HAL_UART_Transmit(&huart1, dataTX, strlen(dataTX), 1000);
	   	  }
	   	  sendCount++;

	   	  sprintf(dataTX, "Final Frequencies = %d, %d, %d, %d, %d\r\n", (int)freq1_Buff[4],
	   			  (int)freq1_Buff[3], (int)freq1_Buff[2], (int)freq1_Buff[1], (int)freq1_Buff[0]);
	   	  HAL_UART_Transmit(&huart1, dataTX, strlen(dataTX), 1000);

	   	  pwm_esc1 = 0;
		  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, pwm_esc1);
	   	  HAL_Delay(200);
		  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, ServoMax);  //Activate brake
		  HAL_Delay(500);
		  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, ServoMin);  //Deactivate brake*/

		  sprintf(dataTX, "Test Finished!\r\n");
		  HAL_UART_Transmit(&huart1, dataTX, strlen(dataTX), 1000);
	  }

	  //Test servo
	  if (strcmp(dataRX, "SERV\r\n") == 0) {
		  sprintf(dataRX, "______");
	  	  sprintf(dataTX, "Testing Servo 1...\r\n");
	  	  HAL_UART_Transmit(&huart1, dataTX, strlen(dataTX), 1000);

	  	  //Servo Test Code

	  	  HAL_Delay(1000);
	  	 __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, ServoMin);
	  	 HAL_Delay(1000);
	  	 __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, ServoMax);
	  	 HAL_Delay(1000);
	  	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, ServoMin);
	   	HAL_Delay(1000);


  	  sprintf(dataTX, "Testing Servo 2...\r\n");
  	  HAL_UART_Transmit(&huart1, dataTX, strlen(dataTX), 1000);

  		  //Servo Test Code
  		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, ServoMin);
  		//__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, ServoMin);
  		HAL_Delay(1000);
  		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, ServoMax);
  		//__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, ServoMax);
  		HAL_Delay(1000);
  		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, ServoMin);
  		//__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, ServoMin);
  		HAL_Delay(1000);


	  sprintf(dataTX, "Test Finished!\r\n");
	  HAL_UART_Transmit(&huart1, dataTX, strlen(dataTX), 1000);
  	  }

	  //Test cube flip
      if (strcmp(dataRX, "FLIP\r\n") == 0) {
	  	  sprintf(dataRX, "______");
	   	  sprintf(dataTX, "Flipping Cube...\r\n");
	   	  HAL_UART_Transmit(&huart1, dataTX, strlen(dataTX), 1000);

	   	  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, 0);

	  	  //Cube Flip Test Code
		  HAL_Delay(5000);                                    //Small delay before flipping program begins
		  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, ServoMin);  //Deactivate brake
		  HAL_Delay(500);                                     //Small delay for timing purposes
	   	  sprintf(dataTX, "Ramping up motor...\r\n");
	   	  HAL_UART_Transmit(&huart1, dataTX, strlen(dataTX), 1000);

	   	  /*for (pwm_esc1 = 100; pwm_esc1 <= MotorMax; pwm_esc1++ ){
			  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, pwm_esc1); //Slowly ramp wheel to max speed
			  HAL_Delay(10);
		  }                                                           //Set wheel to max speed*/

	      __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, ServoMin);  //Deactivate brake
	  	  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, MotorMax); //Ramp wheel to max speed
	   	  HAL_Delay(3000);

	   	  sprintf(dataTX, "About to flip...\r\n");
	   	  HAL_UART_Transmit(&huart1, dataTX, strlen(dataTX), 1000);
		  HAL_Delay(1000);                                     //Gives 1 second for wheel to spin up
		  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 0);    //Cuts power to wheel
		  HAL_Delay(10);                                      //Small delay so events don't overlap
		  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, ServoMax);  //Activate brake
		  HAL_Delay(200);                                     //Waits 1 second for timing purposes
		  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, ServoMin);  //Deactivate brake
		  HAL_Delay(5);
  		  sprintf(dataTX, "Test Finished!\r\n");
  		  HAL_UART_Transmit(&huart1, dataTX, strlen(dataTX), 1000);
  	  }

	  //Get loop time
	  if (strcmp(dataRX, "TIME\r\n") == 0) {
		  sprintf(dataTX, "%lu\r\n", mainCount);
		  HAL_UART_Transmit(&huart1, dataTX, strlen(dataTX), 1000);
	  }

	  //Get frequency
	  if ((strcmp(dataRX, "FREQ\r\n") == 0) && (sendCount % 16)) {
		  sprintf(dataTX, "Frequency = %d\r\n", /*(int)freq1*/ stepCount1);
		  HAL_UART_Transmit(&huart1, dataTX, strlen(dataTX), 1000);
	  }

	  //Calibrate
	  if (strcmp(dataRX, "CALB\r\n") == 0) {
	  //Send confirmation to Bluetooth
		  HAL_TIM_Base_Stop_IT(&htim4); // Stop gyro interrupt
		  sprintf(dataRX, "______");
		  sprintf(dataTX, "Calibration Starting....\r\n");
		  HAL_UART_Transmit(&huart1, dataTX, strlen(dataTX), 1000);

		  sprintf(dataTX, "Keep the device steady.\r\n");
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

		      aX = (i2cBuff[1]<<8  |  i2cBuff[2]);
		      aY = (i2cBuff[3]<<8  |  i2cBuff[4]);
		      aZ = (i2cBuff[5]<<8  |  i2cBuff[6]);
		      gX = (i2cBuff[9]<<8  | i2cBuff[10]);
		      gY = (i2cBuff[11]<<8 | i2cBuff[12]);
		      gZ = (i2cBuff[13]<<8 | i2cBuff[14]);

		      axOff += aX;
		      ayOff += aY;
		      azOff += (aZ - 16384);
		      gxOff += gX;
		      gyOff += gY;
		      gzOff += gZ;
		  }

		  //Find the averages
		  axOff /= 1000;
		  ayOff /= 1000;
		  azOff /= 1000;
		  gxOff /= 1000;
		  gyOff /= 1000;
		  gzOff /= 1000;

		  //Calibrate the magnetometer
		  sprintf(dataTX, "Wave the device in a figure-eight pattern.\r\n");
		  HAL_UART_Transmit(&huart1, dataTX, strlen(dataTX), 1000);

		  //Reset the offsets and temp variables
		  mxOff = 0;
		  myOff = 0;
		  mzOff = 0;
		  mxScale = 0;
		  myScale = 0;
		  mzScale = 0;
		  mx_max = -10000;
		  my_max = -10000;
		  mz_max = -10000;
		  mx_min = 10000;
		  my_min = 10000;
		  mz_min = 10000;

		  //Get max and min measurements from 1500 reads
		  for (loop = 0; loop < 1500; loop++){
		  	i2cBuff[0]= 0x03;
		  	HAL_I2C_Master_Transmit(&hi2c1, ak8963Address, i2cBuff, 1, 20);
		  	i2cBuff[1] = 0x00;
		  	HAL_I2C_Master_Receive(&hi2c1, ak8963Address, &i2cBuff[1], 7, 20);
		  		if (!(i2cBuff[7] & 0x08)){
		  			mx_temp = (i2cBuff[2]<<8  |  i2cBuff[1]);
		  			my_temp = (i2cBuff[4]<<8  |  i2cBuff[3]);
		  			mz_temp = (i2cBuff[6]<<8  |  i2cBuff[5]);

		  			if (mx_temp > mx_max){mx_max = mx_temp;}
		  			if (mx_temp < mx_min){mx_min = mx_temp;}

		  			if (my_temp > my_max){my_max = my_temp;}
		  			if (my_temp < my_min){my_min = my_temp;}

		  			if (mz_temp > mz_max){mz_max = mz_temp;}
		  			if (mz_temp < mz_min){mz_min = mz_temp;}

		  			HAL_Delay(12); //New data should be ready after about 10ms
		  		}
		  }

		  // Get hard iron correction
		  mxOff = (mx_max + mx_min) / 2;
		  myOff = (my_max + my_min) / 2;
		  mzOff = (mz_max + mz_min) / 2;

		  // Get soft iron correction
		  mxScale = (mx_max - mx_min) / 2.0;
		  myScale = (my_max - my_min) / 2.0;
		  mzScale = (mz_max - mz_min) / 2.0;

		  avg_rad = (mxScale + myScale + mzScale) / 3.0;
		  mxScale = avg_rad / mxScale;
		  myScale = avg_rad / myScale;
		  mzScale = avg_rad / mzScale;

		  uint16_t mxScale_temp = mxScale * 100;
		  uint16_t myScale_temp = myScale * 100;
		  uint16_t mzScale_temp = mzScale * 100;

		  //Send confirmation of calibration to Bluetooth
		  sprintf(dataTX, "Finished!\nCalibration Offsets are:\naxOff=%d, ayOff=%d, azOff=%d,\ngxOff=%d, gyOff=%d, gzOff=%d\r\n",
				  axOff, ayOff, azOff, gxOff, gyOff, gzOff);
		  HAL_UART_Transmit(&huart1, dataTX, strlen(dataTX), 1000);

		  sprintf(dataTX, "mx_max=%d, my_max=%d, mz_max=%d,\nmx_min=%d, my_min=%d, mz_min=%d\r\n",
				  mx_max, my_max, mz_max, mx_min, my_min, mz_min);
		  HAL_UART_Transmit(&huart1, dataTX, strlen(dataTX), 1000);

		  sprintf(dataTX, "mxOff=%d, myOff=%d, mzOff=%d,\nmxScale=%d%%, myScale=%d%%, mzScale=%d%%\r\n",
				  mxOff, myOff, mzOff, mxScale_temp, myScale_temp, mzScale_temp);
		  HAL_UART_Transmit(&huart1, dataTX, strlen(dataTX), 1000);

		  HAL_TIM_Base_Start_IT(&htim4); // Restart gyro interrupt
		  sprintf(dataRX, "______");
	  }

	  //Send angle info to Bluetooth for testing purposes
	  if ((strcmp(dataRX, "SEND\r\n") == 0) && ((sendCount % 16) == 0)) {
		  //Clear RX buffer
		  //sprintf(dataRX, "______");

		  //This converts the floats to a series of three ints,
		  //As there is no support for printing floats in the STM32
		  /*XGyrSign = (XgyroAng < 0) ? "-" : " ";
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
				  XSign, XInt1, XInt2, YSign, YInt1, YInt2);*/
		  /*
		  //For testing
		  XSign = (Xmag < 0) ? "-" : " ";
		  XVal = (Xmag < 0) ? -Xmag : Xmag;
		  XInt1 = XVal;                      // Get the integer
		  XFrac = XVal - XInt1;              // Get fraction
		  XInt2 = trunc(XFrac * 100);        // Turn into integer

		  YSign = (Ymag < 0) ? "-" : " ";
		  YVal = (Ymag < 0) ? -Ymag : Ymag;
		  YInt1 = YVal;                      // Get the integer
		  YFrac = YVal - YInt1;              // Get fraction
		  YInt2 = trunc(YFrac * 100);        // Turn into integer

		  ZSign = (Zmag < 0) ? "-" : " ";
		  ZVal = (Zmag < 0) ? -Zmag : Zmag;
		  ZInt1 = ZVal;                      // Get the integer
		  ZFrac = ZVal - ZInt1;              // Get fraction
		  ZInt2 = trunc(ZFrac * 100);        // Turn into integer

		  sprintf(dataTX, "mx=%s%d.%02d, my=%s%d.%02d, mz=%s%d.%02d\r\n",
				  XSign, XInt1, XInt2, YSign, YInt1, YInt2, ZSign, ZInt1, ZInt2);
		  HAL_UART_Transmit(&huart1, dataTX, strlen(dataTX), 1000);
		  */

		  //For testing
		  XSign = (yaw < 0) ? "-" : " ";
		  XVal = (yaw < 0) ? -yaw : yaw;
		  XInt1 = XVal;                      // Get the integer
		  XFrac = XVal - XInt1;              // Get fraction
		  XInt2 = trunc(XFrac * 100);        // Turn into integer

		  YSign = (pitch < 0) ? "-" : " ";
		  YVal = (pitch < 0) ? -pitch : pitch;
		  YInt1 = YVal;                      // Get the integer
		  YFrac = YVal - YInt1;              // Get fraction
		  YInt2 = trunc(YFrac * 100);        // Turn into integer

		  ZSign = (roll < 0) ? "-" : " ";
		  ZVal = (roll < 0) ? -roll : roll;
		  ZInt1 = ZVal;                      // Get the integer
		  ZFrac = ZVal - ZInt1;              // Get fraction
		  ZInt2 = trunc(ZFrac * 100);        // Turn into integer

		  sprintf(dataTX, "yaw=%s%d.%02d, pitch=%s%d.%02d, roll=%s%d.%02d\r\n",
				  XSign, XInt1, XInt2, YSign, YInt1, YInt2, ZSign, ZInt1, ZInt2);

		  //For testing
		  //sprintf(dataTX, "q0 = %d, q1 = %d, q2 = %d, q3 = %d\r\n", (int16_t)(q[0] * 1000), (int16_t)(q[1] * 1000), (int16_t)(q[2] * 1000), (int16_t)(q[3] * 1000));

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
  htim1.Init.Prescaler = 800;
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
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 63;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 1375;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
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

void Increase_Overflow(){
	timer1_Overflow++;
}

void GetSpeed(){
	count1 = (uint32_t)__HAL_TIM_GET_COUNTER(&htim1); // Gets current counter value when interrupt is triggered

	stepCount1 = count1 - countPrev1;
	countPrev1 = count1;
	stepCount1 += (32767 * timer1_Overflow);
	timer1_Overflow = 0;
}

void UpdateMotorSpeed(){
	float freq1_Buff_Copy[5];

	freq1 = ((float)motorCount1 * 10.0); //Period of timer4 interrupt
	lastMotorCount = motorCount1;
	motorCount1 = 0;


	//Take a running average
	freq1_Buff[0] = freq1_Buff[1];
	freq1_Buff[1] = freq1_Buff[2];
	freq1_Buff[2] = freq1_Buff[3];
	freq1_Buff[3] = freq1_Buff[4];
	freq1_Buff[4] = freq1;

	freq1_Buff_Copy[0] = freq1_Buff[0];
	freq1_Buff_Copy[1] = freq1_Buff[1];
	freq1_Buff_Copy[2] = freq1_Buff[2];
	freq1_Buff_Copy[3] = freq1_Buff[3];
	freq1_Buff_Copy[4] = freq1_Buff[4];

	freq1 = median(5, freq1_Buff_Copy);
}

void UpdatePID(){
	pwm_servo1 = ServoMin;
	kp = -1500;
	kp_servo = 15;

	kd = -15000;
	kd_servo = 2;

	if(balancing == 1){
   		if ((PID_sendCount % 16) == 0){
   			YSign = (pitchError < 0) ? "-" : " ";
   			YVal = (pitchError < 0) ? -pitchError : pitchError;
   			YInt1 = YVal;                      // Get the integer
   			YFrac = YVal - YInt1;              // Get fraction
   			YInt2 = trunc(YFrac * 100);        // Turn into integer

   			XSign = (pitchErrorStep < 0) ? "-" : " ";
   			XVal = (pitchErrorStep < 0) ? -pitchErrorStep : pitchErrorStep;
   			XInt1 = XVal;                      // Get the integer
   			XFrac = XVal - XInt1;              // Get fraction
   			XInt2 = trunc(XFrac * 100);        // Turn into integer

   			sprintf(dataTX, "pitch= %s%d.%02d, pitch velocity= %s%d.%02d, motor= %d, servo= %d\r\n",
   					YSign, YInt1, YInt2, XSign, XInt1, XInt2, pwm_esc1, pwm_servo1);
   			HAL_UART_Transmit(&huart1, dataTX, strlen(dataTX), 1000);
   		}
   		PID_sendCount++;

		pitchError = pitch - 45;
		pitchErrorStep = pitchError - pitchErrorPrev;
		pitchErrorPrev = pitchError;

		//Find motor PWM
	   	pwm_esc1 = (pitchError * kp) + (kd * pitchErrorStep);
	   	if (pwm_esc1 < 0){
	   		pwm_esc1 = 0; //Don't let the PWM go negative
	   	}
	   	if (pwm_esc1 > 2500){
	   		pwm_esc1 = 2500; //Max PWM value is 2500
	   	}

	   	//Find servo brake PWM
	   	pwm_servo1 = ServoMin - ((pitchError * kp_servo) + (kd_servo * pitchErrorStep));
	   	if (pwm_servo1 > ServoMin){
	   		pwm_servo1 = ServoMin; //Set highest PWM to servo min
	   	}

	   	if (pwm_servo1 < ServoMax){
	   		pwm_servo1 = ServoMax; //Set lowest PWM to servo max
	   	}

		if ((pitchError > 20) || (pitchError < -20)){
			balancing = 0; //Quit PID loop if cube falls over
			pwm_esc1 = 0;
		}

		__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, pwm_esc1); //Set motor PWM
		//__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, pwm_servo1);  //Set Brake

	}
}

void SendInterruptMessage(){
	if(HAL_I2C_IsDeviceReady(&hi2c1, mpu6050Address, 1, 10) != HAL_OK){
		sprintf(dataTX, "MPU Disconnected\r\n");
		HAL_UART_Transmit(&huart1, dataTX, strlen(dataTX), 1000);
	}
}

// Similar to Madgwick scheme but uses proportional and integral filtering on
// the error between estimated reference vectors and measured ones.
void MahonyQuaternionUpdate(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz, float deltat)
{
  // short name local variable for readability
  float q1 = q[0], q2 = q[1], q3 = q[2], q4 = q[3];
  float norm;
  float hx, hy, bx, bz;
  float vx, vy, vz, wx, wy, wz;
  float ex, ey, ez;
  float pa, pb, pc;

  // Auxiliary variables to avoid repeated arithmetic
  float q1q1 = q1 * q1;
  float q1q2 = q1 * q2;
  float q1q3 = q1 * q3;
  float q1q4 = q1 * q4;
  float q2q2 = q2 * q2;
  float q2q3 = q2 * q3;
  float q2q4 = q2 * q4;
  float q3q3 = q3 * q3;
  float q3q4 = q3 * q4;
  float q4q4 = q4 * q4;

  // Normalise accelerometer measurement
  norm = sqrt(ax * ax + ay * ay + az * az);
  if (norm == 0.0f) return; // Handle NaN
  norm = 1.0f / norm;       // Use reciprocal for division
  ax *= norm;
  ay *= norm;
  az *= norm;

  // Normalise magnetometer measurement
  norm = sqrt(mx * mx + my * my + mz * mz);
  if (norm == 0.0f) return; // Handle NaN
  norm = 1.0f / norm;       // Use reciprocal for division
  mx *= norm;
  my *= norm;
  mz *= norm;

  // Reference direction of Earth's magnetic field
  hx = 2.0f * mx * (0.5f - q3q3 - q4q4) + 2.0f * my * (q2q3 - q1q4) + 2.0f * mz * (q2q4 + q1q3);
  hy = 2.0f * mx * (q2q3 + q1q4) + 2.0f * my * (0.5f - q2q2 - q4q4) + 2.0f * mz * (q3q4 - q1q2);
  bx = sqrt((hx * hx) + (hy * hy));
  bz = 2.0f * mx * (q2q4 - q1q3) + 2.0f * my * (q3q4 + q1q2) + 2.0f * mz * (0.5f - q2q2 - q3q3);

  // Estimated direction of gravity and magnetic field
  vx = 2.0f * (q2q4 - q1q3);
  vy = 2.0f * (q1q2 + q3q4);
  vz = q1q1 - q2q2 - q3q3 + q4q4;
  wx = 2.0f * bx * (0.5f - q3q3 - q4q4) + 2.0f * bz * (q2q4 - q1q3);
  wy = 2.0f * bx * (q2q3 - q1q4) + 2.0f * bz * (q1q2 + q3q4);
  wz = 2.0f * bx * (q1q3 + q2q4) + 2.0f * bz * (0.5f - q2q2 - q3q3);

  // Error is cross product between estimated direction and measured direction of gravity
  ex = (ay * vz - az * vy) + (my * wz - mz * wy);
  ey = (az * vx - ax * vz) + (mz * wx - mx * wz);
  ez = (ax * vy - ay * vx) + (mx * wy - my * wx);
  if (Ki > 0.0f)
  {
    eInt[0] += ex;      // accumulate integral error
    eInt[1] += ey;
    eInt[2] += ez;
  }
  else
  {
    eInt[0] = 0.0f;     // prevent integral wind up
    eInt[1] = 0.0f;
    eInt[2] = 0.0f;
  }

  // Apply feedback terms
  gx = gx + Kp * ex + Ki * eInt[0];
  gy = gy + Kp * ey + Ki * eInt[1];
  gz = gz + Kp * ez + Ki * eInt[2];

  // Integrate rate of change of quaternion
  pa = q2;
  pb = q3;
  pc = q4;
  q1 = q1 + (-q2 * gx - q3 * gy - q4 * gz) * (0.5f * deltat);
  q2 = pa + (q1 * gx + pb * gz - pc * gy) * (0.5f * deltat);
  q3 = pb + (q1 * gy - pa * gz + pc * gx) * (0.5f * deltat);
  q4 = pc + (q1 * gz + pa * gy - pb * gx) * (0.5f * deltat);

  // Normalise quaternion
  norm = sqrt(q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4);
  norm = 1.0f / norm;
  q[0] = q1 * norm;
  q[1] = q2 * norm;
  q[2] = q3 * norm;
  q[3] = q4 * norm;

  //return q;
}


void UpdateGyro(){
	//Get magnetometer info
	  //The magnetometer has a lower refresh rate than the accel & gyro, so the data ready register must be checked
	  i2cBuff[0]= 0x02; //See if mag data is ready
	  HAL_I2C_Master_Transmit(&hi2c1, ak8963Address, i2cBuff, 1, 2);
	  i2cBuff[1] = 0x00;
	  HAL_I2C_Master_Receive(&hi2c1, ak8963Address, &i2cBuff[1], 1, 2);
	  if (i2cBuff[1] & 0x01){
		  //Get mag data
		  i2cBuff[0]= 0x03; //Magnetometer address
		  HAL_I2C_Master_Transmit(&hi2c1, ak8963Address, i2cBuff, 1, 2);
		  i2cBuff[1] = 0x00;
		  HAL_I2C_Master_Receive(&hi2c1, ak8963Address, &i2cBuff[1], 7, 2);
		  //See if magnetometer has overflowed
		  if (!(i2cBuff[7] & 0x08)){
			  mX = (i2cBuff[2]<<8  |  i2cBuff[1]);
			  mY = (i2cBuff[4]<<8  |  i2cBuff[3]);
			  mZ = (i2cBuff[6]<<8  |  i2cBuff[5]);
			  //sprintf(dataTX, "Xmag = %d, Ymag = %d, Zmag = %d\r\n", mx, my, mz);
			  //HAL_UART_Transmit(&huart1, dataTX, strlen(dataTX), 1000);
		  }
	  }

	  //Get gyroscope and accelerometer info
	  i2cBuff[0]= 0x3B; //Accelerometer addresses
	  HAL_I2C_Master_Transmit(&hi2c1, mpu6050Address, i2cBuff, 1, 2);
	  i2cBuff[1] = 0x00;
	  HAL_I2C_Master_Receive(&hi2c1, mpu6050Address, &i2cBuff[1], 14, 2);
	  aX = (i2cBuff[1]<<8 | i2cBuff[2]);
	  aY = (i2cBuff[3]<<8 | i2cBuff[4]);
	  aZ = (i2cBuff[5]<<8 | i2cBuff[6]);
	  gX = (i2cBuff[9]<<8  | i2cBuff[10]);
	  gY = (i2cBuff[11]<<8 | i2cBuff[12]);
	  gZ = (i2cBuff[13]<<8 | i2cBuff[14]);

	  //Subtract offsets
	  gX -=  gxOff;
	  gY -=  gyOff;
	  gZ -=  gzOff;
	  aX -=  axOff;
	  aY -=  ayOff;
	  aZ -=  azOff;
	  mX -=  mxOff;
	  mY -=  myOff;
	  mZ -=  mzOff;

	  //Convert accelerations to g's
	  Xacc = ((float)aX / 32768.0) * 2.0;
	  Yacc = ((float)aY / 32768.0) * 2.0;
	  Zacc = ((float)aZ / 32768.0) * 2.0;

	  //Convert angular velocities to radians per second
	  XangVel = ((float)gX / 32768.0) * 250.0 * (3.141592 / 180.0);
	  YangVel = ((float)gY / 32768.0) * 250.0 * (3.141592 / 180.0);
	  ZangVel = ((float)gZ / 32768.0) * 250.0 * (3.141592 / 180.0);

	  //Convert magnetometer readings to milliGauss
	  Xmag = ((float)mX / 32768.0) * 49120.0 * mxScale;
	  Ymag = ((float)mY / 32768.0) * 49120.0 * myScale;
	  Zmag = ((float)mZ / 32768.0) * 49120.0 * mzScale;

	  //Get the time since the last MPU read
	  /*mainCountCurr = (uint32_t)__HAL_TIM_GET_COUNTER(&htim4); //Using timer 1 for testing
	  mainCount = mainCountCurr - mainCountPrev;
	  if (mainCount > 66000){
		  mainCount = mainCount + 65535; //Make mainCount positive if the timer rolled over
	  }
	  mainCountPrev = mainCountCurr;
	  mainCount = (mainCount * 64) / 8; //Main loop time in us
	  float time = (float) mainCount / 1000000; //Find time for quaternion filter in seconds*/

	  // Sensors x (y)-axis of the accelerometer is aligned with the y (x)-axis of
	  // the magnetometer; the magnetometer z-axis (+ down) is opposite to z-axis
	  // (+ up) of accelerometer and gyro! We have to make some allowance for this
	  // orientation mismatch in feeding the output to the quaternion filter. For the
	  // MPU-9250, we have chosen a magnetic rotation that keeps the sensor forward
	  // along the x-axis just like in the LSM9DS0 sensor. This rotation can be
	  // modified to allow any convenient orientation convention. This is ok by
	  // aircraft orientation standards! Pass gyro rate as rad/s
	  MahonyQuaternionUpdate(Xacc, Yacc, Zacc, XangVel, YangVel, ZangVel, Xmag, Ymag, Zmag, 0.005/*time*/);

	  //Find the Yaw, Pitch, and Roll from the Filter
    //myIMU.yaw   = atan2(2.0f * (*(getQ()+1) * *(getQ()+2) + *getQ()* *(getQ()+3)), *getQ() * *getQ() + *(getQ()+1)* *(getQ()+1) - *(getQ()+2) * *(getQ()+2) - *(getQ()+3)* *(getQ()+3));
    //myIMU.pitch = -asin(2.0f * (*(getQ()+1) * *(getQ()+3) - *getQ()* *(getQ()+2)));
    //myIMU.roll  = atan2(2.0f * (*getQ() * *(getQ()+1) + *(getQ()+2)* *(getQ()+3)), *getQ() * *getQ() - *(getQ()+1)* *(getQ()+1) - *(getQ()+2) * *(getQ()+2) + *(getQ()+3)* *(getQ()+3));

    yaw   = atan2(2.0 * (q[1] * q[2] + q[0] * q[3]), q[0] * q[0] + q[1] * q[1] - q[2] * q[2] - q[3] * q[3]);
    pitch = -asin(2.0 * (q[1] * q[3] - q[0] * q[2]));
    roll  = atan2(2.0 * (q[0] * q[1] + q[2] * q[3]), q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3]);

    //Convert to degrees
    yaw   *= (180.0 / 3.141592);
    pitch *= (180.0 / 3.141592);
    roll  *= (180.0 / 3.141592);

    // Correct yaw for magnetic declination
    yaw -= 3.1;

	  /*//Find angle from direction of gravitational acceleration
	  XaccAng = atan2(Yacc, sqrt((Xacc * Xacc) + (Zacc * Zacc))) * 57.2958;
	  YaccAng = -atan2(Xacc, sqrt((Yacc * Yacc) + (Zacc * Zacc))) * 57.2958;
	  ZaccAng = atan2(Zacc, sqrt((Xacc * Xacc) + (Yacc * Yacc))) * 57.2958;

	  //Find angle from direction of gravitational acceleration using other formula
	  //XaccAng = atan2(Yacc, Zacc) * 57.2958;
	  //YaccAng = atan2(Xacc, Zacc) * 57.2958;
	  //ZaccAng = atan2(Xacc, Yacc) * 57.2958;

	  //Find degrees turned since last read
	  XDegree = XangVel * ((float)mainCount / 1000000); //Using a sampling rate of 10 ms
	  YDegree = YangVel * ((float)mainCount / 1000000); //Main loop seems to take an extra 4 ms
	  ZDegree = ZangVel * ((float)mainCount / 1000000);

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
	  Zang = ((1.0 - alpha) * (Zang + ZDegree)) + (alpha * ZaccAng);*/
}

float median(uint8_t n, float x[]) {
    float temp;
    int i, j;
    // the following two loops sort the array x in ascending order
    for(i=0; i<n-1; i++) {
        for(j=i+1; j<n; j++) {
            if(x[j] < x[i]) {
                // swap elements
                temp = x[i];
                x[i] = x[j];
                x[j] = temp;
            }
        }
    }

    if(n%2==0) {
        // if there is an even number of elements, return mean of the two elements in the middle
        return((x[n/2] + x[n/2 - 1]) / 2.0);
    } else {
        // else return the element in the middle
        return x[n/2];
    }
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
