/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdbool.h>
#include <string.h>
//#include "SparkFun_Qwiic_Scale_NAU7802_Arduino_library.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct TEMPERATURE_DATA{
	int16_t temperature;
}TEMPERATURE_DATA;
typedef struct IMU_DATA{
	int16_t accel_x;
	int16_t accel_y;
	int16_t accel_z;
	int16_t gyro_x;
	int16_t gyro_y;
	int16_t gyro_z;
	int16_t mag_x;
	int16_t mag_y;
	int16_t mag_z;
}IMU_DATA;
typedef struct TILT_DATA{
	int16_t accel_x;
	int16_t accel_y;
	int16_t accel_z;
	int16_t pitch_x;
	int16_t roll_y;
}TILT_DATA;
typedef struct COND_DATA{
	uint16_t result;
}COND_DATA;
typedef struct PRES_DATA{
	uint16_t pressure_result;
	uint16_t temp_result;
}PRES_DATA;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define GENERAL_CALL_ADDR 0
#define CI_Byte 0xAC

/*WAIT TIMES*/
#define TMP117_I2C_DELAY 500
#define MPU9250_I2C_DELAY 500
#define TILT_UART_DELAY 1000
#define ADC_POLL_TIMEOUT 1000

#define TMP117_EEPROM_WRITE_WAIT 7 //ms
/*WAIT TIMES*/

/*TEMPERATURE SENSOR*/
#define TMP117_DEVICE_ID_REG 0x0F

#define TMP117_TEMPERATURE_REG 0x00
#define TMP117_CONFIG_REG 0x01
#define TMP117_EEPROM_UNLOCK_REG 0x04
#define TMP117_EEPROM_2_REG 0x06

#define TMP117_EEPROM_UNLOCK_BIT 15
#define TMP117_EEPROM_BUSY_BIT 14
#define TMP117_RESET_CMD 6
/*TEMPERATURE SENSOR*/

/*IMU SENSOR*/
/*MPU-9250 ADDRESS*/
#define MPU9250_WHO_AM_I 0x75 //default 0x71
#define MPU9250_USER_CTRL 0x6A
#define MPU9250_INT_PIN_CFG 0x37

/*MPU-9250 Accelerometer Data Registers*/
#define MPU9250_ACCEL_XOUT_H 0x3B
#define MPU9250_ACCEL_XOUT_L 0x3C
#define MPU9250_ACCEL_YOUT_H 0x3D
#define MPU9250_ACCEL_YOUT_L 0x3E
#define MPU9250_ACCEL_ZOUT_H 0x3F
#define MPU9250_ACCEL_ZOUT_L 0x40

/*MPU-9250 Temperature Data Registers*/
#define MPU9250_TEMP_OUT_H 0x41
#define MPU9250_TEMP_OUT_L 0x42

/*MPU-9250 Gyro Data Registers*/
#define MPU9250_GYRO_XOUT_H 0x43
#define MPU9250_GYRO_XOUT_L 0x44
#define MPU9250_GYRO_YOUT_H 0x45
#define MPU9250_GYRO_YOUT_L 0x46
#define MPU9250_GYRO_ZOUT_H 0x47
#define MPU9250_GYRO_ZOUT_L 0x48

/*MPU-9250 Magnetometer Data Registers*/
#define MPU9250_MAG_XOUT_L 0x03 //HXL
#define MPU9250_MAG_XOUT_H 0x04 //HXH
#define MPU9250_MAG_YOUT_L 0x05 //HYL
#define MPU9250_MAG_YOUT_H 0x06 //HYH
#define MPU9250_MAG_ZOUT_L 0x07 //HZL
#define MPU9250_MAG_ZOUT_H 0x08 //HZH

#define MPU9250_MAG_STATUS 0x02 //ST1
#define MPU9250_MAG_STATUS2 0x09 //ST2
#define MPU9250_MAG_CNTL 0x0A //CNTL
#define MPU9250_MAG_CNTL2 0x0B

#define ICM20948_INT_PIN_CFG 0x0F

#define ICM20948_MAG_XOUT_L 0x11 //HXL
#define ICM20948_MAG_XOUT_H 0x12 //HXH
#define ICM20948_MAG_YOUT_L 0x13 //HYL
#define ICM20948_MAG_YOUT_H 0x14 //HYH
#define ICM20948_MAG_ZOUT_L 0x15 //HZL
#define ICM20948_MAG_ZOUT_H 0x16 //HZH

#define ICM20948_MAG_STATUS 0x10 //ST1
#define ICM20948_MAG_STATUS2 0x18 //ST2
#define ICM20948_MAG_CNTL2 0x31 //CNTL
#define ICM20948_MAG_CNTL3 0x32
/*IMU SENSOR*/

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

I2C_HandleTypeDef hi2c2;
I2C_HandleTypeDef hi2c3;

RTC_HandleTypeDef hrtc;

TIM_HandleTypeDef htim1;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;



/* USER CODE BEGIN PV */
bool flag = false;
static const uint8_t TMP117_ADDR = (0x48 << 1);
static const uint8_t ICM20948_ADDR = (0x68 << 1);
static const uint8_t AK09916_ADDR = (0x0C << 1);
static const uint8_t PRESSURE_ADDR = (0x40 << 1);

bool load_cell_connected = false;
bool pressure_sensor_connected = false;
bool tilt_sensor_connected = false;
bool imu_connected = false;
bool temperature_sensor_connected = false;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C3_Init(void);
static void MX_RTC_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM1_Init(void);
static void MX_ADC1_Init(void);
static void MX_I2C2_Init(void);
/* USER CODE BEGIN PFP */
bool setup_temperature_sensor(void);
TEMPERATURE_DATA read_temperature_sensor(void);
bool setup_imu_sensor(void);
IMU_DATA read_imu_sensor(void);
bool setup_tilt_sensor(void);
bool setup_load_cell_sensor(void);
TILT_DATA read_tilt_sensor(void);
COND_DATA read_conductivity_sensor(void);
bool setup_pressure_sensor(void);
PRES_DATA read_pressure_sensor(void);
uint16_t read_battery_voltage();
void convert_int32_t_to_three_bytes(int32_t input_value, uint8_t * output_array);
bool assemble_and_send_packet(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint8_t TILT_RX_BUF[50];
char TILT_RX_DATA[50];
int TILT_RX_DATA_INDEX = 0;
volatile bool TILT_RX_FLAG = false;
volatile bool TILT_ERROR_FLAG = false;
uint8_t TILT_CHKSUM = 0;
char TILT_CHKSUM_RETREIVED[2];

TEMPERATURE_DATA temperature_data;
IMU_DATA imu_data;
TILT_DATA tilt_data;
COND_DATA cond_data;
PRES_DATA pres_data;
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
  MX_USART2_UART_Init();
  MX_I2C3_Init();
  MX_RTC_Init();
  MX_USART1_UART_Init();
  MX_TIM1_Init();
  MX_ADC1_Init();
  MX_I2C2_Init();
  /* USER CODE BEGIN 2 */

  /*CRYOEGG 19 WAKEUP CODE*/
  // wakeup functionality
  /* Check and handle if the system was resumed from StandBy mode */
  if(__HAL_PWR_GET_FLAG(PWR_FLAG_SB) != RESET)
  {
	  /* Clear Standby flag */
	  __HAL_PWR_CLEAR_FLAG(PWR_FLAG_SB);
  }
  /* The Following Wakeup sequence is highly recommended prior to each Standby mode entry
   * mainly  when using more than one wakeup source this is to not miss any wakeup event.
   * - Disable all used wakeup sources,
   * - Clear all related wakeup flags,
   * - Re-enable all used wakeup sources,
   * - Enter the Standby mode.
   * */
  /* Disable all used wakeup sources*/
  HAL_RTCEx_DeactivateWakeUpTimer(&hrtc);
  /* Clear all related wakeup flags */
  __HAL_PWR_CLEAR_FLAG(PWR_FLAG_WU);
  /*CRYOEGG 19 WAKEUP CODE*/

  //Enable Sensor 3v3
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, GPIO_PIN_SET);
   HAL_Delay(500);

   load_cell_connected = setup_load_cell_sensor(); // returns true on success
   //tilt_sensor_connected = setup_tilt_sensor();
   tilt_sensor_connected = false; // temporary removal of tilt sensor function as we're not using it and it seems to sometimes cause the micro to hang.
   imu_connected = setup_imu_sensor();
   temperature_sensor_connected = setup_temperature_sensor();
   pressure_sensor_connected = setup_pressure_sensor();


  //Enable Radio Modem
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);
  HAL_Delay(500);
  assemble_and_send_packet();
  //turn on led
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_SET);

  HAL_Delay(500);
  //Disable Sensor 3v3
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, GPIO_PIN_RESET);
  //Disable Radio Modem
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
  //HAL_Delay(2000);

  // set sleep period
  // this varies depending on whether we are in "deployment mode" (after a power-on or hardware reset) or in "long-term mode"

  // RTC backup register 0 counts out our initial "deployment" packets at a high sample rate

  uint32_t deployment_packet_count;

  uint32_t sleep_time;
  // note that it takes 5 seconds to wake up and produce a packet, so all sleep times need to be shortened by 5s.
  #define DEPLOYMENT_PACKET_SLEEP 5 // sleep for 5 seconds between deployment packets (giving a packet every 10s)
  #define DEPLOYMENT_PACKET_TOTAL_COUNT 720 // 720 deployment packets before we go to long-term mode. At one every 10s, that's two hours (6 packets/minute * 60 minutes * 2 hours = 720)

  #define LONG_TERM_SLEEP 595 // 595 seconds sleep + 5s = 600s between packets, which is 1 packet every 10 minutes

  // read values from RTC backup registers
  HAL_PWR_EnableBkUpAccess(); // enable access to the Backup Registers

  deployment_packet_count = HAL_RTCEx_BKUPRead(&hrtc, 0); // get the number of deployment packets transmitted so far

  if (deployment_packet_count < DEPLOYMENT_PACKET_TOTAL_COUNT ) {

	  // we're still in deployment mode
	  deployment_packet_count++;
	  sleep_time = DEPLOYMENT_PACKET_SLEEP;

  } else {

	  // we're in long-term mode
	  sleep_time = LONG_TERM_SLEEP;
  }

  // write back the deployment packet count to the backup register
  HAL_RTCEx_BKUPWrite(&hrtc, 0, deployment_packet_count);



  HAL_StatusTypeDef rtc_ret;
  // configure wakeup timer - set the clock to count in seconds, 0x3C seconds = 60 seconds
  rtc_ret = HAL_RTCEx_SetWakeUpTimer_IT(&hrtc, sleep_time, RTC_WAKEUPCLOCK_CK_SPRE_16BITS);

//  strcpy((char*)buf, "sleep\r\n");
//  HAL_UART_Transmit(&huart2, buf, strlen((char*)buf), HAL_MAX_DELAY);
  /* Enter the Shutdown mode */
  HAL_PWREx_EnterSHUTDOWNMode();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

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
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();
  __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSE;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 10;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */
  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_5;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_2CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.Timing = 0x10909CEC;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c2, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c2, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief I2C3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C3_Init(void)
{

  /* USER CODE BEGIN I2C3_Init 0 */

  /* USER CODE END I2C3_Init 0 */

  /* USER CODE BEGIN I2C3_Init 1 */

  /* USER CODE END I2C3_Init 1 */
  hi2c3.Instance = I2C3;
  hi2c3.Init.Timing = 0x10909CEC;
  hi2c3.Init.OwnAddress1 = 0;
  hi2c3.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c3.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c3.Init.OwnAddress2 = 0;
  hi2c3.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c3.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c3.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c3) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c3, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c3, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C3_Init 2 */

  /* USER CODE END I2C3_Init 2 */

}

/**
  * @brief RTC Initialization Function
  * @param None
  * @retval None
  */
static void MX_RTC_Init(void)
{

  /* USER CODE BEGIN RTC_Init 0 */

  /* USER CODE END RTC_Init 0 */

  /* USER CODE BEGIN RTC_Init 1 */

  /* USER CODE END RTC_Init 1 */
  /** Initialize RTC Only
  */
  hrtc.Instance = RTC;
  hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
  hrtc.Init.AsynchPrediv = 127;
  hrtc.Init.SynchPrediv = 255;
  hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
  hrtc.Init.OutPutRemap = RTC_OUTPUT_REMAP_NONE;
  hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RTC_Init 2 */

  /* USER CODE END RTC_Init 2 */

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
  htim1.Init.Prescaler = 20000;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 6;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 3;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCPolarity = TIM_OCPOLARITY_LOW;
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
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
  sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
  sBreakDeadTimeConfig.Break2Filter = 0;
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
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

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
  huart2.Init.BaudRate = 19200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_HalfDuplex_Init(&huart2) != HAL_OK)
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

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);

  /*Configure GPIO pin : PA1 */
  GPIO_InitStruct.Pin = GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PC5 */
  GPIO_InitStruct.Pin = GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PB0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if (huart->Instance == USART1)
	{


		//"$" is char found at start of sensor data string
		if(TILT_RX_BUF[0] == '$'){
			TILT_RX_DATA_INDEX = 0;
		}

		TILT_RX_DATA[TILT_RX_DATA_INDEX] = TILT_RX_BUF[0];

		//"\n" char is found at end of sensor data string
		if(TILT_RX_BUF[0] == '\n'){
			TILT_RX_DATA_INDEX = 0;
			TILT_RX_FLAG = true;
		}else{
			//Checksum is calculated by xoring all characters between "$" and "*" ("*" is at index 43)
			if(TILT_RX_DATA_INDEX == 1){
				TILT_CHKSUM = TILT_RX_DATA[TILT_RX_DATA_INDEX];
			}else if(TILT_RX_DATA_INDEX < 43){
				TILT_CHKSUM ^= TILT_RX_DATA[TILT_RX_DATA_INDEX];
			}

			//Checksum is hex byte transmitted by sensor as characters, values are found at indexes 44 45
			if(TILT_RX_DATA_INDEX == 44){
				TILT_CHKSUM_RETREIVED[0] = TILT_RX_DATA[TILT_RX_DATA_INDEX];
			}else if(TILT_RX_DATA_INDEX == 45){
				TILT_CHKSUM_RETREIVED[1] = TILT_RX_DATA[TILT_RX_DATA_INDEX];
			}

			//Infinite loop mitigation
			//If index is past the 48 characters expected, set flag to true to exit while loop in read_tilt_sensor()
			if(TILT_RX_DATA_INDEX + 1 > 47){
				TILT_RX_FLAG = true;
			}
			TILT_RX_DATA_INDEX = (TILT_RX_DATA_INDEX + 1) > 47? 0 : TILT_RX_DATA_INDEX + 1;
			HAL_UART_Receive_IT(&huart1, TILT_RX_BUF, 1);
		}

	}
}
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart){
	if (huart->Instance == USART1)
	{
		//if error set rx flag to true to exit while loop
		TILT_RX_FLAG = true;
		TILT_ERROR_FLAG = true;
	}
}

/**
  * @brief Tilt Sensor Helper Character to int16_t Function
  * @note	Function used to convert characters in string received
  * 		by TILT-05 sensor to int.
  * @param start Start index of section in TILT_RX_DATA string to be converted (int)
  * @param unit_size Size of character section in TILT_RX_DATA string (int)
  * @retval int16_t Character section converted to int16_t
  */
int16_t tilt_helper_char_to_int(int start, int size, int unit_size){
	int16_t temp = 0;
	for(int i=start + 1; i<size + start; i++){
		if(TILT_RX_DATA[i] == '.'){
			//skip decimal dot
		}else{
			temp += (TILT_RX_DATA[i] - '0') * unit_size;
			unit_size /= 10;
		}
	}
	temp = TILT_RX_DATA[start] == '-'? temp * (-1) : temp;
	return temp;
}
uint8_t hexchar2int(char ch)
{
    if (ch >= '0' && ch <= '9')
        return ch - '0';
    if (ch >= 'A' && ch <= 'F')
        return ch - 'A' + 10;
    if (ch >= 'a' && ch <= 'f')
        return ch - 'a' + 10;
    return 16;
}
/**
  * @brief Setup Temperature Sensor Function
  * @note Function used to set TMP117 to shutdown mode on startup.
  * @param None
  * @retval bool Status
  */
bool setup_temperature_sensor(void){
	//1.5ms wait after startup
	HAL_Delay(50);
	uint8_t cmd_buf[3];
	uint8_t rec_buf[2];

	HAL_StatusTypeDef HAL_temperature_ret;

	//Fill pointer register with device id reg address
	uint16_t id_reg = 0;
	cmd_buf[0] = TMP117_DEVICE_ID_REG;
	HAL_temperature_ret = HAL_I2C_Master_Transmit(&hi2c3, TMP117_ADDR, cmd_buf, 1, TMP117_I2C_DELAY);
	//Read device id register
	HAL_temperature_ret = HAL_I2C_Master_Receive(&hi2c3, TMP117_ADDR, rec_buf, 2, TMP117_I2C_DELAY);
	id_reg = ((uint16_t)rec_buf[0] << 8 | rec_buf[1]);

	//default id reg 279 base10

	uint16_t config_reg = 0;

	//Read config register
	cmd_buf[0] = TMP117_CONFIG_REG;
	HAL_temperature_ret = HAL_I2C_Master_Transmit(&hi2c3, TMP117_ADDR, cmd_buf, 1, TMP117_I2C_DELAY);
	HAL_temperature_ret = HAL_I2C_Master_Receive(&hi2c3, TMP117_ADDR, rec_buf, 2, TMP117_I2C_DELAY);
	config_reg = ((uint16_t)rec_buf[0] << 8 | rec_buf[1]);

	//Set bit [10, 11] so that sensor can go from continuous -> shutdown mode
	config_reg |= 0x0400;
	cmd_buf[0] = TMP117_CONFIG_REG;
	cmd_buf[1] = config_reg >> 8;
	cmd_buf[2] = config_reg & 0x00FF;
	HAL_temperature_ret = HAL_I2C_Master_Transmit(&hi2c3, TMP117_ADDR, cmd_buf, 3, TMP117_I2C_DELAY);

	if(HAL_temperature_ret == HAL_OK && id_reg == 279){
		return true;
	}
	return false;

}
/**
  * @brief Read Temperature Sensor Function
  * @note	Function used to read raw temperature from TMP117.
  * @param None
  * @retval TEMPERATURE_DATA Raw sensor data
  */
TEMPERATURE_DATA read_temperature_sensor(void){
	TEMPERATURE_DATA data;
	uint8_t cmd_buf[3];
	uint8_t rec_buf[2];

	uint16_t config_reg = 0;

	//Read config register
	cmd_buf[0] = TMP117_CONFIG_REG;
	HAL_I2C_Master_Transmit(&hi2c3, TMP117_ADDR, cmd_buf, 1, TMP117_I2C_DELAY);
	HAL_I2C_Master_Receive(&hi2c3, TMP117_ADDR, rec_buf, 2, TMP117_I2C_DELAY);
	config_reg = ((uint16_t)rec_buf[0] << 8 | rec_buf[1]);

	//Set bit [10, 11] so that sensor can perform one-shot conversion
	config_reg |= 0x0C00;
	cmd_buf[0] = TMP117_CONFIG_REG;
	cmd_buf[1] = config_reg >> 8;
	cmd_buf[2] = config_reg & 0x00FF;
	HAL_I2C_Master_Transmit(&hi2c3, TMP117_ADDR, cmd_buf, 3, TMP117_I2C_DELAY);

	//Wait for conversion to finish
	HAL_Delay(1000);

	//Temperature register is where temperature value is stored
	cmd_buf[0] = TMP117_TEMPERATURE_REG;
	HAL_I2C_Master_Transmit(&hi2c3, TMP117_ADDR, cmd_buf, 1, TMP117_I2C_DELAY);
	HAL_I2C_Master_Receive(&hi2c3, TMP117_ADDR, rec_buf, 2, TMP117_I2C_DELAY);
	data.temperature = ((uint16_t)rec_buf[0] << 8 | rec_buf[1]);

	return data;
}
/**
  * @brief Setup IMU Sensor Function
  * @note	Function used to setup MPU9250.
  * @param None
  * @retval bool Status
  */
bool setup_imu_sensor(void){
	HAL_StatusTypeDef HAL_imu_ret;
	uint8_t cmd_buf[2];
	uint8_t rec_buf[2];

	/*MAGNETOMETER DATA REGISTER READS ---- START*/

	uint8_t mag_id = 0;
	volatile uint8_t mag_status = 0;
	uint8_t mag_status2 = 0;
	uint8_t mag_control = 0;
	bool mag_setup_status = false;

	uint8_t icm20948_pwr_mgt_2 = 0;
	//pwr mgt 2
	cmd_buf[0] = 0x07;
	HAL_imu_ret = HAL_I2C_Master_Transmit(&hi2c3, ICM20948_ADDR, cmd_buf, 1, MPU9250_I2C_DELAY);
	HAL_imu_ret = HAL_I2C_Master_Receive(&hi2c3, ICM20948_ADDR, rec_buf, 1, MPU9250_I2C_DELAY);
	icm20948_pwr_mgt_2 = rec_buf[0];

	//turn off gyro and accel
	cmd_buf[0] = 0x07;
	cmd_buf[1] = icm20948_pwr_mgt_2 | 0x3F;
	HAL_imu_ret = HAL_I2C_Master_Transmit(&hi2c3, ICM20948_ADDR, cmd_buf, 2, MPU9250_I2C_DELAY);

	uint8_t icm20948_pwr_mgt_1 = 0;
	//pwr mgt 1
	cmd_buf[0] = 0x06;
	HAL_imu_ret = HAL_I2C_Master_Transmit(&hi2c3, ICM20948_ADDR, cmd_buf, 1, MPU9250_I2C_DELAY);
	HAL_imu_ret = HAL_I2C_Master_Receive(&hi2c3, ICM20948_ADDR, rec_buf, 1, MPU9250_I2C_DELAY);
	icm20948_pwr_mgt_1 = rec_buf[0];

	//turn on sensor by clearing sleep bit
	cmd_buf[0] = 0x06;
	cmd_buf[1] = icm20948_pwr_mgt_1 & 0xBF;
	HAL_imu_ret = HAL_I2C_Master_Transmit(&hi2c3, ICM20948_ADDR, cmd_buf, 2, MPU9250_I2C_DELAY);

	uint8_t icm20948_int_pin_cfg = 0;
	//Read int_pin_cfg register on icm20948
	cmd_buf[0] = ICM20948_INT_PIN_CFG;
	HAL_imu_ret = HAL_I2C_Master_Transmit(&hi2c3, ICM20948_ADDR, cmd_buf, 1, MPU9250_I2C_DELAY);
	HAL_imu_ret = HAL_I2C_Master_Receive(&hi2c3, ICM20948_ADDR, rec_buf, 1, MPU9250_I2C_DELAY);

	icm20948_int_pin_cfg = rec_buf[0];

	//Set BYPASS bit to high to allow I2C communication from mcu to magnetometer
	cmd_buf[0] = ICM20948_INT_PIN_CFG;
	cmd_buf[1] = icm20948_int_pin_cfg | 0x02;
	HAL_imu_ret = HAL_I2C_Master_Transmit(&hi2c3, ICM20948_ADDR, cmd_buf, 2, MPU9250_I2C_DELAY);

	//Read Device ID, default 0x09
	cmd_buf[0] = 0x01;
	HAL_imu_ret = HAL_I2C_Master_Transmit(&hi2c3, AK09916_ADDR, cmd_buf, 1, MPU9250_I2C_DELAY);
	HAL_imu_ret = HAL_I2C_Master_Receive(&hi2c3, AK09916_ADDR, rec_buf, 1, MPU9250_I2C_DELAY);
	mag_id = rec_buf[0];

	//Read CNTL2 register
	cmd_buf[0] = ICM20948_MAG_CNTL2;
	HAL_imu_ret = HAL_I2C_Master_Transmit(&hi2c3, AK09916_ADDR, cmd_buf, 1, MPU9250_I2C_DELAY);
	HAL_imu_ret = HAL_I2C_Master_Receive(&hi2c3, AK09916_ADDR, rec_buf, 1, MPU9250_I2C_DELAY);
	mag_control = rec_buf[0];

	//Wait at least 100microseconds before switching to single measurement mode
	HAL_Delay(1);

	//Set power down mode (xxxx0000 == power down mode)
	cmd_buf[0] = ICM20948_MAG_CNTL2;
	cmd_buf[1] = (mag_control & 0x0) & 0xE;
	HAL_imu_ret = HAL_I2C_Master_Transmit(&hi2c3, AK09916_ADDR, cmd_buf, 2, MPU9250_I2C_DELAY);

	if(mag_id == 0x09 && HAL_imu_ret == HAL_OK){
		mag_setup_status = true;
	}

	/*GYROSCOPE DATA REGISTER READS ---- START*/

	/*
	bool gyro_setup_status = false;

	HAL_Delay(35); //35ms cold start gyro

	cmd_buf[0] = MPU9250_GYRO_XOUT_H;
	HAL_imu_ret = HAL_I2C_Master_Transmit(&hi2c1, MPU9250_ADDR, cmd_buf, 1, MPU9250_I2C_DELAY);
	HAL_imu_ret = HAL_I2C_Master_Receive(&hi2c1, MPU9250_ADDR, rec_buf, 2, MPU9250_I2C_DELAY);

	if(HAL_imu_ret == HAL_OK){
		gyro_setup_status = true;
	}
	*/

	/*ACCELEROMETER DATA REGISTER READS ---- START*/

	/*
	bool accel_setup_status = false;

	HAL_Delay(30); //30ms cold start accel

	cmd_buf[0] = MPU9250_ACCEL_XOUT_H;
	HAL_imu_ret = HAL_I2C_Master_Transmit(&hi2c1, MPU9250_ADDR, cmd_buf, 1, MPU9250_I2C_DELAY);
	HAL_imu_ret = HAL_I2C_Master_Receive(&hi2c1, MPU9250_ADDR, rec_buf, 2, MPU9250_I2C_DELAY);

	if(HAL_imu_ret == HAL_OK){
		accel_setup_status = true;
	}
	*/

	//if(mag_setup_status && gyro_setup_status && accel_setup_status){
	if(mag_setup_status){
		return true;
	}
	return false;
}
/**
  * @brief Read IMU Sensor Function
  * @note	Function used to read raw gyro, accelerometer, magnetometer values from MPU9250.
  * @param None
  * @retval IMU_DATA Raw sensor data
  */
IMU_DATA read_imu_sensor(void){
	IMU_DATA data;

	HAL_StatusTypeDef HAL_imu_ret;
	uint8_t cmd_buf[2];
	uint8_t rec_buf[2];

	/*MAGNETOMETER DATA REGISTER READS ---- START*/

	int16_t mag_read_x = 0;
	int16_t mag_read_y = 0;
	int16_t mag_read_z = 0;
	uint8_t mag_id = 0;
	volatile uint8_t mag_status = 0;
	uint8_t mag_status2 = 0;
	uint8_t mag_control = 0;

	//Read CNTL1 register
	cmd_buf[0] = ICM20948_MAG_CNTL2;
	HAL_imu_ret = HAL_I2C_Master_Transmit(&hi2c3, AK09916_ADDR, cmd_buf, 1, MPU9250_I2C_DELAY);
	HAL_imu_ret = HAL_I2C_Master_Receive(&hi2c3, AK09916_ADDR, rec_buf, 1, MPU9250_I2C_DELAY);
	mag_control = rec_buf[0];

	//Set single measurement mode (xxxx0001 == single measurement mode)
	cmd_buf[0] = ICM20948_MAG_CNTL2;
	cmd_buf[1] = (mag_control & 0x0) | 0x1;
	HAL_imu_ret = HAL_I2C_Master_Transmit(&hi2c3, AK09916_ADDR, cmd_buf, 2, MPU9250_I2C_DELAY);

	HAL_Delay(100); // short delay to ensure we have time for IMU to catch up

	//Read CNTL1 register
	cmd_buf[0] = ICM20948_MAG_CNTL2;
	HAL_imu_ret = HAL_I2C_Master_Transmit(&hi2c3, AK09916_ADDR, cmd_buf, 1, MPU9250_I2C_DELAY);
	HAL_imu_ret = HAL_I2C_Master_Receive(&hi2c3, AK09916_ADDR, rec_buf, 1, MPU9250_I2C_DELAY);
	mag_control = rec_buf[0];


	cmd_buf[0] = ICM20948_MAG_STATUS;
	HAL_imu_ret = HAL_I2C_Master_Transmit(&hi2c3, AK09916_ADDR, cmd_buf, 1, MPU9250_I2C_DELAY);
	HAL_imu_ret = HAL_I2C_Master_Receive(&hi2c3, AK09916_ADDR, rec_buf, 1, MPU9250_I2C_DELAY);
	mag_status = rec_buf[0];

	if(mag_status & 0x1){
		//Data ready
		//otherwise data has already been read

		//LittleEndian
		cmd_buf[0] = ICM20948_MAG_XOUT_L;
		HAL_imu_ret = HAL_I2C_Master_Transmit(&hi2c3, AK09916_ADDR, cmd_buf, 1, MPU9250_I2C_DELAY);
		HAL_imu_ret = HAL_I2C_Master_Receive(&hi2c3, AK09916_ADDR, rec_buf, 2, MPU9250_I2C_DELAY);
		mag_read_x = (rec_buf[1] << 8) | rec_buf[0];

		cmd_buf[0] = ICM20948_MAG_YOUT_L;
		HAL_imu_ret = HAL_I2C_Master_Transmit(&hi2c3, AK09916_ADDR, cmd_buf, 1, MPU9250_I2C_DELAY);
		HAL_imu_ret = HAL_I2C_Master_Receive(&hi2c3, AK09916_ADDR, rec_buf, 2, MPU9250_I2C_DELAY);
		mag_read_y = (rec_buf[1] << 8) | rec_buf[0];

		cmd_buf[0] = ICM20948_MAG_ZOUT_L;
		HAL_imu_ret = HAL_I2C_Master_Transmit(&hi2c3, AK09916_ADDR, cmd_buf, 1, MPU9250_I2C_DELAY);
		HAL_imu_ret = HAL_I2C_Master_Receive(&hi2c3, AK09916_ADDR, rec_buf, 2, MPU9250_I2C_DELAY);
		mag_read_z = (rec_buf[1] << 8) | rec_buf[0];
	}

	//Read Status Register 2 (ST2) to signal end of read
	cmd_buf[0] = ICM20948_MAG_STATUS2;
	HAL_imu_ret = HAL_I2C_Master_Transmit(&hi2c3, AK09916_ADDR, cmd_buf, 1, MPU9250_I2C_DELAY);
	HAL_imu_ret = HAL_I2C_Master_Receive(&hi2c3, AK09916_ADDR, rec_buf, 1, MPU9250_I2C_DELAY);
	mag_status2 = rec_buf[0];

	data.mag_x = mag_read_x;
	data.mag_y = mag_read_y;
	data.mag_z = mag_read_z;

	/*MAGNETOMETER DATA REGISTER READS ---- END*/

	/*GYROSCOPE DATA REGISTER READS ---- START*/

	data.gyro_x = 0;
	data.gyro_y = 0;
	data.gyro_z = 0;
	/*
	int16_t gyro_read_x = 0;
	int16_t gyro_read_y = 0;
	int16_t gyro_read_z = 0;

	//BigEndian
	cmd_buf[0] = MPU9250_GYRO_XOUT_H;
	HAL_imu_ret = HAL_I2C_Master_Transmit(&hi2c3, MPU9250_ADDR, cmd_buf, 1, MPU9250_I2C_DELAY);
	HAL_imu_ret = HAL_I2C_Master_Receive(&hi2c3, MPU9250_ADDR, rec_buf, 2, MPU9250_I2C_DELAY);
	gyro_read_x = (rec_buf[0] << 8) | rec_buf[1];

	cmd_buf[0] = MPU9250_GYRO_YOUT_H;
	HAL_imu_ret = HAL_I2C_Master_Transmit(&hi2c3, MPU9250_ADDR, cmd_buf, 1, MPU9250_I2C_DELAY);
	HAL_imu_ret = HAL_I2C_Master_Receive(&hi2c3, MPU9250_ADDR, rec_buf, 2, MPU9250_I2C_DELAY);
	gyro_read_y = (rec_buf[0] << 8) | rec_buf[1];

	cmd_buf[0] = MPU9250_GYRO_ZOUT_H;
	HAL_imu_ret = HAL_I2C_Master_Transmit(&hi2c3, MPU9250_ADDR, cmd_buf, 1, MPU9250_I2C_DELAY);
	HAL_imu_ret = HAL_I2C_Master_Receive(&hi2c3, MPU9250_ADDR, rec_buf, 2, MPU9250_I2C_DELAY);
	gyro_read_z = (rec_buf[0] << 8) | rec_buf[1];

	data.gyro_x = gyro_read_x;
	data.gyro_y = gyro_read_y;
	data.gyro_z = gyro_read_z;

	*/
	/*GYROSCOPE DATA REGISTER READS ---- END*/

	/*ACCELEROMETER DATA REGISTER READS ---- START*/

	data.accel_x = 0;
	data.accel_y = 0;
	data.accel_z = 0;
	/*
	int16_t accel_read_x = 0;
	int16_t accel_read_y = 0;
	int16_t accel_read_z = 0;

	cmd_buf[0] = MPU9250_ACCEL_XOUT_H;
	HAL_imu_ret = HAL_I2C_Master_Transmit(&hi2c3, MPU9250_ADDR, cmd_buf, 1, MPU9250_I2C_DELAY);
	HAL_imu_ret = HAL_I2C_Master_Receive(&hi2c3, MPU9250_ADDR, rec_buf, 2, MPU9250_I2C_DELAY);
	accel_read_x = (rec_buf[0] << 8) | rec_buf[1];

	cmd_buf[0] = MPU9250_ACCEL_YOUT_H;
	HAL_imu_ret = HAL_I2C_Master_Transmit(&hi2c3, MPU9250_ADDR, cmd_buf, 1, MPU9250_I2C_DELAY);
	HAL_imu_ret = HAL_I2C_Master_Receive(&hi2c3, MPU9250_ADDR, rec_buf, 2, MPU9250_I2C_DELAY);
	accel_read_y = (rec_buf[0] << 8) | rec_buf[1];

	cmd_buf[0] = MPU9250_ACCEL_ZOUT_H;
	HAL_imu_ret = HAL_I2C_Master_Transmit(&hi2c3, MPU9250_ADDR, cmd_buf, 1, MPU9250_I2C_DELAY);
	HAL_imu_ret = HAL_I2C_Master_Receive(&hi2c3, MPU9250_ADDR, rec_buf, 2, MPU9250_I2C_DELAY);
	accel_read_z = (rec_buf[0] << 8) | rec_buf[1];

	data.accel_x = accel_read_x;
	data.accel_y = accel_read_y;
	data.accel_z = accel_read_z;
	*/

	/*ACCELEROMETER DATA REGISTER READS ---- END*/

	return data;
}
/**
  * @brief Setup Tilt Sensor Function
  * @note	Function used to setup TILT-05 sensor.
  * @param None
  * @retval bool Status
  */
bool setup_tilt_sensor(void){
	HAL_StatusTypeDef ret_status;
	char cmd_buf[10];

	//This command turns off continuous inclinometer data output
	strcpy((char*)cmd_buf, "[1MIX\r");
	ret_status = HAL_UART_Transmit(&huart1, cmd_buf, strlen((char*)cmd_buf), TILT_UART_DELAY);
	HAL_Delay(1000);
	//This command turns off continuous accelerometer data output
	strcpy((char*)cmd_buf, "[1MAX\r");
	ret_status = HAL_UART_Transmit(&huart1, cmd_buf, strlen((char*)cmd_buf), TILT_UART_DELAY);
	HAL_Delay(1000);

	//as soon as tilt sensor is powered on, uart gives overrun error, tilt sensor possibly sends burst of data on start up
	//this will clear overrun flag so that uart can be used to receive valid data during read sequence
	__HAL_UART_CLEAR_FLAG(&huart1, UART_CLEAR_OREF);

	// try and read data from the sensor
	// if the sensor is not connected, HAL_UART_ErrorCallback will set TILT_ERROR_FLAG to be true
	// reading the sensor here also ensures that the data is fresh.

	read_tilt_sensor();

	if (TILT_ERROR_FLAG == true) {
		return false; // we have a comms error
	}

	if(ret_status == HAL_OK){
		return true;
	}
	return false;
}
/**
  * @brief Read Tilt Sensor Function
  * @note	Function used to read raw accelerometer, inclinometer values from TILT-05 sensor.
  * @param None
  * @retval TILT_DATA Raw sensor data
  */
TILT_DATA read_tilt_sensor(void){
	TILT_DATA data;
	HAL_StatusTypeDef ret_status;
	char cmd_buf[10];

	data.accel_x = 0;
	data.accel_y = 0;
	data.accel_z = 0;
	data.pitch_x = 0;
	data.roll_y = 0;

	//This command turns on single inclinometer data output
	//This out put is based on last sensor position, must call twice to get real-time data
	strcpy((char*)cmd_buf, "[1MIS\r");
	ret_status = HAL_UART_Transmit(&huart1, cmd_buf, strlen((char*)cmd_buf), TILT_UART_DELAY);

	TILT_RX_FLAG = false;

	ret_status = HAL_UART_Receive_IT(&huart1, TILT_RX_BUF, 1);


	//Wait for flag to be set by interrupt receive completing
	while(!TILT_RX_FLAG){
	}

	//check data validity
	uint8_t chksum = 0;
	uint8_t top_4bit_of_byte = hexchar2int(TILT_CHKSUM_RETREIVED[0]);
	uint8_t bot_4bit_of_byte = hexchar2int(TILT_CHKSUM_RETREIVED[1]);
	if(top_4bit_of_byte >= 16 || bot_4bit_of_byte >= 16){
		return data;
	}
	chksum = top_4bit_of_byte << 4 | bot_4bit_of_byte;

	if(chksum != TILT_CHKSUM){
		return data;
	}

	data.accel_x = tilt_helper_char_to_int(7,5,1000);
	data.accel_y = tilt_helper_char_to_int(13,5,1000);
	data.accel_z = tilt_helper_char_to_int(19,5,1000);
	data.pitch_x = tilt_helper_char_to_int(25,6,1000);
	data.roll_y = tilt_helper_char_to_int(32,6,1000);


	return data;
}
/**
  * @brief Read Conductivity Sensor Function
  * @note	Function used to read raw adc values conductivity sensor.
  * @param None
  * @retval COND_DATA Raw sensor data
  */
COND_DATA read_conductivity_sensor(void){
	COND_DATA data;

	data.result = 0;

	// configure ADC to the correct channel
	ADC_ChannelConfTypeDef sConfig = {0};

	sConfig.Channel = ADC_CHANNEL_5;
	sConfig.Rank = ADC_REGULAR_RANK_1;
	sConfig.SamplingTime = ADC_SAMPLETIME_2CYCLES_5;
	sConfig.SingleDiff = ADC_SINGLE_ENDED;
	sConfig.OffsetNumber = ADC_OFFSET_NONE;
	sConfig.Offset = 0;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
	{
		Error_Handler();
	}

	//Turn on PWM square waves
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);

	HAL_Delay(1000);
	// start conversion
	HAL_ADC_Start(&hadc1);

	// poll ADC until conversion completes - note that we should check for the success of the conversion here really
	HAL_ADC_PollForConversion(&hadc1, ADC_POLL_TIMEOUT);

	// get result
	data.result = HAL_ADC_GetValue(&hadc1);

	// switch off ADC
	HAL_ADC_Stop(&hadc1);

	//Turn off PWM square waves
	HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
	HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_4);

	return data;
}

bool setup_pressure_sensor(void) {

	HAL_StatusTypeDef status = HAL_BUSY;

	uint32_t trials = 2; // number of tries to connect before we give up
	uint32_t timeout = 100; // number of millisconds to wait before we give up

	status = HAL_I2C_IsDeviceReady(&hi2c2, PRESSURE_ADDR, trials, timeout);

	if (status != HAL_OK) {
		return false;
	} else {
		return true;
	}

	return false;
}


/**
  * @brief Read Pressure Sensor Function
  * @note	Function used to read raw keller pressure sensor values.
  * @param None
  * @retval PRES_DATA Raw sensor data
  */
PRES_DATA read_pressure_sensor(void){
	PRES_DATA data;

	data.pressure_result = 0;

	uint8_t command_byte = 0xAC; // request measurement command
	uint8_t pressure_received_bytes[3]; // array to store 5 received bytes from sensor
	bool pressure_okay = false;
	// send conversion request
	HAL_I2C_Master_Transmit(&hi2c2, PRESSURE_ADDR, &command_byte, sizeof(command_byte), 500 ); // 500ms timeout
	// wait 10ms - 8ms minimum required for result to be ready
	HAL_Delay(10);
	// read back results
	HAL_I2C_Master_Receive(&hi2c2, PRESSURE_ADDR, pressure_received_bytes, sizeof(pressure_received_bytes), 500); // 500ms timeout
	// check pressure reading was okay - status byte should be 0x40 if everything is operating normally
	if (pressure_received_bytes[0] == 0x40) {
		pressure_okay = true;
	}

	uint16_t raw_result = pressure_received_bytes[1];
	if (pressure_okay){
		data.pressure_result = (raw_result << 8) | pressure_received_bytes[2];
	} else {
		data.pressure_result = 0;
	}

	return data;
}

bool setup_load_cell_sensor(void) {

	bool begin_ok = false;

	begin_ok = NAU7802_begin(&hi2c2, true); // initialise sensor



	return begin_ok;
}


// function to turn load cell readings into three bytes rather than four
// we're taking a 32 bit signed value and making a 24 bit signed value but represented as three unsigned bytes
// output_arrray a pointer to a 3-byte array of uint8_t
void convert_int32_t_to_three_bytes(int32_t input_value, uint8_t * output_array) {

	*output_array = 0; // MSB
	*(output_array + 1) = 0; // middle byte
	*(output_array + 2) = 0; // LSB

	*output_array = (uint8_t)((uint32_t)(input_value & 0xFF0000) >> 16); // MSB magnitude
	*output_array = (*output_array) | ((uint32_t)(input_value & 0x8000000) >> 24); // MSB sign
	*(output_array + 1) = (uint8_t)((uint32_t)(input_value & 0xFF00) >> 8); // middle byte
	*(output_array + 2) = (uint8_t)(input_value & 0xFF); // LSB

	return;
}

// function to read battery voltage using internal voltage reference on ADC
uint16_t read_battery_voltage() {

	uint16_t battery_millivolts = 0; // return value is in integer mV, so 3.6V = 3600
	uint16_t vref_adc_val = 0; // this is the measurement of VREF from the ADC

	ADC_ChannelConfTypeDef sConfig = {0};
	// reconfigure the ADC to read the internal voltage reference
	sConfig.Channel = ADC_CHANNEL_VREFINT;
	sConfig.Rank = ADC_REGULAR_RANK_1;
	sConfig.SamplingTime = ADC_SAMPLETIME_640CYCLES_5;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
	{
		Error_Handler();
	}

	// start ADC conversion
	HAL_ADC_Start(&hadc1);

	// poll ADC until conversion completes - note that we should check for the success of the conversion here really
	HAL_ADC_PollForConversion(&hadc1, ADC_POLL_TIMEOUT);

	// get result
	vref_adc_val = HAL_ADC_GetValue(&hadc1);

	// switch off ADC
	HAL_ADC_Stop(&hadc1);

	// convert the ADC value to the supply voltage (VDDA/VREF+), which on Cryowurst is the raw battery voltage
	// we use this handy macro from the HAL

	battery_millivolts = __HAL_ADC_CALC_VREFANALOG_VOLTAGE ( vref_adc_val,  ADC_RESOLUTION_12B );




	return battery_millivolts;
}


/**
  * @brief Assemble and send packet Function
  * @note	Function used to read peripheral sensor data (TILT-05, MPU9250, TMP117)
  * 		then send assembled data packet of raw sensor values to Radio modem.
  * @param None
  * @retval bool Status
  */
bool assemble_and_send_packet(void){
	HAL_StatusTypeDef radio_ret;

	temperature_data = read_temperature_sensor();

	imu_data = read_imu_sensor();

	if (tilt_sensor_connected) {
		tilt_data = read_tilt_sensor();
	}

	cond_data = read_conductivity_sensor();

	if (pressure_sensor_connected == true) {
		pres_data = read_pressure_sensor();
	}
	uint16_t battery_voltage;
	battery_voltage = read_battery_voltage();

	int32_t load_cell_ch1_result = 0;
	int32_t load_cell_ch2_result = 0;

	if (load_cell_connected == true) {
		NAU7802_setChannel(NAU7802_CHANNEL_1); // make sure we have Channel 1 selected
		NAU7802_calibrateAFE(); // recalibrate after channel change

		load_cell_ch1_result = NAU7802_getReading();
		// change to channel 2
		NAU7802_setChannel(NAU7802_CHANNEL_2);
		NAU7802_calibrateAFE(); // recalibrate after channel change
		load_cell_ch2_result = NAU7802_getReading();
	}

	uint8_t load_cell_ch1_bytes[3];
	uint8_t load_cell_ch2_bytes[3];

	convert_int32_t_to_three_bytes(load_cell_ch1_result, load_cell_ch1_bytes);
	convert_int32_t_to_three_bytes(load_cell_ch2_result, load_cell_ch2_bytes);

	// sequence number

	HAL_PWR_EnableBkUpAccess(); // enable access to the Backup Registers
	uint8_t sequence_number;
	sequence_number = HAL_RTCEx_BKUPRead(&hrtc, 2); // read Backup Register 2

	uint8_t packet[33];

	packet[0] = sizeof(packet)-1; // number of bytes to follow
	packet[1] = CI_Byte;

	/*TEMPERATURE PACKET*/
	packet[2] = temperature_data.temperature >> 8;
	packet[3] = temperature_data.temperature & 0x00FF;

	/*IMU PACKET*/
	/*
	packet[4] = imu_data.accel_x >> 8;
	packet[5] = imu_data.accel_x & 0x00FF;
	packet[6] = imu_data.accel_y >> 8;
	packet[7] = imu_data.accel_y & 0x00FF;
	packet[8] = imu_data.accel_z >> 8;
	packet[9] = imu_data.accel_z & 0x00FF;

	packet[10] = imu_data.gyro_x >> 8;
	packet[11] = imu_data.gyro_x & 0x00FF;
	packet[12] = imu_data.gyro_y >> 8;
	packet[13] = imu_data.gyro_y & 0x00FF;
	packet[14] = imu_data.gyro_z >> 8;
	packet[15] = imu_data.gyro_z & 0x00FF;
	*/
	packet[4] = imu_data.mag_x >> 8;
	packet[5] = imu_data.mag_x & 0x00FF;
	packet[6] = imu_data.mag_y >> 8;
	packet[7] = imu_data.mag_y & 0x00FF;
	packet[8] = imu_data.mag_z >> 8;
	packet[9] = imu_data.mag_z & 0x00FF;

	/*TILT PACKET*/
	packet[10] = tilt_data.accel_x >> 8;
	packet[11] = tilt_data.accel_x & 0x00FF;
	packet[12] = tilt_data.accel_y >> 8;
	packet[13] = tilt_data.accel_y & 0x00FF;
	packet[14] = tilt_data.accel_z >> 8;
	packet[15] = tilt_data.accel_z & 0x00FF;

	packet[16] = tilt_data.pitch_x >> 8;
	packet[17] = tilt_data.pitch_x & 0x00FF;
	packet[18] = tilt_data.roll_y >> 8;
	packet[19] = tilt_data.roll_y & 0x00FF;

	/*Conductivity PACKET */
	packet[20] = cond_data.result >> 8;
	packet[21] = cond_data.result & 0x00FF;

	/*Pressure PACKET*/
	packet[22] = pres_data.pressure_result >> 8;
	packet[23] = pres_data.pressure_result & 0x00FF;

	/*Load cell*/
	packet[24] = load_cell_ch1_bytes[0];
	packet[25] = load_cell_ch1_bytes[1];
	packet[26] = load_cell_ch1_bytes[2];
	packet[27] = load_cell_ch2_bytes[0];
	packet[28] = load_cell_ch2_bytes[1];
	packet[29] = load_cell_ch2_bytes[2];

	/* battery voltage */
	packet[30] = battery_voltage >> 8;
	packet[31] = battery_voltage & 0x00FF;

	/* sequence number */
	packet[32] = sequence_number;

	radio_ret = HAL_UART_Transmit(&huart2, packet, packet[0] + 1, 100);

	// increment sequence number and write back to Backup Register
	if (sequence_number == 255) {
		sequence_number = 0; // reset back to zero if we're at the top
	} else {
		sequence_number++; // otherwise increment
	}

	 HAL_RTCEx_BKUPWrite(&hrtc, 2, sequence_number); // write the sequence number to Backup Register 2


	if(radio_ret == HAL_OK){
		return true;
	}
	return false;
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
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

