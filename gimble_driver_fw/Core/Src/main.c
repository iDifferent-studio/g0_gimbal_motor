/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
#include "adc.h"
#include "dma.h"
#include "i2c.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

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
uint32_t time_cntr;
float target;
uint8_t controller_state, controller_error, motor_en;
uint8_t i2c_rx_buf[10];
uint8_t flag_500ms=0;
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
  MX_ADC1_Init();
  MX_I2C1_Init();
  MX_SPI1_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
	HAL_TIM_Base_Start_IT(&htim2);
	
	HAL_ADC_Start_DMA(&hadc1, (uint32_t *)&adc_buf, 5);
	HAL_I2C_EnableListen_IT(&hi2c1);
	
	HAL_Delay(500);
	
	MagneticSensor_Init();
	LowsideCurrentSense(0.001,20,2,1,0);
	LowsideCurrentSense_Init();
	
	LPF_init(); 
	PID_init();
	
	voltage_power_supply=12;    //V
	voltage_limit=6.9;          //V，最大值需小于12/1.732=6.9，航模电机设置的小一点，不超过2；云台电机设置的大一点，不小于2，电压越大能达到的转速就越大
	velocity_limit=100;         //rad/s angleOpenloop() use it	
	current_limit=2;            //A，foc_current和dc_current模式限制电流，不能为0。速度模式和位置模式起作用
	voltage_sensor_align=2;     //V     alignSensor() and driverAlign() use it，大功率电机0.5-1，小功率电机2-3
	torque_controller=Type_foc_current;  
	motion_controller=Type_angle; 
	target=0;
	
	Motor_init();
	Motor_initFOC(0,UNKNOWN);
	
	motor_en=1;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		if(motor_en)
		{
			move(target);
			loopFOC();
		}
		else
		{
			shaft_angle = shaftAngle();
		}
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		if(flag_500ms)
		{
			HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
			flag_500ms=0;
		}
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
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSIDiv = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV1;
  RCC_OscInitStruct.PLL.PLLN = 8;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
uint32_t t_sum=0, inc_num=0;

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	time_cntr++;
	
	if(time_cntr%500==0)
		flag_500ms=1;
	
	t_sum = t_sum + __LL_ADC_CALC_TEMPERATURE(v_vref, adc_buf[3], ADC_RESOLUTION_12B);
	inc_num++;
	if(inc_num==200)
	{
		temperature = t_sum / inc_num;
		t_sum = 0;
		inc_num = 0;
	}
}

uint8_t state=0;

union rx_data
{
	float val_f;
	uint32_t val_u32;
	uint16_t val_u16[2];
	uint8_t buf[4];
}rx_data;

union tx_data
{
	float val_f;
	uint32_t val_u32;
	uint16_t val_u16[2];
	uint8_t buf[4];
}tx_data;

void HAL_I2C_ListenCpltCallback(I2C_HandleTypeDef *hi2c)//!
{
	HAL_I2C_EnableListen_IT(hi2c); // slave is ready again
}

void HAL_I2C_AddrCallback(I2C_HandleTypeDef *hi2c, uint8_t TransferDirection, uint16_t AddrMatchCode)
{
	if( TransferDirection==I2C_DIRECTION_TRANSMIT )//need to receive
	{
		if(state==0)//receive reg addr
		{
			HAL_I2C_Slave_Sequential_Receive_IT(hi2c, i2c_rx_buf, 1, I2C_LAST_FRAME);
		}
		else//receive reg data
		{
			HAL_I2C_Slave_Sequential_Receive_IT(&hi2c1, rx_data.buf, 4, I2C_LAST_FRAME);
		}
		
	} 
	else//need to send 
	{
		switch (i2c_rx_buf[0])
		{
			case 0x01:
				tx_data.val_f=target;
			break;
			
			case 0x02:
				tx_data.val_f=shaft_angle;
			break;
			
			case 0x03:
				tx_data.val_f=shaft_velocity;
			break;
			
			case 0x04:
				tx_data.val_f=current.q;
			break;
			
			case 0x05:
				tx_data.val_u32=temperature;
			break;
			
			case 0x06:
				
			break;
			
			case 0x07:
				tx_data.buf[0]=(uint8_t)torque_controller;
			  tx_data.buf[1]=(uint8_t)motion_controller;
			break;
			
			case 0x08:
				tx_data.val_f=voltage_limit;
			break;
			
			case 0x09:
				tx_data.val_f=velocity_limit;
			break;
			
			case 0x10:
				tx_data.val_f=current_limit;
			break;
			
			case 0x11:
				tx_data.val_f=voltage_sensor_align;
			break;
			
			case 0x12:
				tx_data.val_f=PID_angle.P;
			break;
			
			case 0x13:
				tx_data.val_f=PID_velocity.P;
			break;
			
			case 0x14:
				tx_data.val_f=PID_velocity.I;
			break;
			
			case 0x15:
				tx_data.val_f=PID_velocity.D;
			break;
			
			default:
			break;
		}
		HAL_I2C_Slave_Seq_Transmit_IT(hi2c, tx_data.buf, 4,  I2C_LAST_FRAME);
		state=0;
	}
	
}
 
void HAL_I2C_SlaveRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
	if(state==1)
	{
		switch (i2c_rx_buf[0])
		{
			case 0x01:
				target=rx_data.val_f;
			break;
			
			case 0x02:
			case 0x03:
			case 0x04:
			case 0x05:
			case 0x06:
			break;
			
			case 0x07:
				torque_controller=(TorqueControlType)rx_data.buf[0];
				motion_controller=(MotionControlType)rx_data.buf[1];
			break;
			
			case 0x08:
				voltage_limit=rx_data.val_f;
			break;
			
			case 0x09:
				velocity_limit=rx_data.val_f;
			break;
			
			case 0x10:
				current_limit=rx_data.val_f;
			break;
			
			case 0x11:
				voltage_sensor_align=rx_data.val_f;
			break;
			
			case 0x12:
				PID_angle.P=rx_data.val_f;
			break;
			
			case 0x13:
				PID_velocity.P=rx_data.val_f;
			break;
			
			case 0x14:
				PID_velocity.I=rx_data.val_f;
			break;
			
			case 0x15:
				PID_velocity.D=rx_data.val_f;
			break;
			
			default:
			break;
		}
		state=0;
	}
	else
	{
		switch(i2c_rx_buf[0])
		{
			case 0x81:
				if(motor_en)
				{
					motor_en=0;
					M1_Disable;
				}
				else
				{
					motor_en=1;
					M1_Enable;
					target=shaft_angle;
				}
			break;
			
			default:
				state=1;
			break;
		}
	}
}
 
void HAL_I2C_SlaveTxCpltCallback(I2C_HandleTypeDef *hi2c)
{

}

void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c)
{
	HAL_I2C_DeInit(hi2c);
	MX_I2C1_Init();
	HAL_I2C_Slave_Receive_IT(&hi2c1, i2c_rx_buf, 1);
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

