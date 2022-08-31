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
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include "PID.h"
#include "Motor.h"
#include "oled.h"
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
float pitch,roll,yaw; 		//欧拉角
short aacx,aacy,aacz;		//加速度传感器原始数据
short gyrox,gyroy,gyroz;	//陀螺仪原始数据
short temp;					//温度

float pwm_velocity=0;
int pwm_value=0,pwm_Upright=0;
int tmp_x=0,tmp_z=0;
float tmp_theta=0;


int count=0;
int A1=0,A2=0,B1=0,B2=0;
int A1_tmp=0,A2_tmp=0,B1_tmp=0,B2_tmp=0;
int L_count=0,R_count=0;
uint8_t PWM_PUT_OLED;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int fputc(int ch,FILE* f)
{
	uint8_t temp[1]={ch};
	HAL_UART_Transmit(&huart1,temp,1,2);
	return ch;	
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
  MX_USART1_UART_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_I2C2_Init();
  /* USER CODE BEGIN 2 */
HAL_TIM_Base_Start_IT(&htim1);
HAL_TIM_Base_Start_IT(&htim2);
HAL_TIM_Base_Start_IT(&htim3);
HAL_TIM_PWM_Start_IT(&htim2,TIM_CHANNEL_1);
HAL_TIM_PWM_Start_IT(&htim3,TIM_CHANNEL_1);
	OLED_CLS();
	while(MPU_Init());					//初始化MPU6050


	while(mpu_dmp_init())
	{
		delay_ms(200);
		printf("%s\r\n","Mpu6050 Init Wrong!");
	}
	printf("%s\r\n","Mpu6050 Init OK!");
	
	MPU_Get_Gyroscope_Mean(&gyrox,&gyroy,&gyroz);	//取平均
	MPU_Get_Accelerometer_Mean(&aacx,&aacy,&aacz);	//取平均
	
	OLED_Init();
	OLED_CLS();
	OLED_ShowStr(40,2,"zihao",2);
	HAL_Delay(1000);
	OLED_CLS();
	OLED_ShowStr(0,2,"    balancing car",1);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	
//				OLED_ShowStr(0,0,"    balancing car",1);
//				OLED_ShowStr(0,2,"pitch:",1);
//				OLED_ShowStr(0,4,"theta:",1);
//				OLED_ShowStr(0,6,"pwm:",1);
//				//OLED_ShowNum(32,2,(float)aacx/(0xffff/4),3,12);
//				static int aacx_oled ;
//				static int theta_oled;
//				static int pwm_oled;
//				aacx_oled= pitch;
//				theta_oled = tmp_theta;
//				pwm_oled = pwm_value;
//				aacx_oled = (aacx_oled<0)?(-aacx_oled):aacx_oled;
//				OLED_ShowNum(50,2,aacx_oled,3,12);
//				OLED_ShowNum(50,4,theta_oled,3,12);
//				OLED_ShowNum(50,6,pwm_oled,3,12);
  while(1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		if(mpu_dmp_get_data(&pitch,&roll,&yaw)==0)
		{
				MPU_Get_Accelerometer(&aacx,&aacy,&aacz);	//得到加速度传感器数据
				MPU_Get_Gyroscope(&gyrox,&gyroy,&gyroz);	//得到陀螺仪数据

//				printf("三轴角度(deg)：%.1f  %.1f  %.1f\r\n",pitch,roll,yaw);
//				printf("三轴加速度a(m/s^2)：%.3fg  %.3fg  %.3fg\r\n",(float)aacx/(0xffff/4),(float)aacy/(0xffff/4),(float)aacz/(0xffff/4));	
//				printf("三轴角速度w(deg/s)：%.1f  %.1f  %.1f\r\n",(float)gyrox/(0xffff/500),(float)gyroy/(0xffff/500),(float)gyroz/(0xffff/500));

		}


//电机计数

		A1_tmp = A1;
		A2_tmp = A2;
		B1_tmp = B1;
		B2_tmp = B2;
		A1=HAL_GPIO_ReadPin(AOUT1_GPIO_Port,AOUT1_Pin);
		B1=HAL_GPIO_ReadPin(BOUT1_GPIO_Port,BOUT1_Pin);
		A2=HAL_GPIO_ReadPin(AOUT2_GPIO_Port,AOUT2_Pin);
		B2=HAL_GPIO_ReadPin(BOUT2_GPIO_Port,BOUT2_Pin);
		
		if((A1!=A1_tmp)&&A1)
		{
			if(B1&&B1_tmp)
				L_count++;
			if(!B1&&!B1_tmp)
				L_count--;
		}
		
		if((A2!=A2_tmp)&&A2)
		{
			if(B2&&B2_tmp)
				R_count--;
			if(!B2&&!B2_tmp)
				R_count++;
		}
		
		tmp_theta=(float)gyroy/(0xffff/500);

		pwm_velocity = velocity(0,L_count,R_count);
		pwm_velocity = (pwm_velocity>=30)?30:pwm_velocity;
		pwm_velocity = (pwm_velocity<=-30)?-30:pwm_velocity;
		
		pwm_Upright=Upright(pwm_velocity,pitch,tmp_theta);
//		pwm_Upright = (pwm_Upright>=1000)?1000:pwm_Upright;
//		pwm_Upright = (pwm_Upright<=-1000)?-1000:pwm_Upright;
		
		pwm_value = pwm_Upright;
		rotation(pwm_value);//正反转
		pwm_value = (pwm_value>0)?pwm_value:(-pwm_value);
		__HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_1,pwm_value);
		__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_1,pwm_value);

//		HAL_Delay(100);
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

/* USER CODE BEGIN 4 */


void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim->Instance == htim1.Instance)
	{
		count++;
		if(count>=500)
		{
			printf("pwm_Upright： %d   pitch： %.2f   pwm_value: %d\r\n",pwm_Upright,pitch,pwm_value);
			count = 0; 
			L_count = 0;
			R_count = 0;
		}
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

