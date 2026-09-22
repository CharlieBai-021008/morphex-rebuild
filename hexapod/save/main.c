/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "servo.h"
#include <string.h>
#include "stdio.h"
#include "hexapod_control.h"
#include "sbus.h"

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
// SBUS接收相关变量
uint8_t sbus_rx_byte;                // 当前接收到的单个字节
uint8_t sbus_rx_buffer[SBUS_FRAME_SIZE]; // 完整的SBUS帧缓冲区
uint8_t sbus_rx_index = 0;           // 当前接收到的字节索引
uint8_t sbus_frame_ready = 0;        // 一帧数据接收完成标志
uint32_t sbus_last_receive_time = 0; // 上次接收字节的时间

// USART1打印用缓冲区
char print_buffer[100];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */


#ifdef __GNUC__									//串口重定向
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif
PUTCHAR_PROTOTYPE
{
    HAL_UART_Transmit(&huart1 , (uint8_t *)&ch, 1, 0xFFFF);
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
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_TIM1_Init();
  MX_USART6_UART_Init();
  /* USER CODE BEGIN 2 */
  Servo_Init();
  InitHexapod();

  // 启动USART6接收中断
  HAL_UART_Receive_IT(&huart6, &sbus_rx_byte, 1);

  // 初始化时间戳
  sbus_last_receive_time = HAL_GetTick();

  // 发送启动消息
  printf("SBUS Receiver started\r\n");
  printf("SBUS Config: Start=0x0F, End=0x00, Flag=0x%02X\r\n");


/*  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);

  // 初始化舵机位置为90度，立即设置
  Servo_SetAngle(90, 0);*/


  HAL_Delay(2000);



/*  SetSpecificJointAnglesStepwise();*/


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  /* 处理舵机通信状态 */
	  Servo_Process();
	  // 获取当前时间
	    uint32_t current_time = HAL_GetTick();
	    static uint32_t last_print_time = 0;

	    // 检查是否有新的SBUS帧接收完成，并且距离上次打印已过去1秒
	    if (sbus_frame_ready && (current_time - last_print_time >= 1000))
	    {
	      // 更新上次打印时间
	      last_print_time = current_time;

	      // 清除标志
	      sbus_frame_ready = 0;

	      // 打印原始数据帧的关键字节
	      sprintf(print_buffer, "Frame: Start=0x%02X, Flag=0x%02X, End=0x%02X\r\n",
	              sbus_rx_buffer[0], sbus_rx_buffer[23], sbus_rx_buffer[24]);
	      HAL_UART_Transmit(&huart1, (uint8_t *)print_buffer, strlen(print_buffer), HAL_MAX_DELAY);

	      // 解析SBUS数据
	      if (update_sbus(sbus_rx_buffer) == 0)
	      {
	        // 打印通道数据
	        sprintf(print_buffer, "Connected: CH1=%d, CH2=%d, CH3=%d, CH4=%d, CH5=%d, CH6=%d\r\n",
	                SBUS_CH.CH1, SBUS_CH.CH2, SBUS_CH.CH3, SBUS_CH.CH4, SBUS_CH.CH5,SBUS_CH.CH6);
	        HAL_UART_Transmit(&huart1, (uint8_t *)print_buffer, strlen(print_buffer), HAL_MAX_DELAY);

	        // 将SBUS值转换为PWM值并打印
	        uint16_t pwm1 = sbus_to_pwm(SBUS_CH.CH1);
	        uint16_t pwm2 = sbus_to_pwm(SBUS_CH.CH2);
	        sprintf(print_buffer, "PWM: CH1=%d, CH2=%d\r\n", pwm1, pwm2);
	        HAL_UART_Transmit(&huart1, (uint8_t *)print_buffer, strlen(print_buffer), HAL_MAX_DELAY);
	      }
	      else
	      {
	        sprintf(print_buffer, "Not connected, Flag=0x%02X\r\n", sbus_rx_buffer[23]);
	        HAL_UART_Transmit(&huart1, (uint8_t *)print_buffer, strlen(print_buffer), HAL_MAX_DELAY);
	      }
	    }
	    else if (sbus_frame_ready)
	    {
	      // 如果有新数据但还没到打印时间，只更新数据不打印
	      update_sbus(sbus_rx_buffer);
	      sbus_frame_ready = 0;
	    }

	    // 超时检测 - 如果长时间没有收到数据，打印提示
	    // 改为使用与数据打印相同的时间控制变量，确保这部分也是1秒打印一次
	    if (current_time - last_print_time >= 1000 && current_time - sbus_last_receive_time > 1000)
	    {
	      last_print_time = current_time;

	      sprintf(print_buffer, "No SBUS data received for 1s, RX_index=%d\r\n", sbus_rx_index);
	      HAL_UART_Transmit(&huart1, (uint8_t *)print_buffer, strlen(print_buffer), HAL_MAX_DELAY);

	      // 打印接收缓冲区内容，用于调试
	      sprintf(print_buffer, "Buffer[0-4]: %02X %02X %02X %02X %02X\r\n",
	              sbus_rx_buffer[0], sbus_rx_buffer[1], sbus_rx_buffer[2],
	              sbus_rx_buffer[3], sbus_rx_buffer[4]);
	      HAL_UART_Transmit(&huart1, (uint8_t *)print_buffer, strlen(print_buffer), HAL_MAX_DELAY);

	      // 如果没有接收到数据，重新启动接收
	      if (HAL_UART_GetState(&huart6) != HAL_UART_STATE_BUSY_RX)
	      {
	        HAL_UART_Receive_IT(&huart6, &sbus_rx_byte, 1);
	        sprintf(print_buffer, "UART restarted\r\n");
	        HAL_UART_Transmit(&huart1, (uint8_t *)print_buffer, strlen(print_buffer), HAL_MAX_DELAY);
	      }
	    }
  }
	    HAL_Delay(10); // 短暂延时，降低CPU负载
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
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 168;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART6)
  {
    uint32_t current_time = HAL_GetTick();

    // 如果距离上次接收超过5ms，认为是新帧的开始（说明书中提到14ms一帧）
    if (current_time - sbus_last_receive_time > 5 || sbus_rx_byte == 0x0F)
    {
      sbus_rx_index = 0;
    }
    sbus_last_receive_time = current_time;

    // 存储接收到的字节
    sbus_rx_buffer[sbus_rx_index++] = sbus_rx_byte;

    // 检查是否接收到完整的SBUS帧
    if (sbus_rx_index >= SBUS_FRAME_SIZE)
    {
      // 验证帧格式：第一个字节是0x0F，最后一个字节是0x00
      if (sbus_rx_buffer[0] == 0x0F && sbus_rx_buffer[24] == 0x00)
      {
        sbus_frame_ready = 1;
      }
      sbus_rx_index = 0;
    }

    // 重新启动中断接收下一个字节
    HAL_UART_Receive_IT(&huart6, &sbus_rx_byte, 1);
  }
  if(huart->Instance == USART2) // 如果是 UART2 中断
      {
          Servo_UART_RxCallback(); // 调用舵机驱动接收回调
      }
}
// UART 发送完成中断回调
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if(huart->Instance == USART2) // 如果是 UART2 中断
    {
        Servo_UART_TxCallback(); // 调用舵机驱动发送回调
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
