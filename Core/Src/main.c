/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2023 STMicroelectronics.
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
#include "stdio.h"
#include "string.h"
#include "stdbool.h"
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
ADC_HandleTypeDef hadc;

UART_HandleTypeDef hlpuart1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
extern uint8_t Res2_Buf[256];
extern uint8_t Res2_Count;
extern uint8_t Res2_Sign;
extern uint8_t Res2;

extern uint8_t Res1_Buf[256];
extern uint8_t Res1_Count;
extern uint8_t Res1_Sign;
extern uint8_t Res1;

extern void Get_BAT_Value(void);
extern uint8_t Set_Cmd2NB(uint8_t *cmd, uint8_t len, char *recdata);
extern void NB_Rec_Handle(void);

uint16_t BAT_DATA;
float BAT_VALUE;
uint16_t VREFINT_CAL;
uint16_t VREFINT_DATA;
float VDDA_Value;

uint32_t main_count = 25000;
uint8_t i = 0;
uint8_t NB_SINGAL_VALUE = 0;
float Latitude;
float Longitude;
float Altitude;

bool iot_flag = false;
char iot_info[127];
char iot_info2[127];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC_Init(void);
static void MX_LPUART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int fputc(int ch, FILE *f)
{
  while ((USART2->ISR&0X40) == 0);
  USART2->TDR = (uint8_t)ch;
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
  float temp = 0.0f;
  uint8_t BAT_Q = 0;
  uint8_t BAT_Q_LAST = 0;
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
  MX_ADC_Init();
  MX_LPUART1_UART_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  HAL_GPIO_WritePin(PWR_LED_GPIO_Port, PWR_LED_Pin, GPIO_PIN_RESET);  //点亮电源指示灯
  HAL_GPIO_WritePin(PWR_EN_GPIO_Port, PWR_EN_Pin, GPIO_PIN_SET); //让电路板保持上电

  //给NB20模组开机
  HAL_GPIO_WritePin(NB_PWR_GPIO_Port, NB_PWR_Pin, GPIO_PIN_SET);
  HAL_Delay(600);
  HAL_GPIO_WritePin(NB_PWR_GPIO_Port, NB_PWR_Pin, GPIO_PIN_RESET);

  HAL_Delay(30000); //等待BC20连接基站

  while(Set_Cmd2NB("AT\r\n", 4, "OK") != 0)  //AT回复正确之后再进行以下操作
  {
    HAL_Delay(1000);
  }
  printf("NB模组正常\r\n");
  
  Set_Cmd2NB("ATE0\r\n", 6, "OK"); //关闭回显
  printf("已关闭回显\r\n");

  while((NB_SINGAL_VALUE == 0) || (NB_SINGAL_VALUE == 99)) //检测信号
  {
    if(Set_Cmd2NB("AT+CSQ\r\n", 8, "OK") == 0)
    {
      if(Res1_Buf[9] == ',')  //信号量为个位数
      {
        NB_SINGAL_VALUE = Res1_Buf[8] - 48;
      }
      else if(Res1_Buf[10] == ',')  //信号量为10个位数
      {
        NB_SINGAL_VALUE = (Res1_Buf[8]-48)*10+(Res1_Buf[9]-48);
      }
    }
    HAL_Delay(1000);
  }
  printf("检测信号值:%d\r\n",NB_SINGAL_VALUE);

  while(Set_Cmd2NB("AT+CEREG?\r\n",11,"+CEREG: 0,1")  != 0) //等待ESP网络注册
  {
    HAL_Delay(1000);
  }
  printf("EPS网络注册成功\r\n");

  while(Set_Cmd2NB("AT+CGATT?\r\n",11,"+CGATT: 1")  != 0) //等待PS附着
  {
    HAL_Delay(1000);
  }
  printf("PS附着成功\r\n");

  while(Set_Cmd2NB("AT+QGNSSC=1\r\n",13,"OK")  != 0) //打开GNSS电源
  {
    HAL_Delay(1000);
  }
  HAL_Delay(2000);
  while(Set_Cmd2NB("AT+QGNSSC?\r\n",12,"+QGNSSC: 1")  != 0) //检测GNSS电源
  {
    HAL_Delay(1000);
  }
  printf("GNSS电源已打开\r\n");

  // Set_Cmd2NB("AT+QLWSERV=\"221.229.214.202\",5683\r\n",35,"OK");  //设置电信 loT 平台 IP 地址和端口。
  // Set_Cmd2NB("AT+QLWCONF=\"862177041550669\"\r\n",30,"OK"); //设置连接到电信 loT 平台设备的 IMEI 号。
  // Set_Cmd2NB("AT+QLWADDOBJ=19,0,1,\"0\"\r\n",25,"OK");  //添加 LwM2M 对象 19/0/0。
  // Set_Cmd2NB("AT+QLWOPEN=0\r\n",14,"OK"); //以直吐模式注册到电信 loT 平台。


  Set_Cmd2NB("AT+QSCLK=0\r\n", 12, "OK"); //关闭休眠模式

  // Set_Cmd2NB("AT+QGNSSRD=\"NMEA/RMC\"\r\n", 23, "OK"); //查询定位

  
  

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    
    if (Res2_Sign == 1) //如果串口发送数据
    {
      //接收直至最后一个字节，再延时10ms
      do {
        Res2++;
        HAL_Delay(1);
      } while (Res2 < 10);
      Res2_Sign = 0;
      HAL_UART_Transmit(&hlpuart1, Res2_Buf, Res2_Count, 1000); //把接收到的数据发送给BC20模组
      Res2_Count = 0;
    }

    if (Res1_Sign == 1) //如果BC20发送数据
    {
      //接收直至最后一个字节，再延时10ms
      do {
        Res1++;
        HAL_Delay(1);
      } while (Res1 < 10);
      Res1_Sign = 0;
      HAL_UART_Transmit(&huart2, Res1_Buf, Res1_Count, 1000); //把接收到的数据发送给USB串口
      Res1_Count = 0;
      // NB_Rec_Handle();
    }
    
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    if (HAL_GPIO_ReadPin(SHUT_DOWN_GPIO_Port, SHUT_DOWN_Pin) == 0)  //循环检测引脚状态控制开关机
    {
      HAL_GPIO_WritePin(PWR_LED_GPIO_Port, PWR_LED_Pin, GPIO_PIN_SET);  //关闭电源指示灯
      while(HAL_GPIO_ReadPin(SHUT_DOWN_GPIO_Port, SHUT_DOWN_Pin) == 0); //等待按键松开
      HAL_GPIO_WritePin(PWR_EN_GPIO_Port, PWR_EN_Pin, GPIO_PIN_RESET); //让电路板掉电
    }

    HAL_Delay(1);
    main_count++;
    if (main_count > 30000)
    {
      // if (iot_flag)
      {
        main_count = 0;
        Get_BAT_Value();
        temp = 0;
        for (size_t i = 0; i < 10; i++)
        {
          Get_BAT_Value();
          temp += BAT_VALUE;
        }
        BAT_VALUE = temp/10;
        if (BAT_VALUE > 4.1)
        {
          BAT_Q = 100;
        }
        else if (BAT_VALUE < 3.0)
        {
          BAT_Q = 0;
        }
        else
        {
          BAT_Q = 100 - (4.1 - BAT_VALUE) * (100/(4.1-3.0));
        }
        
        // printf("发送定位请求,返回代码值:%d\r\n",Set_Cmd2NB("AT+QGNSSRD=\"NMEA/GGA\"\r\n",23,"+QGNSSRD: $GNGGA"));
        // // Set_Cmd2NB("AT+QGNSSRD=\"NMEA/GGA\"\r\n",23,"+QGNSSRD: $GNGGA");  //发送定位请求
        // printf("处理返回数据:->%s<-\r\n",Res1_Buf);
        // NB_Rec_Handle();  //处理返回数据
        // printf("处理返回数据结束\r\n");


        // memset(iot_info, 0, 127);
        // memset(iot_info2, 0, 127);
        // if (BAT_Q_LAST != BAT_Q)
        // {
        //   sprintf(iot_info,"AT+QLWDATASEND=889,0,0,%d,\"%s\",0x0000\r\n",sizeof(BAT_Q),BAT_Q);
        //   Set_Cmd2NB((uint8_t *)iot_info,strlen(iot_info),"OK");
        //   BAT_Q_LAST = BAT_Q;
        // }
        
        // sprintf(iot_info,"Lat=%.4f,Lon=%.4f,Alt=%.2f,BAT_Q=%d",Latitude,Longitude,Altitude,BAT_Q);
        // sprintf(iot_info2,"AT+QLWDATASEND=19,0,0,%d,\"%s\",0x0000\r\n",strlen(iot_info),iot_info);
        // Set_Cmd2NB((uint8_t *)iot_info2,strlen(iot_info2),"OK");
      }
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2|RCC_PERIPHCLK_LPUART1;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  PeriphClkInit.Lpuart1ClockSelection = RCC_LPUART1CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC_Init(void)
{

  /* USER CODE BEGIN ADC_Init 0 */
  uint8_t i = 0;
  /* USER CODE END ADC_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC_Init 1 */

  /* USER CODE END ADC_Init 1 */
  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc.Instance = ADC1;
  hadc.Init.OversamplingMode = DISABLE;
  hadc.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc.Init.Resolution = ADC_RESOLUTION_12B;
  hadc.Init.SamplingTime = ADC_SAMPLETIME_160CYCLES_5;
  hadc.Init.ScanConvMode = ADC_SCAN_DIRECTION_FORWARD;
  hadc.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc.Init.ContinuousConvMode = ENABLE;
  hadc.Init.DiscontinuousConvMode = DISABLE;
  hadc.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc.Init.DMAContinuousRequests = DISABLE;
  hadc.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc.Init.LowPowerAutoWait = ENABLE;
  hadc.Init.LowPowerFrequencyMode = DISABLE;
  hadc.Init.LowPowerAutoPowerOff = ENABLE;
  if (HAL_ADC_Init(&hadc) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel to be converted.
  */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = ADC_RANK_CHANNEL_NUMBER;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel to be converted.
  */
  sConfig.Channel = ADC_CHANNEL_VREFINT;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC_Init 2 */
  ADC1->CR |= ADC_CR_ADCAL;
  i = 0;
  while(((ADC1->ISR&ADC_ISR_EOCAL)!=ADC_ISR_EOCAL) && (i < 10))
  {
    i++;
    HAL_Delay(1);
  };
  if (i == 10)
    printf("ADC校准失败\r\n");
  ADC1->ISR |= ADC_ISR_EOCAL;
  VREFINT_CAL = *(uint16_t *)0X1FF80078;
  // HAL_ADC_Start(&hadc);
  ADC1->CR |= ADC_CR_ADSTART;
  
  /* USER CODE END ADC_Init 2 */

}

/**
  * @brief LPUART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_LPUART1_UART_Init(void)
{

  /* USER CODE BEGIN LPUART1_Init 0 */

  /* USER CODE END LPUART1_Init 0 */

  /* USER CODE BEGIN LPUART1_Init 1 */

  /* USER CODE END LPUART1_Init 1 */
  hlpuart1.Instance = LPUART1;
  hlpuart1.Init.BaudRate = 115200;
  hlpuart1.Init.WordLength = UART_WORDLENGTH_8B;
  hlpuart1.Init.StopBits = UART_STOPBITS_1;
  hlpuart1.Init.Parity = UART_PARITY_NONE;
  hlpuart1.Init.Mode = UART_MODE_TX_RX;
  hlpuart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  hlpuart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  hlpuart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&hlpuart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN LPUART1_Init 2 */
  __HAL_UART_ENABLE_IT(&hlpuart1, UART_IT_RXNE);  //打开串口接收中断
  /* USER CODE END LPUART1_Init 2 */

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
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */
  __HAL_UART_ENABLE_IT(&huart2, UART_IT_RXNE);  //打开串口接收中断
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
  HAL_GPIO_WritePin(PWR_LED_GPIO_Port, PWR_LED_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, NB_PSM_Pin|VBAT_ADC_EN_Pin|NB_RST_Pin|NB_PWR_Pin
                          |PWR_EN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : PWR_LED_Pin */
  GPIO_InitStruct.Pin = PWR_LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(PWR_LED_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : NB_PSM_Pin VBAT_ADC_EN_Pin NB_RST_Pin NB_PWR_Pin
                           PWR_EN_Pin */
  GPIO_InitStruct.Pin = NB_PSM_Pin|VBAT_ADC_EN_Pin|NB_RST_Pin|NB_PWR_Pin
                          |PWR_EN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : SHUT_DOWN_Pin */
  GPIO_InitStruct.Pin = SHUT_DOWN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(SHUT_DOWN_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void Get_BAT_Value(void)
{
  i = 0;
  while(((ADC1->ISR&ADC_ISR_EOC)!=ADC_ISR_EOC) && (i<10))
  {
    HAL_Delay(1);
    i++;
  }
  if (i == 10)
  {
    printf("通道4转换超时\r\n");
  }
  BAT_DATA = ADC1->DR;
  i = 0;
  while(((ADC1->ISR&ADC_ISR_EOC)!=ADC_ISR_EOC) && (i<10))
  {
    HAL_Delay(1);
    i++;
  }
  if (i == 10)
  {
    printf("通道17转换超时\r\n");
  }
  VREFINT_DATA = ADC1->DR;
  VDDA_Value = 3.0f*VREFINT_CAL/VREFINT_DATA;
  BAT_VALUE = VDDA_Value*BAT_DATA/4096*2;
}

/**
 * @brief 向BC20发送命令
 * 
 * @param cmd 表示要发送的命令
 * @param len 命令的长度
 * @param recdata 接收的响应是否包含该字符串
 * @return uint8_t 0正确；1没有接收到响应；2接收到响应但不符合要求
 */
uint8_t Set_Cmd2NB(uint8_t *cmd, uint8_t len, char *recdata)
{
  uint8_t ret;
  uint16_t count = 0;
  memset(Res1_Buf,0,sizeof(Res1_Buf));

  HAL_UART_Transmit(&hlpuart1, cmd, len, 1000);
  while ((Res1_Sign == 0) && (count < 300))
  {
    count++;
    HAL_Delay(1);
  }

  if (count == 300)
  {
    ret = 1;  //未收到BC20的响应
  }
  else
  {
    do {
        Res1++;
        HAL_Delay(1);
      } while (Res1 < 10);
    ////////////////////////////////
    Res1_Sign = 0;
    HAL_UART_Transmit(&huart2, Res1_Buf, Res1_Count, 1000); //把接收到的数据发送给USB串口
    Res1_Count = 0;

    ret = 2;
    if(strstr((const char*)Res1_Buf,recdata)) //如果接受到的数据里面有这个字符串
    {
      ret = 0;  //正确收到相应
    }
  }
  
  return ret;

}

/**
 * @brief 处理NB的返回数据
 * 
 */
void NB_Rec_Handle(void)
{
  if((!iot_flag) && (strstr((const char*)Res1_Buf,"+QLWOBSERVE: 0,19,0,0")))  //接收订阅成功（19/0/0）请求。
  {
    iot_flag = true;
    while(Set_Cmd2NB("AT+QGNSSC?\r\n",12,"+QGNSSC: 1")  != 0)  //检测GNSS状态
    {
      Set_Cmd2NB("AT+QGNSSC=1\r\n",13,"OK");  //开启GNSS
        HAL_Delay(1000);
    }
    printf("开启GNSS成功\r\n");
  }

  if((strstr((const char*)Res1_Buf,"+QGNSSRD: $GNGGA")) && (Res1_Buf[19] != ','))
  {
    //+QGNSSRD: $GNGGA,021326.00,3155.9607,N,11715.6982,E,1,09,1.54,26.7,M,,M,,*6E
    uint8_t i = 0;
    char *temp = strtok((char *)Res1_Buf,",");
    while(temp)
    {
        if (i == 2)
        {
          Latitude = atof(temp);
          Latitude = (uint8_t)Latitude/100 + (Latitude - (uint8_t)Latitude/100*100)/60;
        }
        else if (i == 4)
        {
          Longitude = atof(temp);
          Longitude = (uint8_t)Longitude/100 + (Longitude - (uint8_t)Longitude/100*100)/60;
        }
        else if (i == 9)
        {
          Altitude = atof(temp);
        }

        temp = strtok(NULL,",");
        i++;
    }
  }
  else if ((strstr((const char*)Res1_Buf,"+QGNSSRD: $GNGGA")) && (Res1_Buf[19] == ','))
  {
    Latitude = 0;
    Longitude = 0;
    Altitude = 0;
    printf("GNSS信号差定位不成功\r\n");
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
