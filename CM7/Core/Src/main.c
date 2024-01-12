/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */

#include <stdarg.h>
#include <stdlib.h>
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "string.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#define USE_FOR_LORA 	0
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
// #define ITM_TRACE_EN          *((volatile uint32_t *) 0xE0000000)
// #define ITM_STIMULUS_PORT0    *((volatile uint32_t *) 0xE0000E00)
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#ifndef HSEM_ID_0
#define HSEM_ID_0 (0U) /* HW semaphore 0*/
#endif
#define 	BUFFER_SIZE		50000
/* timestamp in BCD format */
#define HOURS     0x10
#define MINUTES   0x06
#define DATE      0x10
#define MONTH     RTC_MONTH_OCTOBER
#define YEAR      0x23

#define BFR_SIZE	1000
uint32_t buffer1[BFR_SIZE];
uint32_t buffer2[BFR_SIZE];
int gidxA = 0;
int gidxB = 0;
int rateA = 0;
int rateB = 0;
int millis = 0;
int a_shot = 0;
volatile int b_shot = 0;
volatile int c_shot = 0;
volatile int flag_buffer_complete = 0;

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
#if defined ( __ICCARM__ ) /*!< IAR Compiler */
#pragma location=0x30000000
ETH_DMADescTypeDef  DMARxDscrTab[ETH_RX_DESC_CNT]; /* Ethernet Rx DMA Descriptors */
#pragma location=0x30000200
ETH_DMADescTypeDef  DMATxDscrTab[ETH_TX_DESC_CNT]; /* Ethernet Tx DMA Descriptors */

#elif defined ( __CC_ARM )  /* MDK ARM Compiler */

__attribute__((at(0x30000000))) ETH_DMADescTypeDef  DMARxDscrTab[ETH_RX_DESC_CNT]; /* Ethernet Rx DMA Descriptors */
__attribute__((at(0x30000200))) ETH_DMADescTypeDef  DMATxDscrTab[ETH_TX_DESC_CNT]; /* Ethernet Tx DMA Descriptors */

#elif defined ( __GNUC__ ) /* GNU Compiler */
ETH_DMADescTypeDef DMARxDscrTab[ETH_RX_DESC_CNT] __attribute__((section(".RxDecripSection"))); /* Ethernet Rx DMA Descriptors */
ETH_DMADescTypeDef DMATxDscrTab[ETH_TX_DESC_CNT] __attribute__((section(".TxDecripSection")));   /* Ethernet Tx DMA Descriptors */

#endif

ETH_TxPacketConfig TxConfig;

ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;
DMA_HandleTypeDef hdma_adc1;
DMA_HandleTypeDef hdma_adc2;

ETH_HandleTypeDef heth;

RTC_HandleTypeDef hrtc;

UART_HandleTypeDef huart3;

PCD_HandleTypeDef hpcd_USB_OTG_FS;

/* USER CODE BEGIN PV */
UART_HandleTypeDef hlpuart1;
HAL_StatusTypeDef HAL_ADCEx_MultiModeRegStart_DMA(ADC_HandleTypeDef *hadc, uint32_t *pData, uint32_t Length);
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void PeriphCommonClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_USB_OTG_FS_PCD_Init(void);
static void MX_ADC1_Init(void);
static void MX_ETH_Init(void);
static void MX_ADC2_Init(void);
static void MX_RTC_Init(void);
/* USER CODE BEGIN PFP */
static void MX_LPUART1_UART_Init(void);
static void TransferDataADC(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

//__IO uint16_t PA5_Data[BUFFER_SIZE] = {0};
__IO uint16_t PA6_Data[BUFFER_SIZE] = {0};
__IO uint32_t ADC_DualModeVal[4] = {0};
//__IO uint16_t ADC1Val[2] = {0};
//__IO uint16_t ADC2Val[2] = {0};
uint8_t adc_complete = 0;
uint32_t ctr = 0;

void SWD_Init(void)
{
  *(__IO uint32_t*)(0x5C001004) |= 0x00700000; // DBGMCU_CR D3DBGCKEN D1DBGCKEN TRACECLKEN

  //UNLOCK FUNNEL
  *(__IO uint32_t*)(0x5C004FB0) = 0xC5ACCE55; // SWTF_LAR
  *(__IO uint32_t*)(0x5C003FB0) = 0xC5ACCE55; // SWO_LAR

  //SWO current output divisor register
  //This divisor value (0x000000C7) corresponds to 400Mhz
  //To change it, you can use the following rule
  // value = (CPU Freq/sw speed )-1
   *(__IO uint32_t*)(0x5C003010) = ((SystemCoreClock / 2000000) - 1); // SWO_CODR

  //SWO selected pin protocol register
   *(__IO uint32_t*)(0x5C0030F0) = 0x00000002; // SWO_SPPR

  //Enable ITM input of SWO trace funnel
   *(__IO uint32_t*)(0x5C004000) |= 0x00000001; // SWFT_CTRL

  //RCC_AHB4ENR enable GPIOB clock
   *(__IO uint32_t*)(0x580244E0) |= 0x00000002;

  // Configure GPIOB pin 3 as AF
   *(__IO uint32_t*)(0x58020400) = (*(__IO uint32_t*)(0x58020400) & 0xffffff3f) | 0x00000080;

  // Configure GPIOB pin 3 Speed
   *(__IO uint32_t*)(0x58020408) |= 0x00000080;

  // Force AF0 for GPIOB pin 3
   *(__IO uint32_t*)(0x58020420) &= 0xFFFF0FFF;
}

int __io_putchar(int ch){
   ITM_SendChar(ch);
}

void printBuffers(uint32_t *bfr1, uint32_t *bfr2, int size);


void myprintf(const char *fmt, ...) {
  static char buffer[100];
  va_list args;
  va_start(args, fmt);
  vsnprintf(buffer, sizeof(buffer), fmt, args);
  va_end(args);

  int len = strlen(buffer);
  _write(0, (char*)buffer,len);
  //HAL_USART_Transmit(&husart2, (uint8_t*)buffer, len, -1);

}

void myprintf2(const char *fmt, ...) {
  static char buffer[100];
  va_list args;
  va_start(args, fmt);
  vsnprintf(buffer, sizeof(buffer), fmt, args);
  va_end(args);

  int len = strlen(buffer);
//  _write(0, (char*)buffer,len);
  HAL_UART_Transmit(&huart3, (uint8_t*)buffer, len, -1);

}

uint8_t temp = 0;
uint16_t located_fallEdge[50] = {0};
uint16_t dataPeak[50] = {0};
uint16_t located_peak[50] ={0};
uint8_t iPeak = 0;

static void PeakDetectnEdge(void)
{
    int diff = 0;
    uint16_t _tmpMax = BUFFER_SIZE-3;
    uint16_t tempPeak = 0;
    uint16_t tempLocPeak = 0;
    uint16_t ctr_cycles = 0;
    uint8_t iter = 0;
    uint8_t edge = 0;

    iPeak = 0;

   for(int i=8;i<_tmpMax;++i){


      diff = PA6_Data[i] - PA6_Data[i-8];
      if(diff < -42  ){
      // sprintf(temp_str,"found fall edge on : %d\n", i);
      // printf(temp_str);
            /* detected falling edge */
          if((i-located_fallEdge[iter]) > 90){


              if(located_fallEdge[iter])iter++;
              ctr_cycles=0;

              /* stored peak data on the next edge */
              if(edge==1 && tempPeak!=0){
                  dataPeak[iPeak] = tempPeak;
                  located_peak[iPeak] = tempLocPeak;
                  iPeak++;
                  tempPeak=0;
              }
              edge=1;

          }
          located_fallEdge[iter] = i;

      }

      if(edge){

          diff = PA6_Data[i] - PA6_Data[i-3];
          if(PA6_Data[i] > tempPeak){
              tempPeak = PA6_Data[i];
              tempLocPeak = ctr_cycles;
          }
          ctr_cycles++;
      }


   }
}

static void MovingAvg(void)
{
#if 0
  uint16_t AvgVal = 0;

  AvgVal = (AvgVal + PA6_Data[0] + PA6_Data[1]  + PA6_Data[2]) >> 2;

  for(int i=3;i<BUFFER_SIZE;++i){
    AvgVal = (AvgVal + PA6_Data[i]) >> 1;
    PA6_Data[i-3] = AvgVal;

  }
#else
  uint16_t ema[2] = {0};

  ema[0] = PA6_Data[0];

  for(int i=1;i<BUFFER_SIZE;++i){
	  ema[1] = ((PA6_Data[i]-ema[0])>>1) + ema[0];
	  PA6_Data[i-1] = ema[1];
	  ema[0] = ema[1];
  }
#endif

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
/* USER CODE BEGIN Boot_Mode_Sequence_0 */
  int32_t timeout;
  RTC_DateTypeDef _date;
  RTC_TimeTypeDef _time;
/* USER CODE END Boot_Mode_Sequence_0 */

/* USER CODE BEGIN Boot_Mode_Sequence_1 */
  /* Wait until CPU2 boots and enters in stop mode or timeout*/
  timeout = 0xFFFF;
  while((__HAL_RCC_GET_FLAG(RCC_FLAG_D2CKRDY) != RESET) && (timeout-- > 0));
  if ( timeout < 0 )
  {
  Error_Handler();
  }
/* USER CODE END Boot_Mode_Sequence_1 */
  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

/* Configure the peripherals common clocks */
  PeriphCommonClock_Config();
/* USER CODE BEGIN Boot_Mode_Sequence_2 */
/* When system initialization is finished, Cortex-M7 will release Cortex-M4 by means of
HSEM notification */
/*HW semaphore Clock enable*/
__HAL_RCC_HSEM_CLK_ENABLE();
/*Take HSEM */
HAL_HSEM_FastTake(HSEM_ID_0);
/*Release HSEM in order to notify the CPU2(CM4)*/
HAL_HSEM_Release(HSEM_ID_0,0);
/* wait until CPU2 wakes up from stop mode */
timeout = 0xFFFF;
while((__HAL_RCC_GET_FLAG(RCC_FLAG_D2CKRDY) == RESET) && (timeout-- > 0));
if ( timeout < 0 )
{
Error_Handler();
}
/* USER CODE END Boot_Mode_Sequence_2 */

  /* USER CODE BEGIN SysInit */
  //SWD_Init();
  //write(0,"Hello",5);
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART3_UART_Init();
  MX_USB_OTG_FS_PCD_Init();
  MX_ADC1_Init();
  MX_ETH_Init();
  MX_ADC2_Init();
  MX_RTC_Init();
  /* USER CODE BEGIN 2 */

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

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_EXTERNAL_SOURCE_SUPPLY);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 120;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief Peripherals Common Clock Configuration
  * @retval None
  */
void PeriphCommonClock_Config(void)
{
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Initializes the peripherals clock
  */
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInitStruct.PLL2.PLL2M = 1;
  PeriphClkInitStruct.PLL2.PLL2N = 21;
  PeriphClkInitStruct.PLL2.PLL2P = 5;
  PeriphClkInitStruct.PLL2.PLL2Q = 2;
  PeriphClkInitStruct.PLL2.PLL2R = 2;
  PeriphClkInitStruct.PLL2.PLL2RGE = RCC_PLL2VCIRANGE_3;
  PeriphClkInitStruct.PLL2.PLL2VCOSEL = RCC_PLL2VCOMEDIUM;
  PeriphClkInitStruct.PLL2.PLL2FRACN = 7168;
  PeriphClkInitStruct.AdcClockSelection = RCC_ADCCLKSOURCE_PLL2;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
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

  ADC_MultiModeTypeDef multimode = {0};
  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ConversionDataManagement = ADC_CONVERSIONDATA_DMA_CIRCULAR;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.LeftBitShift = ADC_LEFTBITSHIFT_NONE;
  hadc1.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the ADC multi-mode
  */
  multimode.Mode = ADC_DUALMODE_REGSIMULT;
  multimode.DualModeData = ADC_DUALMODEDATAFORMAT_32_10_BITS;
  multimode.TwoSamplingDelay = ADC_TWOSAMPLINGDELAY_1CYCLE;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_19;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  sConfig.OffsetSignedSaturation = DISABLE;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief ADC2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC2_Init(void)
{

  /* USER CODE BEGIN ADC2_Init 0 */

  /* USER CODE END ADC2_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC2_Init 1 */

  /* USER CODE END ADC2_Init 1 */

  /** Common config
  */
  hadc2.Instance = ADC2;
  hadc2.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc2.Init.Resolution = ADC_RESOLUTION_12B;
  hadc2.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc2.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc2.Init.LowPowerAutoWait = DISABLE;
  hadc2.Init.ContinuousConvMode = ENABLE;
  hadc2.Init.NbrOfConversion = 1;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.ConversionDataManagement = ADC_CONVERSIONDATA_DMA_CIRCULAR;
  hadc2.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc2.Init.LeftBitShift = ADC_LEFTBITSHIFT_NONE;
  hadc2.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  sConfig.OffsetSignedSaturation = DISABLE;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC2_Init 2 */
/* Run the ADC calibration in single-ended mode */
  if (HAL_ADCEx_Calibration_Start(&hadc2, ADC_CALIB_OFFSET_LINEARITY, ADC_SINGLE_ENDED) != HAL_OK)
  {
    /* Calibration Error */
    Error_Handler();
  }
  /* USER CODE END ADC2_Init 2 */

}

/**
  * @brief ETH Initialization Function
  * @param None
  * @retval None
  */
static void MX_ETH_Init(void)
{

  /* USER CODE BEGIN ETH_Init 0 */

  /* USER CODE END ETH_Init 0 */

   static uint8_t MACAddr[6];

  /* USER CODE BEGIN ETH_Init 1 */

  /* USER CODE END ETH_Init 1 */
  heth.Instance = ETH;
  MACAddr[0] = 0x00;
  MACAddr[1] = 0x80;
  MACAddr[2] = 0xE1;
  MACAddr[3] = 0x00;
  MACAddr[4] = 0x00;
  MACAddr[5] = 0x00;
  heth.Init.MACAddr = &MACAddr[0];
  heth.Init.MediaInterface = HAL_ETH_RMII_MODE;
  heth.Init.TxDesc = DMATxDscrTab;
  heth.Init.RxDesc = DMARxDscrTab;
  heth.Init.RxBuffLen = 1524;

  /* USER CODE BEGIN MACADDRESS */

  /* USER CODE END MACADDRESS */

  if (HAL_ETH_Init(&heth) != HAL_OK)
  {
    Error_Handler();
  }

  memset(&TxConfig, 0 , sizeof(ETH_TxPacketConfig));
  TxConfig.Attributes = ETH_TX_PACKETS_FEATURES_CSUM | ETH_TX_PACKETS_FEATURES_CRCPAD;
  TxConfig.ChecksumCtrl = ETH_CHECKSUM_IPHDR_PAYLOAD_INSERT_PHDR_CALC;
  TxConfig.CRCPadCtrl = ETH_CRC_PAD_INSERT;
  /* USER CODE BEGIN ETH_Init 2 */

  /* USER CODE END ETH_Init 2 */

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

  RTC_TimeTypeDef sTime = {0};
  RTC_DateTypeDef sDate = {0};

  /* USER CODE BEGIN RTC_Init 1 */

  /* USER CODE END RTC_Init 1 */

  /** Initialize RTC Only
  */
  hrtc.Instance = RTC;
  hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
  hrtc.Init.AsynchPrediv = 127;
  hrtc.Init.SynchPrediv = 255;
  hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
  hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
  hrtc.Init.OutPutRemap = RTC_OUTPUT_REMAP_POS1;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }

  /* USER CODE BEGIN Check_RTC_BKUP */

  /* USER CODE END Check_RTC_BKUP */

  /** Initialize RTC and set the Time and Date
  */
  sTime.Hours = 0x0;
  sTime.Minutes = 0x0;
  sTime.Seconds = 0x0;
  sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
  sTime.StoreOperation = RTC_STOREOPERATION_RESET;
  if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }
  sDate.WeekDay = RTC_WEEKDAY_MONDAY;
  sDate.Month = RTC_MONTH_JANUARY;
  sDate.Date = 0x1;
  sDate.Year = 0x0;

  if (HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }

  /** Enable Calibrartion
  */
  if (HAL_RTCEx_SetCalibrationOutPut(&hrtc, RTC_CALIBOUTPUT_1HZ) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RTC_Init 2 */

  /* USER CODE END RTC_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart3, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart3, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * @brief USB_OTG_FS Initialization Function
  * @param None
  * @retval None
  */
static void MX_USB_OTG_FS_PCD_Init(void)
{

  /* USER CODE BEGIN USB_OTG_FS_Init 0 */

  /* USER CODE END USB_OTG_FS_Init 0 */

  /* USER CODE BEGIN USB_OTG_FS_Init 1 */

  /* USER CODE END USB_OTG_FS_Init 1 */
  hpcd_USB_OTG_FS.Instance = USB_OTG_FS;
  hpcd_USB_OTG_FS.Init.dev_endpoints = 9;
  hpcd_USB_OTG_FS.Init.speed = PCD_SPEED_FULL;
  hpcd_USB_OTG_FS.Init.dma_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.phy_itface = PCD_PHY_EMBEDDED;
  hpcd_USB_OTG_FS.Init.Sof_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.low_power_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.lpm_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.battery_charging_enable = ENABLE;
  hpcd_USB_OTG_FS.Init.vbus_sensing_enable = ENABLE;
  hpcd_USB_OTG_FS.Init.use_dedicated_ep1 = DISABLE;
  if (HAL_PCD_Init(&hpcd_USB_OTG_FS) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USB_OTG_FS_Init 2 */

  /* USER CODE END USB_OTG_FS_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream0_IRQn);
  /* DMA1_Stream1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream1_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */


void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *AdcHandle)
{
	/*
    if(ctr < BUFFER_SIZE){
        PA6_Data[ctr++] = ADC_DualModeVal[0] & 0XFFFF;
        PA6_Data[ctr++] = (ADC_DualModeVal[0] >> 16) & 0XFFFF ;
        PA6_Data[ctr++] = ADC_DualModeVal[1] & 0XFFFF ;
        PA6_Data[ctr++] = (ADC_DualModeVal[1] >> 16) & 0XFFFF ;
		PA6_Data[ctr++] = ADC_DualModeVal[2] & 0XFFFF ;
		PA6_Data[ctr++] = (ADC_DualModeVal[2] >> 16) & 0XFFFF ;
		PA6_Data[ctr++] = ADC_DualModeVal[3] & 0XFFFF ;
		PA6_Data[ctr++] = (ADC_DualModeVal[3] >> 16) & 0XFFFF ;
        //ctr++;
    }
    else{
    	HAL_NVIC_DisableIRQ(DMA1_Stream0_IRQn);
    	//HAL_DMA_Abort(&hdma_adc1);
		//HAL_ADC_Stop(&hadc1);
		//HAL_ADC_Stop(&hadc2);
    	//HAL_ADCEx_MultiModeStop_DMA(&hdma_adc1);
    }
    */

	if( HAL_GetTick() > (b_shot + 1000))
	{
		b_shot  = HAL_GetTick();
		//HAL_NVIC_DisableIRQ(DMA1_Stream0_IRQn);
		rateB = rateA;
		rateA = 0;
	}
	else
	{
		rateA++;
	}

    if(gidxA < BFR_SIZE)
    {
    	buffer1[gidxA] = (ADC_DualModeVal[0] >> 16);
    	buffer2[gidxA] = (ADC_DualModeVal[1] >> 16);
    	gidxA++;
    }
    else
    {
    	HAL_NVIC_DisableIRQ(DMA1_Stream0_IRQn);
    	flag_buffer_complete = 1;
    	gidxA = 0;
    }
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
  hlpuart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  hlpuart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  hlpuart1.FifoMode = UART_FIFOMODE_DISABLE;
  if (HAL_UART_Init(&hlpuart1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&hlpuart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&hlpuart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&hlpuart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN LPUART1_Init 2 */

  /* USER CODE END LPUART1_Init 2 */

}
uint16_t period_signal[100] = {0};
static void TransferDataADC(void)
{
  uint16_t tmpMaxData=BUFFER_SIZE;

  RTC_DateTypeDef _date;
  RTC_TimeTypeDef _time;

  HAL_RTC_GetTime(&hrtc, &_time, RTC_FORMAT_BCD);
  HAL_RTC_GetDate(&hrtc, &_date, RTC_FORMAT_BCD);

#if !USE_FOR_LORA
  /* transfer raw data ADC */
   for(int i= 0;i<tmpMaxData;++i){
     // sprintf(str,"%d",PA6_Data[i]);
     temp = PA6_Data[i] >>8;
     HAL_UART_Transmit(&hlpuart1, &PA6_Data[i] , 1, -1);
     HAL_UART_Transmit(&hlpuart1, &temp, 1, -1);
     // myprintf("%d\n",PA6_Data[i]);
     //HAL_Delay(50);
   }
    // HAL_UART_Transmit(&hlpuart1, ';', 1, -1);
#else
       /* find period each cycles */
  for(int i= 1;i<=iPeak;++i){
      period_signal[i-1] = located_fallEdge[i] - located_fallEdge[i-1];
      // HAL_UART_Transmit(&hlpuart1, &period_signal[i-1] , 1, -1);
      // temp = period_signal[i-1] >>8;
      // HAL_UART_Transmit(&hlpuart1, &temp, 1, -1);
  }

  /* transfer peak data & cycles */
  for(int i= 0;i<iPeak;++i){
      // sprintf(str,"%d",PA6_Data[i]);

      temp = dataPeak[i] >>8;
      HAL_UART_Transmit(&hlpuart1, &dataPeak[i] , 1, 500);
      HAL_UART_Transmit(&hlpuart1, &temp, 1, 500);
      /* transfer location of peak in each cycles */
      HAL_UART_Transmit(&hlpuart1, &located_peak[i] , 1, 500);
      temp = located_peak[i] >>8;
      HAL_UART_Transmit(&hlpuart1, &temp, 1, 500);
      /* transfer period of each cycles */
      HAL_UART_Transmit(&hlpuart1, &period_signal[i] , 1, 500);
      temp = period_signal[i] >>8;
      HAL_UART_Transmit(&hlpuart1, &temp, 1, 500);

      // myprintf("%d\n",PA6_Data[i]);
      //HAL_Delay(50);
    }




    // temp = ';';
    // HAL_UART_Transmit(&hlpuart1, &temp, 1, -1);
    // HAL_UART_Transmit(&hlpuart1, &temp, 1, -1);

     /* transfer timestamp */
  	 HAL_UART_Transmit(&hlpuart1, &_time.Seconds , 1, -1);
  	 HAL_UART_Transmit(&hlpuart1, &_time.Minutes , 1, -1);
     HAL_UART_Transmit(&hlpuart1, &_time.Hours , 1, -1);
     HAL_UART_Transmit(&hlpuart1, &_date.Date , 1, -1);
    // HAL_UART_Transmit(&hlpuart1, &_date.Month , 1, -1);
    // HAL_UART_Transmit(&hlpuart1, &_date.Date , 1, -1);

     /* transfer End of data Number */
     temp = 0xAA;
     HAL_UART_Transmit(&hlpuart1, &temp , 1, -1);
     HAL_UART_Transmit(&hlpuart1, &temp , 1, -1);
#endif
    /* clear buffer */
    for(int i= 0;i<iPeak;++i){
      dataPeak[i] = 0;
      located_peak[i] = 0;
      located_fallEdge[i] = 0;
    }


}

void printBuffers(uint32_t *bfr1, uint32_t *bfr2, int size)
{
	int lidx = 0;

	for(lidx=0;lidx < size; lidx++)
	{
		myprintf2("%d, %d\r\n", bfr1[lidx], bfr2[lidx]);
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
