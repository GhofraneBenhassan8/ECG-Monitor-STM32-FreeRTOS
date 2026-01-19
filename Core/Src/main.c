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
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "ssd1306.h"
#include "ssd1306_fonts.h"
#include <math.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define ECG_BASELINE 38
#define WAVEFORM_TOP 14
#define WAVEFORM_BOTTOM 62
#define ADC_BUFFER_SIZE 1
#define FILTER_SIZE 5
#define SAMPLE_RATE 200
#define PEAK_THRESHOLD_BASE 200
#define ADAPTIVE_THRESHOLD_FACTOR 0.65f  // 65% of signal range
#define MIN_RR_INTERVAL 300  // 200 BPM max
#define MAX_RR_INTERVAL 2000  // 30 BPM min
#define REFRACTORY_PERIOD 200  // 200ms refractory period (prevents double counting)

// High-pass filter parameters_DC blocking at 0.5 Hz
#define HPF_ALPHA 0.995f
#define EMA_ALPHA 0.98f
#define STABILIZATION_SAMPLES 600  // 3 seconds @ 200Hz


#define HR_AVERAGE_SIZE 5
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

I2C_HandleTypeDef hi2c1;

/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* USER CODE BEGIN PV */

// Heart bitmap - 16x16 pixels
const unsigned char heart_icon_16x16[] = {
  0x00, 0x00, 0x1C, 0x70, 0x3E, 0xF8, 0x7F, 0xFC,
  0xFF, 0xFE, 0xFF, 0xFE, 0xFF, 0xFE, 0x7F, 0xFC,
  0x7F, 0xFC, 0x3F, 0xF8, 0x1F, 0xF0, 0x0F, 0xE0,
  0x07, 0xC0, 0x03, 0x80, 0x01, 0x00, 0x00, 0x00
};

// Small heart bitmap - 8x8 pixels (for beat indicator)
const unsigned char heart_small_8x8[] = {
  0x00, 0x66, 0xFF, 0xFF, 0xFF, 0x7E, 0x3C, 0x18
};

// Large heart for animation - 24x24 pixels
const unsigned char heart_large_24x24[] = {
  0x00, 0x00, 0x00, 0x07, 0x0E, 0x00, 0x0F, 0x9F, 0x00, 0x1F, 0xFF, 0x80,
  0x3F, 0xFF, 0xC0, 0x7F, 0xFF, 0xE0, 0xFF, 0xFF, 0xF0, 0xFF, 0xFF, 0xF0,
  0xFF, 0xFF, 0xF0, 0xFF, 0xFF, 0xF0, 0xFF, 0xFF, 0xF0, 0x7F, 0xFF, 0xE0,
  0x7F, 0xFF, 0xE0, 0x3F, 0xFF, 0xC0, 0x1F, 0xFF, 0x80, 0x0F, 0xFF, 0x00,
  0x07, 0xFE, 0x00, 0x03, 0xFC, 0x00, 0x01, 0xF8, 0x00, 0x00, 0xF0, 0x00,
  0x00, 0x60, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};

typedef struct {
    uint16_t adc_value;
    uint32_t timestamp;
} ADC_Data_t;

typedef enum {
    HR_NORMAL,
    HR_BRADYCARDIA,
    HR_TACHYCARDIA
} HR_Status_t;

typedef struct {
    int16_t waveform[128];
    uint8_t buffer_index;
    uint32_t heart_rate;
    uint8_t lead_off;
    uint8_t signal_quality;
    uint16_t raw_adc;
    float hrv_sdnn;
    uint8_t is_stable;
} ECG_Data_t;

ECG_Data_t ecg_data = {0};
HR_Status_t AnalyzeHeartRate(uint32_t hr);

osThreadId_t ADC_TaskHandle;
osThreadId_t Process_TaskHandle;
osThreadId_t Display_TaskHandle;
osThreadId_t Monitor_TaskHandle;

osMessageQueueId_t ADC_QueueHandle;
osMutexId_t ECG_Data_MutexHandle;
osSemaphoreId_t Display_SemaphoreHandle;

uint16_t adc_dma_buffer[4];
uint16_t filter_buffer[FILTER_SIZE] = {2048, 2048, 2048, 2048, 2048};
uint8_t filter_index = 0;
uint32_t last_peak_time = 0;
uint32_t rr_intervals[10] = {0};
uint8_t rr_index = 0;
uint8_t peak_detected = 0;
uint16_t prev_filtered = 2048;
uint8_t above_threshold = 0;
uint8_t lead_off_detected = 0;


float hpf_prev_input = 2048.0f;
float hpf_prev_output = 0.0f;


float ema_baseline = 2048.0f;


volatile uint32_t stabilization_counter = STABILIZATION_SAMPLES;
volatile uint8_t system_stable = 1;


float signal_min = 4096.0f;
float signal_max = 0.0f;
uint16_t adaptive_samples = 0;
#define ADAPTIVE_WINDOW 100


uint32_t hr_history[HR_AVERAGE_SIZE] = {0};
uint8_t hr_history_index = 0;
uint8_t hr_history_count = 0;


uint32_t last_valid_peak_time = 0;
float peak_value_threshold = 0.0f;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_I2C1_Init(void);
static void MX_ADC1_Init(void);
void StartDefaultTask(void *argument);

/* USER CODE BEGIN PFP */
void ADC_Task(void *argument);
void Process_Task(void *argument);
void Display_Task(void *argument);
void Monitor_Task(void *argument);

uint16_t MedianFilter(uint16_t new_value);
float HighPassFilter(uint16_t input);
void CalculateHeartRate(float filtered_val, uint32_t timestamp);
float CalculateHRV(void);
uint8_t CheckLeadOff(void);
uint8_t AssessSignalQuality(void);
uint32_t GetAveragedHeartRate(void);
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
  MX_I2C1_Init();
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */
  ssd1306_Init();
  HAL_Delay(100);


  for(int beat = 0; beat < 2; beat++) {
    ssd1306_Fill(Black);


    if(beat % 2 == 0) {

      ssd1306_DrawBitmap(52, 20, heart_large_24x24, 24, 24, White);
    } else {

      ssd1306_DrawBitmap(51, 19, heart_large_24x24, 24, 24, White);
      ssd1306_DrawBitmap(52, 20, heart_large_24x24, 24, 24, White);
    }

    ssd1306_UpdateScreen();
    HAL_Delay(300);
  }

  HAL_Delay(400);

  ssd1306_Fill(Black);

  ssd1306_DrawBitmap(7, 5, heart_icon_16x16, 16, 16, White);

  ssd1306_SetCursor(32, 5);
  ssd1306_WriteString("ECG", Font_11x18, White);
  ssd1306_SetCursor(8, 24);
  ssd1306_WriteString("MONITOR", Font_11x18, White);

  for(int x = 0; x < 128; x += 2) {
    ssd1306_DrawPixel(x, 44, White);
  }

  ssd1306_SetCursor(4, 48);
  ssd1306_WriteString("Ghofrane & Esra", Font_7x10, White);

  ssd1306_UpdateScreen();
  HAL_Delay(2500);

  // Clear screen before starting signal display
  ssd1306_Fill(Black);
  ssd1306_UpdateScreen();

  for(int i = 0; i < 128; i++) {
     ecg_data.waveform[i] = ECG_BASELINE;
  }
  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  ECG_Data_MutexHandle = osMutexNew(NULL);
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  Display_SemaphoreHandle = osSemaphoreNew(1, 0, NULL);
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  ADC_QueueHandle = osMessageQueueNew(100, sizeof(ADC_Data_t), NULL);
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  const osThreadAttr_t ADC_Task_attr = {.name = "ADC_Task", .stack_size = 256 * 4, .priority = osPriorityHigh};
  const osThreadAttr_t Process_Task_attr = {.name = "Process_Task", .stack_size = 512 * 4, .priority = osPriorityAboveNormal};
  const osThreadAttr_t Display_Task_attr = {.name = "Display_Task", .stack_size = 512 * 4, .priority = osPriorityNormal};
  const osThreadAttr_t Monitor_Task_attr = {.name = "Monitor_Task", .stack_size = 256 * 4, .priority = osPriorityHigh};

  ADC_TaskHandle = osThreadNew(ADC_Task, NULL, &ADC_Task_attr);
  Process_TaskHandle = osThreadNew(Process_Task, NULL, &Process_Task_attr);
  Display_TaskHandle = osThreadNew(Display_Task, NULL, &Display_Task_attr);
  Monitor_TaskHandle = osThreadNew(Monitor_Task, NULL, &Monitor_Task_attr);
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

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
  RCC_OscInitStruct.PLL.PLLN = 168;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
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

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }


  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_84CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

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
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pins : LO_PLUS_Pin LO_MINUS_Pin */
  GPIO_InitStruct.Pin = LO_PLUS_Pin|LO_MINUS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void ADC_Task(void *argument)
{
  ADC_Data_t adc_data;
  HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_dma_buffer, ADC_BUFFER_SIZE);

  for(;;)
  {
    if(!lead_off_detected) {
      adc_data.adc_value = adc_dma_buffer[0];
      adc_data.timestamp = osKernelGetTickCount();
      osMessageQueuePut(ADC_QueueHandle, &adc_data, 0, 0);
    }
    osDelay(5);  // 200 Hz sampling rate
  }
}

float HighPassFilter(uint16_t input)
{
  float input_f = (float)input;
  float output = HPF_ALPHA * (hpf_prev_output + input_f - hpf_prev_input);
  hpf_prev_input = input_f;
  hpf_prev_output = output;
  return output;
}

void Process_Task(void *argument)
{
  ADC_Data_t adc_data;
  float hpf_output;
  int16_t scaled_value;

  for(;;)
  {
    if(osMessageQueueGet(ADC_QueueHandle, &adc_data, NULL, osWaitForever) == osOK)
    {
      if(lead_off_detected) {
        osDelay(10);
        continue;
      }

      // 1: Apply median filter to remove spikes
      uint16_t median_filtered = MedianFilter(adc_data.adc_value);

      // 2: Apply high-pass filter for DC blocking
      hpf_output = HighPassFilter(median_filtered);

      // 3: Update adaptive baseline using exponential moving average
      ema_baseline = EMA_ALPHA * ema_baseline + (1.0f - EMA_ALPHA) * (float)median_filtered;

      // 4: Calculate AC signal
      float ac_signal = hpf_output;

      // 5: Scale for display
      scaled_value = ECG_BASELINE - (int16_t)(ac_signal * 15.0f / 500.0f);


      if(scaled_value < 2) scaled_value = 2;
      if(scaled_value >= SCREEN_HEIGHT - 2) scaled_value = SCREEN_HEIGHT - 3;

      // 6: Update adaptive threshold
      adaptive_samples++;
      if(adaptive_samples >= ADAPTIVE_WINDOW) {
        adaptive_samples = 0;
        signal_min = 4096.0f;
        signal_max = 0.0f;
      }

      float abs_signal = fabsf(ac_signal);
      if(abs_signal < signal_min) signal_min = abs_signal;
      if(abs_signal > signal_max) signal_max = abs_signal;

      // 7: Heart rate detection with adaptive threshold
      CalculateHeartRate(ac_signal, adc_data.timestamp);

      // 8: Update shared data structure
      if(osMutexAcquire(ECG_Data_MutexHandle, 10) == osOK)
      {
        ecg_data.waveform[ecg_data.buffer_index] = scaled_value;
        ecg_data.buffer_index = (ecg_data.buffer_index + 1) % SCREEN_WIDTH;
        ecg_data.raw_adc = adc_data.adc_value;
        ecg_data.signal_quality = AssessSignalQuality();

        // Update heart rate with averaged value
        ecg_data.heart_rate = GetAveragedHeartRate();

        if(rr_index >= 10) {
          ecg_data.hrv_sdnn = CalculateHRV();
        }

        osMutexRelease(ECG_Data_MutexHandle);
        osSemaphoreRelease(Display_SemaphoreHandle);
      }
    }
  }
}

HR_Status_t AnalyzeHeartRate(uint32_t hr) {
    if(hr < 60) return HR_BRADYCARDIA;
    if(hr > 100) return HR_TACHYCARDIA;
    return HR_NORMAL;
}

void Display_Task(void *argument)
{
  char str_buffer[32];
  uint32_t last_update = 0;

  for(;;)
  {
    if(osSemaphoreAcquire(Display_SemaphoreHandle, osWaitForever) == osOK)
    {
      uint32_t current_time = osKernelGetTickCount();
      if(current_time - last_update < 50) {
        osDelay(50 - (current_time - last_update));
      }
      last_update = osKernelGetTickCount();

      // Priority: Show lead off message immediately
      if(ecg_data.lead_off) {
        ssd1306_Fill(Black);
        ssd1306_SetCursor(10, 20);
        ssd1306_WriteString("LEADS OFF", Font_11x18, White);
        ssd1306_SetCursor(5, 45);
        ssd1306_WriteString("Check Electrodes", Font_7x10, White);
        ssd1306_UpdateScreen();
        continue;
      }

      if(osMutexAcquire(ECG_Data_MutexHandle, 10) == osOK)
      {
        ssd1306_Fill(Black);

        // Draw baseline reference
        for(int x = 0; x < SCREEN_WIDTH; x += 8) {
          ssd1306_DrawPixel(x, ECG_BASELINE, White);
        }

        // Draw waveform
        for(int i = 0; i < SCREEN_WIDTH - 1; i++) {
          int buf_idx1 = (ecg_data.buffer_index + i) % SCREEN_WIDTH;
          int buf_idx2 = (ecg_data.buffer_index + i + 1) % SCREEN_WIDTH;
          ssd1306_Line(i, ecg_data.waveform[buf_idx1], i + 1, ecg_data.waveform[buf_idx2], White);
        }

        // Display HR with status
        if(ecg_data.heart_rate > 30 && ecg_data.heart_rate < 220) {
          snprintf(str_buffer, sizeof(str_buffer), "HR:%lu", ecg_data.heart_rate);
          ssd1306_SetCursor(2, 0);
          ssd1306_WriteString(str_buffer, Font_7x10, White);

          HR_Status_t status = AnalyzeHeartRate(ecg_data.heart_rate);
          if(status == HR_BRADYCARDIA) {
            ssd1306_SetCursor(50, 0);
            ssd1306_WriteString("BRADY", Font_7x10, White);
          } else if(status == HR_TACHYCARDIA) {
            ssd1306_SetCursor(50, 0);
            ssd1306_WriteString("TACHY", Font_7x10, White);
          }

          // Heart beat indicator using small bitmap
          if(peak_detected) {
            ssd1306_DrawBitmap(116, 2, heart_small_8x8, 8, 8, White);
            peak_detected = 0;
          }
        }

        // Signal quality indicator
        if(ecg_data.signal_quality > 60) {
          ssd1306_FillCircle(SCREEN_WIDTH - 5, 5, 2, White);
        }

        osMutexRelease(ECG_Data_MutexHandle);
        ssd1306_UpdateScreen();
      }
    }
  }
}

void Monitor_Task(void *argument)
{
  uint8_t lead_off_count = 0;
  const uint8_t LEAD_OFF_THRESHOLD = 3;

  for(;;)
  {
    uint8_t lo_status = CheckLeadOff();


    if(lo_status) {
      lead_off_count++;
      if(lead_off_count >= LEAD_OFF_THRESHOLD) {
        lead_off_detected = 1;
      }
    } else {
      if(lead_off_count > 0) {
        lead_off_count--;
      }
      if(lead_off_count == 0) {
        lead_off_detected = 0;
      }
    }

    if(osMutexAcquire(ECG_Data_MutexHandle, 10) == osOK)
    {
      ecg_data.lead_off = lead_off_detected;
      osMutexRelease(ECG_Data_MutexHandle);
    }

    osDelay(20);
  }
}

uint16_t MedianFilter(uint16_t new_value)
{
    filter_buffer[filter_index] = new_value;
    filter_index = (filter_index + 1) % FILTER_SIZE;

    uint16_t sorted[FILTER_SIZE];
    memcpy(sorted, filter_buffer, sizeof(sorted));


    for(int i = 0; i < FILTER_SIZE-1; i++) {
        for(int j = 0; j < FILTER_SIZE-i-1; j++) {
            if(sorted[j] > sorted[j+1]) {
                uint16_t temp = sorted[j];
                sorted[j] = sorted[j+1];
                sorted[j+1] = temp;
            }
        }
    }

    return sorted[FILTER_SIZE/2];
}


void CalculateHeartRate(float filtered_val, uint32_t timestamp)
{

  float adaptive_threshold = PEAK_THRESHOLD_BASE;
  if(signal_max > signal_min) {
    adaptive_threshold = (signal_max - signal_min) * ADAPTIVE_THRESHOLD_FACTOR;
    if(adaptive_threshold < 150.0f) adaptive_threshold = 150.0f;
    if(adaptive_threshold > 600.0f) adaptive_threshold = 600.0f;
  }

  float abs_signal = fabsf(filtered_val);
  static float prev_abs_signal = 0.0f;
  static float prev_prev_signal = 0.0f;


  uint32_t time_since_last_peak = timestamp - last_valid_peak_time;
  uint8_t in_refractory = (time_since_last_peak < REFRACTORY_PERIOD);

  // peak: must be local maximum AND above threshold AND not in refractory
  uint8_t is_local_max = (abs_signal > prev_abs_signal) && (prev_abs_signal > prev_prev_signal);
  uint8_t is_above_threshold = (abs_signal > adaptive_threshold);

  if(is_local_max && is_above_threshold && !in_refractory && !above_threshold) {
    above_threshold = 1;

    if(last_peak_time > 0) {
      uint32_t rr_interval = timestamp - last_peak_time;

      // Validate RR interval
      if(rr_interval >= MIN_RR_INTERVAL && rr_interval <= MAX_RR_INTERVAL) {
        rr_intervals[rr_index] = rr_interval;
        rr_index = (rr_index + 1) % 10;

        // Calculate HR from this RR interval
        uint32_t calculated_hr = 60000 / rr_interval;

        // Add to history for averaging
        if(calculated_hr >= 30 && calculated_hr <= 220) {
          hr_history[hr_history_index] = calculated_hr;
          hr_history_index = (hr_history_index + 1) % HR_AVERAGE_SIZE;
          if(hr_history_count < HR_AVERAGE_SIZE) {
            hr_history_count++;
          }
          peak_detected = 1;
        }
      }
    }
    last_peak_time = timestamp;
    last_valid_peak_time = timestamp;  // Update valid peak time
  }
  // Reset when signal drops significantly
  else if(abs_signal < (adaptive_threshold * 0.3f)) {
    above_threshold = 0;
  }

  prev_prev_signal = prev_abs_signal;
  prev_abs_signal = abs_signal;
}
uint32_t GetAveragedHeartRate(void)
{
  if(hr_history_count == 0) {
    return 0;
  }

  uint32_t sum = 0;
  for(int i = 0; i < hr_history_count; i++) {
    sum += hr_history[i];
  }

  return sum / hr_history_count;
}

float CalculateHRV(void)
{
  float mean = 0;
  for(int i = 0; i < 10; i++) {
    mean += rr_intervals[i];
  }
  mean /= 10.0f;

  float variance = 0;
  for(int i = 0; i < 10; i++) {
    float diff = rr_intervals[i] - mean;
    variance += diff * diff;
  }
  variance /= 10.0f;
  return sqrtf(variance);
}

uint8_t CheckLeadOff(void)
{
  uint8_t lo_plus = HAL_GPIO_ReadPin(LO_PLUS_GPIO_Port, LO_PLUS_Pin);
  uint8_t lo_minus = HAL_GPIO_ReadPin(LO_MINUS_GPIO_Port, LO_MINUS_Pin);
  return (lo_plus == GPIO_PIN_SET || lo_minus == GPIO_PIN_SET);
}

uint8_t AssessSignalQuality(void)
{
  int32_t min_val = 4096, max_val = 0;

  for(int i = 0; i < FILTER_SIZE; i++) {
    if(filter_buffer[i] < min_val) min_val = filter_buffer[i];
    if(filter_buffer[i] > max_val) max_val = filter_buffer[i];
  }

  int32_t range = max_val - min_val;

  if(range > 200 && range < 1500) return 80;
  if(range > 100 && range < 2000) return 60;
  if(range > 50) return 40;
  return 20;
}

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN 5 */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END 5 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
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
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
